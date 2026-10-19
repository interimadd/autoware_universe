// Copyright 2021 Tier IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/trajectory_follower_node/trajectory_follower_node.hpp"

#include "autoware/mpc_lateral_controller/mpc_utils.hpp"
#include "autoware/trajectory_follower_node/controller_factory.hpp"
#include "autoware/trajectory_follower_node/create_config.hpp"
#include "autoware_utils/ros/marker_helper.hpp"

#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control::trajectory_follower_node
{
using mpc_lateral_controller::MPCParam;

TrajectoryFollowerNode::TrajectoryFollowerNode(const rclcpp::NodeOptions & node_options)
: Node("controller", node_options),
  diag_updater_(std::make_shared<diagnostic_updater::Updater>(this))
{
  config_ = createConfig(*this);
  const auto & config = config_;
  cyclic_message_timeout_thr_sec_ = config.cyclic_message_timeout_thr_sec;
  enable_control_cmd_horizon_pub_ = config.enable_control_cmd_horizon;

  diag_updater_->setHardwareID("trajectory_follower_node");
  diag_updater_->add(
    "incoming_message_timeout", this, &TrajectoryFollowerNode::check_cyclic_message_timeout);

  log_writer_ =
    std::make_unique<RosLogWriter>(get_logger().get_child("lateral_controller"), get_clock());

  auto lateral = createLateralController(config, *log_writer_);
  auto longitudinal = createLongitudinalController(config);
  mpc_ = lateral.mpc;
  pure_pursuit_ = lateral.pure_pursuit;
  pid_ = longitudinal.pid;

  trajectory_follower_ = std::make_unique<TrajectoryFollower>(
    config, std::move(lateral.controller), std::move(longitudinal.controller));

  setupControllerTopics();

  control_cmd_pub_ = create_publisher<autoware_control_msgs::msg::Control>(
    "~/output/control_cmd", rclcpp::QoS{1}.transient_local());
  pub_processing_time_lat_ms_ =
    create_publisher<Float64Stamped>("~/lateral/debug/processing_time_ms", 1);
  pub_processing_time_lon_ms_ =
    create_publisher<Float64Stamped>("~/longitudinal/debug/processing_time_ms", 1);
  debug_marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/output/debug_marker", rclcpp::QoS{1});

  sub_steering_offset_update_ =
    create_subscription<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "~/input/steering_offset_update", rclcpp::QoS{1}.transient_local(),
      [this](const autoware_internal_debug_msgs::msg::Float32Stamped::ConstSharedPtr msg) {
        steering_offset_ = static_cast<double>(msg->data);
      });

  if (enable_control_cmd_horizon_pub_) {
    control_cmd_horizon_pub_ = create_publisher<autoware_control_msgs::msg::ControlHorizon>(
      "~/debug/control_cmd_horizon", 1);
  }

  // Timer
  {
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(config.ctrl_period));
    timer_control_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&TrajectoryFollowerNode::callbackTimerControl, this));
  }

  logger_configure_ = std::make_unique<autoware_utils::LoggerLevelConfigure>(this);

  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);
}

void TrajectoryFollowerNode::check_cyclic_message_timeout(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const auto traj_timestamp = sub_ref_path_.last_taken_data_timestamp();

  if (!traj_timestamp) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "timeout");
    stat.add("trajectory", "no message received");
    return;
  }

  const auto elapsed = (this->now() - traj_timestamp.value()).seconds();
  if (elapsed > cyclic_message_timeout_thr_sec_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "timeout");
    stat.add("trajectory", "timeout (elapsed: " + std::to_string(elapsed) + " sec)");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
    stat.add("trajectory", "OK");
  }
}

bool TrajectoryFollowerNode::processData(rclcpp::Clock & clock)
{
  bool is_ready = true;

  const auto & logData = [&clock, this](const std::string & data_type) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), clock, logger_throttle_interval, "Waiting for %s data", data_type.c_str());
  };

  const auto & getData = [&logData](auto & dest, auto & sub, const std::string & data_type = "") {
    const auto temp = sub.take_data();
    if (temp) {
      dest = temp;
      return true;
    }
    if (!data_type.empty()) logData(data_type);
    return false;
  };

  is_ready &= getData(current_accel_ptr_, sub_accel_, "acceleration");
  is_ready &= getData(current_steering_ptr_, sub_steering_, "steering");
  is_ready &= getData(current_trajectory_ptr_, sub_ref_path_, "trajectory");
  is_ready &= getData(current_odometry_ptr_, sub_odometry_, "odometry");
  is_ready &= getData(current_operation_mode_ptr_, sub_operation_mode_, "operation mode");

  return is_ready;
}

std::optional<InputData> TrajectoryFollowerNode::createInputData(rclcpp::Clock & clock)
{
  if (!processData(clock)) {
    return {};
  }

  InputData input_data;
  input_data.current_trajectory = *current_trajectory_ptr_;
  input_data.current_odometry = *current_odometry_ptr_;
  input_data.current_steering = *current_steering_ptr_;
  input_data.current_accel = *current_accel_ptr_;
  input_data.current_operation_mode = *current_operation_mode_ptr_;
  input_data.steering_offset = steering_offset_;

  return input_data;
}

void TrajectoryFollowerNode::callbackTimerControl()
{
  const auto stamp = this->now();

  // 1. create input data
  const auto input_data = createInputData(*get_clock());
  if (!input_data) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), logger_throttle_interval,
      "Control is skipped since input data is not ready.");
    return;
  }

  // 2. run the follower
  const auto result = trajectory_follower_->run(*input_data, stamp);

  // 3. publish how long each controller took, whether or not it produced a command
  if (result) {
    publishProcessingTime(result->lateral_processing_time_ms, pub_processing_time_lat_ms_);
    publishProcessingTime(result->longitudinal_processing_time_ms, pub_processing_time_lon_ms_);
  }

  // 4. update diagnostics
  diag_updater_->force_update();

  if (!result) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), logger_throttle_interval, "Control is skipped since %s.",
      result.error().c_str());
    return;
  }

  // 5. publish control command
  control_cmd_pub_->publish(result->control_cmd);

  // 5.5. write out what the controllers produced
  publishControllerOutput();

  // 6. publish debug
  published_time_publisher_->publish_if_subscribed(control_cmd_pub_, result->control_cmd.stamp);
  publishDebugMarker(*input_data, *result);

  // 7. publish experimental topic
  if (enable_control_cmd_horizon_pub_ && !result->control_cmd_horizon.controls.empty()) {
    control_cmd_horizon_pub_->publish(result->control_cmd_horizon);
  }
}

void TrajectoryFollowerNode::publishDebugMarker(
  const InputData & input_data, const TrajectoryFollowerResult & result) const
{
  visualization_msgs::msg::MarkerArray debug_marker_array{};

  // steer converged marker
  {
    auto marker = autoware_utils::create_default_marker(
      "map", this->now(), "steer_converged", 0, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      autoware_utils::create_marker_scale(0.0, 0.0, 1.0),
      autoware_utils::create_marker_color(1.0, 1.0, 1.0, 0.99));
    marker.pose = input_data.current_odometry.pose.pose;

    std::stringstream ss;
    const double current = input_data.current_steering.steering_tire_angle;
    const double cmd = result.control_cmd.lateral.steering_tire_angle;
    const double diff = current - cmd;
    ss << "current:" << current << " cmd:" << cmd << " diff:" << diff
       << (result.lateral_feedback.is_steer_converged ? " (converged)" : " (not converged)");
    marker.text = ss.str();

    debug_marker_array.markers.push_back(marker);
  }

  debug_marker_pub_->publish(debug_marker_array);
}

void TrajectoryFollowerNode::publishProcessingTime(
  const double t_ms, const rclcpp::Publisher<Float64Stamped>::SharedPtr pub)
{
  Float64Stamped msg{};
  msg.stamp = this->now();
  msg.data = t_ms;
  pub->publish(msg);
}

void TrajectoryFollowerNode::setupControllerTopics()
{
  using std::placeholders::_1;

  if (mpc_) {
    pub_mpc_predicted_traj_ = create_publisher<Trajectory>("~/output/predicted_trajectory", 1);
    pub_mpc_predicted_traj_frenet_ =
      create_publisher<Trajectory>("~/debug/predicted_trajectory_in_frenet_coordinate", 1);
    pub_mpc_resampled_reference_traj_ =
      create_publisher<Trajectory>("~/debug/resampled_reference_trajectory", 1);
    pub_mpc_nearest_pose_ = create_publisher<PoseStamped>("~/debug/nearest_pose", 1);
    pub_mpc_nearest_segment_traj_ = create_publisher<Trajectory>("~/debug/nearest_segment", 1);
    pub_mpc_nearest_info_ = create_publisher<Float32MultiArrayStamped>("~/debug/nearest_info", 1);
    pub_mpc_debug_values_ =
      create_publisher<Float32MultiArrayStamped>("~/output/lateral_diagnostic", 1);
    pub_mpc_steer_offset_ = create_publisher<Float32Stamped>("~/output/estimated_steer_offset", 1);

    diag_updater_->add("MPC_solve_checker", this, &TrajectoryFollowerNode::checkMpcSolve);
  }

  if (pure_pursuit_) {
    pub_pure_pursuit_debug_marker_ = create_publisher<MarkerArray>("~/debug/markers", 0);
    pub_pure_pursuit_debug_values_ =
      create_publisher<Float32MultiArrayStamped>("~/debug/ld_outputs", rclcpp::QoS{1});
    pub_pure_pursuit_predicted_traj_ =
      create_publisher<Trajectory>("~/output/predicted_trajectory", 1);
  }

  if (pid_) {
    pub_pid_slope_ =
      create_publisher<Float32MultiArrayStamped>("~/output/slope_angle", rclcpp::QoS{1});
    pub_pid_debug_ = create_publisher<Float32MultiArrayStamped>(
      "~/output/longitudinal_diagnostic", rclcpp::QoS{1});
    pub_pid_virtual_wall_marker_ = create_publisher<MarkerArray>("~/virtual_wall", 1);

    diag_updater_->add("control_state", this, &TrajectoryFollowerNode::checkControlState);
  }

  set_param_res_ =
    add_on_set_parameters_callback(std::bind(&TrajectoryFollowerNode::paramCallback, this, _1));
}

void TrajectoryFollowerNode::publishControllerOutput()
{
  if (mpc_) {
    const auto & result = mpc_->lastResult();

    pub_mpc_predicted_traj_->publish(result.mpc.predicted_trajectory);

    if (result.mpc.debug_msgs) {
      const auto & debug_msgs = *result.mpc.debug_msgs;
      pub_mpc_predicted_traj_frenet_->publish(debug_msgs.predicted_trajectory_frenet);
      pub_mpc_resampled_reference_traj_->publish(debug_msgs.resampled_reference_trajectory);
      pub_mpc_nearest_pose_->publish(debug_msgs.nearest_pose);
      pub_mpc_nearest_segment_traj_->publish(debug_msgs.nearest_segment_trajectory);
      pub_mpc_nearest_info_->publish(debug_msgs.nearest_info);
    }

    pub_mpc_debug_values_->publish(result.mpc.diagnostic);
    pub_mpc_steer_offset_->publish(result.steering_offset);
  }

  if (pure_pursuit_) {
    const auto & result = pure_pursuit_->lastResult();
    const auto lateral_logger = get_logger().get_child("lateral_controller");

    if (result.trajectory_too_short_to_smooth) {
      RCLCPP_ERROR(lateral_logger, "Cannot smooth path! Trajectory size is too low!");
    }
    if (result.curvature_calculation_failed) {
      RCLCPP_WARN(lateral_logger, "failed to calculate curvature");
    }
    if (result.received_short_trajectory) {
      RCLCPP_WARN_THROTTLE(
        lateral_logger, *get_clock(), 5000, "received path size is < 3, ignored");
    }
    if (result.closest_waypoint_not_found) {
      RCLCPP_ERROR(lateral_logger, "cannot find closest waypoint");
    }
    if (result.control_command_solve_failed) {
      RCLCPP_WARN_THROTTLE(
        lateral_logger, *get_clock(), 5000,
        "failed to solve pure_pursuit for control command calculation");
    }
    if (result.prediction_solve_failed) {
      RCLCPP_WARN_THROTTLE(
        lateral_logger, *get_clock(), 5000, "failed to solve pure_pursuit for prediction");
    }
    if (!result.predicted_trajectory) {
      RCLCPP_ERROR(lateral_logger, "Failed to generate predicted trajectory.");
    }

    pub_pure_pursuit_debug_values_->publish(result.lookahead_distance_values);
    if (result.debug_markers) {
      pub_pure_pursuit_debug_marker_->publish(*result.debug_markers);
    }
    if (result.predicted_trajectory) {
      pub_pure_pursuit_predicted_traj_->publish(*result.predicted_trajectory);
    }
  }

  if (pid_) {
    const auto & result = pid_->lastResult();
    const auto longitudinal_logger = get_logger().get_child("longitudinal_controller");
    pid_control_state_ = result.control_state;

    if (result.received_invalid_trajectory) {
      RCLCPP_ERROR_THROTTLE(
        longitudinal_logger, *get_clock(), 3000, "received invalid trajectory. ignore.");
    }
    if (result.emergency_stop_reason) {
      RCLCPP_ERROR(
        longitudinal_logger, "Emergency Stop since %s", result.emergency_stop_reason->c_str());
    }

    pub_pid_debug_->publish(result.debug_message);
    pub_pid_slope_->publish(result.slope_message);
    if (result.virtual_wall_marker) {
      pub_pid_virtual_wall_marker_->publish(*result.virtual_wall_marker);
    }
  }
}

void TrajectoryFollowerNode::checkMpcSolve(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const auto & mpc_result = mpc_->lastResult().mpc;
  if (mpc_result.result) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "MPC succeeded.");
  } else {
    const std::string error_msg = "MPC failed due to " + mpc_result.reason;
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, error_msg);
  }
}

void TrajectoryFollowerNode::checkControlState(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  auto level = DiagnosticStatus::OK;
  std::string msg = "OK";

  if (pid_control_state_ == pid_longitudinal_controller::ControlState::EMERGENCY) {
    level = DiagnosticStatus::ERROR;
    msg = "emergency occurred due to ";
  }

  stat.add<int32_t>("control_state", static_cast<int32_t>(pid_control_state_));
  stat.summary(level, msg);
}

rcl_interfaces::msg::SetParametersResult TrajectoryFollowerNode::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  if (mpc_) {
    // strong exception safety wrt MPCParam
    MPCParam param = mpc_->mpcParam();

    using mpc_lateral_controller::MPCUtils::update_param;
    try {
      auto & nw = param.nominal_weight;
      auto & lcw = param.low_curvature_weight;

      update_param(parameters, "mpc_prediction_horizon", param.prediction_horizon);
      update_param(parameters, "mpc_prediction_dt", param.prediction_dt);

      const std::string ns_nw = "mpc_weight_";
      update_param(parameters, ns_nw + "lat_error", nw.lat_error);
      update_param(parameters, ns_nw + "heading_error", nw.heading_error);
      update_param(parameters, ns_nw + "heading_error_squared_vel", nw.heading_error_squared_vel);
      update_param(parameters, ns_nw + "steering_input", nw.steering_input);
      update_param(parameters, ns_nw + "steering_input_squared_vel", nw.steering_input_squared_vel);
      update_param(parameters, ns_nw + "lat_jerk", nw.lat_jerk);
      update_param(parameters, ns_nw + "steer_rate", nw.steer_rate);
      update_param(parameters, ns_nw + "steer_acc", nw.steer_acc);
      update_param(parameters, ns_nw + "terminal_lat_error", nw.terminal_lat_error);
      update_param(parameters, ns_nw + "terminal_heading_error", nw.terminal_heading_error);

      const std::string ns_lcw = "mpc_low_curvature_weight_";
      update_param(parameters, ns_lcw + "lat_error", lcw.lat_error);
      update_param(parameters, ns_lcw + "heading_error", lcw.heading_error);
      update_param(parameters, ns_lcw + "heading_error_squared_vel", lcw.heading_error_squared_vel);
      update_param(parameters, ns_lcw + "steering_input", lcw.steering_input);
      update_param(
        parameters, ns_lcw + "steering_input_squared_vel", lcw.steering_input_squared_vel);
      update_param(parameters, ns_lcw + "lat_jerk", lcw.lat_jerk);
      update_param(parameters, ns_lcw + "steer_rate", lcw.steer_rate);
      update_param(parameters, ns_lcw + "steer_acc", lcw.steer_acc);

      update_param(
        parameters, "mpc_low_curvature_thresh_curvature", param.low_curvature_thresh_curvature);

      update_param(parameters, "mpc_zero_ff_steer_deg", param.zero_ff_steer_deg);
      update_param(parameters, "mpc_acceleration_limit", param.acceleration_limit);
      update_param(parameters, "mpc_velocity_time_constant", param.velocity_time_constant);
      update_param(parameters, "mpc_min_prediction_length", param.min_prediction_length);

      update_param(parameters, "input_delay", param.input_delay);

      // transaction succeeds, now assign values
      mpc_->setMpcParam(param);
    } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
      result.successful = false;
      result.reason = e.what();
    }
  }

  if (pid_) {
    auto update_param = [&](const std::string & name, double & v) {
      auto it = std::find_if(
        parameters.cbegin(), parameters.cend(),
        [&name](const rclcpp::Parameter & parameter) { return parameter.get_name() == name; });
      if (it != parameters.cend()) {
        v = it->as_double();
        return true;
      }
      return false;
    };

    // delay compensation
    update_param("delay_compensation_time", config_.pid.delay_compensation_time);

    // state transition
    {
      auto & p = config_.pid.state_transition_params;
      update_param("drive_state_stop_dist", p.drive_state_stop_dist);
      update_param("stopping_state_stop_dist", p.stopping_state_stop_dist);
      update_param("stopped_state_entry_duration_time", p.stopped_state_entry_duration_time);
      update_param("stopped_state_entry_vel", p.stopped_state_entry_vel);
      update_param("stopped_state_entry_acc", p.stopped_state_entry_acc);
      update_param("emergency_state_overshoot_stop_dist", p.emergency_state_overshoot_stop_dist);
    }

    // drive state
    {
      auto & gains = config_.pid.pid_gains;
      update_param("kp", gains.kp);
      update_param("ki", gains.ki);
      update_param("kd", gains.kd);

      update_param("lpf_vel_error_gain", config_.pid.lpf_vel_error_gain);

      auto & limits = config_.pid.pid_limits;
      update_param("max_out", limits.max_out);
      update_param("min_out", limits.min_out);
      update_param("max_p_effort", limits.max_p_effort);
      update_param("min_p_effort", limits.min_p_effort);
      update_param("max_i_effort", limits.max_i_effort);
      update_param("min_i_effort", limits.min_i_effort);
      update_param("max_d_effort", limits.max_d_effort);
      update_param("min_d_effort", limits.min_d_effort);

      update_param(
        "current_vel_threshold_pid_integration", config_.pid.current_vel_threshold_pid_integrate);
      update_param(
        "time_threshold_before_pid_integration", config_.pid.time_threshold_before_pid_integrate);
      update_param("ff_scale_min", config_.pid.ff_scale_min);
      update_param("ff_scale_max", config_.pid.ff_scale_max);
    }

    // stopping state
    {
      auto & p = config_.pid.smooth_stop_params;
      update_param("smooth_stop_max_strong_acc", p.max_strong_acc);
      update_param("smooth_stop_min_strong_acc", p.min_strong_acc);
      update_param("smooth_stop_weak_acc", p.weak_acc);
      update_param("smooth_stop_weak_stop_acc", p.weak_stop_acc);
      update_param("smooth_stop_strong_stop_acc", p.strong_stop_acc);
      update_param("smooth_stop_max_fast_vel", p.min_fast_vel);
      update_param("smooth_stop_min_running_vel", p.min_running_vel);
      update_param("smooth_stop_min_running_acc", p.min_running_acc);
      update_param("smooth_stop_weak_stop_time", p.weak_stop_time);
      update_param("smooth_stop_weak_stop_dist", p.weak_stop_dist);
      update_param("smooth_stop_strong_stop_dist", p.strong_stop_dist);
    }

    // stop state
    {
      auto & p = config_.pid.stopped_state_params;
      update_param("stopped_vel", p.vel);
      update_param("stopped_acc", p.acc);
    }

    // emergency state
    {
      auto & p = config_.pid.emergency_state_params;
      update_param("emergency_vel", p.vel);
      update_param("emergency_acc", p.acc);
      update_param("emergency_jerk", p.jerk);
    }

    // acceleration feedback
    update_param("acc_feedback_gain", config_.pid.acc_feedback_gain);
    update_param("lpf_acc_error_gain", config_.pid.lpf_acc_error_gain);

    // acceleration limit
    update_param("min_acc", config_.pid.min_acc);

    // jerk limit
    update_param("max_jerk", config_.pid.max_jerk);
    update_param("min_jerk", config_.pid.min_jerk);
    update_param("max_acc_cmd_diff", config_.pid.max_acc_cmd_diff);

    // slope compensation
    update_param("max_pitch_rad", config_.pid.max_pitch_rad);
    update_param("min_pitch_rad", config_.pid.min_pitch_rad);
    update_param("adaptive_trajectory_velocity_th", config_.pid.adaptive_trajectory_velocity_th);

    pid_->setConfig(config_.pid);
  }

  return result;
}

}  // namespace autoware::motion::control::trajectory_follower_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::motion::control::trajectory_follower_node::TrajectoryFollowerNode)

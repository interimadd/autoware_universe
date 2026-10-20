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

#include "autoware/trajectory_follower_node/controller_node.hpp"

#include "autoware/mpc_lateral_controller/mpc_lateral_controller_node.hpp"
#include "autoware/pid_longitudinal_controller/pid_longitudinal_controller_node.hpp"
#include "autoware/pure_pursuit/autoware_pure_pursuit_lateral_controller_node.hpp"
#include "autoware_utils/ros/marker_helper.hpp"

#include <autoware/trajectory_follower_base/lateral_controller.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
/// Build the lateral controller a mode names. Every lateral algorithm lives behind the
/// same interface, so this is the only place that knows which ones exist.
std::unique_ptr<autoware::motion::control::trajectory_follower::LateralController>
createLateralController(
  const std::string & mode, rclcpp::Node & node, diagnostic_updater::Updater & diag_updater)
{
  namespace mpc_lateral_controller = autoware::motion::control::mpc_lateral_controller;

  if (mode == "mpc") {
    return std::make_unique<mpc_lateral_controller::MpcLateralControllerNode>(node, diag_updater);
  }
  if (mode == "pure_pursuit") {
    return std::make_unique<autoware::pure_pursuit::PurePursuitLateralControllerNode>(node);
  }
  throw std::domain_error("[LateralController] invalid algorithm: " + mode);
}

/// Build the longitudinal controller a mode names. PID is the only one, but it reaches
/// the follower through the same interface as the lateral algorithms do.
std::unique_ptr<autoware::motion::control::trajectory_follower::LongitudinalController>
createLongitudinalController(
  const std::string & mode, rclcpp::Node & node, diagnostic_updater::Updater & diag_updater)
{
  namespace pid_longitudinal_controller = autoware::motion::control::pid_longitudinal_controller;

  if (mode == "pid") {
    return std::make_unique<pid_longitudinal_controller::PidLongitudinalControllerNode>(
      node, diag_updater);
  }
  throw std::domain_error("[LongitudinalController] invalid algorithm: " + mode);
}
}  // namespace

namespace autoware::motion::control::trajectory_follower_node
{
Controller::Controller(const rclcpp::NodeOptions & node_options)
: Node("controller", node_options),
  diag_updater_(std::make_shared<diagnostic_updater::Updater>(this))
{
  const double ctrl_period = declare_parameter<double>("ctrl_period");
  TrajectoryFollowerConfig trajectory_follower_config;
  trajectory_follower_config.timeout_thr_sec = declare_parameter<double>("timeout_thr_sec");
  cyclic_message_timeout_thr_sec_ = declare_parameter<double>("cyclic_message_timeout_thr_sec");
  // NOTE: It is possible that using control_horizon could be expected to enhance performance,
  // but it is not a formal interface topic, only an experimental one.
  // So it is disabled by default.
  enable_control_cmd_horizon_pub_ =
    declare_parameter<bool>("enable_control_cmd_horizon_pub", false);
  trajectory_follower_config.enable_control_cmd_horizon = enable_control_cmd_horizon_pub_;

  diag_updater_->setHardwareID("trajectory_follower_node");
  diag_updater_->add("incoming_message_timeout", this, &Controller::check_cyclic_message_timeout);

  auto lateral_controller = createLateralController(
    declare_parameter<std::string>("lateral_controller_mode"), *this, *diag_updater_);
  auto longitudinal_controller = createLongitudinalController(
    declare_parameter<std::string>("longitudinal_controller_mode"), *this, *diag_updater_);

  trajectory_follower_ = std::make_unique<TrajectoryFollower>(
    trajectory_follower_config, std::move(lateral_controller), std::move(longitudinal_controller));

  control_cmd_pub_ = create_publisher<autoware_control_msgs::msg::Control>(
    "~/output/control_cmd", rclcpp::QoS{1}.transient_local());
  predicted_trajectory_pub_ =
    create_publisher<autoware_planning_msgs::msg::Trajectory>("~/output/predicted_trajectory", 1);
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
      std::chrono::duration<double>(ctrl_period));
    timer_control_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&Controller::callbackTimerControl, this));
  }

  logger_configure_ = std::make_unique<autoware_utils::LoggerLevelConfigure>(this);

  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);
}

void Controller::check_cyclic_message_timeout(diagnostic_updater::DiagnosticStatusWrapper & stat)
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

bool Controller::processData(rclcpp::Clock & clock)
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

std::optional<trajectory_follower::InputData> Controller::createInputData(rclcpp::Clock & clock)
{
  if (!processData(clock)) {
    return {};
  }

  trajectory_follower::InputData input_data;
  input_data.current_trajectory = *current_trajectory_ptr_;
  input_data.current_odometry = *current_odometry_ptr_;
  input_data.current_steering = *current_steering_ptr_;
  input_data.current_accel = *current_accel_ptr_;
  input_data.current_operation_mode = *current_operation_mode_ptr_;
  input_data.steering_offset = steering_offset_;

  return input_data;
}

void Controller::callbackTimerControl()
{
  const rclcpp::Time stamp = this->now();

  // 1. create input data
  const auto input_data = createInputData(*get_clock());
  if (!input_data) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "Control is skipped since input data is not ready.");
    return;
  }

  // 2. run the control logic
  const auto result = trajectory_follower_->run(*input_data, stamp);
  if (!result) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "Control is skipped since %s", result.error().c_str());
    return;
  }

  // 3. publish how long each controller took
  publishProcessingTime(result->lateral_processing_time_ms, pub_processing_time_lat_ms_);
  publishProcessingTime(result->longitudinal_processing_time_ms, pub_processing_time_lon_ms_);

  // 4. update diagnostics
  diag_updater_->force_update();

  // 5. publish control command and what the lateral controller expects it to achieve
  control_cmd_pub_->publish(result->control_cmd);
  if (result->lateral_output.predicted_trajectory) {
    predicted_trajectory_pub_->publish(*result->lateral_output.predicted_trajectory);
  }

  // 6. publish debug
  published_time_publisher_->publish_if_subscribed(control_cmd_pub_, result->control_cmd.stamp);
  publishDebugMarker(*input_data, result->lateral_output);

  // 7. publish experimental topic
  if (result->control_cmd_horizon) {
    control_cmd_horizon_pub_->publish(*result->control_cmd_horizon);
  }
}

void Controller::publishDebugMarker(
  const trajectory_follower::InputData & input_data,
  const trajectory_follower::LateralOutput & lat_out) const
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
    const double cmd = lat_out.control_cmd.steering_tire_angle;
    const double diff = current - cmd;
    ss << "current:" << current << " cmd:" << cmd << " diff:" << diff
       << (lat_out.sync_data.is_steer_converged ? " (converged)" : " (not converged)");
    marker.text = ss.str();

    debug_marker_array.markers.push_back(marker);
  }

  debug_marker_pub_->publish(debug_marker_array);
}

void Controller::publishProcessingTime(
  const double t_ms, const rclcpp::Publisher<Float64Stamped>::SharedPtr pub)
{
  Float64Stamped msg{};
  msg.stamp = this->now();
  msg.data = t_ms;
  pub->publish(msg);
}

}  // namespace autoware::motion::control::trajectory_follower_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::motion::control::trajectory_follower_node::Controller)

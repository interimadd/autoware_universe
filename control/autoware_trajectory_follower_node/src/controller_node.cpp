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
template <typename T>
std::vector<T> resampleHorizonByZeroOrderHold(
  const std::vector<T> & original_horizon, const double original_time_step_ms,
  const double new_time_step_ms)
{
  std::vector<T> resampled_horizon{};
  const size_t step_factor = static_cast<size_t>(original_time_step_ms / new_time_step_ms);
  const size_t resampled_size = original_horizon.size() * step_factor;
  resampled_horizon.reserve(resampled_size);
  for (const auto & command : original_horizon) {
    for (size_t i = 0; i < step_factor; ++i) {
      resampled_horizon.push_back(command);
    }
  }
  return resampled_horizon;
}

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
}  // namespace

namespace autoware::motion::control::trajectory_follower_node
{
Controller::Controller(const rclcpp::NodeOptions & node_options)
: Node("controller", node_options),
  diag_updater_(std::make_shared<diagnostic_updater::Updater>(this))
{
  using std::placeholders::_1;

  const double ctrl_period = declare_parameter<double>("ctrl_period");
  timeout_thr_sec_ = declare_parameter<double>("timeout_thr_sec");
  cyclic_message_timeout_thr_sec_ = declare_parameter<double>("cyclic_message_timeout_thr_sec");
  // NOTE: It is possible that using control_horizon could be expected to enhance performance,
  // but it is not a formal interface topic, only an experimental one.
  // So it is disabled by default.
  enable_control_cmd_horizon_pub_ =
    declare_parameter<bool>("enable_control_cmd_horizon_pub", false);

  diag_updater_->setHardwareID("trajectory_follower_node");
  diag_updater_->add("incoming_message_timeout", this, &Controller::check_cyclic_message_timeout);
  diag_updater_->add("control_state", this, &Controller::check_longitudinal_control_state);

  lateral_controller_ = createLateralController(
    declare_parameter<std::string>("lateral_controller_mode"), *this, *diag_updater_);

  // PID is the only longitudinal algorithm, so the mode is validated rather than dispatched on.
  if (declare_parameter<std::string>("longitudinal_controller_mode") != "pid") {
    throw std::domain_error("[LongitudinalController] invalid algorithm");
  }
  longitudinal_config_ = pid_longitudinal_controller::create_config(*this);
  longitudinal_controller_ =
    std::make_unique<pid_longitudinal_controller::PidLongitudinalController>(longitudinal_config_);
  longitudinal_reporter_ =
    std::make_unique<pid_longitudinal_controller::PidLongitudinalControllerReporter>(*this);
  set_param_res_ =
    add_on_set_parameters_callback(std::bind(&Controller::onSetParameters, this, _1));

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
      std::chrono::duration<double>(ctrl_period));
    timer_control_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&Controller::callbackTimerControl, this));
  }

  logger_configure_ = std::make_unique<autoware_utils::LoggerLevelConfigure>(this);

  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);
}

rcl_interfaces::msg::SetParametersResult Controller::onSetParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  pid_longitudinal_controller::update_config(longitudinal_config_, parameters);
  longitudinal_controller_->setConfig(longitudinal_config_);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
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

void Controller::check_longitudinal_control_state(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  auto level = DiagnosticStatus::OK;
  std::string msg = "OK";

  if (longitudinal_control_state_ == pid_longitudinal_controller::ControlState::EMERGENCY) {
    level = DiagnosticStatus::ERROR;
    msg = "emergency occurred due to ";
  }

  stat.add<int32_t>("control_state", static_cast<int32_t>(longitudinal_control_state_));
  stat.summary(level, msg);
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

bool Controller::isTimeOut(
  const trajectory_follower::LongitudinalOutput & lon_out,
  const trajectory_follower::LateralOutput & lat_out)
{
  const auto now = this->now();
  if ((now - lat_out.control_cmd.stamp).seconds() > timeout_thr_sec_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000 /*ms*/,
      "Lateral control command too old, control_cmd will not be published.");
    return true;
  }
  if ((now - lon_out.control_cmd.stamp).seconds() > timeout_thr_sec_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000 /*ms*/,
      "Longitudinal control command too old, control_cmd will not be published.");
    return true;
  }
  return false;
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
  autoware_control_msgs::msg::Control out;
  const rclcpp::Time stamp = this->now();
  out.stamp = stamp;

  // 1. create input data
  const auto input_data = createInputData(*get_clock());
  if (!input_data) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "Control is skipped since input data is not ready.");
    return;
  }

  // 2. run controllers
  stop_watch_.tic("lateral");
  const auto lat_result = lateral_controller_->run(*input_data, stamp);
  publishProcessingTime(stop_watch_.toc("lateral"), pub_processing_time_lat_ms_);

  if (!lat_result) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "Control is skipped since %s", lat_result.error().c_str());
    return;
  }
  const auto & lat_out = lat_result.value();

  stop_watch_.tic("longitudinal");
  // NOTE: the steer convergence of the previous cycle is handed over, as the removed sync() did.
  const auto lon_result = longitudinal_controller_->run(*input_data, stamp, is_steer_converged_);
  publishProcessingTime(stop_watch_.toc("longitudinal"), pub_processing_time_lon_ms_);

  // 3. carry the lateral result over to the next cycle of the longitudinal controller
  is_steer_converged_ = lat_out.sync_data.is_steer_converged;
  longitudinal_control_state_ = lon_result.control_state;
  longitudinal_reporter_->report(lon_result);

  const auto & lon_out = lon_result.output;

  // TODO(Horibe): Think specification. This comes from the old implementation.
  if (isTimeOut(lon_out, lat_out)) return;

  // 4.5. update diagnostics
  diag_updater_->force_update();

  // 5. publish control command
  out.lateral = lat_out.control_cmd;
  out.longitudinal = lon_out.control_cmd;
  control_cmd_pub_->publish(out);

  // 6. publish debug
  published_time_publisher_->publish_if_subscribed(control_cmd_pub_, out.stamp);
  publishDebugMarker(*input_data, lat_out);

  // 7. publish experimental topic
  if (enable_control_cmd_horizon_pub_) {
    const auto control_horizon =
      mergeLatLonHorizon(lat_out.control_cmd_horizon, lon_out.control_cmd_horizon, this->now());
    if (control_horizon.has_value()) {
      control_cmd_horizon_pub_->publish(control_horizon.value());
    }
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

std::optional<ControlHorizon> Controller::mergeLatLonHorizon(
  const LateralHorizon & lateral_horizon, const LongitudinalHorizon & longitudinal_horizon,
  const rclcpp::Time & stamp)
{
  if (lateral_horizon.controls.empty() || longitudinal_horizon.controls.empty()) {
    return std::nullopt;
  }

  autoware_control_msgs::msg::ControlHorizon control_horizon{};
  control_horizon.stamp = stamp;

  // If either of the horizons has only one control, repeat the control to match the other horizon.
  if (lateral_horizon.controls.size() == 1) {
    control_horizon.time_step_ms = longitudinal_horizon.time_step_ms;
    const auto lateral = lateral_horizon.controls.front();
    for (const auto & longitudinal : longitudinal_horizon.controls) {
      autoware_control_msgs::msg::Control control;
      control.longitudinal = longitudinal;
      control.lateral = lateral;
      control.stamp = stamp;
      control_horizon.controls.push_back(control);
    }
    return control_horizon;
  }
  if (longitudinal_horizon.controls.size() == 1) {
    control_horizon.time_step_ms = lateral_horizon.time_step_ms;
    const auto longitudinal = longitudinal_horizon.controls.front();
    for (const auto & lateral : lateral_horizon.controls) {
      autoware_control_msgs::msg::Control control;
      control.longitudinal = longitudinal;
      control.lateral = lateral;
      control.stamp = stamp;
      control_horizon.controls.push_back(control);
    }
    return control_horizon;
  }

  // If both horizons have multiple controls, align the time steps and zero-order hold the controls.
  // calculate greatest common divisor of time steps
  const auto gcd_double = [](const double a, const double b) {
    const double precision = 1e9;
    const int int_a = static_cast<int>(round(a * precision));
    const int int_b = static_cast<int>(round(b * precision));
    return static_cast<double>(std::gcd(int_a, int_b)) / precision;
  };
  const double time_step_ms =
    gcd_double(lateral_horizon.time_step_ms, longitudinal_horizon.time_step_ms);
  control_horizon.time_step_ms = time_step_ms;

  const auto lateral_controls = resampleHorizonByZeroOrderHold(
    lateral_horizon.controls, lateral_horizon.time_step_ms, time_step_ms);
  const auto longitudinal_controls = resampleHorizonByZeroOrderHold(
    longitudinal_horizon.controls, longitudinal_horizon.time_step_ms, time_step_ms);

  if (lateral_controls.size() != longitudinal_controls.size()) {
    return std::nullopt;
  }

  const size_t num_steps = lateral_controls.size();
  for (size_t i = 0; i < num_steps; ++i) {
    autoware_control_msgs::msg::Control control{};
    control.stamp = stamp;
    control.lateral = lateral_controls.at(i);
    control.longitudinal = longitudinal_controls.at(i);
    control_horizon.controls.push_back(control);
  }
  return control_horizon;
}

}  // namespace autoware::motion::control::trajectory_follower_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::motion::control::trajectory_follower_node::Controller)

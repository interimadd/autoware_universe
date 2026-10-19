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

#include "autoware/trajectory_follower_node/create_config.hpp"
#include "autoware/trajectory_follower_node/ros_controller_factory.hpp"
#include "autoware_utils/ros/marker_helper.hpp"

#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

namespace autoware::motion::control::trajectory_follower_node
{
TrajectoryFollowerNode::TrajectoryFollowerNode(const rclcpp::NodeOptions & node_options)
: Node("controller", node_options),
  diag_updater_(std::make_shared<diagnostic_updater::Updater>(this))
{
  const auto config = createConfig(*this);
  cyclic_message_timeout_thr_sec_ = config.cyclic_message_timeout_thr_sec;
  enable_control_cmd_horizon_pub_ = config.enable_control_cmd_horizon;

  diag_updater_->setHardwareID("trajectory_follower_node");
  diag_updater_->add(
    "incoming_message_timeout", this, &TrajectoryFollowerNode::check_cyclic_message_timeout);

  trajectory_follower_ = std::make_unique<TrajectoryFollower>(
    config, createLateralController(config, *this, diag_updater_),
    createLongitudinalController(config, *this, diag_updater_));

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

}  // namespace autoware::motion::control::trajectory_follower_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::motion::control::trajectory_follower_node::TrajectoryFollowerNode)

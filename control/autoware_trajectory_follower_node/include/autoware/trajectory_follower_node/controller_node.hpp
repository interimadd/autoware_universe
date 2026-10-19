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

#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_NODE__CONTROLLER_NODE_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_NODE__CONTROLLER_NODE_HPP_

#include "autoware/trajectory_follower_node/trajectory_follower.hpp"
#include "autoware/trajectory_follower_node/visibility_control.hpp"
#include "autoware_utils/ros/logger_level_configure.hpp"
#include "autoware_utils/ros/polling_subscriber.hpp"
#include "rclcpp/rclcpp.hpp"

#include <autoware_utils/ros/published_time_publisher.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_control_msgs/msg/control_horizon.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>

#include <memory>
#include <optional>

namespace autoware::motion::control::trajectory_follower_node
{
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_internal_debug_msgs::msg::Float64Stamped;

/// \classController
/// \brief Drives a TrajectoryFollower from the topics of ROS 2. It takes the input
/// messages, hands them to the follower once per control period, and publishes what the
/// follower returned.
class TRAJECTORY_FOLLOWER_PUBLIC Controller : public rclcpp::Node
{
public:
  explicit Controller(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::TimerBase::SharedPtr timer_control_;
  double cyclic_message_timeout_thr_sec_;
  bool enable_control_cmd_horizon_pub_{false};

  std::shared_ptr<diagnostic_updater::Updater>
    diag_updater_;  // Diagnostic updater for publishing diagnostic data.

  std::unique_ptr<TrajectoryFollower> trajectory_follower_;

  // Subscribers
  autoware_utils::InterProcessPollingSubscriber<autoware_planning_msgs::msg::Trajectory>
    sub_ref_path_{this, "~/input/reference_trajectory"};

  autoware_utils::InterProcessPollingSubscriber<nav_msgs::msg::Odometry> sub_odometry_{
    this, "~/input/current_odometry"};

  autoware_utils::InterProcessPollingSubscriber<autoware_vehicle_msgs::msg::SteeringReport>
    sub_steering_{this, "~/input/current_steering"};

  autoware_utils::InterProcessPollingSubscriber<geometry_msgs::msg::AccelWithCovarianceStamped>
    sub_accel_{this, "~/input/current_accel"};

  autoware_utils::InterProcessPollingSubscriber<OperationModeState> sub_operation_mode_{
    this, "~/input/current_operation_mode", rclcpp::QoS{1}.transient_local()};

  rclcpp::Subscription<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr
    sub_steering_offset_update_;

  // Publishers
  rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr control_cmd_pub_;
  rclcpp::Publisher<Float64Stamped>::SharedPtr pub_processing_time_lat_ms_;
  rclcpp::Publisher<Float64Stamped>::SharedPtr pub_processing_time_lon_ms_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;
  rclcpp::Publisher<autoware_control_msgs::msg::ControlHorizon>::SharedPtr control_cmd_horizon_pub_;

  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr current_trajectory_ptr_;
  nav_msgs::msg::Odometry::ConstSharedPtr current_odometry_ptr_;
  autoware_vehicle_msgs::msg::SteeringReport::ConstSharedPtr current_steering_ptr_;
  geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr current_accel_ptr_;
  OperationModeState::ConstSharedPtr current_operation_mode_ptr_;
  double steering_offset_{0.0};

  /// Assemble the input of one cycle from the latest message of each topic.
  std::optional<InputData> createInputData(rclcpp::Clock & clock);
  bool processData(rclcpp::Clock & clock);

  /// Run the follower once and publish what it returned.
  void callbackTimerControl();

  void check_cyclic_message_timeout(diagnostic_updater::DiagnosticStatusWrapper & stat);

  void publishDebugMarker(
    const InputData & input_data, const TrajectoryFollowerResult & result) const;
  void publishProcessingTime(
    const double t_ms, const rclcpp::Publisher<Float64Stamped>::SharedPtr pub);

  std::unique_ptr<autoware_utils::LoggerLevelConfigure> logger_configure_;
  std::unique_ptr<autoware_utils::PublishedTimePublisher> published_time_publisher_;

  static constexpr double logger_throttle_interval = 5000;
};
}  // namespace autoware::motion::control::trajectory_follower_node

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_NODE__CONTROLLER_NODE_HPP_

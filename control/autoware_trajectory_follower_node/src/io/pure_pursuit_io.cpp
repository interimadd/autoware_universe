// Copyright 2026 Tier IV, Inc.
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

#include "io/pure_pursuit_io.hpp"

namespace autoware::motion::control::trajectory_follower_node
{

PurePursuitIo::PurePursuitIo(rclcpp::Node & node, PurePursuitLateralController & controller)
: controller_(controller),
  clock_(node.get_clock()),
  logger_(node.get_logger().get_child("lateral_controller"))
{
  // Debug Publishers
  pub_debug_marker_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/markers", 0);
  pub_debug_values_ =
    node.create_publisher<autoware_internal_debug_msgs::msg::Float32MultiArrayStamped>(
      "~/debug/ld_outputs", rclcpp::QoS{1});

  // Publish predicted trajectory
  pub_predicted_trajectory_ = node.create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "~/output/predicted_trajectory", 1);
}

void PurePursuitIo::publish()
{
  const auto & result = controller_.lastResult();
  emitLogs(result);
  publishMessage(result);
}

void PurePursuitIo::publishMessage(const PurePursuitLateralControllerResult & result)
{
  pub_debug_values_->publish(result.lookahead_distance_values);

  if (result.debug_markers) {
    pub_debug_marker_->publish(*result.debug_markers);
  }
  if (result.predicted_trajectory) {
    pub_predicted_trajectory_->publish(*result.predicted_trajectory);
  }
}

void PurePursuitIo::emitLogs(const PurePursuitLateralControllerResult & result)
{
  if (result.trajectory_too_short_to_smooth) {
    RCLCPP_ERROR(logger_, "Cannot smooth path! Trajectory size is too low!");
  }
  if (result.curvature_calculation_failed) {
    RCLCPP_WARN(logger_, "failed to calculate curvature");
  }
  if (result.received_short_trajectory) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "received path size is < 3, ignored");
  }
  if (result.closest_waypoint_not_found) {
    RCLCPP_ERROR(logger_, "cannot find closest waypoint");
  }
  if (result.control_command_solve_failed) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000, "failed to solve pure_pursuit for control command calculation");
  }
  if (result.prediction_solve_failed) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "failed to solve pure_pursuit for prediction");
  }
  if (!result.predicted_trajectory) {
    RCLCPP_ERROR(logger_, "Failed to generate predicted trajectory.");
  }
}

}  // namespace autoware::motion::control::trajectory_follower_node

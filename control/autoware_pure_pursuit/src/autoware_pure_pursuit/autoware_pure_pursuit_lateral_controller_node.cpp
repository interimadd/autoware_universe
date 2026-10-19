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

#include "autoware/pure_pursuit/autoware_pure_pursuit_lateral_controller_node.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

namespace autoware::pure_pursuit
{
PurePursuitLateralControllerConfig PurePursuitLateralControllerNode::createConfig(
  rclcpp::Node & node)
{
  PurePursuitLateralControllerConfig config;

  // Vehicle Parameters
  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
  config.wheel_base = vehicle_info.wheel_base_m;
  config.max_steering_angle = vehicle_info.max_steer_angle_rad;

  // Algorithm Parameters
  config.ld_velocity_ratio = node.declare_parameter<double>("ld_velocity_ratio");
  config.ld_lateral_error_ratio = node.declare_parameter<double>("ld_lateral_error_ratio");
  config.ld_curvature_ratio = node.declare_parameter<double>("ld_curvature_ratio");
  config.long_ld_lateral_error_threshold =
    node.declare_parameter<double>("long_ld_lateral_error_threshold");
  config.min_lookahead_distance = node.declare_parameter<double>("min_lookahead_distance");
  config.max_lookahead_distance = node.declare_parameter<double>("max_lookahead_distance");
  config.reverse_min_lookahead_distance =
    node.declare_parameter<double>("reverse_min_lookahead_distance");
  config.converged_steer_rad = node.declare_parameter<double>("converged_steer_rad");
  config.prediction_ds = node.declare_parameter<double>("prediction_ds");
  config.prediction_distance_length = node.declare_parameter<double>("prediction_distance_length");
  config.resampling_ds = node.declare_parameter<double>("resampling_ds");
  config.curvature_calculation_distance =
    node.declare_parameter<double>("curvature_calculation_distance");
  config.enable_path_smoothing = node.declare_parameter<bool>("enable_path_smoothing");
  config.path_filter_moving_ave_num = node.declare_parameter<int64_t>("path_filter_moving_ave_num");

  return config;
}

PurePursuitLateralControllerNode::PurePursuitLateralControllerNode(
  rclcpp::Node & node, const PurePursuitLateralControllerConfig & config)
: clock_(node.get_clock()),
  logger_(node.get_logger().get_child("lateral_controller")),
  controller_(config)
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

bool PurePursuitLateralControllerNode::isReady(
  [[maybe_unused]] const trajectory_follower::InputData & input_data)
{
  return true;
}

trajectory_follower::LateralOutput PurePursuitLateralControllerNode::run(
  const trajectory_follower::InputData & input_data)
{
  // Timestamp of this control cycle, shared by every message this call publishes.
  const auto stamp = clock_->now();

  const auto result = controller_.run(input_data, stamp);

  emitLogs(result);
  publish(result);

  return result.output;
}

void PurePursuitLateralControllerNode::publish(const PurePursuitLateralControllerResult & result)
{
  pub_debug_values_->publish(result.lookahead_distance_values);

  if (result.debug_markers) {
    pub_debug_marker_->publish(*result.debug_markers);
  }
  if (result.predicted_trajectory) {
    pub_predicted_trajectory_->publish(*result.predicted_trajectory);
  }
}

void PurePursuitLateralControllerNode::emitLogs(const PurePursuitLateralControllerResult & result)
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

}  // namespace autoware::pure_pursuit

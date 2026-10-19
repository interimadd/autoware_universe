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

#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_NODE__TRAJECTORY_FOLLOWER_NODE_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_NODE__TRAJECTORY_FOLLOWER_NODE_HPP_

#include "autoware/trajectory_follower_node/controller_factory.hpp"
#include "autoware/trajectory_follower_node/ros_log_writer.hpp"
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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <autoware_internal_debug_msgs/msg/float32_multi_array_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::motion::control::trajectory_follower_node
{
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_internal_debug_msgs::msg::Float32MultiArrayStamped;
using autoware_internal_debug_msgs::msg::Float32Stamped;
using autoware_internal_debug_msgs::msg::Float64Stamped;
using autoware_planning_msgs::msg::Trajectory;
using geometry_msgs::msg::PoseStamped;
using visualization_msgs::msg::MarkerArray;

/// \class TrajectoryFollowerNode
/// \brief Drives a TrajectoryFollower from the topics of ROS 2. It takes the input
/// messages, hands them to the follower once per control period, and publishes what the
/// follower returned.
class TRAJECTORY_FOLLOWER_PUBLIC TrajectoryFollowerNode : public rclcpp::Node
{
public:
  explicit TrajectoryFollowerNode(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::TimerBase::SharedPtr timer_control_;
  double cyclic_message_timeout_thr_sec_;
  bool enable_control_cmd_horizon_pub_{false};

  std::shared_ptr<diagnostic_updater::Updater>
    diag_updater_;  // Diagnostic updater for publishing diagnostic data.

  /// Declared before the follower so that it outlives the controller that writes to it.
  std::unique_ptr<RosLogWriter> log_writer_;

  std::unique_ptr<TrajectoryFollower> trajectory_follower_;

  /// Views of the controllers the follower owns, for the kinds this node was configured
  /// with. Null for every other kind. The controllers read no parameter and write no
  /// topic; everything below does that for them.
  mpc_lateral_controller::MpcLateralController * mpc_{nullptr};
  pure_pursuit::PurePursuitLateralController * pure_pursuit_{nullptr};
  pid_longitudinal_controller::PidLongitudinalController * pid_{nullptr};

  /// The config the controllers were built with. A parameter update changes it field by
  /// field and hands it back to the controller it belongs to.
  TrajectoryFollowerConfig config_;

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

  // Publishers of the MPC lateral controller
  rclcpp::Publisher<Trajectory>::SharedPtr pub_mpc_predicted_traj_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_mpc_predicted_traj_frenet_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_mpc_resampled_reference_traj_;
  rclcpp::Publisher<PoseStamped>::SharedPtr pub_mpc_nearest_pose_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_mpc_nearest_segment_traj_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr pub_mpc_nearest_info_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr pub_mpc_debug_values_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_mpc_steer_offset_;

  // Publishers of the pure pursuit lateral controller
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_pure_pursuit_debug_marker_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr pub_pure_pursuit_debug_values_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_pure_pursuit_predicted_traj_;

  // Publishers of the PID longitudinal controller
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr pub_pid_slope_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr pub_pid_debug_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_pid_virtual_wall_marker_;

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

  /// Create the topics of the controllers this node was configured with, and register
  /// their diagnostics.
  void setupControllerTopics();

  /// Write out and log everything the controllers produced during the last cycle.
  void publishControllerOutput();

  /// Hand a changed parameter to the controller it belongs to.
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  void checkMpcSolve(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void checkControlState(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /// The control state of the last PID cycle, which its diagnostic reports.
  pid_longitudinal_controller::ControlState pid_control_state_{
    pid_longitudinal_controller::ControlState::STOPPED};

  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  void publishProcessingTime(
    const double t_ms, const rclcpp::Publisher<Float64Stamped>::SharedPtr pub);

  std::unique_ptr<autoware_utils::LoggerLevelConfigure> logger_configure_;
  std::unique_ptr<autoware_utils::PublishedTimePublisher> published_time_publisher_;

  static constexpr double logger_throttle_interval = 5000;
};
}  // namespace autoware::motion::control::trajectory_follower_node

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_NODE__TRAJECTORY_FOLLOWER_NODE_HPP_

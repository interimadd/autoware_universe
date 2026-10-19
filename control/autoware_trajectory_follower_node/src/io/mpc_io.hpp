// Copyright 2026 TIER IV, Inc.
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

#ifndef IO__MPC_IO_HPP_
#define IO__MPC_IO_HPP_

#include "autoware/mpc_lateral_controller/mpc_lateral_controller.hpp"
#include "autoware/trajectory_follower_node/controller_io.hpp"
#include "rclcpp/rclcpp.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>

#include "autoware_internal_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "autoware_internal_debug_msgs/msg/float32_stamped.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <memory>
#include <vector>

namespace autoware::motion::control::trajectory_follower_node
{
using autoware_internal_debug_msgs::msg::Float32MultiArrayStamped;
using autoware_internal_debug_msgs::msg::Float32Stamped;
using autoware_planning_msgs::msg::Trajectory;
using geometry_msgs::msg::PoseStamped;
using mpc_lateral_controller::MpcLateralController;
using mpc_lateral_controller::MpcLateralControllerResult;
using mpc_lateral_controller::MPCParam;

/// The topics, the diagnostic and the runtime parameters of the MPC lateral controller.
/// The controller itself knows none of them.
class MpcIo : public ControllerIo
{
public:
  /// \param node The node that owns the topics and the parameters.
  /// \param controller The controller whose output is written out. It has to outlive this.
  MpcIo(
    rclcpp::Node & node, MpcLateralController & controller,
    const std::shared_ptr<diagnostic_updater::Updater> & diag_updater);

  void publish() override;

private:
  void setStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);

  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  MpcLateralController & controller_;
  std::shared_ptr<diagnostic_updater::Updater> diag_updater_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr m_set_param_res;

  rclcpp::Publisher<Trajectory>::SharedPtr m_pub_predicted_traj;
  rclcpp::Publisher<Trajectory>::SharedPtr m_pub_predicted_traj_frenet;
  rclcpp::Publisher<Trajectory>::SharedPtr m_pub_resampled_reference_traj;
  rclcpp::Publisher<PoseStamped>::SharedPtr m_pub_nearest_pose;
  rclcpp::Publisher<Trajectory>::SharedPtr m_pub_nearest_segment_traj;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr m_pub_nearest_info;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr m_pub_debug_values;
  rclcpp::Publisher<Float32Stamped>::SharedPtr m_pub_steer_offset;
};

}  // namespace autoware::motion::control::trajectory_follower_node

#endif  // IO__MPC_IO_HPP_

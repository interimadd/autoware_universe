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

#ifndef IO__PURE_PURSUIT_IO_HPP_
#define IO__PURE_PURSUIT_IO_HPP_

#include "autoware/pure_pursuit/autoware_pure_pursuit_lateral_controller.hpp"
#include "autoware/trajectory_follower_node/controller_io.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_internal_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace autoware::motion::control::trajectory_follower_node
{
using pure_pursuit::PurePursuitLateralController;
using pure_pursuit::PurePursuitLateralControllerResult;

/// The topics and the log lines of the pure pursuit lateral controller. The controller
/// itself knows none of them.
class PurePursuitIo : public ControllerIo
{
public:
  /// \param node The node that owns the topics.
  /// \param controller The controller whose output is written out. It has to outlive this.
  PurePursuitIo(rclcpp::Node & node, PurePursuitLateralController & controller);

  void publish() override;

private:
  void publishMessage(const PurePursuitLateralControllerResult & result);
  void emitLogs(const PurePursuitLateralControllerResult & result);

  PurePursuitLateralController & controller_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_marker_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr
    pub_debug_values_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr pub_predicted_trajectory_;
};

}  // namespace autoware::motion::control::trajectory_follower_node

#endif  // IO__PURE_PURSUIT_IO_HPP_

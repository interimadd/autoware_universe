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

#ifndef AUTOWARE__PURE_PURSUIT__AUTOWARE_PURE_PURSUIT_LATERAL_CONTROLLER_NODE_HPP_
#define AUTOWARE__PURE_PURSUIT__AUTOWARE_PURE_PURSUIT_LATERAL_CONTROLLER_NODE_HPP_

#include "autoware/pure_pursuit/autoware_pure_pursuit_lateral_controller.hpp"
#include "autoware/trajectory_follower_base/lateral_controller.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_internal_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <string>

namespace autoware::pure_pursuit
{
/// The node side of the lateral controller. It declares the parameters, builds the
/// controller from them, publishes what a cycle produced and writes what it reported.
class PurePursuitLateralControllerNode : public trajectory_follower::LateralController
{
public:
  /// \param node Reference to the node used only for the component and parameter initialization.
  explicit PurePursuitLateralControllerNode(rclcpp::Node & node);

  /// Read the parameters the controller needs from the node, declaring them on it.
  static PurePursuitLateralControllerConfig createConfig(rclcpp::Node & node);

private:
  tl::expected<trajectory_follower::LateralOutput, std::string> run(
    const trajectory_follower::InputData & input_data, const rclcpp::Time & stamp) override;

  void publish(const PurePursuitLateralControllerResult & result);

  void emitLogs(const PurePursuitLateralControllerResult & result);

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;

  PurePursuitLateralController controller_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_marker_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr
    pub_debug_values_;
};

}  // namespace autoware::pure_pursuit

#endif  // AUTOWARE__PURE_PURSUIT__AUTOWARE_PURE_PURSUIT_LATERAL_CONTROLLER_NODE_HPP_

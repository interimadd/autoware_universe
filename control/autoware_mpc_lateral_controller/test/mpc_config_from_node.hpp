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

#ifndef MPC_CONFIG_FROM_NODE_HPP_
#define MPC_CONFIG_FROM_NODE_HPP_

#include "autoware/mpc_lateral_controller/mpc_lateral_controller.hpp"
#include "autoware/trajectory_follower_base/common_params.hpp"
#include "rclcpp/rclcpp.hpp"

// Test scaffolding only. The controller reads no parameter of ROS 2; the node that owns it
// does, in autoware_trajectory_follower_node. These tests set a controller up from the
// parameter file this package ships, so they keep a copy of that reading here.
//
// TODO(mpc maintainers): build MpcLateralControllerConfig in the test itself and delete
// this file, together with the node these tests still create. The CMakeLists of this
// package already notes that a test without a node is the goal.

namespace autoware::motion::control::mpc_lateral_controller
{
MpcLateralControllerConfig createMpcConfigFromNode(
  rclcpp::Node & node, const trajectory_follower::CommonParams & common);

}  // namespace autoware::motion::control::mpc_lateral_controller

#endif  // MPC_CONFIG_FROM_NODE_HPP_

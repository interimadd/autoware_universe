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

#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_NODE__CREATE_CONFIG_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_NODE__CREATE_CONFIG_HPP_

#include "autoware/trajectory_follower_node/trajectory_follower_config.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autoware::motion::control::trajectory_follower_node
{
/// Read every parameter the follower needs from the node, declaring them on it. This is the
/// one place where a parameter of this node is declared.
TrajectoryFollowerConfig createConfig(rclcpp::Node & node);

}  // namespace autoware::motion::control::trajectory_follower_node

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_NODE__CREATE_CONFIG_HPP_

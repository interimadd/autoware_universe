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

#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_NODE__ROS_CONTROLLER_FACTORY_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_NODE__ROS_CONTROLLER_FACTORY_HPP_

#include "autoware/trajectory_follower_base/lateral_controller.hpp"
#include "autoware/trajectory_follower_base/longitudinal_controller.hpp"
#include "autoware/trajectory_follower_node/trajectory_follower_config.hpp"
#include "rclcpp/rclcpp.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <memory>

namespace autoware::motion::control::trajectory_follower_node
{
/// Build the lateral controller the config names, wrapped in the node layer that publishes
/// what a cycle produced, writes its log lines and reports its diagnostics.
std::unique_ptr<trajectory_follower::LateralController> createLateralController(
  const TrajectoryFollowerConfig & config, rclcpp::Node & node,
  const std::shared_ptr<diagnostic_updater::Updater> & diag_updater);

/// Build the longitudinal controller the config names, wrapped in the same way.
std::unique_ptr<trajectory_follower::LongitudinalController> createLongitudinalController(
  const TrajectoryFollowerConfig & config, rclcpp::Node & node,
  const std::shared_ptr<diagnostic_updater::Updater> & diag_updater);

}  // namespace autoware::motion::control::trajectory_follower_node

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_NODE__ROS_CONTROLLER_FACTORY_HPP_

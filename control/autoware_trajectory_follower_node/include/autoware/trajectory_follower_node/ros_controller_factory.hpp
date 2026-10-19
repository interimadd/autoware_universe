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

#include "autoware/mpc_lateral_controller/log_writer.hpp"
#include "autoware/trajectory_follower_base/lateral_controller.hpp"
#include "autoware/trajectory_follower_base/longitudinal_controller.hpp"
#include "autoware/trajectory_follower_node/controller_io.hpp"
#include "autoware/trajectory_follower_node/trajectory_follower_config.hpp"
#include "rclcpp/rclcpp.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <memory>

namespace autoware::motion::control::trajectory_follower_node
{
/// A controller and the ROS side that writes out what it produced. The controller is
/// handed to the TrajectoryFollower; the io stays with the node, which calls publish()
/// after each cycle that produced a command.
template <typename Controller>
struct ControllerWithIo
{
  std::unique_ptr<Controller> controller;
  std::unique_ptr<ControllerIo> io;
};

/// \param writer Where the lateral controller reports what it meets. It has to outlive
///        the controller this returns.
ControllerWithIo<trajectory_follower::LateralController> createLateralController(
  const TrajectoryFollowerConfig & config, rclcpp::Node & node,
  const std::shared_ptr<diagnostic_updater::Updater> & diag_updater,
  const mpc_lateral_controller::LogWriter & writer);

ControllerWithIo<trajectory_follower::LongitudinalController> createLongitudinalController(
  const TrajectoryFollowerConfig & config, rclcpp::Node & node,
  const std::shared_ptr<diagnostic_updater::Updater> & diag_updater);

}  // namespace autoware::motion::control::trajectory_follower_node

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_NODE__ROS_CONTROLLER_FACTORY_HPP_

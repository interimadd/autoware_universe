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

#include "autoware/trajectory_follower_node/ros_controller_factory.hpp"

#include "autoware/mpc_lateral_controller/mpc_lateral_controller_node.hpp"
#include "autoware/pid_longitudinal_controller/pid_longitudinal_controller_node.hpp"
#include "autoware/pure_pursuit/autoware_pure_pursuit_lateral_controller_node.hpp"

#include <memory>

namespace autoware::motion::control::trajectory_follower_node
{
std::unique_ptr<trajectory_follower::LateralController> createLateralController(
  const TrajectoryFollowerConfig & config, rclcpp::Node & node,
  const std::shared_ptr<diagnostic_updater::Updater> & diag_updater)
{
  switch (config.lateral_controller_kind) {
    case LateralControllerKind::Mpc:
      return std::make_unique<mpc_lateral_controller::MpcLateralControllerNode>(
        node, config.mpc, diag_updater);
    case LateralControllerKind::PurePursuit:
      return std::make_unique<pure_pursuit::PurePursuitLateralControllerNode>(
        node, config.pure_pursuit);
  }
  return nullptr;
}

std::unique_ptr<trajectory_follower::LongitudinalController> createLongitudinalController(
  const TrajectoryFollowerConfig & config, rclcpp::Node & node,
  const std::shared_ptr<diagnostic_updater::Updater> & diag_updater)
{
  switch (config.longitudinal_controller_kind) {
    case LongitudinalControllerKind::Pid:
      return std::make_unique<pid_longitudinal_controller::PidLongitudinalControllerNode>(
        node, config.pid, diag_updater);
  }
  return nullptr;
}

}  // namespace autoware::motion::control::trajectory_follower_node

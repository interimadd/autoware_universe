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

#include "io/mpc_io.hpp"
#include "io/pid_io.hpp"
#include "io/pure_pursuit_io.hpp"

#include <memory>
#include <utility>

namespace autoware::motion::control::trajectory_follower_node
{
ControllerWithIo<trajectory_follower::LateralController> createLateralController(
  const TrajectoryFollowerConfig & config, rclcpp::Node & node,
  const std::shared_ptr<diagnostic_updater::Updater> & diag_updater,
  const mpc_lateral_controller::LogWriter & writer)
{
  switch (config.lateral_controller_kind) {
    case LateralControllerKind::Mpc: {
      auto controller = std::make_unique<MpcLateralController>(config.mpc, writer);
      auto io = std::make_unique<MpcIo>(node, *controller, diag_updater);
      return {std::move(controller), std::move(io)};
    }
    case LateralControllerKind::PurePursuit: {
      auto controller = std::make_unique<PurePursuitLateralController>(config.pure_pursuit);
      auto io = std::make_unique<PurePursuitIo>(node, *controller);
      return {std::move(controller), std::move(io)};
    }
  }
  return {};
}

ControllerWithIo<trajectory_follower::LongitudinalController> createLongitudinalController(
  const TrajectoryFollowerConfig & config, rclcpp::Node & node,
  const std::shared_ptr<diagnostic_updater::Updater> & diag_updater)
{
  switch (config.longitudinal_controller_kind) {
    case LongitudinalControllerKind::Pid: {
      auto controller = std::make_unique<PidLongitudinalController>(config.pid);
      auto io = std::make_unique<PidIo>(node, *controller, config.pid, diag_updater);
      return {std::move(controller), std::move(io)};
    }
  }
  return {};
}

}  // namespace autoware::motion::control::trajectory_follower_node

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

#include "autoware/trajectory_follower_node/controller_factory.hpp"

#include <memory>
#include <utility>

namespace autoware::motion::control::trajectory_follower_node
{
LateralControllerHandle createLateralController(
  const TrajectoryFollowerConfig & config, const mpc_lateral_controller::LogWriter & writer)
{
  LateralControllerHandle handle;
  switch (config.lateral_controller_kind) {
    case LateralControllerKind::Mpc: {
      auto controller =
        std::make_unique<mpc_lateral_controller::MpcLateralController>(config.mpc, writer);
      handle.mpc = controller.get();
      handle.controller = std::move(controller);
      break;
    }
    case LateralControllerKind::PurePursuit: {
      auto controller =
        std::make_unique<pure_pursuit::PurePursuitLateralController>(config.pure_pursuit);
      handle.pure_pursuit = controller.get();
      handle.controller = std::move(controller);
      break;
    }
  }
  return handle;
}

LongitudinalControllerHandle createLongitudinalController(const TrajectoryFollowerConfig & config)
{
  LongitudinalControllerHandle handle;
  switch (config.longitudinal_controller_kind) {
    case LongitudinalControllerKind::Pid: {
      auto controller =
        std::make_unique<pid_longitudinal_controller::PidLongitudinalController>(config.pid);
      handle.pid = controller.get();
      handle.controller = std::move(controller);
      break;
    }
  }
  return handle;
}

}  // namespace autoware::motion::control::trajectory_follower_node

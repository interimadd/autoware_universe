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

#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_NODE__CONTROLLER_FACTORY_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_NODE__CONTROLLER_FACTORY_HPP_

#include "autoware/mpc_lateral_controller/log_writer.hpp"
#include "autoware/trajectory_follower_base/lateral_controller.hpp"
#include "autoware/trajectory_follower_base/longitudinal_controller.hpp"
#include "autoware/trajectory_follower_node/trajectory_follower_config.hpp"

#include <memory>

namespace autoware::motion::control::trajectory_follower_node
{
/// The lateral controller a config names. `controller` owns it; the pointer that matches
/// the chosen kind views the same object and is null for every other kind. The node reads
/// that pointer to publish what the controller produced and to update its parameters,
/// which it can only do for the kind it knows.
struct LateralControllerHandle
{
  std::unique_ptr<trajectory_follower::LateralController> controller;
  mpc_lateral_controller::MpcLateralController * mpc{nullptr};
  pure_pursuit::PurePursuitLateralController * pure_pursuit{nullptr};
};

struct LongitudinalControllerHandle
{
  std::unique_ptr<trajectory_follower::LongitudinalController> controller;
  pid_longitudinal_controller::PidLongitudinalController * pid{nullptr};
};

/// \param writer Where a controller reports what it meets. It has to outlive the
///        controller. The default drops every line, which is what a caller outside ROS 2
///        wants.
LateralControllerHandle createLateralController(
  const TrajectoryFollowerConfig & config,
  const mpc_lateral_controller::LogWriter & writer = mpc_lateral_controller::no_log);

LongitudinalControllerHandle createLongitudinalController(const TrajectoryFollowerConfig & config);

}  // namespace autoware::motion::control::trajectory_follower_node

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_NODE__CONTROLLER_FACTORY_HPP_

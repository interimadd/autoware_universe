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

#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_NODE__TRAJECTORY_FOLLOWER_CONFIG_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_NODE__TRAJECTORY_FOLLOWER_CONFIG_HPP_

#include "autoware/mpc_lateral_controller/mpc_lateral_controller.hpp"
#include "autoware/pid_longitudinal_controller/pid_longitudinal_controller.hpp"
#include "autoware/pure_pursuit/autoware_pure_pursuit_lateral_controller.hpp"

#include <string>

namespace autoware::motion::control::trajectory_follower_node
{
enum class LateralControllerKind { Mpc, PurePursuit };
enum class LongitudinalControllerKind { Pid };

/// Everything the trajectory follower reads to set itself up. The node layer fills it from
/// the parameters it declares, so that the follower itself needs no node.
struct TrajectoryFollowerConfig
{
  double ctrl_period{0.0};
  double timeout_thr_sec{0.0};
  double cyclic_message_timeout_thr_sec{0.0};
  bool enable_control_cmd_horizon{false};

  LateralControllerKind lateral_controller_kind{LateralControllerKind::Mpc};
  LongitudinalControllerKind longitudinal_controller_kind{LongitudinalControllerKind::Pid};

  mpc_lateral_controller::MpcLateralControllerConfig mpc{};
  pure_pursuit::PurePursuitLateralControllerConfig pure_pursuit{};
  pid_longitudinal_controller::PidLongitudinalControllerConfig pid{};
};

/// @throw std::invalid_argument when the name is not a lateral controller this node builds.
LateralControllerKind parseLateralControllerKind(const std::string & name);

/// @throw std::invalid_argument when the name is not a longitudinal controller this node builds.
LongitudinalControllerKind parseLongitudinalControllerKind(const std::string & name);

}  // namespace autoware::motion::control::trajectory_follower_node

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_NODE__TRAJECTORY_FOLLOWER_CONFIG_HPP_

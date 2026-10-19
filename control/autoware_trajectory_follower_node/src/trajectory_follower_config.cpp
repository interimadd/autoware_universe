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

#include "autoware/trajectory_follower_node/trajectory_follower_config.hpp"

#include <stdexcept>
#include <string>

namespace autoware::motion::control::trajectory_follower_node
{
LateralControllerKind parseLateralControllerKind(const std::string & name)
{
  if (name == "mpc") return LateralControllerKind::Mpc;
  if (name == "pure_pursuit") return LateralControllerKind::PurePursuit;
  throw std::invalid_argument("[LateralController] invalid algorithm: " + name);
}

LongitudinalControllerKind parseLongitudinalControllerKind(const std::string & name)
{
  if (name == "pid") return LongitudinalControllerKind::Pid;
  throw std::invalid_argument("[LongitudinalController] invalid algorithm: " + name);
}

}  // namespace autoware::motion::control::trajectory_follower_node

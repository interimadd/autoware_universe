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

#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_BASE__COMMON_PARAMS_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_BASE__COMMON_PARAMS_HPP_

namespace autoware::motion::control::trajectory_follower
{
/// The settings that more than one controller reads. The node that owns the controllers
/// declares them once and hands them to each controller's own config builder, so that no
/// controller has to know whether another one declared them first.
struct CommonParams
{
  double ctrl_period{0.0};
  bool use_temporal_trajectory{false};
  double ego_nearest_dist_threshold{0.0};
  double ego_nearest_yaw_threshold{0.0};
};

}  // namespace autoware::motion::control::trajectory_follower

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_BASE__COMMON_PARAMS_HPP_

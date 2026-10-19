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

#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_BASE__FEEDBACK_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_BASE__FEEDBACK_HPP_

namespace autoware::motion::control::trajectory_follower
{
/// The part of the lateral controller's output that the longitudinal controller reads.
/// The caller carries it from one controller to the other.
///
/// NOTE: There is no feedback in the other direction. The longitudinal controller tells the
/// lateral controller nothing, so no type says that it does.
struct LateralFeedback
{
  bool is_steer_converged{false};
};

}  // namespace autoware::motion::control::trajectory_follower

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_BASE__FEEDBACK_HPP_

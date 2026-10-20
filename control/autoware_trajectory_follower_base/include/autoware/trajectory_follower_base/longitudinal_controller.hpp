// Copyright 2022 The Autoware Foundation
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

#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_BASE__LONGITUDINAL_CONTROLLER_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_BASE__LONGITUDINAL_CONTROLLER_HPP_

#include "autoware/trajectory_follower_base/control_horizon.hpp"
#include "autoware/trajectory_follower_base/input_data.hpp"

#include <rclcpp/time.hpp>
#include <tl_expected/expected.hpp>

#include "autoware_control_msgs/msg/longitudinal.hpp"

#include <string>

namespace autoware::motion::control::trajectory_follower
{
using autoware_control_msgs::msg::Longitudinal;
struct LongitudinalOutput
{
  Longitudinal control_cmd;
  LongitudinalHorizon control_cmd_horizon;
};

/// What a longitudinal controller has to do, and nothing else. One call takes everything
/// the controller reads for a cycle and returns everything the caller needs, so there is no
/// order a caller has to know.
class LongitudinalController
{
public:
  virtual ~LongitudinalController() = default;

  /// Compute the command for one control cycle.
  /// \param input_data What the vehicle and the planner report for this cycle.
  /// \param stamp The time of this cycle, read once by the caller.
  /// \param is_steer_converged What the lateral controller reported about its steering. It
  ///        is not input data, so it stays an argument of its own.
  /// \return The command, or the reason the controller cannot produce one yet.
  virtual tl::expected<LongitudinalOutput, std::string> run(
    const InputData & input_data, const rclcpp::Time & stamp, const bool is_steer_converged) = 0;
};
}  // namespace autoware::motion::control::trajectory_follower

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_BASE__LONGITUDINAL_CONTROLLER_HPP_

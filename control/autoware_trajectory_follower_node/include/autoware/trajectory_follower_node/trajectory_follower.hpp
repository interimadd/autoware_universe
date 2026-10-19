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

#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_NODE__TRAJECTORY_FOLLOWER_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_NODE__TRAJECTORY_FOLLOWER_HPP_

#include "autoware/trajectory_follower_base/feedback.hpp"
#include "autoware/trajectory_follower_base/input_data.hpp"
#include "autoware/trajectory_follower_base/lateral_controller.hpp"
#include "autoware/trajectory_follower_base/longitudinal_controller.hpp"
#include "autoware/trajectory_follower_node/trajectory_follower_config.hpp"

#include <rclcpp/time.hpp>
#include <tl_expected/expected.hpp>

#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_control_msgs/msg/control_horizon.hpp"

#include <memory>
#include <optional>
#include <string>

namespace autoware::motion::control::trajectory_follower_node
{
using autoware_control_msgs::msg::Control;
using autoware_control_msgs::msg::ControlHorizon;
using trajectory_follower::InputData;
using trajectory_follower::LateralController;
using trajectory_follower::LateralFeedback;
using trajectory_follower::LateralHorizon;
using trajectory_follower::LongitudinalController;
using trajectory_follower::LongitudinalHorizon;

/// What one cycle of the trajectory follower produced.
struct TrajectoryFollowerResult
{
  Control control_cmd;
  /// Holds no control when the config does not ask for the horizon, or when the two
  /// horizons could not be aligned. There is then nothing to publish.
  ControlHorizon control_cmd_horizon;

  /// What the lateral controller reported, which the caller shows for debugging.
  LateralFeedback lateral_feedback;

  double lateral_processing_time_ms{0.0};
  double longitudinal_processing_time_ms{0.0};
};

/// Merge a lateral and a longitudinal horizon into one.
///
/// If one of them holds a single control, that control is repeated to match the other. If
/// the two use different time steps, both are resampled to the greatest common divisor of
/// the steps. Returns nothing when either horizon is empty or the two cannot be aligned.
std::optional<ControlHorizon> mergeLatLonHorizon(
  const LateralHorizon & lateral_horizon, const LongitudinalHorizon & longitudinal_horizon,
  const rclcpp::Time & stamp);

/// Runs a lateral and a longitudinal controller for one cycle and joins what they produced
/// into one vehicle command.
///
/// Nothing here reads a clock, a parameter or a topic, so a caller outside ROS 2 can drive
/// this class in a loop and evaluate a controller without starting an executor.
class TrajectoryFollower
{
public:
  TrajectoryFollower(
    const TrajectoryFollowerConfig & config, std::unique_ptr<LateralController> lateral_controller,
    std::unique_ptr<LongitudinalController> longitudinal_controller);

  /// Compute the vehicle command for one control cycle.
  /// \param input_data What the vehicle and the planner report for this cycle.
  /// \param stamp The time of this cycle, read once by the caller.
  /// \return The command, or the reason this cycle produced none.
  tl::expected<TrajectoryFollowerResult, std::string> run(
    const InputData & input_data, const rclcpp::Time & stamp);

private:
  TrajectoryFollowerConfig config_;
  std::unique_ptr<LateralController> lateral_controller_;
  std::unique_ptr<LongitudinalController> longitudinal_controller_;

  /// What the lateral controller reported on the previous cycle. The longitudinal
  /// controller of this cycle reads it, as it did before this class existed.
  LateralFeedback lateral_feedback_;
};

}  // namespace autoware::motion::control::trajectory_follower_node

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_NODE__TRAJECTORY_FOLLOWER_HPP_

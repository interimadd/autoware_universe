// Copyright 2026 TIER IV, Inc.
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

#include "autoware/trajectory_follower_base/control_horizon.hpp"
#include "autoware/trajectory_follower_base/input_data.hpp"
#include "autoware/trajectory_follower_base/lateral_controller.hpp"
#include "autoware/trajectory_follower_base/longitudinal_controller.hpp"

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

/// What the trajectory follower reads to set itself up. The node layer fills it from the
/// parameters it declares, so that the follower itself needs no node.
struct TrajectoryFollowerConfig
{
  /// How old a command may be before this cycle produces none [s].
  double timeout_thr_sec{0.0};

  /// Whether to build the merged control horizon, which is an experimental output.
  bool enable_control_cmd_horizon{false};
};

/// What one cycle of the trajectory follower produced.
struct TrajectoryFollowerResult
{
  Control control_cmd;

  /// Holds nothing when the config does not ask for the horizon, or when the two horizons
  /// could not be aligned.
  std::optional<ControlHorizon> control_cmd_horizon{std::nullopt};

  /// What the lateral controller reported. It carries the predicted trajectory, which the
  /// command itself does not hold.
  trajectory_follower::LateralOutput lateral_output;
};

/// Merge a lateral and a longitudinal horizon into one.
///
/// If one of them holds a single control, that control is repeated to match the other. If
/// the two use different time steps, both are resampled to the greatest common divisor of
/// the steps. Returns nothing when either horizon is empty or the two cannot be aligned.
std::optional<ControlHorizon> mergeLatLonHorizon(
  const trajectory_follower::LateralHorizon & lateral_horizon,
  const trajectory_follower::LongitudinalHorizon & longitudinal_horizon,
  const rclcpp::Time & stamp);

/// Runs a lateral and a longitudinal controller for one cycle and joins what they produced
/// into one vehicle command.
///
/// Nothing here reads a clock, a parameter or a topic, so a caller outside ROS 2 can drive
/// this class in a loop and evaluate a controller without starting an executor. The same
/// input data and the same stamp always produce the same result.
class TrajectoryFollower
{
public:
  TrajectoryFollower(
    const TrajectoryFollowerConfig & config,
    std::unique_ptr<trajectory_follower::LateralController> lateral_controller,
    std::unique_ptr<trajectory_follower::LongitudinalController> longitudinal_controller);

  /// Compute the vehicle command for one control cycle.
  /// \param input_data What the vehicle and the planner report for this cycle.
  /// \param stamp The time of this cycle, read once by the caller.
  /// \return What the cycle produced, or the reason it produced no command.
  tl::expected<TrajectoryFollowerResult, std::string> run(
    const trajectory_follower::InputData & input_data, const rclcpp::Time & stamp);

private:
  TrajectoryFollowerConfig config_;
  std::unique_ptr<trajectory_follower::LateralController> lateral_controller_;
  std::unique_ptr<trajectory_follower::LongitudinalController> longitudinal_controller_;

  /// Steer convergence of the previous cycle, handed to the longitudinal controller.
  bool is_steer_converged_{false};
};

}  // namespace autoware::motion::control::trajectory_follower_node

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_NODE__TRAJECTORY_FOLLOWER_HPP_

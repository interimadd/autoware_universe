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

#include "autoware/trajectory_follower_node/trajectory_follower.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control::trajectory_follower_node
{
namespace
{
template <typename T>
std::vector<T> resampleHorizonByZeroOrderHold(
  const std::vector<T> & original_horizon, const double original_time_step_ms,
  const double new_time_step_ms)
{
  std::vector<T> resampled_horizon{};
  const size_t step_factor = static_cast<size_t>(original_time_step_ms / new_time_step_ms);
  const size_t resampled_size = original_horizon.size() * step_factor;
  resampled_horizon.reserve(resampled_size);
  for (const auto & command : original_horizon) {
    for (size_t i = 0; i < step_factor; ++i) {
      resampled_horizon.push_back(command);
    }
  }
  return resampled_horizon;
}
}  // namespace

std::optional<ControlHorizon> mergeLatLonHorizon(
  const trajectory_follower::LateralHorizon & lateral_horizon,
  const trajectory_follower::LongitudinalHorizon & longitudinal_horizon, const rclcpp::Time & stamp)
{
  if (lateral_horizon.controls.empty() || longitudinal_horizon.controls.empty()) {
    return std::nullopt;
  }

  ControlHorizon control_horizon{};
  control_horizon.stamp = stamp;

  // If either of the horizons has only one control, repeat the control to match the other horizon.
  if (lateral_horizon.controls.size() == 1) {
    control_horizon.time_step_ms = longitudinal_horizon.time_step_ms;
    const auto lateral = lateral_horizon.controls.front();
    for (const auto & longitudinal : longitudinal_horizon.controls) {
      Control control;
      control.longitudinal = longitudinal;
      control.lateral = lateral;
      control.stamp = stamp;
      control_horizon.controls.push_back(control);
    }
    return control_horizon;
  }
  if (longitudinal_horizon.controls.size() == 1) {
    control_horizon.time_step_ms = lateral_horizon.time_step_ms;
    const auto longitudinal = longitudinal_horizon.controls.front();
    for (const auto & lateral : lateral_horizon.controls) {
      Control control;
      control.longitudinal = longitudinal;
      control.lateral = lateral;
      control.stamp = stamp;
      control_horizon.controls.push_back(control);
    }
    return control_horizon;
  }

  // If both horizons have multiple controls, align the time steps and zero-order hold the controls.
  // calculate greatest common divisor of time steps
  // NOTE: int64_t is needed, not int. A time step of 100 ms times the precision below is
  // 1e11, which does not fit in an int, and the conversion is undefined.
  const auto gcd_double = [](const double a, const double b) {
    const double precision = 1e9;
    const auto int_a = static_cast<std::int64_t>(std::round(a * precision));
    const auto int_b = static_cast<std::int64_t>(std::round(b * precision));
    return static_cast<double>(std::gcd(int_a, int_b)) / precision;
  };
  const double time_step_ms =
    gcd_double(lateral_horizon.time_step_ms, longitudinal_horizon.time_step_ms);
  control_horizon.time_step_ms = time_step_ms;

  const auto lateral_controls = resampleHorizonByZeroOrderHold(
    lateral_horizon.controls, lateral_horizon.time_step_ms, time_step_ms);
  const auto longitudinal_controls = resampleHorizonByZeroOrderHold(
    longitudinal_horizon.controls, longitudinal_horizon.time_step_ms, time_step_ms);

  if (lateral_controls.size() != longitudinal_controls.size()) {
    return std::nullopt;
  }

  const size_t num_steps = lateral_controls.size();
  for (size_t i = 0; i < num_steps; ++i) {
    Control control{};
    control.stamp = stamp;
    control.lateral = lateral_controls.at(i);
    control.longitudinal = longitudinal_controls.at(i);
    control_horizon.controls.push_back(control);
  }
  return control_horizon;
}

TrajectoryFollower::TrajectoryFollower(
  const TrajectoryFollowerConfig & config,
  std::unique_ptr<trajectory_follower::LateralController> lateral_controller,
  std::unique_ptr<trajectory_follower::LongitudinalController> longitudinal_controller)
: config_(config),
  lateral_controller_(std::move(lateral_controller)),
  longitudinal_controller_(std::move(longitudinal_controller))
{
}

tl::expected<TrajectoryFollowerResult, std::string> TrajectoryFollower::run(
  const trajectory_follower::InputData & input_data, const rclcpp::Time & stamp)
{
  const auto lateral_result = lateral_controller_->run(input_data, stamp);
  if (!lateral_result) {
    return tl::unexpected(lateral_result.error());
  }

  // NOTE: the steer convergence of the previous cycle is handed over, as the removed sync() did.
  const auto longitudinal_result =
    longitudinal_controller_->run(input_data, stamp, is_steer_converged_);

  // Carry the lateral result over to the next cycle of the longitudinal controller.
  is_steer_converged_ = lateral_result->sync_data.is_steer_converged;

  if (!longitudinal_result) {
    return tl::unexpected(longitudinal_result.error());
  }

  const auto & lateral_output = lateral_result.value();
  const auto & longitudinal_output = longitudinal_result.value();

  // TODO(Horibe): Think specification. This comes from the old implementation.
  if ((stamp - lateral_output.control_cmd.stamp).seconds() > config_.timeout_thr_sec) {
    return tl::unexpected<std::string>("the lateral control command is too old");
  }
  if ((stamp - longitudinal_output.control_cmd.stamp).seconds() > config_.timeout_thr_sec) {
    return tl::unexpected<std::string>("the longitudinal control command is too old");
  }

  TrajectoryFollowerResult result;
  result.control_cmd.stamp = stamp;
  result.control_cmd.lateral = lateral_output.control_cmd;
  result.control_cmd.longitudinal = longitudinal_output.control_cmd;
  result.lateral_output = lateral_output;

  if (config_.enable_control_cmd_horizon) {
    result.control_cmd_horizon = mergeLatLonHorizon(
      lateral_output.control_cmd_horizon, longitudinal_output.control_cmd_horizon, stamp);
  }

  return result;
}

}  // namespace autoware::motion::control::trajectory_follower_node

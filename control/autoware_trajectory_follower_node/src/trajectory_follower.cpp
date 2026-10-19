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

#include "autoware/trajectory_follower_node/trajectory_follower.hpp"

#include <autoware_utils/system/stop_watch.hpp>

#include <algorithm>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <utility>
#include <vector>

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

namespace autoware::motion::control::trajectory_follower_node
{
std::optional<ControlHorizon> mergeLatLonHorizon(
  const LateralHorizon & lateral_horizon, const LongitudinalHorizon & longitudinal_horizon,
  const rclcpp::Time & stamp)
{
  if (lateral_horizon.controls.empty() || longitudinal_horizon.controls.empty()) {
    return std::nullopt;
  }

  autoware_control_msgs::msg::ControlHorizon control_horizon{};
  control_horizon.stamp = stamp;

  // If either of the horizons has only one control, repeat the control to match the other horizon.
  if (lateral_horizon.controls.size() == 1) {
    control_horizon.time_step_ms = longitudinal_horizon.time_step_ms;
    const auto lateral = lateral_horizon.controls.front();
    for (const auto & longitudinal : longitudinal_horizon.controls) {
      autoware_control_msgs::msg::Control control;
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
      autoware_control_msgs::msg::Control control;
      control.longitudinal = longitudinal;
      control.lateral = lateral;
      control.stamp = stamp;
      control_horizon.controls.push_back(control);
    }
    return control_horizon;
  }

  // If both horizons have multiple controls, align the time steps and zero-order hold the controls.
  // calculate greatest common divisor of time steps
  const auto gcd_double = [](const double a, const double b) {
    const double precision = 1e9;
    const int int_a = static_cast<int>(round(a * precision));
    const int int_b = static_cast<int>(round(b * precision));
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
    autoware_control_msgs::msg::Control control{};
    control.stamp = stamp;
    control.lateral = lateral_controls.at(i);
    control.longitudinal = longitudinal_controls.at(i);
    control_horizon.controls.push_back(control);
  }
  return control_horizon;
}

TrajectoryFollower::TrajectoryFollower(
  const TrajectoryFollowerConfig & config, std::unique_ptr<LateralController> lateral_controller,
  std::unique_ptr<LongitudinalController> longitudinal_controller)
: config_(config),
  lateral_controller_(std::move(lateral_controller)),
  longitudinal_controller_(std::move(longitudinal_controller))
{
}

tl::expected<TrajectoryFollowerResult, std::string> TrajectoryFollower::run(
  const InputData & input_data, const rclcpp::Time & stamp)
{
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  TrajectoryFollowerResult result;

  stop_watch.tic("lateral");
  const auto lateral = lateral_controller_->run(input_data, stamp);
  result.lateral_processing_time_ms = stop_watch.toc("lateral");
  if (!lateral) {
    return tl::unexpected("the lateral controller cannot run: " + lateral.error());
  }

  stop_watch.tic("longitudinal");
  const auto longitudinal = longitudinal_controller_->run(input_data, lateral_feedback_, stamp);
  result.longitudinal_processing_time_ms = stop_watch.toc("longitudinal");
  if (!longitudinal) {
    return tl::unexpected("the longitudinal controller cannot run: " + longitudinal.error());
  }

  // Carry what the lateral controller reported to the longitudinal controller of the next
  // cycle. This is the one value the two controllers share.
  lateral_feedback_ = lateral->feedback;
  result.lateral_feedback = lateral->feedback;

  // TODO(Horibe): Think specification. This comes from the old implementation.
  if ((stamp - rclcpp::Time(lateral->control_cmd.stamp)).seconds() > config_.timeout_thr_sec) {
    return tl::unexpected<std::string>("the lateral control command is too old");
  }
  if ((stamp - rclcpp::Time(longitudinal->control_cmd.stamp)).seconds() > config_.timeout_thr_sec) {
    return tl::unexpected<std::string>("the longitudinal control command is too old");
  }

  result.control_cmd.stamp = stamp;
  result.control_cmd.lateral = lateral->control_cmd;
  result.control_cmd.longitudinal = longitudinal->control_cmd;

  if (config_.enable_control_cmd_horizon) {
    const auto merged =
      mergeLatLonHorizon(lateral->control_cmd_horizon, longitudinal->control_cmd_horizon, stamp);
    if (merged) {
      result.control_cmd_horizon = *merged;
    }
  }

  return result;
}

}  // namespace autoware::motion::control::trajectory_follower_node

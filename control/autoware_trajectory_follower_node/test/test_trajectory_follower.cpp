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

// No test below starts a ROS 2 context. TrajectoryFollower reads no clock, no parameter
// and no topic, so the whole control cycle runs in the test process.

#include "autoware/trajectory_follower_node/trajectory_follower.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
using autoware::motion::control::trajectory_follower::InputData;
using autoware::motion::control::trajectory_follower::LateralController;
using autoware::motion::control::trajectory_follower::LateralFeedback;
using autoware::motion::control::trajectory_follower::LateralHorizon;
using autoware::motion::control::trajectory_follower::LateralOutput;
using autoware::motion::control::trajectory_follower::LongitudinalController;
using autoware::motion::control::trajectory_follower::LongitudinalHorizon;
using autoware::motion::control::trajectory_follower::LongitudinalOutput;
using autoware::motion::control::trajectory_follower_node::mergeLatLonHorizon;
using autoware::motion::control::trajectory_follower_node::TrajectoryFollower;
using autoware::motion::control::trajectory_follower_node::TrajectoryFollowerConfig;
using autoware_control_msgs::msg::Lateral;
using autoware_control_msgs::msg::Longitudinal;

constexpr double timeout_thr_sec = 0.5;

/// The input of a cycle, left at its defaults. Every test below is about what
/// TrajectoryFollower does with what the controllers answer, not about the input.
InputData make_input_data()
{
  return InputData();
}

rclcpp::Time make_time(const double seconds)
{
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME);
}

/// A lateral controller the test drives directly. It answers with whatever the test asked
/// for, and records what it was given, so that each test below is about TrajectoryFollower
/// and not about an algorithm.
class FakeLateralController : public LateralController
{
public:
  tl::expected<LateralOutput, std::string> run(
    const InputData & input_data, const rclcpp::Time & stamp) override
  {
    ++call_count;
    last_stamp = stamp;
    last_steering_offset = input_data.steering_offset;
    if (!failure_reason.empty()) {
      return tl::unexpected(failure_reason);
    }
    output.control_cmd.stamp = command_stamp.value_or(stamp);
    return output;
  }

  LateralOutput output;
  std::string failure_reason;
  /// Set to answer with a command older than the cycle the follower is running.
  std::optional<rclcpp::Time> command_stamp;

  int call_count{0};
  rclcpp::Time last_stamp{0, 0, RCL_ROS_TIME};
  double last_steering_offset{0.0};
};

class FakeLongitudinalController : public LongitudinalController
{
public:
  tl::expected<LongitudinalOutput, std::string> run(
    [[maybe_unused]] const InputData & input_data, const LateralFeedback & lateral_feedback,
    const rclcpp::Time & stamp) override
  {
    ++call_count;
    last_feedback = lateral_feedback;
    if (!failure_reason.empty()) {
      return tl::unexpected(failure_reason);
    }
    output.control_cmd.stamp = stamp;
    return output;
  }

  LongitudinalOutput output;
  std::string failure_reason;

  int call_count{0};
  LateralFeedback last_feedback;
};

struct Followers
{
  FakeLateralController * lateral;
  FakeLongitudinalController * longitudinal;
  std::unique_ptr<TrajectoryFollower> follower;
};

/// Build a follower over two controllers the test keeps a pointer to.
Followers make_followers(const bool enable_control_cmd_horizon = false)
{
  auto lateral = std::make_unique<FakeLateralController>();
  auto longitudinal = std::make_unique<FakeLongitudinalController>();

  TrajectoryFollowerConfig config;
  config.ctrl_period = 0.03;
  config.timeout_thr_sec = timeout_thr_sec;
  config.enable_control_cmd_horizon = enable_control_cmd_horizon;

  Followers followers{lateral.get(), longitudinal.get(), nullptr};
  followers.follower =
    std::make_unique<TrajectoryFollower>(config, std::move(lateral), std::move(longitudinal));
  return followers;
}

Lateral make_lateral_command(const double steering_tire_angle)
{
  Lateral command;
  command.steering_tire_angle = static_cast<float>(steering_tire_angle);
  return command;
}

Longitudinal make_longitudinal_command(const double acceleration)
{
  Longitudinal command;
  command.acceleration = static_cast<float>(acceleration);
  return command;
}

LateralHorizon make_lateral_horizon(const std::vector<double> & angles, const double time_step_ms)
{
  LateralHorizon horizon;
  horizon.time_step_ms = time_step_ms;
  for (const auto angle : angles) {
    horizon.controls.push_back(make_lateral_command(angle));
  }
  return horizon;
}

LongitudinalHorizon make_longitudinal_horizon(
  const std::vector<double> & accelerations, const double time_step_ms)
{
  LongitudinalHorizon horizon;
  horizon.time_step_ms = time_step_ms;
  for (const auto acceleration : accelerations) {
    horizon.controls.push_back(make_longitudinal_command(acceleration));
  }
  return horizon;
}
}  // namespace

TEST(TrajectoryFollowerTest, JoinsTheTwoCommandsIntoOneVehicleCommand)
{
  auto followers = make_followers();
  followers.lateral->output.control_cmd = make_lateral_command(0.25);
  followers.longitudinal->output.control_cmd = make_longitudinal_command(1.5);

  const auto result = followers.follower->run(make_input_data(), make_time(10.0));

  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->control_cmd.lateral.steering_tire_angle, 0.25f);
  EXPECT_FLOAT_EQ(result->control_cmd.longitudinal.acceleration, 1.5f);
}

TEST(TrajectoryFollowerTest, StampsTheCommandWithTheTimeOfTheCycle)
{
  auto followers = make_followers();

  const auto result = followers.follower->run(make_input_data(), make_time(10.0));

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(rclcpp::Time(result->control_cmd.stamp), make_time(10.0));
}

TEST(TrajectoryFollowerTest, GivesBothControllersTheSameCycleTime)
{
  auto followers = make_followers();

  followers.follower->run(make_input_data(), make_time(10.0)).value();

  EXPECT_EQ(followers.lateral->last_stamp, make_time(10.0));
}

TEST(TrajectoryFollowerTest, PassesTheInputDataThroughToTheController)
{
  auto followers = make_followers();
  InputData input_data;
  input_data.steering_offset = 0.125;

  followers.follower->run(input_data, make_time(10.0)).value();

  EXPECT_DOUBLE_EQ(followers.lateral->last_steering_offset, 0.125);
}

TEST(TrajectoryFollowerTest, ReportsWhyTheLateralControllerCouldNotRun)
{
  auto followers = make_followers();
  followers.lateral->failure_reason = "no vehicle model";

  const auto result = followers.follower->run(make_input_data(), make_time(10.0));

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), "the lateral controller cannot run: no vehicle model");
}

TEST(TrajectoryFollowerTest, SkipsTheLongitudinalControllerWhenTheLateralOneCannotRun)
{
  auto followers = make_followers();
  followers.lateral->failure_reason = "no vehicle model";

  const auto result = followers.follower->run(make_input_data(), make_time(10.0));

  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(followers.longitudinal->call_count, 0);
}

TEST(TrajectoryFollowerTest, ReportsWhyTheLongitudinalControllerCouldNotRun)
{
  auto followers = make_followers();
  followers.longitudinal->failure_reason = "no trajectory";

  const auto result = followers.follower->run(make_input_data(), make_time(10.0));

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), "the longitudinal controller cannot run: no trajectory");
}

/// The longitudinal controller reads what the lateral controller reported on the cycle
/// before, not on the cycle they are both running.
TEST(TrajectoryFollowerTest, GivesTheLongitudinalControllerTheFeedbackOfThePreviousCycle)
{
  auto followers = make_followers();
  followers.lateral->output.feedback.is_steer_converged = true;

  followers.follower->run(make_input_data(), make_time(10.0)).value();
  followers.follower->run(make_input_data(), make_time(10.03)).value();

  EXPECT_TRUE(followers.longitudinal->last_feedback.is_steer_converged);
}

TEST(TrajectoryFollowerTest, GivesTheLongitudinalControllerNoFeedbackOnTheFirstCycle)
{
  auto followers = make_followers();
  followers.lateral->output.feedback.is_steer_converged = true;

  followers.follower->run(make_input_data(), make_time(10.0)).value();

  EXPECT_FALSE(followers.longitudinal->last_feedback.is_steer_converged);
}

TEST(TrajectoryFollowerTest, RejectsACommandOlderThanTheTimeout)
{
  auto followers = make_followers();
  followers.lateral->command_stamp = make_time(10.0 - timeout_thr_sec - 0.1);

  const auto result = followers.follower->run(make_input_data(), make_time(10.0));

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), "the lateral control command is too old");
}

TEST(TrajectoryFollowerTest, AcceptsACommandWithinTheTimeout)
{
  auto followers = make_followers();
  followers.lateral->command_stamp = make_time(10.0 - timeout_thr_sec + 0.1);

  const auto result = followers.follower->run(make_input_data(), make_time(10.0));

  EXPECT_TRUE(result.has_value());
}

TEST(TrajectoryFollowerTest, LeavesTheHorizonEmptyWhileItIsDisabled)
{
  auto followers = make_followers(false);
  followers.lateral->output.control_cmd_horizon = make_lateral_horizon({0.1}, 100.0);
  followers.longitudinal->output.control_cmd_horizon = make_longitudinal_horizon({1.0}, 100.0);

  const auto result = followers.follower->run(make_input_data(), make_time(10.0));

  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->control_cmd_horizon.controls.empty());
}

TEST(TrajectoryFollowerTest, FillsTheHorizonWhileItIsEnabled)
{
  auto followers = make_followers(true);
  followers.lateral->output.control_cmd_horizon = make_lateral_horizon({0.1, 0.2}, 100.0);
  followers.longitudinal->output.control_cmd_horizon = make_longitudinal_horizon({1.0, 2.0}, 100.0);

  const auto result = followers.follower->run(make_input_data(), make_time(10.0));

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->control_cmd_horizon.controls.size(), 2u);
}

TEST(MergeLatLonHorizonTest, RepeatsASingleLateralControlOverTheLongitudinalHorizon)
{
  const auto lateral = make_lateral_horizon({0.3}, 100.0);
  const auto longitudinal = make_longitudinal_horizon({1.0, 2.0, 3.0}, 50.0);

  const auto merged = mergeLatLonHorizon(lateral, longitudinal, make_time(10.0));

  ASSERT_TRUE(merged.has_value());
  EXPECT_EQ(merged->controls.size(), 3u);
  EXPECT_DOUBLE_EQ(merged->time_step_ms, 50.0);
  EXPECT_FLOAT_EQ(merged->controls.back().lateral.steering_tire_angle, 0.3f);
}

TEST(MergeLatLonHorizonTest, RepeatsASingleLongitudinalControlOverTheLateralHorizon)
{
  const auto lateral = make_lateral_horizon({0.1, 0.2}, 100.0);
  const auto longitudinal = make_longitudinal_horizon({1.5}, 50.0);

  const auto merged = mergeLatLonHorizon(lateral, longitudinal, make_time(10.0));

  ASSERT_TRUE(merged.has_value());
  EXPECT_EQ(merged->controls.size(), 2u);
  EXPECT_DOUBLE_EQ(merged->time_step_ms, 100.0);
  EXPECT_FLOAT_EQ(merged->controls.back().longitudinal.acceleration, 1.5f);
}

/// Two horizons of different time steps are both resampled to the greatest common divisor
/// of the steps, by holding each control until the next one starts.
TEST(MergeLatLonHorizonTest, ResamplesTwoHorizonsToTheCommonTimeStep)
{
  const auto lateral = make_lateral_horizon({0.1, 0.2}, 100.0);
  const auto longitudinal = make_longitudinal_horizon({1.0, 2.0, 3.0, 4.0}, 50.0);

  const auto merged = mergeLatLonHorizon(lateral, longitudinal, make_time(10.0));

  ASSERT_TRUE(merged.has_value());
  EXPECT_DOUBLE_EQ(merged->time_step_ms, 50.0);
  EXPECT_EQ(merged->controls.size(), 4u);
  EXPECT_FLOAT_EQ(merged->controls.at(0).lateral.steering_tire_angle, 0.1f);
  EXPECT_FLOAT_EQ(merged->controls.at(1).lateral.steering_tire_angle, 0.1f);
  EXPECT_FLOAT_EQ(merged->controls.at(2).lateral.steering_tire_angle, 0.2f);
  EXPECT_FLOAT_EQ(merged->controls.at(3).longitudinal.acceleration, 4.0f);
}

TEST(MergeLatLonHorizonTest, ReturnsNothingWhenTheLateralHorizonIsEmpty)
{
  const auto lateral = make_lateral_horizon({}, 100.0);
  const auto longitudinal = make_longitudinal_horizon({1.0}, 100.0);

  const auto merged = mergeLatLonHorizon(lateral, longitudinal, make_time(10.0));

  EXPECT_FALSE(merged.has_value());
}

TEST(MergeLatLonHorizonTest, ReturnsNothingWhenTheLongitudinalHorizonIsEmpty)
{
  const auto lateral = make_lateral_horizon({0.1}, 100.0);
  const auto longitudinal = make_longitudinal_horizon({}, 100.0);

  const auto merged = mergeLatLonHorizon(lateral, longitudinal, make_time(10.0));

  EXPECT_FALSE(merged.has_value());
}

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

// Nothing in this file builds an rclcpp::Node or an executor. That is the point of the
// tests: TrajectoryFollower is the control logic of the node, and it runs without the
// ROS 2 runtime, so a desk evaluation can drive it in a loop.

#include "autoware/trajectory_follower_node/trajectory_follower.hpp"
#include "gtest/gtest.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
using autoware::motion::control::trajectory_follower::InputData;
using autoware::motion::control::trajectory_follower::LateralControllerInterface;
using autoware::motion::control::trajectory_follower::LateralHorizon;
using autoware::motion::control::trajectory_follower::LateralOutput;
using autoware::motion::control::trajectory_follower::LongitudinalControllerInterface;
using autoware::motion::control::trajectory_follower::LongitudinalHorizon;
using autoware::motion::control::trajectory_follower::LongitudinalOutput;
using autoware::motion::control::trajectory_follower_node::mergeLatLonHorizon;
using autoware::motion::control::trajectory_follower_node::TrajectoryFollower;
using autoware::motion::control::trajectory_follower_node::TrajectoryFollowerConfig;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

// Helper function that builds a time of the clock the node uses, so that a command stamp
// taken from a message can be subtracted from it.
rclcpp::Time make_time(const double seconds)
{
  const auto whole_seconds = static_cast<int32_t>(std::floor(seconds));
  const auto nanoseconds = static_cast<uint32_t>(std::llround((seconds - whole_seconds) * 1e9));
  return rclcpp::Time(whole_seconds, nanoseconds, RCL_ROS_TIME);
}

/// A lateral controller a test drives directly. It reports what it was told to report, so
/// that a test can say what the lateral side of a cycle produced without running MPC.
class FakeLateralController : public LateralControllerInterface
{
public:
  tl::expected<LateralOutput, std::string> run(
    const InputData &, const rclcpp::Time & stamp) override
  {
    ++cycles_run;
    if (failure_reason) {
      return tl::unexpected(*failure_reason);
    }

    LateralOutput output;
    output.control_cmd.stamp = command_stamp ? *command_stamp : stamp;
    output.control_cmd.steering_tire_angle = steering_tire_angle;
    output.sync_data.is_steer_converged = is_steer_converged;
    output.control_cmd_horizon = horizon;
    return output;
  }

  /// What the next cycle reports.
  std::optional<std::string> failure_reason{std::nullopt};
  std::optional<rclcpp::Time> command_stamp{std::nullopt};
  float steering_tire_angle{0.1f};
  bool is_steer_converged{false};
  LateralHorizon horizon{};

  int cycles_run{0};
};

/// A longitudinal controller a test drives directly. It records the steer convergence it
/// was handed, so that a test can say what the follower passed on from the cycle before.
class FakeLongitudinalController : public LongitudinalControllerInterface
{
public:
  tl::expected<LongitudinalOutput, std::string> run(
    const InputData &, const rclcpp::Time & stamp, const bool is_steer_converged) override
  {
    steer_convergence_received.push_back(is_steer_converged);
    if (failure_reason) {
      return tl::unexpected(*failure_reason);
    }

    LongitudinalOutput output;
    output.control_cmd.stamp = command_stamp ? *command_stamp : stamp;
    output.control_cmd.velocity = velocity;
    output.control_cmd_horizon = horizon;
    return output;
  }

  /// What the next cycle reports.
  std::optional<std::string> failure_reason{std::nullopt};
  std::optional<rclcpp::Time> command_stamp{std::nullopt};
  float velocity{0.0f};
  LongitudinalHorizon horizon{};

  /// What every cycle so far was handed, oldest first.
  std::vector<bool> steer_convergence_received;
};

// Helper function that builds a straight, flat trajectory along the +x axis. Every point
// shares the given constant velocity. Points are spaced 1.0 m apart.
Trajectory make_straight_trajectory(const std::size_t num_points, const double velocity)
{
  Trajectory trajectory;
  for (std::size_t point_index = 0; point_index < num_points; ++point_index) {
    TrajectoryPoint point;
    point.pose.position.x = static_cast<double>(point_index);
    point.pose.orientation.w = 1.0;  // identity orientation, heading along +x
    point.longitudinal_velocity_mps = static_cast<float>(velocity);
    trajectory.points.push_back(point);
  }
  return trajectory;
}

// Helper function that builds input data for a vehicle standing at the start of a path.
InputData make_input_data(const Trajectory & trajectory)
{
  InputData input_data;
  input_data.current_trajectory = trajectory;
  input_data.current_odometry.pose.pose.orientation.w = 1.0;
  input_data.current_operation_mode.is_autoware_control_enabled = true;
  input_data.current_operation_mode.mode = OperationModeState::AUTONOMOUS;
  return input_data;
}

struct Setup
{
  FakeLateralController * lateral{nullptr};
  FakeLongitudinalController * longitudinal{nullptr};
  std::unique_ptr<TrajectoryFollower> follower;
};

// Helper function that builds a follower out of two fake controllers, and keeps a view of
// each one so that a test can steer what it reports and read what it was handed.
Setup make_follower(const TrajectoryFollowerConfig & config = TrajectoryFollowerConfig{1.0, false})
{
  auto lateral = std::make_unique<FakeLateralController>();
  auto longitudinal = std::make_unique<FakeLongitudinalController>();
  auto * lateral_view = lateral.get();
  auto * longitudinal_view = longitudinal.get();
  return Setup{
    lateral_view, longitudinal_view,
    std::make_unique<TrajectoryFollower>(config, std::move(lateral), std::move(longitudinal))};
}
}  // namespace

TEST(TrajectoryFollower, JoinsWhatBothControllersProducedIntoOneCommand)
{
  // Arrange
  auto setup = make_follower();
  setup.lateral->steering_tire_angle = 0.25f;
  setup.longitudinal->velocity = 2.5f;
  const auto input_data = make_input_data(make_straight_trajectory(10, 0.0));

  // Act
  const auto result = setup.follower->run(input_data, make_time(1.0));

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->control_cmd.lateral.steering_tire_angle, 0.25f);
  EXPECT_FLOAT_EQ(result->control_cmd.longitudinal.velocity, 2.5f);
  EXPECT_EQ(rclcpp::Time(result->control_cmd.stamp), make_time(1.0));
}

TEST(TrajectoryFollower, ReportsTheReasonWhenTheLongitudinalControllerCannotRun)
{
  // Arrange
  auto setup = make_follower();
  setup.longitudinal->failure_reason = "the vehicle is not ready yet.";
  const auto input_data = make_input_data(make_straight_trajectory(10, 0.0));

  // Act
  const auto result = setup.follower->run(input_data, make_time(1.0));

  // Assert
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), "the vehicle is not ready yet.");
}

TEST(TrajectoryFollower, ReportsTheReasonWhenTheLateralControllerCannotRun)
{
  // Arrange
  auto setup = make_follower();
  setup.lateral->failure_reason = "trajectory size is zero.";
  const auto input_data = make_input_data(make_straight_trajectory(10, 0.0));

  // Act
  const auto result = setup.follower->run(input_data, make_time(1.0));

  // Assert
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), "trajectory size is zero.");
}

TEST(TrajectoryFollower, ProducesNoCommandWhenTheLateralCommandIsTooOld)
{
  // Arrange
  auto setup = make_follower(TrajectoryFollowerConfig{0.5, false});
  setup.lateral->command_stamp = make_time(0.0);
  const auto input_data = make_input_data(make_straight_trajectory(10, 0.0));

  // Act
  const auto result = setup.follower->run(input_data, make_time(10.0));

  // Assert
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), "the lateral control command is too old");
}

TEST(TrajectoryFollower, HandsTheSteerConvergenceOfThePreviousCycleToTheLongitudinalController)
{
  // Arrange
  // The first cycle hands over the value from before any cycle ran, which is false, even
  // though the lateral controller reports a converged steering from the first cycle on.
  auto setup = make_follower();
  setup.lateral->is_steer_converged = true;
  const auto input_data = make_input_data(make_straight_trajectory(20, 5.0));

  // Act
  const auto first_cycle = setup.follower->run(input_data, make_time(1.0));
  const auto second_cycle = setup.follower->run(input_data, make_time(1.03));
  const auto third_cycle = setup.follower->run(input_data, make_time(1.06));

  // Assert
  ASSERT_TRUE(first_cycle.has_value());
  ASSERT_TRUE(second_cycle.has_value());
  ASSERT_TRUE(third_cycle.has_value());
  EXPECT_EQ(setup.longitudinal->steer_convergence_received, std::vector<bool>({false, true, true}));
}

TEST(TrajectoryFollower, HoldsNoHorizonWhenTheConfigDoesNotAskForOne)
{
  // Arrange
  auto setup = make_follower(TrajectoryFollowerConfig{1.0, false});
  setup.lateral->horizon.time_step_ms = 100.0;
  setup.lateral->horizon.controls.resize(2);
  const auto input_data = make_input_data(make_straight_trajectory(10, 0.0));

  // Act
  const auto result = setup.follower->run(input_data, make_time(1.0));

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->control_cmd_horizon.has_value());
}

TEST(MergeLatLonHorizon, RepeatsASingleLateralControlOverTheLongitudinalOne)
{
  // Arrange
  LateralHorizon lateral;
  lateral.time_step_ms = 100.0;
  lateral.controls.resize(1);
  lateral.controls.front().steering_tire_angle = 0.2f;
  LongitudinalHorizon longitudinal;
  longitudinal.time_step_ms = 50.0;
  longitudinal.controls.resize(3);

  // Act
  const auto horizon = mergeLatLonHorizon(lateral, longitudinal, make_time(1.0));

  // Assert
  ASSERT_TRUE(horizon.has_value());
  EXPECT_DOUBLE_EQ(horizon->time_step_ms, 50.0);
  ASSERT_EQ(horizon->controls.size(), 3u);
  for (const auto & control : horizon->controls) {
    EXPECT_FLOAT_EQ(control.lateral.steering_tire_angle, 0.2f);
  }
}

TEST(MergeLatLonHorizon, ResamplesBothHorizonsToTheGreatestCommonDivisorOfTheTimeSteps)
{
  // Arrange
  LateralHorizon lateral;
  lateral.time_step_ms = 100.0;
  lateral.controls.resize(2);
  LongitudinalHorizon longitudinal;
  longitudinal.time_step_ms = 50.0;
  longitudinal.controls.resize(4);

  // Act
  const auto horizon = mergeLatLonHorizon(lateral, longitudinal, make_time(1.0));

  // Assert
  ASSERT_TRUE(horizon.has_value());
  EXPECT_DOUBLE_EQ(horizon->time_step_ms, 50.0);
  EXPECT_EQ(horizon->controls.size(), 4u);
}

TEST(MergeLatLonHorizon, HoldsNothingWhenAHorizonIsEmpty)
{
  // Arrange
  LateralHorizon lateral;
  lateral.time_step_ms = 100.0;
  LongitudinalHorizon longitudinal;
  longitudinal.time_step_ms = 100.0;
  longitudinal.controls.resize(2);

  // Act
  const auto horizon = mergeLatLonHorizon(lateral, longitudinal, make_time(1.0));

  // Assert
  EXPECT_FALSE(horizon.has_value());
}

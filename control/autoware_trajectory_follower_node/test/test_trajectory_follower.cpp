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
using autoware::motion::control::pid_longitudinal_controller::ControlState;
using autoware::motion::control::pid_longitudinal_controller::PidLongitudinalController;
using autoware::motion::control::pid_longitudinal_controller::PidLongitudinalControllerConfig;
using autoware::motion::control::trajectory_follower::InputData;
using autoware::motion::control::trajectory_follower::LateralController;
using autoware::motion::control::trajectory_follower::LateralHorizon;
using autoware::motion::control::trajectory_follower::LateralOutput;
using autoware::motion::control::trajectory_follower::LongitudinalHorizon;
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
class FakeLateralController : public LateralController
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

// Helper function that builds a config equivalent to the shipped default parameters, with
// only the fields these tests depend on set to a value they can reason about.
PidLongitudinalControllerConfig make_longitudinal_config()
{
  PidLongitudinalControllerConfig config{};

  config.wheel_base = 2.79;
  config.front_overhang = 1.0;
  config.longitudinal_ctrl_period = 0.03;

  config.delay_compensation_time = 0.17;
  config.use_temporal_trajectory = false;

  config.enable_smooth_stop = true;
  config.enable_overshoot_emergency = true;
  config.enable_large_tracking_error_emergency = true;
  config.enable_slope_compensation = true;
  config.enable_keep_stopped_until_steer_convergence = false;

  config.state_transition_params.drive_state_stop_dist = 0.5;
  config.state_transition_params.drive_state_offset_stop_dist = 1.0;
  config.state_transition_params.stopping_state_stop_dist = 0.5;
  config.state_transition_params.stopped_state_entry_duration_time = 0.1;
  config.state_transition_params.stopped_state_entry_vel = 0.01;
  config.state_transition_params.stopped_state_entry_acc = 0.1;
  config.state_transition_params.emergency_state_overshoot_stop_dist = 1.5;

  config.pid_gains.kp = 1.0;
  config.pid_gains.ki = 0.1;
  config.pid_gains.kd = 0.0;

  config.pid_limits.max_out = 1.0;
  config.pid_limits.min_out = -1.0;
  config.pid_limits.max_p_effort = 1.0;
  config.pid_limits.min_p_effort = -1.0;
  config.pid_limits.max_i_effort = 0.3;
  config.pid_limits.min_i_effort = -0.3;
  config.pid_limits.max_d_effort = 0.0;
  config.pid_limits.min_d_effort = 0.0;

  config.lpf_vel_error_gain = 0.9;
  config.enable_integration_at_low_speed = false;
  config.current_vel_threshold_pid_integrate = 0.5;
  config.time_threshold_before_pid_integrate = 2.0;
  config.ff_scale_min = 0.5;
  config.ff_scale_max = 2.0;
  config.enable_brake_keeping_before_stop = false;
  config.brake_keeping_acc = -0.2;

  config.smooth_stop_params.max_strong_acc = -0.5;
  config.smooth_stop_params.min_strong_acc = -0.8;
  config.smooth_stop_params.weak_acc = -0.3;
  config.smooth_stop_params.weak_stop_acc = -0.8;
  config.smooth_stop_params.strong_stop_acc = -3.4;
  config.smooth_stop_params.min_fast_vel = 0.5;
  config.smooth_stop_params.min_running_vel = 0.01;
  config.smooth_stop_params.min_running_acc = 0.01;
  config.smooth_stop_params.weak_stop_time = 0.8;
  config.smooth_stop_params.weak_stop_dist = -0.3;
  config.smooth_stop_params.strong_stop_dist = -0.5;

  config.stopped_state_params.vel = 0.0;
  config.stopped_state_params.acc = -3.4;

  config.emergency_state_params.vel = 0.0;
  config.emergency_state_params.acc = -5.0;
  config.emergency_state_params.jerk = -3.0;

  config.acc_feedback_gain = 0.0;
  config.lpf_acc_error_gain = 0.98;

  config.max_acc = 3.0;
  config.min_acc = -5.0;
  config.max_jerk = 2.0;
  config.min_jerk = -5.0;
  config.max_acc_cmd_diff = 50.0;

  config.slope_source = PidLongitudinalControllerConfig::SlopeSource::TRAJECTORY_GOAL_ADAPTIVE;
  config.adaptive_trajectory_velocity_th = 1.0;
  config.lpf_pitch_gain = 0.95;
  config.max_pitch_rad = 0.1;
  config.min_pitch_rad = -0.1;

  config.ego_nearest_dist_threshold = 3.0;
  config.ego_nearest_yaw_threshold = 1.0472;

  return config;
}

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
  std::unique_ptr<TrajectoryFollower> follower;
};

// Helper function that builds a follower out of a fake lateral controller and the real
// longitudinal one, and keeps a view of the fake so a test can steer what it reports.
Setup make_follower(
  const TrajectoryFollowerConfig & config = TrajectoryFollowerConfig{1.0, false},
  const PidLongitudinalControllerConfig & longitudinal_config = make_longitudinal_config())
{
  auto lateral = std::make_unique<FakeLateralController>();
  auto * lateral_view = lateral.get();
  auto longitudinal = std::make_unique<PidLongitudinalController>(longitudinal_config);
  return Setup{
    lateral_view,
    std::make_unique<TrajectoryFollower>(config, std::move(lateral), std::move(longitudinal))};
}
}  // namespace

TEST(TrajectoryFollower, JoinsWhatBothControllersProducedIntoOneCommand)
{
  // Arrange
  auto setup = make_follower();
  setup.lateral->steering_tire_angle = 0.25f;
  const auto input_data = make_input_data(make_straight_trajectory(10, 0.0));

  // Act
  const auto result = setup.follower->run(input_data, make_time(1.0));

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->control_cmd.lateral.steering_tire_angle, 0.25f);
  EXPECT_FLOAT_EQ(result->control_cmd.longitudinal.velocity, 0.0f);
  EXPECT_EQ(rclcpp::Time(result->control_cmd.stamp), make_time(1.0));
  EXPECT_EQ(result->longitudinal_result.control_state, ControlState::STOPPED);
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
  // The keep-stopped guard lets a test read which steer convergence the longitudinal
  // controller was given. It departs from STOPPED once it has been told twice in a row
  // that the steering converged, so the lateral controller reporting a converged steering
  // from the first cycle on must take three cycles to depart: the first hands over the
  // value from before any cycle ran, which is false. Handing over the value of the running
  // cycle instead would depart one cycle earlier.
  auto longitudinal_config = make_longitudinal_config();
  longitudinal_config.enable_keep_stopped_until_steer_convergence = true;
  auto setup = make_follower(TrajectoryFollowerConfig{1.0, false}, longitudinal_config);
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
  EXPECT_EQ(first_cycle->longitudinal_result.control_state, ControlState::STOPPED);
  EXPECT_EQ(second_cycle->longitudinal_result.control_state, ControlState::STOPPED);
  EXPECT_EQ(third_cycle->longitudinal_result.control_state, ControlState::DRIVE);
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

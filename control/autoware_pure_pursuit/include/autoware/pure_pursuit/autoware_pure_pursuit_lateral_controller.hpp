// Copyright 2020-2022 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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
/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef AUTOWARE__PURE_PURSUIT__AUTOWARE_PURE_PURSUIT_LATERAL_CONTROLLER_HPP_
#define AUTOWARE__PURE_PURSUIT__AUTOWARE_PURE_PURSUIT_LATERAL_CONTROLLER_HPP_

#include "autoware/pure_pursuit/autoware_pure_pursuit.hpp"
#include "autoware/trajectory_follower_base/input_data.hpp"
#include "autoware/trajectory_follower_base/lateral_controller.hpp"

#include <rclcpp/time.hpp>

#include "autoware_control_msgs/msg/lateral.hpp"
#include "autoware_internal_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::pure_pursuit
{
namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;

using autoware_control_msgs::msg::Lateral;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

struct PpOutput
{
  double curvature;
  double velocity;
};

/// Everything PurePursuitLateralController reads to set itself up. The node layer fills it
/// from the parameters it declares, so that the controller itself needs no node.
struct PurePursuitLateralControllerConfig
{
  // Vehicle
  double wheel_base{0.0};
  double max_steering_angle{0.0};  // [rad]

  // Algorithm
  double ld_velocity_ratio{0.0};
  double ld_lateral_error_ratio{0.0};
  double ld_curvature_ratio{0.0};
  double min_lookahead_distance{0.0};
  double max_lookahead_distance{0.0};
  double reverse_min_lookahead_distance{0.0};  // min_lookahead_distance in reverse gear
  double converged_steer_rad{0.0};
  double prediction_ds{0.0};
  double prediction_distance_length{0.0};  // Total distance of prediction trajectory
  double resampling_ds{0.0};
  double curvature_calculation_distance{0.0};
  double long_ld_lateral_error_threshold{0.0};
  bool enable_path_smoothing{false};
  int64_t path_filter_moving_ave_num{0};
};

/// What one cycle of PurePursuitLateralController produced, beside the command itself. The
/// node layer publishes the messages and reports the flags.
struct PurePursuitLateralControllerResult
{
  trajectory_follower::LateralOutput output;
  autoware_internal_debug_msgs::msg::Float32MultiArrayStamped lookahead_distance_values;
  std::optional<visualization_msgs::msg::MarkerArray> debug_markers{std::nullopt};

  // What the cycle met, which the node layer turns into log lines.
  bool trajectory_too_short_to_smooth{false};
  bool curvature_calculation_failed{false};
  bool received_short_trajectory{false};
  bool closest_waypoint_not_found{false};
  bool control_command_solve_failed{false};
  bool prediction_solve_failed{false};
};

struct DebugData
{
  geometry_msgs::msg::Point next_target;
};

class PurePursuitLateralController : public trajectory_follower::LateralController
{
public:
  explicit PurePursuitLateralController(const PurePursuitLateralControllerConfig & config);

  /**
   * @brief compute control command for path follow with a constant control period
   * @param input_data input data required for control calculation
   * @param stamp timestamp of this control cycle, captured once by the caller
   * @return everything the cycle produced. The node layer publishes what this carries
   *         beside the command.
   */
  PurePursuitLateralControllerResult runDetailed(
    const trajectory_follower::InputData & input_data, const rclcpp::Time & stamp);

  /// The command alone, which is what LateralController asks for. This controller always
  /// produces one, so the error side is never taken.
  tl::expected<trajectory_follower::LateralOutput, std::string> run(
    const trajectory_follower::InputData & input_data, const rclcpp::Time & stamp) override;

private:
  PurePursuitLateralControllerConfig config_;

  std::vector<TrajectoryPoint> output_tp_array_;
  Trajectory trajectory_resampled_;
  nav_msgs::msg::Odometry current_odometry_;
  autoware_vehicle_msgs::msg::SteeringReport current_steering_;
  std::optional<Lateral> prev_cmd_{std::nullopt};

  // Algorithm
  std::unique_ptr<PurePursuit> pure_pursuit_;

  // What the running cycle met. Reset at the top of run() and copied into the result.
  PurePursuitLateralControllerResult result_;

  // Timestamp of the running cycle, set at the top of run().
  rclcpp::Time stamp_;

  DebugData debug_data_;

  void setResampledTrajectory(const Trajectory & trajectory);

  void averageFilterTrajectory(Trajectory & u);

  double calcCurvature(const size_t closest_idx);

  double calcLookaheadDistance(
    const double lateral_error, const double curvature, const double velocity, const double min_ld,
    const bool is_control_cmd);

  std::optional<PpOutput> calcTargetCurvature(
    bool is_control_output, const geometry_msgs::msg::Pose & pose);

  Lateral generateCtrlCmdMsg(const double target_curvature);

  Lateral generateOutputControlCmd();

  bool calcIsSteerConverged(const Lateral & cmd) const;

  /**
   * @brief It takes current pose, control command, and delta distance. Then it calculates next pose
   * of vehicle.
   */
  TrajectoryPoint calcNextPose(const double ds, TrajectoryPoint & point, Lateral cmd) const;

  std::optional<Trajectory> generatePredictedTrajectory();

  visualization_msgs::msg::MarkerArray createDebugMarker() const;
};

}  // namespace autoware::pure_pursuit

#endif  // AUTOWARE__PURE_PURSUIT__AUTOWARE_PURE_PURSUIT_LATERAL_CONTROLLER_HPP_

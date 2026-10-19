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

#include "autoware/pure_pursuit/autoware_pure_pursuit_lateral_controller.hpp"

#include "autoware/pure_pursuit/autoware_pure_pursuit_viz.hpp"
#include "autoware/pure_pursuit/util/planning_utils.hpp"

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <tf2/utils.hpp>

#include <algorithm>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace
{
enum TYPE {
  VEL_LD = 0,
  CURVATURE_LD = 1,
  LATERAL_ERROR_LD = 2,
  TOTAL_LD = 3,
  CURVATURE = 4,
  LATERAL_ERROR = 5,
  VELOCITY = 6,
  SIZE  // this is the number of enum elements
};
}  // namespace

namespace autoware::pure_pursuit
{
PurePursuitLateralController::PurePursuitLateralController(
  const PurePursuitLateralControllerConfig & config)
: config_(config), stamp_(0, 0, RCL_ROS_TIME)
{
  pure_pursuit_ = std::make_unique<PurePursuit>();
}

double PurePursuitLateralController::calcLookaheadDistance(
  const double lateral_error, const double curvature, const double velocity, const double min_ld,
  const bool is_control_cmd)
{
  const double vel_ld = abs(config_.ld_velocity_ratio * velocity);
  const double curvature_ld = -abs(config_.ld_curvature_ratio * curvature);
  double lateral_error_ld = 0.0;

  if (abs(lateral_error) >= config_.long_ld_lateral_error_threshold) {
    // If lateral error is higher than threshold, we should make ld larger to prevent entering the
    // road with high heading error.
    lateral_error_ld = abs(config_.ld_lateral_error_ratio * lateral_error);
  }

  const double total_ld =
    std::clamp(vel_ld + curvature_ld + lateral_error_ld, min_ld, config_.max_lookahead_distance);

  if (is_control_cmd) {
    auto & debug_msg = result_.lookahead_distance_values;
    debug_msg.data.resize(TYPE::SIZE);
    debug_msg.data.at(TYPE::VEL_LD) = static_cast<float>(vel_ld);
    debug_msg.data.at(TYPE::CURVATURE_LD) = static_cast<float>(curvature_ld);
    debug_msg.data.at(TYPE::LATERAL_ERROR_LD) = static_cast<float>(lateral_error_ld);
    debug_msg.data.at(TYPE::TOTAL_LD) = static_cast<float>(total_ld);
    debug_msg.data.at(TYPE::VELOCITY) = static_cast<float>(velocity);
    debug_msg.data.at(TYPE::CURVATURE) = static_cast<float>(curvature);
    debug_msg.data.at(TYPE::LATERAL_ERROR) = static_cast<float>(lateral_error);
    debug_msg.stamp = stamp_;
  }

  return total_ld;
}

TrajectoryPoint PurePursuitLateralController::calcNextPose(
  const double ds, TrajectoryPoint & point, Lateral cmd) const
{
  geometry_msgs::msg::Transform transform;
  transform.translation = autoware_utils::create_translation(ds, 0.0, 0.0);
  transform.rotation = planning_utils::getQuaternionFromYaw(
    ((tan(cmd.steering_tire_angle) * ds) / config_.wheel_base));
  TrajectoryPoint output_p;

  tf2::Transform tf_pose;
  tf2::Transform tf_offset;
  tf2::fromMsg(transform, tf_offset);
  tf2::fromMsg(point.pose, tf_pose);
  tf2::toMsg(tf_pose * tf_offset, output_p.pose);
  return output_p;
}

void PurePursuitLateralController::setResampledTrajectory(const Trajectory & trajectory)
{
  // Interpolate with constant interval distance.
  std::vector<double> out_arclength;
  const auto input_tp_array = autoware::motion_utils::convertToTrajectoryPointArray(trajectory);
  const auto traj_length = autoware::motion_utils::calcArcLength(input_tp_array);
  for (double s = 0; s < traj_length; s += config_.resampling_ds) {
    out_arclength.push_back(s);
  }
  trajectory_resampled_ = autoware::motion_utils::resampleTrajectory(
    autoware::motion_utils::convertToTrajectory(input_tp_array), out_arclength);
  trajectory_resampled_.points.back() = trajectory.points.back();
  trajectory_resampled_.header = trajectory.header;
  output_tp_array_ = autoware::motion_utils::convertToTrajectoryPointArray(trajectory_resampled_);
}

double PurePursuitLateralController::calcCurvature(const size_t closest_idx)
{
  // Calculate current curvature
  const size_t idx_dist = static_cast<size_t>(std::max(
    static_cast<int>((config_.curvature_calculation_distance) / config_.resampling_ds), 1));

  // Find the points in trajectory to calculate curvature
  size_t next_idx = trajectory_resampled_.points.size() - 1;
  size_t prev_idx = 0;

  if (static_cast<size_t>(closest_idx) >= idx_dist) {
    prev_idx = closest_idx - idx_dist;
  } else {
    // return zero curvature when backward distance is not long enough in the trajectory
    return 0.0;
  }

  if (trajectory_resampled_.points.size() - 1 >= closest_idx + idx_dist) {
    next_idx = closest_idx + idx_dist;
  } else {
    // return zero curvature when forward distance is not long enough in the trajectory
    return 0.0;
  }
  // TODO(k.sugahara): shift the center point of the curvature calculation to allow sufficient
  // distance, because if sufficient distance cannot be obtained in front or behind, the curvature
  // will be zero in the current implementation.

  // Calculate curvature assuming the trajectory points interval is constant
  double current_curvature = 0.0;

  try {
    current_curvature = autoware_utils::calc_curvature(
      autoware_utils::get_point(trajectory_resampled_.points.at(prev_idx)),
      autoware_utils::get_point(trajectory_resampled_.points.at(closest_idx)),
      autoware_utils::get_point(trajectory_resampled_.points.at(next_idx)));
  } catch (std::exception const &) {
    // ...code that handles the error...
    result_.curvature_calculation_failed = true;
    current_curvature = 0.0;
  }
  return current_curvature;
}

void PurePursuitLateralController::averageFilterTrajectory(Trajectory & u)
{
  if (static_cast<int>(u.points.size()) <= 2 * config_.path_filter_moving_ave_num) {
    result_.trajectory_too_short_to_smooth = true;
    return;
  }

  Trajectory filtered_trajectory(u);

  for (int64_t i = 0; i < static_cast<int64_t>(u.points.size()); ++i) {
    TrajectoryPoint tmp{};
    int64_t num_tmp = config_.path_filter_moving_ave_num;
    int64_t count = 0;
    double yaw = 0.0;
    if (i - num_tmp < 0) {
      num_tmp = i;
    }
    if (i + num_tmp > static_cast<int64_t>(u.points.size()) - 1) {
      num_tmp = static_cast<int64_t>(u.points.size()) - i - 1;
    }
    for (int64_t j = -num_tmp; j <= num_tmp; ++j) {
      const auto & p = u.points.at(static_cast<size_t>(i + j));

      tmp.pose.position.x += p.pose.position.x;
      tmp.pose.position.y += p.pose.position.y;
      tmp.pose.position.z += p.pose.position.z;
      tmp.longitudinal_velocity_mps += p.longitudinal_velocity_mps;
      tmp.acceleration_mps2 += p.acceleration_mps2;
      tmp.front_wheel_angle_rad += p.front_wheel_angle_rad;
      tmp.heading_rate_rps += p.heading_rate_rps;
      yaw += tf2::getYaw(p.pose.orientation);
      tmp.lateral_velocity_mps += p.lateral_velocity_mps;
      tmp.rear_wheel_angle_rad += p.rear_wheel_angle_rad;
      ++count;
    }
    auto & p = filtered_trajectory.points.at(static_cast<size_t>(i));

    p.pose.position.x = tmp.pose.position.x / count;
    p.pose.position.y = tmp.pose.position.y / count;
    p.pose.position.z = tmp.pose.position.z / count;
    p.longitudinal_velocity_mps = tmp.longitudinal_velocity_mps / count;
    p.acceleration_mps2 = tmp.acceleration_mps2 / count;
    p.front_wheel_angle_rad = tmp.front_wheel_angle_rad / count;
    p.heading_rate_rps = tmp.heading_rate_rps / count;
    p.lateral_velocity_mps = tmp.lateral_velocity_mps / count;
    p.rear_wheel_angle_rad = tmp.rear_wheel_angle_rad / count;
    p.pose.orientation = autoware::pure_pursuit::planning_utils::getQuaternionFromYaw(yaw / count);
  }
  trajectory_resampled_ = filtered_trajectory;
}

std::optional<Trajectory> PurePursuitLateralController::generatePredictedTrajectory()
{
  const auto closest_idx_result = autoware::motion_utils::findNearestIndex(
    output_tp_array_, current_odometry_.pose.pose, 3.0, M_PI_4);

  if (!closest_idx_result) {
    return std::nullopt;
  }

  const double remaining_distance = planning_utils::calcArcLengthFromWayPoint(
    trajectory_resampled_, *closest_idx_result, trajectory_resampled_.points.size() - 1);

  const auto num_of_iteration = std::max(
    static_cast<int>(std::ceil(
      std::min(remaining_distance, config_.prediction_distance_length) / config_.prediction_ds)),
    1);
  Trajectory predicted_trajectory;

  // Iterative prediction:
  for (int i = 0; i < num_of_iteration; i++) {
    if (i == 0) {
      // For first point, use the odometry for velocity, and use the current_pose for prediction.

      TrajectoryPoint p;
      p.pose = current_odometry_.pose.pose;
      p.longitudinal_velocity_mps = current_odometry_.twist.twist.linear.x;
      predicted_trajectory.points.push_back(p);

      const auto pp_output = calcTargetCurvature(true, predicted_trajectory.points.at(i).pose);
      Lateral tmp_msg;

      if (pp_output) {
        tmp_msg = generateCtrlCmdMsg(pp_output->curvature);
        predicted_trajectory.points.at(i).longitudinal_velocity_mps = pp_output->velocity;
      } else {
        result_.prediction_solve_failed = true;
        tmp_msg = generateCtrlCmdMsg(0.0);
      }
      TrajectoryPoint p2;
      p2 = calcNextPose(config_.prediction_ds, predicted_trajectory.points.at(i), tmp_msg);
      predicted_trajectory.points.push_back(p2);

    } else {
      const auto pp_output = calcTargetCurvature(false, predicted_trajectory.points.at(i).pose);
      Lateral tmp_msg;

      if (pp_output) {
        tmp_msg = generateCtrlCmdMsg(pp_output->curvature);
        predicted_trajectory.points.at(i).longitudinal_velocity_mps = pp_output->velocity;
      } else {
        result_.prediction_solve_failed = true;
        tmp_msg = generateCtrlCmdMsg(0.0);
      }
      predicted_trajectory.points.push_back(
        calcNextPose(config_.prediction_ds, predicted_trajectory.points.at(i), tmp_msg));
    }
  }

  // for last point
  predicted_trajectory.points.back().longitudinal_velocity_mps = 0.0;
  predicted_trajectory.header.frame_id = trajectory_resampled_.header.frame_id;
  predicted_trajectory.header.stamp = trajectory_resampled_.header.stamp;

  return predicted_trajectory;
}

PurePursuitLateralControllerResult PurePursuitLateralController::run(
  const trajectory_follower::InputData & input_data, const rclcpp::Time & stamp)
{
  result_ = PurePursuitLateralControllerResult();
  stamp_ = stamp;

  current_odometry_ = input_data.current_odometry;
  current_steering_ = input_data.current_steering;

  setResampledTrajectory(input_data.current_trajectory);
  if (config_.enable_path_smoothing) {
    averageFilterTrajectory(trajectory_resampled_);
  }
  const auto cmd_msg = generateOutputControlCmd();

  result_.output.control_cmd = cmd_msg;
  result_.output.sync_data.is_steer_converged = calcIsSteerConverged(cmd_msg);

  // calculate predicted trajectory with iterative calculation
  result_.predicted_trajectory = generatePredictedTrajectory();

  return result_;
}

bool PurePursuitLateralController::calcIsSteerConverged(const Lateral & cmd) const
{
  return std::abs(cmd.steering_tire_angle - current_steering_.steering_tire_angle) <
         static_cast<float>(config_.converged_steer_rad);
}

Lateral PurePursuitLateralController::generateOutputControlCmd()
{
  // Generate the control command
  const auto pp_output = calcTargetCurvature(true, current_odometry_.pose.pose);
  Lateral output_cmd;

  if (pp_output) {
    output_cmd = generateCtrlCmdMsg(pp_output->curvature);
    prev_cmd_ = output_cmd;
    result_.debug_markers = createDebugMarker();
  } else {
    result_.control_command_solve_failed = true;
    if (prev_cmd_) {
      output_cmd = *prev_cmd_;
    } else {
      output_cmd = generateCtrlCmdMsg(0.0);
    }
  }
  return output_cmd;
}

Lateral PurePursuitLateralController::generateCtrlCmdMsg(const double target_curvature)
{
  const double tmp_steering =
    planning_utils::convertCurvatureToSteeringAngle(config_.wheel_base, target_curvature);
  Lateral cmd;
  cmd.stamp = stamp_;
  cmd.steering_tire_angle = static_cast<float>(
    std::min(std::max(tmp_steering, -config_.max_steering_angle), config_.max_steering_angle));

  return cmd;
}

visualization_msgs::msg::MarkerArray PurePursuitLateralController::createDebugMarker() const
{
  visualization_msgs::msg::MarkerArray marker_array;

  marker_array.markers.push_back(createNextTargetMarker(debug_data_.next_target));
  marker_array.markers.push_back(
    createTrajectoryCircleMarker(debug_data_.next_target, current_odometry_.pose.pose));

  return marker_array;
}

std::optional<PpOutput> PurePursuitLateralController::calcTargetCurvature(
  bool is_control_output, const geometry_msgs::msg::Pose & pose)
{
  // Ignore invalid trajectory
  if (trajectory_resampled_.points.size() < 3) {
    result_.received_short_trajectory = true;
    return std::nullopt;
  }

  // Calculate target point for velocity/acceleration

  const auto closest_idx_result =
    autoware::motion_utils::findNearestIndex(output_tp_array_, pose, 3.0, M_PI_4);
  if (!closest_idx_result) {
    result_.closest_waypoint_not_found = true;
    return std::nullopt;
  }

  const double target_vel =
    trajectory_resampled_.points.at(*closest_idx_result).longitudinal_velocity_mps;

  // calculate the lateral error

  const double lateral_error =
    autoware::motion_utils::calcLateralOffset(trajectory_resampled_.points, pose.position);

  // calculate the current curvature

  const double current_curvature = calcCurvature(*closest_idx_result);

  // Calculate lookahead distance

  const bool is_reverse = (target_vel < 0);
  const double min_lookahead_distance =
    is_reverse ? config_.reverse_min_lookahead_distance : config_.min_lookahead_distance;
  double lookahead_distance =
    is_control_output
      ? calcLookaheadDistance(
          lateral_error, current_curvature, current_odometry_.twist.twist.linear.x,
          min_lookahead_distance, is_control_output)
      : calcLookaheadDistance(
          lateral_error, current_curvature, target_vel, min_lookahead_distance, is_control_output);

  // Set PurePursuit data
  pure_pursuit_->setCurrentPose(pose);
  pure_pursuit_->setWaypoints(planning_utils::extractPoses(trajectory_resampled_));
  pure_pursuit_->setLookaheadDistance(lookahead_distance);

  // Run PurePursuit
  const auto pure_pursuit_result = pure_pursuit_->run();
  if (!pure_pursuit_result.first) {
    return std::nullopt;
  }

  const auto kappa = pure_pursuit_result.second;

  // Set debug data
  if (is_control_output) {
    debug_data_.next_target = pure_pursuit_->getLocationOfNextTarget();
  }
  PpOutput output{};
  output.curvature = kappa;
  if (!is_control_output) {
    output.velocity = current_odometry_.twist.twist.linear.x;
  } else {
    output.velocity = target_vel;
  }

  return output;
}
}  // namespace autoware::pure_pursuit

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

#include "autoware/trajectory_follower_node/create_config.hpp"

#include "autoware/trajectory_follower_base/common_params.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control::trajectory_follower_node
{
using mpc_lateral_controller::MpcLateralControllerConfig;
using mpc_lateral_controller::MPCParam;
using pid_longitudinal_controller::PidLongitudinalControllerConfig;
using pure_pursuit::PurePursuitLateralControllerConfig;

namespace
{
MPCParam declareMpcParam(rclcpp::Node & node)
{
  MPCParam mpc_param;
  mpc_param.prediction_horizon = node.declare_parameter<int>("mpc_prediction_horizon");
  mpc_param.prediction_dt = node.declare_parameter<double>("mpc_prediction_dt");

  const auto dp = [&](const auto & param) { return node.declare_parameter<double>(param); };

  auto & nw = mpc_param.nominal_weight;
  nw.lat_error = dp("mpc_weight_lat_error");
  nw.heading_error = dp("mpc_weight_heading_error");
  nw.heading_error_squared_vel = dp("mpc_weight_heading_error_squared_vel");
  nw.steering_input = dp("mpc_weight_steering_input");
  nw.steering_input_squared_vel = dp("mpc_weight_steering_input_squared_vel");
  nw.lat_jerk = dp("mpc_weight_lat_jerk");
  nw.steer_rate = dp("mpc_weight_steer_rate");
  nw.steer_acc = dp("mpc_weight_steer_acc");
  nw.terminal_lat_error = dp("mpc_weight_terminal_lat_error");
  nw.terminal_heading_error = dp("mpc_weight_terminal_heading_error");

  auto & lcw = mpc_param.low_curvature_weight;
  lcw.lat_error = dp("mpc_low_curvature_weight_lat_error");
  lcw.heading_error = dp("mpc_low_curvature_weight_heading_error");
  lcw.heading_error_squared_vel = dp("mpc_low_curvature_weight_heading_error_squared_vel");
  lcw.steering_input = dp("mpc_low_curvature_weight_steering_input");
  lcw.steering_input_squared_vel = dp("mpc_low_curvature_weight_steering_input_squared_vel");
  lcw.lat_jerk = dp("mpc_low_curvature_weight_lat_jerk");
  lcw.steer_rate = dp("mpc_low_curvature_weight_steer_rate");
  lcw.steer_acc = dp("mpc_low_curvature_weight_steer_acc");
  mpc_param.low_curvature_thresh_curvature = dp("mpc_low_curvature_thresh_curvature");

  mpc_param.zero_ff_steer_deg = dp("mpc_zero_ff_steer_deg");
  mpc_param.acceleration_limit = dp("mpc_acceleration_limit");
  mpc_param.velocity_time_constant = dp("mpc_velocity_time_constant");
  mpc_param.min_prediction_length = dp("mpc_min_prediction_length");

  return mpc_param;
}

MpcLateralControllerConfig createMpcConfig(
  rclcpp::Node & node, const trajectory_follower::CommonParams & common)
{
  const auto dp_int = [&](const std::string & s) { return node.declare_parameter<int>(s); };
  const auto dp_bool = [&](const std::string & s) { return node.declare_parameter<bool>(s); };
  const auto dp_double = [&](const std::string & s) { return node.declare_parameter<double>(s); };

  MpcLateralControllerConfig config;

  config.ctrl_period = common.ctrl_period;

  auto & p_filt = config.trajectory_filtering;
  p_filt.enable_path_smoothing = dp_bool("enable_path_smoothing");
  p_filt.path_filter_moving_ave_num = dp_int("path_filter_moving_ave_num");
  p_filt.curvature_smoothing_num_traj = dp_int("curvature_smoothing_num_traj");
  p_filt.curvature_smoothing_num_ref_steer = dp_int("curvature_smoothing_num_ref_steer");
  p_filt.traj_resample_dist = dp_double("traj_resample_dist");
  p_filt.extend_trajectory_for_end_yaw_control = dp_bool("extend_trajectory_for_end_yaw_control");

  config.use_steer_prediction = dp_bool("use_steer_prediction");
  const double steer_tau = dp_double("vehicle_model_steer_tau");

  /* stop state parameters */
  config.stop_state_entry_ego_speed = dp_double("stop_state_entry_ego_speed");
  config.stop_state_entry_target_speed = dp_double("stop_state_entry_target_speed");
  config.converged_steer_rad = dp_double("converged_steer_rad");
  config.keep_steer_control_until_converged = dp_bool("keep_steer_control_until_converged");
  config.new_traj_duration_time = dp_double("new_traj_duration_time");            // [s]
  config.new_traj_end_dist = dp_double("new_traj_end_dist");                      // [m]
  config.mpc_converged_threshold_rps = dp_double("mpc_converged_threshold_rps");  // [rad/s]

  /* vehicle */
  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
  config.wheelbase = vehicle_info.wheel_base_m;
  config.steer_lim = vehicle_info.max_steer_angle_rad;
  constexpr double deg2rad = static_cast<double>(M_PI) / 180.0;

  // steer rate limit depending on curvature
  const auto steer_rate_lim_dps_list_by_curvature =
    node.declare_parameter<std::vector<double>>("steer_rate_lim_dps_list_by_curvature");
  const auto curvature_list_for_steer_rate_lim =
    node.declare_parameter<std::vector<double>>("curvature_list_for_steer_rate_lim");
  for (size_t i = 0; i < steer_rate_lim_dps_list_by_curvature.size(); ++i) {
    config.steer_rate_lim_by_curvature.emplace_back(
      curvature_list_for_steer_rate_lim.at(i),
      steer_rate_lim_dps_list_by_curvature.at(i) * deg2rad);
  }

  // steer rate limit depending on velocity
  const auto steer_rate_lim_dps_list_by_velocity =
    node.declare_parameter<std::vector<double>>("steer_rate_lim_dps_list_by_velocity");
  const auto velocity_list_for_steer_rate_lim =
    node.declare_parameter<std::vector<double>>("velocity_list_for_steer_rate_lim");
  for (size_t i = 0; i < steer_rate_lim_dps_list_by_velocity.size(); ++i) {
    config.steer_rate_lim_by_velocity.emplace_back(
      velocity_list_for_steer_rate_lim.at(i), steer_rate_lim_dps_list_by_velocity.at(i) * deg2rad);
  }

  config.vehicle_model_type = node.declare_parameter<std::string>("vehicle_model_type");
  if (config.vehicle_model_type == "dynamics") {
    config.mass_fl = dp_double("vehicle.mass_fl");
    config.mass_fr = dp_double("vehicle.mass_fr");
    config.mass_rl = dp_double("vehicle.mass_rl");
    config.mass_rr = dp_double("vehicle.mass_rr");
    config.cf = dp_double("vehicle.cf");
    config.cr = dp_double("vehicle.cr");
  }

  config.qp_solver_type = node.declare_parameter<std::string>("qp_solver_type");

  config.input_delay = dp_double("input_delay");

  /* steering offset compensation */
  config.enable_auto_steering_offset_removal =
    dp_bool("steering_offset.enable_auto_steering_offset_removal");
  config.steer_offset_max_update_th = dp_double("steering_offset.max_update_th");
  if (config.enable_auto_steering_offset_removal) {
    config.steer_offset_filter_cutoff_hz =
      dp_double("steering_offset.steer_offset_filter_cutoff_hz");
  }

  config.steering_lpf_cutoff_hz = dp_double("steering_lpf_cutoff_hz");
  config.error_deriv_lpf_cutoff_hz = dp_double("error_deriv_lpf_cutoff_hz");

  // ego nearest index search
  config.ego_nearest_dist_threshold = common.ego_nearest_dist_threshold;
  config.ego_nearest_yaw_threshold = common.ego_nearest_yaw_threshold;

  config.use_delayed_initial_state = dp_bool("use_delayed_initial_state");

  config.use_temporal_trajectory = common.use_temporal_trajectory;

  config.publish_debug_trajectories = dp_bool("publish_debug_trajectories");

  config.mpc_param = declareMpcParam(node);
  config.mpc_param.steer_tau = steer_tau;

  return config;
}

PurePursuitLateralControllerConfig createPurePursuitConfig(rclcpp::Node & node)
{
  PurePursuitLateralControllerConfig config;

  // Vehicle Parameters
  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
  config.wheel_base = vehicle_info.wheel_base_m;
  config.max_steering_angle = vehicle_info.max_steer_angle_rad;

  // Algorithm Parameters
  config.ld_velocity_ratio = node.declare_parameter<double>("ld_velocity_ratio");
  config.ld_lateral_error_ratio = node.declare_parameter<double>("ld_lateral_error_ratio");
  config.ld_curvature_ratio = node.declare_parameter<double>("ld_curvature_ratio");
  config.long_ld_lateral_error_threshold =
    node.declare_parameter<double>("long_ld_lateral_error_threshold");
  config.min_lookahead_distance = node.declare_parameter<double>("min_lookahead_distance");
  config.max_lookahead_distance = node.declare_parameter<double>("max_lookahead_distance");
  config.reverse_min_lookahead_distance =
    node.declare_parameter<double>("reverse_min_lookahead_distance");
  config.converged_steer_rad = node.declare_parameter<double>("converged_steer_rad");
  config.prediction_ds = node.declare_parameter<double>("prediction_ds");
  config.prediction_distance_length = node.declare_parameter<double>("prediction_distance_length");
  config.resampling_ds = node.declare_parameter<double>("resampling_ds");
  config.curvature_calculation_distance =
    node.declare_parameter<double>("curvature_calculation_distance");
  config.enable_path_smoothing = node.declare_parameter<bool>("enable_path_smoothing");
  config.path_filter_moving_ave_num = node.declare_parameter<int64_t>("path_filter_moving_ave_num");

  return config;
}

PidLongitudinalControllerConfig createPidConfig(
  rclcpp::Node & node, const trajectory_follower::CommonParams & common)
{
  PidLongitudinalControllerConfig config{};

  // parameters timer
  config.longitudinal_ctrl_period = common.ctrl_period;

  config.use_temporal_trajectory = common.use_temporal_trajectory;

  config.wheel_base =
    autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo().wheel_base_m;
  config.front_overhang =
    autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo().front_overhang_m;

  // parameters for delay compensation
  config.delay_compensation_time =
    node.declare_parameter<double>("delay_compensation_time");  // [s]

  // parameters to enable functions
  config.enable_smooth_stop = node.declare_parameter<bool>("enable_smooth_stop");
  config.enable_overshoot_emergency = node.declare_parameter<bool>("enable_overshoot_emergency");
  config.enable_large_tracking_error_emergency =
    node.declare_parameter<bool>("enable_large_tracking_error_emergency");
  config.enable_slope_compensation = node.declare_parameter<bool>("enable_slope_compensation");
  config.enable_keep_stopped_until_steer_convergence =
    node.declare_parameter<bool>("enable_keep_stopped_until_steer_convergence");

  // parameters for state transition
  {
    auto & p = config.state_transition_params;
    // drive
    p.drive_state_stop_dist = node.declare_parameter<double>("drive_state_stop_dist");  // [m]
    p.drive_state_offset_stop_dist =
      node.declare_parameter<double>("drive_state_offset_stop_dist");  // [m]
    // stopping
    p.stopping_state_stop_dist = node.declare_parameter<double>("stopping_state_stop_dist");  // [m]
    p.stopped_state_entry_duration_time =
      node.declare_parameter<double>("stopped_state_entry_duration_time");  // [s]
    // stop
    p.stopped_state_entry_vel = node.declare_parameter<double>("stopped_state_entry_vel");  // [m/s]
    p.stopped_state_entry_acc =
      node.declare_parameter<double>("stopped_state_entry_acc");  // [m/s²]

    // emergency
    p.emergency_state_overshoot_stop_dist =
      node.declare_parameter<double>("emergency_state_overshoot_stop_dist");  // [m]
  }

  // parameters for drive state
  {
    // initialize PID gain
    auto & gains = config.pid_gains;
    gains.kp = node.declare_parameter<double>("kp");
    gains.ki = node.declare_parameter<double>("ki");
    gains.kd = node.declare_parameter<double>("kd");

    // initialize PID limits
    auto & limits = config.pid_limits;
    limits.max_out = node.declare_parameter<double>("max_out");            // [m/s^2]
    limits.min_out = node.declare_parameter<double>("min_out");            // [m/s^2]
    limits.max_p_effort = node.declare_parameter<double>("max_p_effort");  // [m/s^2]
    limits.min_p_effort = node.declare_parameter<double>("min_p_effort");  // [m/s^2]
    limits.max_i_effort = node.declare_parameter<double>("max_i_effort");  // [m/s^2]
    limits.min_i_effort = node.declare_parameter<double>("min_i_effort");  // [m/s^2]
    limits.max_d_effort = node.declare_parameter<double>("max_d_effort");  // [m/s^2]
    limits.min_d_effort = node.declare_parameter<double>("min_d_effort");  // [m/s^2]

    // set lowpass filter for vel error and pitch
    config.lpf_vel_error_gain = node.declare_parameter<double>("lpf_vel_error_gain");

    config.enable_integration_at_low_speed =
      node.declare_parameter<bool>("enable_integration_at_low_speed");
    config.current_vel_threshold_pid_integrate =
      node.declare_parameter<double>("current_vel_threshold_pid_integration");  // [m/s]

    config.time_threshold_before_pid_integrate =
      node.declare_parameter<double>("time_threshold_before_pid_integration");  // [s]
    config.ff_scale_min = node.declare_parameter<double>("ff_scale_min");
    config.ff_scale_max = node.declare_parameter<double>("ff_scale_max");

    config.enable_brake_keeping_before_stop =
      node.declare_parameter<bool>("enable_brake_keeping_before_stop");              // [-]
    config.brake_keeping_acc = node.declare_parameter<double>("brake_keeping_acc");  // [m/s^2]
  }

  // parameters for smooth stop state
  {
    auto & p = config.smooth_stop_params;
    p.max_strong_acc = node.declare_parameter<double>("smooth_stop_max_strong_acc");    // [m/s^2]
    p.min_strong_acc = node.declare_parameter<double>("smooth_stop_min_strong_acc");    // [m/s^2]
    p.weak_acc = node.declare_parameter<double>("smooth_stop_weak_acc");                // [m/s^2]
    p.weak_stop_acc = node.declare_parameter<double>("smooth_stop_weak_stop_acc");      // [m/s^2]
    p.strong_stop_acc = node.declare_parameter<double>("smooth_stop_strong_stop_acc");  // [m/s^2]

    p.min_fast_vel = node.declare_parameter<double>("smooth_stop_max_fast_vel");        // [m/s]
    p.min_running_vel = node.declare_parameter<double>("smooth_stop_min_running_vel");  // [m/s]
    p.min_running_acc = node.declare_parameter<double>("smooth_stop_min_running_acc");  // [m/s^2]
    p.weak_stop_time = node.declare_parameter<double>("smooth_stop_weak_stop_time");    // [s]

    p.weak_stop_dist = node.declare_parameter<double>("smooth_stop_weak_stop_dist");      // [m]
    p.strong_stop_dist = node.declare_parameter<double>("smooth_stop_strong_stop_dist");  // [m]
  }

  // parameters for stop state
  {
    auto & p = config.stopped_state_params;
    p.vel = node.declare_parameter<double>("stopped_vel");  // [m/s]
    p.acc = node.declare_parameter<double>("stopped_acc");  // [m/s^2]
  }

  // parameters for emergency state
  {
    auto & p = config.emergency_state_params;
    p.vel = node.declare_parameter<double>("emergency_vel");    // [m/s]
    p.acc = node.declare_parameter<double>("emergency_acc");    // [m/s^2]
    p.jerk = node.declare_parameter<double>("emergency_jerk");  // [m/s^3]
  }

  // parameters for acc feedback
  {
    config.lpf_acc_error_gain = node.declare_parameter<double>("lpf_acc_error_gain");
    config.acc_feedback_gain = node.declare_parameter<double>("acc_feedback_gain");
  }

  // parameters for acceleration limit
  config.max_acc = node.declare_parameter<double>("max_acc");  // [m/s^2]
  config.min_acc = node.declare_parameter<double>("min_acc");  // [m/s^2]

  // parameters for jerk limit
  config.max_jerk = node.declare_parameter<double>("max_jerk");                  // [m/s^3]
  config.min_jerk = node.declare_parameter<double>("min_jerk");                  // [m/s^3]
  config.max_acc_cmd_diff = node.declare_parameter<double>("max_acc_cmd_diff");  // [m/s^3]

  // parameters for slope compensation
  config.adaptive_trajectory_velocity_th =
    node.declare_parameter<double>("adaptive_trajectory_velocity_th");  // [m/s^2]
  config.lpf_pitch_gain = node.declare_parameter<double>("lpf_pitch_gain");
  config.max_pitch_rad = node.declare_parameter<double>("max_pitch_rad");  // [rad]
  config.min_pitch_rad = node.declare_parameter<double>("min_pitch_rad");  // [rad]

  // check slope source is proper
  const std::string slope_source = node.declare_parameter<std::string>(
    "slope_source");  // raw_pitch, trajectory_pitch or trajectory_adaptive
  if (slope_source == "raw_pitch") {
    config.slope_source = PidLongitudinalControllerConfig::SlopeSource::RAW_PITCH;
  } else if (slope_source == "trajectory_pitch") {
    config.slope_source = PidLongitudinalControllerConfig::SlopeSource::TRAJECTORY_PITCH;
  } else if (slope_source == "trajectory_adaptive") {
    config.slope_source = PidLongitudinalControllerConfig::SlopeSource::TRAJECTORY_ADAPTIVE;
  } else if (slope_source == "trajectory_goal_adaptive") {
    config.slope_source = PidLongitudinalControllerConfig::SlopeSource::TRAJECTORY_GOAL_ADAPTIVE;
  } else {
    RCLCPP_ERROR(node.get_logger(), "Slope source is not valid. Using raw_pitch option as default");
    config.slope_source = PidLongitudinalControllerConfig::SlopeSource::RAW_PITCH;
  }

  // ego nearest index search
  config.ego_nearest_dist_threshold = common.ego_nearest_dist_threshold;
  config.ego_nearest_yaw_threshold = common.ego_nearest_yaw_threshold;

  return config;
}
}  // namespace

TrajectoryFollowerConfig createConfig(rclcpp::Node & node)
{
  TrajectoryFollowerConfig config;

  config.ctrl_period = node.declare_parameter<double>("ctrl_period");
  config.timeout_thr_sec = node.declare_parameter<double>("timeout_thr_sec");
  config.cyclic_message_timeout_thr_sec =
    node.declare_parameter<double>("cyclic_message_timeout_thr_sec");
  // NOTE: It is possible that using control_horizon could be expected to enhance performance,
  // but it is not a formal interface topic, only an experimental one.
  // So it is disabled by default.
  config.enable_control_cmd_horizon =
    node.declare_parameter<bool>("enable_control_cmd_horizon_pub", false);

  trajectory_follower::CommonParams common;
  common.ctrl_period = config.ctrl_period;

  // spatial: distance-based tracking, temporal: time-step-based tracking
  const auto trajectory_reference_mode =
    node.declare_parameter<std::string>("trajectory_reference_mode", "spatial");
  if (trajectory_reference_mode == "temporal") {
    common.use_temporal_trajectory = true;
  } else if (trajectory_reference_mode == "spatial") {
    common.use_temporal_trajectory = false;
  } else {
    throw std::invalid_argument(
      "Invalid trajectory_reference_mode. Expected \"spatial\" or \"temporal\".");
  }

  common.ego_nearest_dist_threshold =
    node.declare_parameter<double>("ego_nearest_dist_threshold");  // [m]
  common.ego_nearest_yaw_threshold =
    node.declare_parameter<double>("ego_nearest_yaw_threshold");  // [rad]

  config.lateral_controller_kind =
    parseLateralControllerKind(node.declare_parameter<std::string>("lateral_controller_mode"));
  switch (config.lateral_controller_kind) {
    case LateralControllerKind::Mpc:
      config.mpc = createMpcConfig(node, common);
      break;
    case LateralControllerKind::PurePursuit:
      config.pure_pursuit = createPurePursuitConfig(node);
      break;
  }

  config.longitudinal_controller_kind = parseLongitudinalControllerKind(
    node.declare_parameter<std::string>("longitudinal_controller_mode"));
  switch (config.longitudinal_controller_kind) {
    case LongitudinalControllerKind::Pid:
      config.pid = createPidConfig(node, common);
      break;
  }

  return config;
}

}  // namespace autoware::motion::control::trajectory_follower_node

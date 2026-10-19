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

#include "mpc_config_from_node.hpp"

#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"

#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control::mpc_lateral_controller
{
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
}  // namespace

MpcLateralControllerConfig createMpcConfigFromNode(
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
}  // namespace autoware::motion::control::mpc_lateral_controller

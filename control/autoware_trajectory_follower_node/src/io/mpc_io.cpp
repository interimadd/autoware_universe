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

#include "io/mpc_io.hpp"

#include "autoware/mpc_lateral_controller/mpc_utils.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control::trajectory_follower_node
{

MpcIo::MpcIo(
  rclcpp::Node & node, MpcLateralController & controller,
  const std::shared_ptr<diagnostic_updater::Updater> & diag_updater)
: controller_(controller), diag_updater_(diag_updater)
{
  m_pub_predicted_traj = node.create_publisher<Trajectory>("~/output/predicted_trajectory", 1);
  m_pub_predicted_traj_frenet =
    node.create_publisher<Trajectory>("~/debug/predicted_trajectory_in_frenet_coordinate", 1);
  m_pub_resampled_reference_traj =
    node.create_publisher<Trajectory>("~/debug/resampled_reference_trajectory", 1);
  m_pub_nearest_pose = node.create_publisher<PoseStamped>("~/debug/nearest_pose", 1);
  m_pub_nearest_segment_traj = node.create_publisher<Trajectory>("~/debug/nearest_segment", 1);
  m_pub_nearest_info = node.create_publisher<Float32MultiArrayStamped>("~/debug/nearest_info", 1);
  m_pub_debug_values =
    node.create_publisher<Float32MultiArrayStamped>("~/output/lateral_diagnostic", 1);
  m_pub_steer_offset = node.create_publisher<Float32Stamped>("~/output/estimated_steer_offset", 1);

  /* get parameter updates */
  using std::placeholders::_1;
  m_set_param_res = node.add_on_set_parameters_callback(std::bind(&MpcIo::paramCallback, this, _1));

  diag_updater_->add("MPC_solve_checker", [&](auto & stat) { setStatus(stat); });
}

void MpcIo::publish()
{
  const auto & result = controller_.lastResult();

  auto predicted_trajectory = result.mpc.predicted_trajectory;
  m_pub_predicted_traj->publish(predicted_trajectory);

  if (result.mpc.debug_msgs) {
    const auto & debug_msgs = *result.mpc.debug_msgs;
    m_pub_predicted_traj_frenet->publish(debug_msgs.predicted_trajectory_frenet);
    m_pub_resampled_reference_traj->publish(debug_msgs.resampled_reference_trajectory);
    m_pub_nearest_pose->publish(debug_msgs.nearest_pose);
    m_pub_nearest_segment_traj->publish(debug_msgs.nearest_segment_trajectory);
    m_pub_nearest_info->publish(debug_msgs.nearest_info);
  }

  m_pub_debug_values->publish(result.mpc.diagnostic);

  m_pub_steer_offset->publish(result.steering_offset);
}

void MpcIo::setStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const auto & mpc_result = controller_.lastResult().mpc;
  if (mpc_result.result) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "MPC succeeded.");
  } else {
    const std::string error_msg = "MPC failed due to " + mpc_result.reason;
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, error_msg);
  }
}

rcl_interfaces::msg::SetParametersResult MpcIo::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  // strong exception safety wrt MPCParam
  MPCParam param = controller_.mpcParam();

  using mpc_lateral_controller::MPCUtils::update_param;
  try {
    auto & nw = param.nominal_weight;
    auto & lcw = param.low_curvature_weight;

    update_param(parameters, "mpc_prediction_horizon", param.prediction_horizon);
    update_param(parameters, "mpc_prediction_dt", param.prediction_dt);

    const std::string ns_nw = "mpc_weight_";
    update_param(parameters, ns_nw + "lat_error", nw.lat_error);
    update_param(parameters, ns_nw + "heading_error", nw.heading_error);
    update_param(parameters, ns_nw + "heading_error_squared_vel", nw.heading_error_squared_vel);
    update_param(parameters, ns_nw + "steering_input", nw.steering_input);
    update_param(parameters, ns_nw + "steering_input_squared_vel", nw.steering_input_squared_vel);
    update_param(parameters, ns_nw + "lat_jerk", nw.lat_jerk);
    update_param(parameters, ns_nw + "steer_rate", nw.steer_rate);
    update_param(parameters, ns_nw + "steer_acc", nw.steer_acc);
    update_param(parameters, ns_nw + "terminal_lat_error", nw.terminal_lat_error);
    update_param(parameters, ns_nw + "terminal_heading_error", nw.terminal_heading_error);

    const std::string ns_lcw = "mpc_low_curvature_weight_";
    update_param(parameters, ns_lcw + "lat_error", lcw.lat_error);
    update_param(parameters, ns_lcw + "heading_error", lcw.heading_error);
    update_param(parameters, ns_lcw + "heading_error_squared_vel", lcw.heading_error_squared_vel);
    update_param(parameters, ns_lcw + "steering_input", lcw.steering_input);
    update_param(parameters, ns_lcw + "steering_input_squared_vel", lcw.steering_input_squared_vel);
    update_param(parameters, ns_lcw + "lat_jerk", lcw.lat_jerk);
    update_param(parameters, ns_lcw + "steer_rate", lcw.steer_rate);
    update_param(parameters, ns_lcw + "steer_acc", lcw.steer_acc);

    update_param(
      parameters, "mpc_low_curvature_thresh_curvature", param.low_curvature_thresh_curvature);

    update_param(parameters, "mpc_zero_ff_steer_deg", param.zero_ff_steer_deg);
    update_param(parameters, "mpc_acceleration_limit", param.acceleration_limit);
    update_param(parameters, "mpc_velocity_time_constant", param.velocity_time_constant);
    update_param(parameters, "mpc_min_prediction_length", param.min_prediction_length);

    update_param(parameters, "input_delay", param.input_delay);

    // transaction succeeds, now assign values
    controller_.setMpcParam(param);
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}

}  // namespace autoware::motion::control::trajectory_follower_node

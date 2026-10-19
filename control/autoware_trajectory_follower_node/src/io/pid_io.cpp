// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#include "io/pid_io.hpp"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::motion::control::trajectory_follower_node
{
namespace
{
}  // namespace

PidIo::PidIo(
  rclcpp::Node & node, PidLongitudinalController & controller,
  const PidLongitudinalControllerConfig & config,
  const std::shared_ptr<diagnostic_updater::Updater> & diag_updater)
: controller_(controller),
  config_(config),
  clock_(node.get_clock()),
  logger_(node.get_logger().get_child("longitudinal_controller"))
{
  using std::placeholders::_1;

  diag_updater_ = diag_updater;

  // subscriber, publisher
  m_pub_slope = node.create_publisher<autoware_internal_debug_msgs::msg::Float32MultiArrayStamped>(
    "~/output/slope_angle", rclcpp::QoS{1});
  m_pub_debug = node.create_publisher<autoware_internal_debug_msgs::msg::Float32MultiArrayStamped>(
    "~/output/longitudinal_diagnostic", rclcpp::QoS{1});
  m_pub_virtual_wall_marker = node.create_publisher<MarkerArray>("~/virtual_wall", 1);

  // set parameter callback
  m_set_param_res = node.add_on_set_parameters_callback(std::bind(&PidIo::paramCallback, this, _1));

  // diagnostic
  setupDiagnosticUpdater();
}

rcl_interfaces::msg::SetParametersResult PidIo::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto update_param = [&](const std::string & name, double & v) {
    auto it = std::find_if(
      parameters.cbegin(), parameters.cend(),
      [&name](const rclcpp::Parameter & parameter) { return parameter.get_name() == name; });
    if (it != parameters.cend()) {
      v = it->as_double();
      return true;
    }
    return false;
  };

  // delay compensation
  update_param("delay_compensation_time", config_.delay_compensation_time);

  // state transition
  {
    auto & p = config_.state_transition_params;
    update_param("drive_state_stop_dist", p.drive_state_stop_dist);
    update_param("stopping_state_stop_dist", p.stopping_state_stop_dist);
    update_param("stopped_state_entry_duration_time", p.stopped_state_entry_duration_time);
    update_param("stopped_state_entry_vel", p.stopped_state_entry_vel);
    update_param("stopped_state_entry_acc", p.stopped_state_entry_acc);
    update_param("emergency_state_overshoot_stop_dist", p.emergency_state_overshoot_stop_dist);
  }

  // drive state
  {
    auto & gains = config_.pid_gains;
    update_param("kp", gains.kp);
    update_param("ki", gains.ki);
    update_param("kd", gains.kd);

    update_param("lpf_vel_error_gain", config_.lpf_vel_error_gain);

    auto & limits = config_.pid_limits;
    update_param("max_out", limits.max_out);
    update_param("min_out", limits.min_out);
    update_param("max_p_effort", limits.max_p_effort);
    update_param("min_p_effort", limits.min_p_effort);
    update_param("max_i_effort", limits.max_i_effort);
    update_param("min_i_effort", limits.min_i_effort);
    update_param("max_d_effort", limits.max_d_effort);
    update_param("min_d_effort", limits.min_d_effort);

    update_param(
      "current_vel_threshold_pid_integration", config_.current_vel_threshold_pid_integrate);
    update_param(
      "time_threshold_before_pid_integration", config_.time_threshold_before_pid_integrate);
    update_param("ff_scale_min", config_.ff_scale_min);
    update_param("ff_scale_max", config_.ff_scale_max);
  }

  // stopping state
  {
    auto & p = config_.smooth_stop_params;
    update_param("smooth_stop_max_strong_acc", p.max_strong_acc);
    update_param("smooth_stop_min_strong_acc", p.min_strong_acc);
    update_param("smooth_stop_weak_acc", p.weak_acc);
    update_param("smooth_stop_weak_stop_acc", p.weak_stop_acc);
    update_param("smooth_stop_strong_stop_acc", p.strong_stop_acc);
    update_param("smooth_stop_max_fast_vel", p.min_fast_vel);
    update_param("smooth_stop_min_running_vel", p.min_running_vel);
    update_param("smooth_stop_min_running_acc", p.min_running_acc);
    update_param("smooth_stop_weak_stop_time", p.weak_stop_time);
    update_param("smooth_stop_weak_stop_dist", p.weak_stop_dist);
    update_param("smooth_stop_strong_stop_dist", p.strong_stop_dist);
  }

  // stop state
  {
    auto & p = config_.stopped_state_params;
    update_param("stopped_vel", p.vel);
    update_param("stopped_acc", p.acc);
  }

  // emergency state
  {
    auto & p = config_.emergency_state_params;
    update_param("emergency_vel", p.vel);
    update_param("emergency_acc", p.acc);
    update_param("emergency_jerk", p.jerk);
  }

  // acceleration feedback
  update_param("acc_feedback_gain", config_.acc_feedback_gain);
  update_param("lpf_acc_error_gain", config_.lpf_acc_error_gain);

  // acceleration limit
  update_param("min_acc", config_.min_acc);

  // jerk limit
  update_param("max_jerk", config_.max_jerk);
  update_param("min_jerk", config_.min_jerk);
  update_param("max_acc_cmd_diff", config_.max_acc_cmd_diff);

  // slope compensation
  update_param("max_pitch_rad", config_.max_pitch_rad);
  update_param("min_pitch_rad", config_.min_pitch_rad);
  update_param("adaptive_trajectory_velocity_th", config_.adaptive_trajectory_velocity_th);

  controller_.setConfig(config_);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void PidIo::publish()
{
  const auto & result = controller_.lastResult();
  control_state_ = result.control_state;
  emitLogs(result);
  publishMessage(result);
}

void PidIo::emitLogs(const PidLongitudinalControllerResult & result)
{
  if (result.received_invalid_trajectory) {
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 3000, "received invalid trajectory. ignore.");
  }
  if (result.emergency_stop_reason) {
    RCLCPP_ERROR(logger_, "Emergency Stop since %s", result.emergency_stop_reason->c_str());
  }
}

void PidIo::publishMessage(const PidLongitudinalControllerResult & result)
{
  m_pub_debug->publish(result.debug_message);
  m_pub_slope->publish(result.slope_message);

  if (result.virtual_wall_marker) {
    m_pub_virtual_wall_marker->publish(*result.virtual_wall_marker);
  }
}

void PidIo::setupDiagnosticUpdater()
{
  diag_updater_->add("control_state", this, &PidIo::checkControlState);
}

void PidIo::checkControlState(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  auto level = DiagnosticStatus::OK;
  std::string msg = "OK";

  if (control_state_ == ControlState::EMERGENCY) {
    level = DiagnosticStatus::ERROR;
    msg = "emergency occurred due to ";
  }

  stat.add<int32_t>("control_state", static_cast<int32_t>(control_state_));
  stat.summary(level, msg);
}

}  // namespace autoware::motion::control::trajectory_follower_node

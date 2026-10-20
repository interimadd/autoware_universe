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

#ifndef AUTOWARE__PID_LONGITUDINAL_CONTROLLER__PID_LONGITUDINAL_CONTROLLER_NODE_HPP_
#define AUTOWARE__PID_LONGITUDINAL_CONTROLLER__PID_LONGITUDINAL_CONTROLLER_NODE_HPP_

#include "autoware/pid_longitudinal_controller/pid_longitudinal_controller.hpp"
#include "rclcpp/rclcpp.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>

#include "autoware_internal_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoware::motion::control::pid_longitudinal_controller
{
/// \brief declare every longitudinal control parameter on the node and read it into a config
/// \param [in,out] node node that owns the parameters of the longitudinal controller
PidLongitudinalControllerConfig create_config(rclcpp::Node & node);

/// \brief apply the parameters changed at runtime to an existing config
/// \param [in,out] config config to update in place
/// \param [in] parameters parameters given to the set-parameters callback of the node
void update_config(
  PidLongitudinalControllerConfig & config, const std::vector<rclcpp::Parameter> & parameters);

/// \class PidLongitudinalControllerReporter
/// \brief publishes the debug topics of the longitudinal controller and emits its critical logs.
/// It is a one-way sink: it takes the result of a control cycle and never feeds the control
/// logic, so the node that owns the control loop keeps calling PidLongitudinalController itself.
class PidLongitudinalControllerReporter
{
public:
  /// \param [in,out] node node that owns the debug topics of the longitudinal controller
  explicit PidLongitudinalControllerReporter(rclcpp::Node & node);

  /// \brief publish the debug data and the virtual wall marker of this cycle, if any, and emit
  ///        the critical logs the core logic raised during it
  /// \param [in] result output of the core control logic for this cycle
  void report(const PidLongitudinalControllerResult & result);

private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;

  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr
    pub_slope_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr
    pub_diagnostic_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_virtual_wall_marker_;
};
/// The node side of the longitudinal controller. It declares the parameters, builds the
/// controller from them, publishes what a cycle produced and reports what it met.
class PidLongitudinalControllerNode : public trajectory_follower::LongitudinalController
{
public:
  /// \param node Reference to the node used only for the parameter and topic initialization.
  /// \param diag_updater The updater of the node, which outlives this object and stays
  ///        its owner. This class only adds a task to it.
  PidLongitudinalControllerNode(rclcpp::Node & node, diagnostic_updater::Updater & diag_updater);

private:
  tl::expected<trajectory_follower::LongitudinalOutput, std::string> run(
    const trajectory_follower::InputData & input_data, const rclcpp::Time & stamp,
    const bool is_steer_converged) override;

  void check_control_state(diagnostic_updater::DiagnosticStatusWrapper & stat);

  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  PidLongitudinalControllerConfig config_;
  PidLongitudinalController controller_;
  PidLongitudinalControllerReporter reporter_;

  /// What the last cycle reported, which the diagnostic of the node reads.
  ControlState control_state_{ControlState::STOPPED};

  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr set_param_res_;
};
}  // namespace autoware::motion::control::pid_longitudinal_controller

#endif  // AUTOWARE__PID_LONGITUDINAL_CONTROLLER__PID_LONGITUDINAL_CONTROLLER_NODE_HPP_

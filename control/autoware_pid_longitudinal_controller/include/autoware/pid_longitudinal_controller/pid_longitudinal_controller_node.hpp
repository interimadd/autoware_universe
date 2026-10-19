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

#include <vector>

namespace autoware::motion::control::pid_longitudinal_controller
{
/// \brief declare every longitudinal control parameter on the node and read it into a config
/// \param [in,out] node node that owns the parameters of the longitudinal controller
PidLongitudinalControllerConfig create_pid_longitudinal_controller_config(rclcpp::Node & node);

/// \brief apply the parameters changed at runtime to an existing config
/// \param [in,out] config config to update in place
/// \param [in] parameters parameters given to the set-parameters callback of the node
void update_pid_longitudinal_controller_config(
  PidLongitudinalControllerConfig & config, const std::vector<rclcpp::Parameter> & parameters);
}  // namespace autoware::motion::control::pid_longitudinal_controller

#endif  // AUTOWARE__PID_LONGITUDINAL_CONTROLLER__PID_LONGITUDINAL_CONTROLLER_NODE_HPP_

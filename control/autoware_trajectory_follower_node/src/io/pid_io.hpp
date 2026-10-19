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

#ifndef IO__PID_IO_HPP_
#define IO__PID_IO_HPP_

#include "autoware/pid_longitudinal_controller/pid_longitudinal_controller.hpp"
#include "autoware/trajectory_follower_node/controller_io.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_internal_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoware::motion::control::trajectory_follower_node
{
using pid_longitudinal_controller::ControlState;
using pid_longitudinal_controller::PidLongitudinalController;
using pid_longitudinal_controller::PidLongitudinalControllerConfig;
using pid_longitudinal_controller::PidLongitudinalControllerResult;
using visualization_msgs::msg::MarkerArray;

/// The topics, the log lines, the diagnostic and the runtime parameters of the PID
/// longitudinal controller. The controller itself knows none of them.
class PidIo : public ControllerIo
{
public:
  /// \param node The node that owns the topics and the parameters.
  /// \param controller The controller whose output is written out. It has to outlive this.
  /// \param config The config the controller was built with, which a parameter update
  ///        changes field by field before it is handed back to the controller.
  PidIo(
    rclcpp::Node & node, PidLongitudinalController & controller,
    const PidLongitudinalControllerConfig & config,
    const std::shared_ptr<diagnostic_updater::Updater> & diag_updater);

  void publish() override;

private:
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  void setupDiagnosticUpdater();
  void checkControlState(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /// @brief emit the critical logs raised by the core logic during the last cycle
  void emitLogs(const PidLongitudinalControllerResult & result);

  /// @brief publish debug data and the virtual wall marker created during the last cycle
  void publishMessage(const PidLongitudinalControllerResult & result);

  PidLongitudinalController & controller_;
  PidLongitudinalControllerConfig config_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;

  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr
    m_pub_slope;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr
    m_pub_debug;
  rclcpp::Publisher<MarkerArray>::SharedPtr m_pub_virtual_wall_marker;

  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr m_set_param_res;

  // for diagnostics
  ControlState control_state_{ControlState::STOPPED};

  std::shared_ptr<diagnostic_updater::Updater> diag_updater_{};
};

}  // namespace autoware::motion::control::trajectory_follower_node

#endif  // IO__PID_IO_HPP_

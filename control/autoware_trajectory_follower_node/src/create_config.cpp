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

#include "autoware/mpc_lateral_controller/mpc_lateral_controller_node.hpp"
#include "autoware/pid_longitudinal_controller/pid_longitudinal_controller_node.hpp"
#include "autoware/pure_pursuit/autoware_pure_pursuit_lateral_controller_node.hpp"
#include "autoware/trajectory_follower_base/common_params.hpp"

#include <stdexcept>
#include <string>

namespace autoware::motion::control::trajectory_follower_node
{
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
      config.mpc = mpc_lateral_controller::MpcLateralControllerNode::createConfig(node, common);
      break;
    case LateralControllerKind::PurePursuit:
      config.pure_pursuit =
        autoware::pure_pursuit::PurePursuitLateralControllerNode::createConfig(node);
      break;
  }

  config.longitudinal_controller_kind = parseLongitudinalControllerKind(
    node.declare_parameter<std::string>("longitudinal_controller_mode"));
  switch (config.longitudinal_controller_kind) {
    case LongitudinalControllerKind::Pid:
      config.pid =
        pid_longitudinal_controller::PidLongitudinalControllerNode::createConfig(node, common);
      break;
  }

  return config;
}

}  // namespace autoware::motion::control::trajectory_follower_node

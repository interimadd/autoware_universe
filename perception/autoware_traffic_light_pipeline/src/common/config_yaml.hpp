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

// Reading this package's own `config/*.param.yaml` files outside of ROS's parameter machinery.
//
// The Nodes get their parameters from config/ via the launch files, but the offline evaluation
// executables and the ROS-free core test construct their configs directly, without a launch file
// or an rclcpp::Node. They used to carry their own hand-copied duplicates of every threshold, so
// a tuning change in config/ silently left evaluation and tests measuring the old value. These
// helpers let them read the very same files instead.

#ifndef COMMON__CONFIG_YAML_HPP_
#define COMMON__CONFIG_YAML_HPP_

#include <yaml-cpp/yaml.h>

#include <string>

namespace autoware::traffic_light
{

// Returns `node[key]`, throwing if it is missing. `context` names the file/section the key was
// expected in, so the message points at what the caller must fix.
YAML::Node require(const YAML::Node & node, const std::string & key, const std::string & context);

// `<this package's share directory>/config/<filename>`.
std::string package_config_path(const std::string & filename);

// Loads `<this package's share directory>/config/<filename>` -- a ROS parameter file -- and
// returns the parameter tree itself, i.e. the node under `/**:` -> `ros__parameters:`, so callers
// index it by plain parameter names (`node["classifier"]["over_exposure_threshold"]`) exactly as
// the Node's declare_parameter() calls do.
YAML::Node load_package_param_yaml(const std::string & filename);

}  // namespace autoware::traffic_light

#endif  // COMMON__CONFIG_YAML_HPP_

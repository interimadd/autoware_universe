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

#include "common/config_yaml.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <stdexcept>
#include <string>

namespace autoware::traffic_light
{

YAML::Node require(const YAML::Node & node, const std::string & key, const std::string & context)
{
  const auto child = node[key];
  if (!child) {
    throw std::runtime_error(context + ": missing required key '" + key + "'");
  }
  return child;
}

std::string package_config_path(const std::string & filename)
{
  return ament_index_cpp::get_package_share_directory("autoware_traffic_light_pipeline") +
         "/config/" + filename;
}

YAML::Node load_package_param_yaml(const std::string & filename)
{
  const auto path = package_config_path(filename);
  const auto root = YAML::LoadFile(path);
  // Every config file in this package is wildcard-scoped (`/**:`), matching however the launch
  // files name and namespace the Node.
  return require(require(root, "/**", path), "ros__parameters", path);
}

}  // namespace autoware::traffic_light

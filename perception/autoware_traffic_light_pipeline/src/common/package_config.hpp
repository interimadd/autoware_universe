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

// Locating this package's installed config/*.param.yaml files, for the callers that must read
// them without a launch file: the offline evaluation executables and the tests. They load the
// file with autoware::component_test_framework::ParameterLoader (the same rcl yaml parser
// production uses) and hand the result to build_recognition_config() / build_fusion_config(), so
// they build the very config a Node would from the very file the launch files feed it.

#ifndef COMMON__PACKAGE_CONFIG_HPP_
#define COMMON__PACKAGE_CONFIG_HPP_

#include <string>

namespace autoware::traffic_light
{

// `<this package's share directory>/config/<filename>`.
std::string package_config_path(const std::string & filename);

// The two config files the launch files pass to this package's Nodes.
inline constexpr char kRecognitionParamFile[] = "traffic_light_recognition.param.yaml";
inline constexpr char kFusionParamFile[] = "traffic_light_fusion.param.yaml";

}  // namespace autoware::traffic_light

#endif  // COMMON__PACKAGE_CONFIG_HPP_

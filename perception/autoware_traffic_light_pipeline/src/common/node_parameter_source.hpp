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

#ifndef COMMON__NODE_PARAMETER_SOURCE_HPP_
#define COMMON__NODE_PARAMETER_SOURCE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace autoware::traffic_light
{

// Adapts an rclcpp::Node to the named-parameter source build_recognition_config() and
// build_fusion_config() read from, so production and the ROS-free callers share one mapping from
// parameter name to config field instead of keeping two copies of it in step by hand.
//
// This is the production side: get() *declares* the parameter, which is what makes it visible to
// `ros2 param`, to a --params-file / -p override, and to this package's json schemas. The
// ROS-free side is autoware::component_test_framework::ParameterLoader, which reads the same
// param.yaml with the same rcl parser and exposes the same get()/get_or() pair. The two paths
// must agree; that is what the cross-check tests in the underlying component packages (e.g.
// autoware_traffic_light_multi_camera_fusion's test_multi_camera_fusion_params_cross_check.cpp)
// exist to prove.
class NodeParameterSource
{
public:
  explicit NodeParameterSource(rclcpp::Node * node) : node_(node) {}

  // Declares `name` with no default: a launch file (or the config file it passes) must supply it,
  // otherwise the Node fails to start rather than running on a silent fallback.
  template <typename T>
  T get(const std::string & name)
  {
    return node_->declare_parameter<T>(name);
  }

  // Declares `name` with a default, for the few parameters production may legitimately omit.
  template <typename T>
  T get_or(const std::string & name, const T & default_value)
  {
    return node_->declare_parameter<T>(name, default_value);
  }

private:
  rclcpp::Node * node_;
};

}  // namespace autoware::traffic_light

#endif  // COMMON__NODE_PARAMETER_SOURCE_HPP_

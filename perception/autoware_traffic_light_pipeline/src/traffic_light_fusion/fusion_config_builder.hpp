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

#ifndef TRAFFIC_LIGHT_FUSION__FUSION_CONFIG_BUILDER_HPP_
#define TRAFFIC_LIGHT_FUSION__FUSION_CONFIG_BUILDER_HPP_

#include "traffic_light_fusion.hpp"

#include <functional>
#include <iostream>
#include <string>

namespace autoware::traffic_light
{

// Where build_fusion_config() reports a value it had to normalize. TrafficLightFusionNode passes
// an RCLCPP_WARN; the ROS-free callers get the std::cerr default below, since they have no logger.
using ConfigWarningSink = std::function<void(const std::string &)>;

inline ConfigWarningSink default_config_warning_sink()
{
  return [](const std::string & message) { std::cerr << message << std::endl; };
}

// The single mapping from this Node's parameter names to TrafficLightFusionConfig -- the back-end
// counterpart of build_recognition_config(); see that function for what `Source` must provide and
// why the mapping lives in one place.
//
// crosswalk_estimator's values are hardcoded rather than read: they are
// autoware_crosswalk_traffic_light_estimator's own package defaults and no deployment has ever
// needed to change them. `camera_namespaces` is not here either -- it selects which cameras the
// Node subscribes to, which is Node I/O rather than core configuration, so the Node reads it
// separately.
template <typename Source>
TrafficLightFusionConfig build_fusion_config(
  Source & source, const ConfigWarningSink & warn = default_config_warning_sink())
{
  TrafficLightFusionConfig config;

  // These two are the ones MultiCameraFusionNode itself declares and every x2 deployment overrides
  // (see config/traffic_light_fusion.param.yaml for why message_lifespan must exceed the camera
  // period), so they are real parameters rather than hardcoded values.
  config.multi_camera_fusion.message_lifespan =
    source.template get<double>("multi_camera_fusion.message_lifespan");
  config.multi_camera_fusion.prior_log_odds =
    source.template get<double>("multi_camera_fusion.prior_log_odds");
  // Not read: MultiCameraFusionNode declares these two under `signal_consistency_check.*`, but no
  // x2 param file sets either, so both keep the package default (disabled).
  config.multi_camera_fusion.use_signal_consistency_check = false;
  config.multi_camera_fusion.publish_partial_matched_signal = false;
  // lanelet_map_ptr is deliberately left null: TrafficLightFusion's constructor fills it in from
  // the LaneletMapBin its caller supplies.

  config.arbiter.external_delay_tolerance =
    source.template get<double>("arbiter.external_delay_tolerance");
  config.arbiter.external_time_tolerance =
    source.template get<double>("arbiter.external_time_tolerance");
  config.arbiter.perception_time_tolerance =
    source.template get<double>("arbiter.perception_time_tolerance");
  config.arbiter.enable_signal_matching =
    source.template get<bool>("arbiter.enable_signal_matching");

  // Same normalization TrafficLightArbiterNode does, for the same reason: the core treats every
  // unknown value as "confidence" silently, so the typo is caught (and warned about) here.
  auto source_priority = source.template get<std::string>("arbiter.source_priority");
  if (
    source_priority != "external" && source_priority != "perception" &&
    source_priority != "confidence") {
    warn("Unknown arbiter.source_priority '" + source_priority + "', defaulting to 'confidence'");
    source_priority = "confidence";
  }
  config.arbiter.source_priority = source_priority;

  // Fixed values, not parameters: these are autoware_crosswalk_traffic_light_estimator's own
  // package defaults, and no deployment has ever needed to change them.
  config.crosswalk_estimator.use_last_detect_color = true;
  config.crosswalk_estimator.use_pedestrian_signal_detect = true;
  config.crosswalk_estimator.last_detect_color_hold_time = 2.0;
  config.crosswalk_estimator.flashing_detection.last_colors_hold_time = 1.0;

  return config;
}

}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_FUSION__FUSION_CONFIG_BUILDER_HPP_

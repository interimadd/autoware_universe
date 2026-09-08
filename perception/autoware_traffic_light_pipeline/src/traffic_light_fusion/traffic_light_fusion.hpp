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

#ifndef TRAFFIC_LIGHT_FUSION__TRAFFIC_LIGHT_FUSION_HPP_
#define TRAFFIC_LIGHT_FUSION__TRAFFIC_LIGHT_FUSION_HPP_

#include <autoware/crosswalk_traffic_light_estimator/crosswalk_traffic_light_estimator.hpp>
#include <autoware/traffic_light_arbiter/traffic_light_arbiter.hpp>
#include <autoware/traffic_light_multi_camera_fusion/multi_camera_fusion.hpp>
#include <tl_expected/expected.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <string>

// Single-Node composition of the traffic-light back-end ("後段") pipeline:
//   multi_camera_fusion -> arbiter -> crosswalk_traffic_light_estimator
//
// This header (and its .cpp) is a straight port of autoware_traffic_light_component_test's
// TrafficLightFusionPipeline: same composition, same run() body, renamed to be a first-class
// production API alongside this package's front-end TrafficLightRecognition (see
// ../traffic_light_recognition/traffic_light_recognition.hpp). The Component Test package keeps
// using its own copy until it is switched over to this one.

namespace autoware::traffic_light
{

// The back-end's own constructor arguments for TrafficLightArbiter: unlike the front-end's
// TrafficLightClassifierConfig-style structs, production has no TrafficLightArbiterConfig type to
// reuse -- the real constructor takes five plain arguments
// (autoware_traffic_light_arbiter/include/.../traffic_light_arbiter.hpp) -- so this is a small
// local struct that exists only to keep TrafficLightFusionConfig one flat aggregate.
struct TrafficLightArbiterArgs
{
  // "confidence", "external", or "perception"; any other value is treated as "confidence" by
  // TrafficLightArbiter/SignalMatchValidator, mirroring TrafficLightArbiterNode's parameter
  // fallback (which TrafficLightFusionNode's declare_config() reproduces).
  std::string source_priority = "confidence";
  bool enable_signal_matching = false;
  double external_delay_tolerance = 0.0;
  double external_time_tolerance = 0.0;
  double perception_time_tolerance = 0.0;
};

// Configuration for the three back-end cores. `multi_camera_fusion.lanelet_map_ptr` is left null
// here and filled in by TrafficLightFusion's constructor from `map_msg`: the map is a
// construction-time input to the pipeline, not a per-run config value, and every one of the three
// cores needs the exact same LaneletMapBin round-trip, so filling it in the ctor keeps that
// round-trip in one place instead of three.
struct TrafficLightFusionConfig
{
  MultiCameraFusionConfig multi_camera_fusion;
  TrafficLightArbiterArgs arbiter;
  autoware::crosswalk_traffic_light_estimator::CrosswalkTrafficLightEstimatorConfig
    crosswalk_estimator;
};

// The back-end composition: multi_camera_fusion -> arbiter ->
// crosswalk_traffic_light_estimator, run synchronously for one camera's (camera_info, rois,
// signals) triple at a time, exactly mirroring production's per-trigger callback chain. Unlike
// TrafficLightRecognition, this class is deliberately stateful: each of the three cores keeps its
// own internal history (record_arr_set_ / perception_traffic_light_ + external_traffic_lights_ /
// last_detect_color_ + FlashingDetector), so run()'s return value depends on every prior run()
// call, not just its own arguments. It never reads the clock itself -- every timestamp it acts on
// comes from the trigger's own `camera_info.header.stamp` -- so feeding the same input sequence,
// in the same order, through two fresh instances always produces identical output sequences.
//
// V2X / external traffic signals (arbiter's ~/sub/external_traffic_signals) are out of scope
// here, the same way they are in the Component Test this is ported from: nothing calls
// TrafficLightArbiter::ingest_external(), so arbitration always runs on perception input alone.
class TrafficLightFusion
{
public:
  TrafficLightFusion(
    const TrafficLightFusionConfig & config, const autoware_map_msgs::msg::LaneletMapBin & map_msg);

  // Runs one camera's frame through the chain and returns its output.
  // `camera_info.header.stamp` is also the trigger_stamp run() passes to arbitrate()
  // (production's MultiCameraFusionNode ExactTime-syncs exactly these three inputs per camera).
  // The only failure mode surfaced as an error() here is "no map has arrived yet" (arbiter's
  // ArbitrationResult::output == std::nullopt); an empty map still produces a value, just empty.
  // Intermediate signals (fusion's) and diagnostics (unmapped / conflicted / off-map /
  // unregistered ids) are not part of this composition's output -- production only ever logs them
  // as warnings -- so this class does not surface them.
  //
  // TEMPORARY (2026-09-08): the crosswalk_traffic_light_estimator stage is skipped, so what is
  // returned is the *arbiter's* output. See the note in run()'s definition
  // (traffic_light_fusion.cpp) for why, and for how to restore it.
  tl::expected<autoware_perception_msgs::msg::TrafficLightGroupArray, std::string> run(
    const sensor_msgs::msg::CameraInfo & camera_info,
    const tier4_perception_msgs::msg::TrafficLightRoiArray & selected_rois,
    const tier4_perception_msgs::msg::TrafficLightArray & merged_signals);

private:
  MultiCameraFusion multi_camera_fusion_;
  TrafficLightArbiter arbiter_;
  autoware::crosswalk_traffic_light_estimator::CrosswalkTrafficLightEstimator crosswalk_estimator_;
};

}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_FUSION__TRAFFIC_LIGHT_FUSION_HPP_

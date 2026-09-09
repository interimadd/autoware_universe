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

#ifndef TRAFFIC_LIGHT_RECOGNITION__RECOGNITION_CONFIG_BUILDER_HPP_
#define TRAFFIC_LIGHT_RECOGNITION__RECOGNITION_CONFIG_BUILDER_HPP_

#include "traffic_light_recognition.hpp"

#include <string>

namespace autoware::traffic_light
{

// The single mapping from this Node's parameter names to TrafficLightRecognitionConfig, shared by
// every caller that has to build one:
//
//   - TrafficLightRecognitionNode, through NodeParameterSource (declares the parameters, so
//     launch/`ros2 param`/the json schema all keep working);
//   - the offline evaluation tools and this package's tests, through
//     autoware::component_test_framework::ParameterLoader (reads the same param.yaml with the
//     same rcl parser, no rclcpp::Node needed).
//
// `Source` needs `T get<T>(name)` and `T get_or<T>(name, default)`; both of the above provide
// them. Keeping the mapping here rather than duplicating it per caller is the point: a renamed or
// added parameter is a one-line change that cannot leave evaluation reading a name production no
// longer writes.
//
// Only the values production actually supplies through parameters are read here. Every fixed
// value the underlying cores need -- precision, mean/std, gpu_id, classify_traffic_light_type,
// the map_based_detector calibration-error margins and range/angle cutoffs, ... -- is filled in by
// TrafficLightRecognition's constructor / build_engines() (traffic_light_recognition.cpp).
//
// model_path / label_path / roi_remap_path are read as plain parameters but are deliberately
// absent from config/traffic_light_recognition.param.yaml: they live under $HOME/autoware_data and
// are therefore a property of the machine, injected by the launch file (production) or by a
// set_override() (evaluation and tests).
template <typename Source>
TrafficLightRecognitionConfig build_recognition_config(Source & source)
{
  TrafficLightRecognitionConfig config;

  config.whole_image_detector_model_path =
    source.template get<std::string>("whole_image_detector.model_path");
  config.whole_image_detector_label_path =
    source.template get<std::string>("whole_image_detector.label_path");
  config.whole_image_detector_roi_remap_path =
    source.template get_or<std::string>("whole_image_detector.roi_remap_path", "");
  config.whole_image_detector_score_threshold =
    static_cast<float>(source.template get<double>("whole_image_detector.score_threshold"));
  config.whole_image_detector_nms_threshold =
    static_cast<float>(source.template get<double>("whole_image_detector.nms_threshold"));

  config.min_timestamp_offset =
    source.template get<double>("map_based_detector.min_timestamp_offset");
  config.max_timestamp_offset =
    source.template get<double>("map_based_detector.max_timestamp_offset");

  config.car_classifier_model_path = source.template get<std::string>("car_classifier.model_path");
  config.car_classifier_label_path = source.template get<std::string>("car_classifier.label_path");

  config.pedestrian_classifier_model_path =
    source.template get<std::string>("pedestrian_classifier.model_path");
  config.pedestrian_classifier_label_path =
    source.template get<std::string>("pedestrian_classifier.label_path");

  config.over_exposure_threshold =
    source.template get<double>("classifier.over_exposure_threshold");
  config.under_exposure_threshold =
    source.template get<double>("classifier.under_exposure_threshold");

  return config;
}

}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_RECOGNITION__RECOGNITION_CONFIG_BUILDER_HPP_

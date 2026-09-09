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

//
// Cross-check for this package's two config builders, mirroring the same check the underlying
// component packages already carry (e.g. autoware_traffic_light_multi_camera_fusion's
// test_multi_camera_fusion_params_cross_check.cpp): the same production config file must build the
// same config struct whether read through
//
//   (a) an rclcpp::Node via NodeParameterSource -- production's path, the one
//       TrafficLightRecognitionNode / TrafficLightFusionNode take; or
//   (b) autoware::component_test_framework::ParameterLoader with no rclcpp::Node -- the path the
//       offline evaluation tools and the ROS-free core test take.
//
// build_recognition_config() / build_fusion_config() are shared by both sides, so what this
// actually pins down is that the two *sources* behave identically on the real config files:
// rcl's yaml parser (ParameterLoader) and rclcpp's parameter overrides (the Node) must agree on
// every value, including the ones only one side supplies as an override.
//
// No GPU, TensorRT or ONNX model is needed: nothing here constructs a pipeline core, only its
// config struct.
//

#include "../src/common/node_parameter_source.hpp"
#include "../src/common/package_config.hpp"
#include "../src/traffic_light_fusion/fusion_config_builder.hpp"
#include "../src/traffic_light_recognition/recognition_config_builder.hpp"

#include <autoware/component_test_framework/parameter_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

namespace
{
namespace tl = autoware::traffic_light;
using autoware::component_test_framework::ParameterLoader;

// Stand-ins for the model/label paths production injects as launch arguments: this test never
// opens them, it only checks both sources report the same string back.
constexpr char kYoloxModel[] = "/tmp/yolox.onnx";
constexpr char kYoloxLabel[] = "/tmp/yolox_labels.txt";
constexpr char kRoiRemap[] = "/tmp/roi_remap.csv";
constexpr char kClassifierModel[] = "/tmp/classifier.onnx";
constexpr char kClassifierLabel[] = "/tmp/classifier_labels.txt";

// Side (a): a bare Node fed the config file exactly as the launch file feeds it -- `--params-file`
// first, then the model/label paths as `-p` overrides.
std::shared_ptr<rclcpp::Node> make_node_from_config(
  const std::string & node_name, const std::string & param_file,
  const std::vector<std::string> & overrides)
{
  std::vector<std::string> args{"--ros-args", "--params-file", param_file};
  for (const auto & override_arg : overrides) {
    args.emplace_back("-p");
    args.emplace_back(override_arg);
  }
  rclcpp::NodeOptions options;
  options.arguments(args);
  return std::make_shared<rclcpp::Node>(node_name, options);
}

TEST(ConfigBuildersCrossCheck, RecognitionConfigMatchesBetweenNodeAndParameterLoader)
{
  // Arrange
  const auto param_file = tl::package_config_path(tl::kRecognitionParamFile);

  auto node = make_node_from_config(
    "recognition_cross_check", param_file,
    {std::string("whole_image_detector.model_path:=") + kYoloxModel,
     std::string("whole_image_detector.label_path:=") + kYoloxLabel,
     std::string("whole_image_detector.roi_remap_path:=") + kRoiRemap,
     std::string("car_classifier.model_path:=") + kClassifierModel,
     std::string("car_classifier.label_path:=") + kClassifierLabel,
     std::string("pedestrian_classifier.model_path:=") + kClassifierModel,
     std::string("pedestrian_classifier.label_path:=") + kClassifierLabel});
  tl::NodeParameterSource node_source(node.get());

  ParameterLoader loader;
  loader.merge_yaml_file(param_file);
  loader.set_override("whole_image_detector.model_path", rclcpp::ParameterValue(kYoloxModel));
  loader.set_override("whole_image_detector.label_path", rclcpp::ParameterValue(kYoloxLabel));
  loader.set_override("whole_image_detector.roi_remap_path", rclcpp::ParameterValue(kRoiRemap));
  loader.set_override("car_classifier.model_path", rclcpp::ParameterValue(kClassifierModel));
  loader.set_override("car_classifier.label_path", rclcpp::ParameterValue(kClassifierLabel));
  loader.set_override("pedestrian_classifier.model_path", rclcpp::ParameterValue(kClassifierModel));
  loader.set_override("pedestrian_classifier.label_path", rclcpp::ParameterValue(kClassifierLabel));

  // Act
  const auto via_node = tl::build_recognition_config(node_source);
  const auto via_loader = tl::build_recognition_config(loader);

  // Assert
  EXPECT_EQ(via_node.whole_image_detector_model_path, via_loader.whole_image_detector_model_path);
  EXPECT_EQ(via_node.whole_image_detector_label_path, via_loader.whole_image_detector_label_path);
  EXPECT_EQ(
    via_node.whole_image_detector_roi_remap_path, via_loader.whole_image_detector_roi_remap_path);
  EXPECT_FLOAT_EQ(
    via_node.whole_image_detector_score_threshold, via_loader.whole_image_detector_score_threshold);
  EXPECT_FLOAT_EQ(
    via_node.whole_image_detector_nms_threshold, via_loader.whole_image_detector_nms_threshold);
  EXPECT_DOUBLE_EQ(via_node.min_timestamp_offset, via_loader.min_timestamp_offset);
  EXPECT_DOUBLE_EQ(via_node.max_timestamp_offset, via_loader.max_timestamp_offset);
  EXPECT_EQ(via_node.car_classifier_model_path, via_loader.car_classifier_model_path);
  EXPECT_EQ(via_node.car_classifier_label_path, via_loader.car_classifier_label_path);
  EXPECT_EQ(via_node.pedestrian_classifier_model_path, via_loader.pedestrian_classifier_model_path);
  EXPECT_EQ(via_node.pedestrian_classifier_label_path, via_loader.pedestrian_classifier_label_path);
  EXPECT_DOUBLE_EQ(via_node.over_exposure_threshold, via_loader.over_exposure_threshold);
  EXPECT_DOUBLE_EQ(via_node.under_exposure_threshold, via_loader.under_exposure_threshold);

  // And the config file really is what supplied the tuned values, not a default in the struct.
  EXPECT_EQ(via_loader.whole_image_detector_model_path, kYoloxModel);
  EXPECT_GT(via_loader.whole_image_detector_score_threshold, 0.0f);
  EXPECT_LT(via_loader.min_timestamp_offset, 0.0);
}

TEST(ConfigBuildersCrossCheck, FusionConfigMatchesBetweenNodeAndParameterLoader)
{
  // Arrange -- the back-end config file carries every value, so there is nothing to override.
  const auto param_file = tl::package_config_path(tl::kFusionParamFile);

  auto node = make_node_from_config("fusion_cross_check", param_file, {});
  tl::NodeParameterSource node_source(node.get());

  ParameterLoader loader;
  loader.merge_yaml_file(param_file);

  // Act
  const auto via_node = tl::build_fusion_config(node_source);
  const auto via_loader = tl::build_fusion_config(loader);

  // Assert
  EXPECT_DOUBLE_EQ(
    via_node.multi_camera_fusion.message_lifespan, via_loader.multi_camera_fusion.message_lifespan);
  EXPECT_DOUBLE_EQ(
    via_node.multi_camera_fusion.prior_log_odds, via_loader.multi_camera_fusion.prior_log_odds);
  EXPECT_DOUBLE_EQ(
    via_node.arbiter.external_delay_tolerance, via_loader.arbiter.external_delay_tolerance);
  EXPECT_DOUBLE_EQ(
    via_node.arbiter.external_time_tolerance, via_loader.arbiter.external_time_tolerance);
  EXPECT_DOUBLE_EQ(
    via_node.arbiter.perception_time_tolerance, via_loader.arbiter.perception_time_tolerance);
  EXPECT_EQ(via_node.arbiter.enable_signal_matching, via_loader.arbiter.enable_signal_matching);
  EXPECT_EQ(via_node.arbiter.source_priority, via_loader.arbiter.source_priority);

  // The deployed values, not autoware_traffic_light_arbiter's / _multi_camera_fusion's own
  // package defaults: message_lifespan must exceed the camera period, and this pipeline runs
  // without an external (V2X) source. See config/traffic_light_fusion.param.yaml.
  EXPECT_DOUBLE_EQ(via_loader.multi_camera_fusion.message_lifespan, 0.12);
  EXPECT_EQ(via_loader.arbiter.source_priority, "perception");
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}

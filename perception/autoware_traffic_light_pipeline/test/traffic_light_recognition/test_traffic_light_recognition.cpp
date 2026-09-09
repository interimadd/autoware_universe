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
// Tests for TrafficLightRecognition, the ROS-free composed core (plan §3.2, §6).
//
// The constructor builds three TensorRT engines (whole-image detector + car + pedestrian
// classifiers), so every test here needs a GPU, TensorRT and the ONNX models (downloaded under
// autoware_data). The suite self-skips (GTEST_SKIP) when any of these are unavailable, and is
// additionally gated in CMakeLists.txt behind TRT_AVAIL AND CUDA_AVAIL. The engines are built
// once per suite (SetUpTestSuite) since the build is minutes-long.
//
// Scope (plan §6): run()'s behavior with an empty map / no route, tf resolution failure
// producing tl::make_unexpected, and set_route() error propagation. The composition of detection
// -> selection -> classification -> merge itself is exercised end-to-end by
// autoware_traffic_light_component_test's own tests against the identical code this was ported
// from, so it is not re-verified here.
//

#include "../../src/common/config_yaml.hpp"
#include "../../src/traffic_light_recognition/traffic_light_recognition.hpp"

#include <autoware/cuda_utils/cuda_gtest_utils.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>

#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/lanelet_segment.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

#include <cstdlib>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace
{
namespace tl = autoware::traffic_light;

constexpr char kCameraFrame[] = "camera_optical_link";

// The tuned half of the config is read from this package's own config file -- the same one
// launch/traffic_light_recognition.launch.xml feeds the Node -- rather than duplicated here, so
// retuning a threshold cannot leave the test asserting against the old value.
constexpr char kRecognitionParamFile[] = "traffic_light_recognition.param.yaml";

YAML::Node require_param(const YAML::Node & node, const std::string & key)
{
  return tl::require(node, key, kRecognitionParamFile);
}

// --- autoware_data resolution (mirrors autoware_tensorrt_yolox / autoware_traffic_light_classifier
// tests' resolve_autoware_data_file() helpers) -------------------------------------------------

std::string resolve_yolox_file(const std::string & filename)
{
  std::vector<std::string> candidate_dirs;
  if (const char * override_dir = std::getenv("YOLOX_TEST_DATA_DIR")) {
    candidate_dirs.emplace_back(override_dir);
  }
  if (const char * home = std::getenv("HOME")) {
    candidate_dirs.emplace_back(std::string(home) + "/autoware_data/tensorrt_yolox");
    candidate_dirs.emplace_back(std::string(home) + "/autoware_data/ml_models/tensorrt_yolox");
  }
  for (const auto & dir : candidate_dirs) {
    const std::string candidate = dir + "/" + filename;
    if (std::filesystem::exists(candidate)) {
      return candidate;
    }
  }
  return "";
}

std::string resolve_classifier_file(const std::string & filename)
{
  std::vector<std::string> candidate_dirs;
  if (const char * override_dir = std::getenv("TLC_TEST_DATA_DIR")) {
    candidate_dirs.emplace_back(override_dir);
  }
  if (const char * home = std::getenv("HOME")) {
    candidate_dirs.emplace_back(
      std::string(home) + "/autoware_data/ml_models/traffic_light_classifier");
    candidate_dirs.emplace_back(std::string(home) + "/autoware_data/traffic_light_classifier");
  }
  for (const auto & dir : candidate_dirs) {
    const std::string candidate = dir + "/" + filename;
    if (std::filesystem::exists(candidate)) {
      return candidate;
    }
  }
  return "";
}

// A flat solid-color BGR8 image big enough for the yolox model's expected input.
sensor_msgs::msg::Image make_image(int width, int height, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::Image image;
  image.header.frame_id = kCameraFrame;
  image.header.stamp = stamp;
  image.height = static_cast<uint32_t>(height);
  image.width = static_cast<uint32_t>(width);
  image.encoding = "bgr8";
  image.is_bigendian = 0;
  image.step = image.width * 3;
  image.data.assign(static_cast<size_t>(image.step) * image.height, 128);
  return image;
}

sensor_msgs::msg::CameraInfo make_camera_info(int width, int height, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.header.frame_id = kCameraFrame;
  camera_info.header.stamp = stamp;
  camera_info.width = static_cast<uint32_t>(width);
  camera_info.height = static_cast<uint32_t>(height);
  const double fx = width;  // arbitrary but plausible focal length
  const double fy = width;
  const double cx = width / 2.0;
  const double cy = height / 2.0;
  camera_info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  camera_info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  camera_info.distortion_model = "plumb_bob";
  camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  return camera_info;
}

autoware_map_msgs::msg::LaneletMapBin make_empty_map()
{
  const auto lanelet_map = std::make_shared<lanelet::LaneletMap>();
  auto map_bin = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(lanelet_map);
  map_bin.header.frame_id = "map";
  return map_bin;
}

// A static map->camera transform so sample_map_to_camera_transforms() resolves at any stamp.
std::unique_ptr<tf2::BufferCore> make_tf_buffer_with_camera_transform()
{
  auto buffer = std::make_unique<tf2::BufferCore>(tf2::durationFromSec(3600));
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.header.stamp = rclcpp::Time(0);
  transform.child_frame_id = kCameraFrame;
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  buffer->setTransform(transform, "test", /*is_static=*/true);
  return buffer;
}

// Builds the real TrafficLightRecognitionConfig once for the whole suite (each TensorRT engine
// build is minutes-long). When a model or a usable GPU is missing, config_ stays unset and
// skip_reason_ explains why; each test then GTEST_SKIPs.
class TrafficLightRecognitionTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    const std::string yolox_model = resolve_yolox_file("yolox_s_traffic_light_detector.onnx");
    const std::string yolox_label = resolve_yolox_file("traffic_light_label.txt");
    const std::string classifier_model =
      resolve_classifier_file("traffic_light_classifier_mobilenetv2_batch_1.onnx");
    const std::string classifier_label = resolve_classifier_file("lamp_labels.txt");

    if (yolox_model.empty() || yolox_label.empty()) {
      skip_reason_ = "tensorrt_yolox traffic light model/label not found under autoware_data";
      return;
    }
    if (classifier_model.empty() || classifier_label.empty()) {
      skip_reason_ = "traffic_light_classifier model/label not found under autoware_data";
      return;
    }
    if (!autoware::cuda_utils::is_cuda_runtime_available()) {
      skip_reason_ = "CUDA runtime / GPU not available";
      return;
    }

    // Only the values actually supplied via ROS 2 parameters / launch arguments in production
    // (plan §5.1) are set here -- precision, mean/std, gpu_id, classify_traffic_light_type, the
    // map_based_detector calibration-error margins and range/angle cutoffs, etc. are fixed inside
    // TrafficLightRecognition's constructor itself (traffic_light_recognition.cpp). Of those, the
    // thresholds/offsets come from the package config; the model and label paths do not, because
    // they live under the user's $HOME and are injected per machine (see resolve_*_file() above).
    const auto params = tl::load_package_param_yaml(kRecognitionParamFile);
    const auto detector_params = require_param(params, "whole_image_detector");
    const auto map_based_detector_params = require_param(params, "map_based_detector");
    const auto classifier_params = require_param(params, "classifier");

    tl::TrafficLightRecognitionConfig config;
    config.whole_image_detector_model_path = yolox_model;
    config.whole_image_detector_label_path = yolox_label;
    config.whole_image_detector_score_threshold =
      require_param(detector_params, "score_threshold").as<float>();
    config.whole_image_detector_nms_threshold =
      require_param(detector_params, "nms_threshold").as<float>();

    config.min_timestamp_offset =
      require_param(map_based_detector_params, "min_timestamp_offset").as<double>();
    config.max_timestamp_offset =
      require_param(map_based_detector_params, "max_timestamp_offset").as<double>();

    config.car_classifier_model_path = classifier_model;
    config.car_classifier_label_path = classifier_label;

    config.pedestrian_classifier_model_path = classifier_model;
    config.pedestrian_classifier_label_path = classifier_label;

    config.over_exposure_threshold =
      require_param(classifier_params, "over_exposure_threshold").as<double>();
    config.under_exposure_threshold =
      require_param(classifier_params, "under_exposure_threshold").as<double>();

    config_ = std::move(config);
    tf_buffer_ = make_tf_buffer_with_camera_transform();

    try {
      core_ =
        std::make_unique<tl::TrafficLightRecognition>(*config_, make_empty_map(), *tf_buffer_);
    } catch (const std::exception & e) {
      skip_reason_ = std::string("TrafficLightRecognition environment unavailable: ") + e.what();
      core_.reset();
    }
  }

  static void TearDownTestSuite()
  {
    core_.reset();
    tf_buffer_.reset();
    config_.reset();
  }

  static inline std::string skip_reason_;
  static inline std::optional<tl::TrafficLightRecognitionConfig> config_;
  static inline std::unique_ptr<tf2::BufferCore> tf_buffer_;
  static inline std::unique_ptr<tl::TrafficLightRecognition> core_;
};

// An empty map (no traffic lights) and no route: run() still succeeds -- yolox detects whatever
// it detects in the frame, but map_based_detector has no traffic lights to project, so no ROIs
// are selected and both classifiers see an empty ROI array.
TEST_F(TrafficLightRecognitionTest, EmptyMapAndNoRouteProducesEmptySelection)
{
  if (!core_) {
    GTEST_SKIP() << skip_reason_;
  }

  // Arrange
  const rclcpp::Time stamp(10, 0);
  const auto image = make_image(640, 480, stamp);
  const auto camera_info = make_camera_info(640, 480, stamp);

  // Act
  const auto result = core_->run(image, camera_info);

  // Assert
  ASSERT_TRUE(result.has_value()) << (result ? "" : result.error());
  EXPECT_TRUE(result->selected_rois.rois.empty());
}

// A camera frame_id with no tf sample available makes run() fail before any detection runs,
// mirroring MapBasedDetector::camera_info_callback() dropping the callback outright.
TEST_F(TrafficLightRecognitionTest, UnresolvableTfReturnsUnexpected)
{
  if (!core_) {
    GTEST_SKIP() << skip_reason_;
  }

  // Arrange -- a frame_id the tf buffer never learned a "map" transform for.
  const rclcpp::Time stamp(10, 0);
  auto image = make_image(640, 480, stamp);
  auto camera_info = make_camera_info(640, 480, stamp);
  image.header.frame_id = "unknown_camera_frame";
  camera_info.header.frame_id = "unknown_camera_frame";

  // Act
  const auto result = core_->run(image, camera_info);

  // Assert
  ASSERT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("transform"), std::string::npos);
}

// set_route() propagates map_based_detector's own SetRouteError for an ill-formed route (a
// LaneletRoute referencing lanelet IDs that do not exist in the (empty) map).
TEST_F(TrafficLightRecognitionTest, SetRoutePropagatesMapBasedDetectorError)
{
  if (!core_) {
    GTEST_SKIP() << skip_reason_;
  }

  // Arrange -- references a lanelet ID that cannot exist in the empty test map.
  autoware_planning_msgs::msg::LaneletRoute route;
  autoware_planning_msgs::msg::LaneletSegment segment;
  autoware_planning_msgs::msg::LaneletPrimitive primitive;
  primitive.id = 12345;
  primitive.primitive_type = "lane";
  segment.preferred_primitive = primitive;
  segment.primitives.push_back(primitive);
  route.segments.push_back(segment);

  // Act
  const auto error = core_->set_route(route);

  // Assert
  ASSERT_TRUE(error.has_value());
  EXPECT_FALSE(error->message.empty());
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

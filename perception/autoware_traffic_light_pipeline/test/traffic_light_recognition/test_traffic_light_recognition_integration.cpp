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
// Integration tests for TrafficLightRecognitionNode (plan §4, §6).
//
// Stands up the real node, publishes the vector map (transient_local) and a synchronized
// (Image, CameraInfo) pair plus the map->camera tf, and observes the 3 production output topics
// (traffic_signals / rois / camera_info).
//
// Like the core test, the node's constructor builds three TensorRT engines, so this suite needs a
// GPU + TensorRT + the ONNX models and self-skips (GTEST_SKIP) when they are unavailable; it is
// additionally gated in CMakeLists.txt behind TRT_AVAIL AND CUDA_AVAIL.
//

#include "../../src/common/config_yaml.hpp"
#include "../../src/traffic_light_recognition/traffic_light_recognition_node.hpp"

#include <autoware/cuda_utils/cuda_gtest_utils.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace
{
namespace tl = autoware::traffic_light;

constexpr char kCameraFrame[] = "camera_optical_link";

using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;
using std::placeholders::_1;

// --- autoware_data resolution (same convention as test_traffic_light_recognition.cpp) ---------

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

struct RequiredData
{
  std::string yolox_model;
  std::string yolox_label;
  std::string classifier_model;
  std::string classifier_label;
  bool available = false;
  std::string skip_reason;
};

RequiredData resolve_required_data()
{
  RequiredData data;
  data.yolox_model = resolve_yolox_file("yolox_s_traffic_light_detector.onnx");
  data.yolox_label = resolve_yolox_file("traffic_light_label.txt");
  data.classifier_model =
    resolve_classifier_file("traffic_light_classifier_mobilenetv2_batch_1.onnx");
  data.classifier_label = resolve_classifier_file("lamp_labels.txt");

  if (data.yolox_model.empty() || data.yolox_label.empty()) {
    data.skip_reason = "tensorrt_yolox traffic light model/label not found under autoware_data";
    return data;
  }
  if (data.classifier_model.empty() || data.classifier_label.empty()) {
    data.skip_reason = "traffic_light_classifier model/label not found under autoware_data";
    return data;
  }
  if (!autoware::cuda_utils::is_cuda_runtime_available()) {
    data.skip_reason = "CUDA runtime / GPU not available";
    return data;
  }
  data.available = true;
  return data;
}

// Stands the Node up on this package's own config file -- the very one
// launch/traffic_light_recognition.launch.xml passes it -- so the integration test exercises the
// deployed parameter values instead of a hand-copied duplicate of them. Only the model/label
// paths are passed as individual overrides: they live under the user's $HOME, so they are absent
// from the config file by design and the launch file injects them separately too. They come after
// --params-file, which is what makes them win.
rclcpp::NodeOptions make_node_options(const RequiredData & data)
{
  std::vector<std::string> args{
    "--ros-args",
    "--params-file",
    tl::package_config_path("traffic_light_recognition.param.yaml"),
    "-p",
    "whole_image_detector.model_path:=" + data.yolox_model,
    "-p",
    "whole_image_detector.label_path:=" + data.yolox_label,
    "-p",
    "car_classifier.model_path:=" + data.classifier_model,
    "-p",
    "car_classifier.label_path:=" + data.classifier_label,
    "-p",
    "pedestrian_classifier.model_path:=" + data.classifier_model,
    "-p",
    "pedestrian_classifier.label_path:=" + data.classifier_label,
  };
  rclcpp::NodeOptions options;
  options.arguments(args);
  return options;
}

autoware_map_msgs::msg::LaneletMapBin make_empty_map()
{
  const auto lanelet_map = std::make_shared<lanelet::LaneletMap>();
  auto map_bin = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(lanelet_map);
  map_bin.header.frame_id = "map";
  return map_bin;
}

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
  const double fx = width;
  const double fy = width;
  const double cx = width / 2.0;
  const double cy = height / 2.0;
  camera_info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  camera_info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  camera_info.distortion_model = "plumb_bob";
  camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  return camera_info;
}

// Captures whether a message arrived on each output topic, without asserting content -- content
// (merge / selection correctness) is the ROS-free core's responsibility
// (test_traffic_light_recognition.cpp).
struct OutputCapture
{
  bool got_signals = false;
  bool got_rois = false;
  bool got_camera_info = false;
};

class IntegrationTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { data_ = resolve_required_data(); }

  static inline RequiredData data_;
};

// Publishes the vector map (transient_local) and repeatedly publishes an (image, camera_info)
// pair plus a static map->camera transform until every expected output topic has fired, or the
// timeout elapses.
OutputCapture run_node(std::shared_ptr<tl::TrafficLightRecognitionNode> node)
{
  OutputCapture capture;
  auto tester = std::make_shared<rclcpp::Node>("integration_tester");

  auto signals_sub = tester->create_subscription<tier4_perception_msgs::msg::TrafficLightArray>(
    "/traffic_light_recognition/output/traffic_signals", rclcpp::QoS{1},
    [&capture](tier4_perception_msgs::msg::TrafficLightArray::ConstSharedPtr) {
      capture.got_signals = true;
    });
  auto rois_sub = tester->create_subscription<tier4_perception_msgs::msg::TrafficLightRoiArray>(
    "/traffic_light_recognition/output/rois", rclcpp::QoS{1},
    [&capture](tier4_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr) {
      capture.got_rois = true;
    });
  auto camera_info_sub = tester->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/traffic_light_recognition/output/camera_info", rclcpp::QoS{1},
    [&capture](sensor_msgs::msg::CameraInfo::ConstSharedPtr) { capture.got_camera_info = true; });

  auto map_pub = tester->create_publisher<autoware_map_msgs::msg::LaneletMapBin>(
    "/traffic_light_recognition/input/vector_map", rclcpp::QoS{1}.transient_local());
  auto image_pub = tester->create_publisher<sensor_msgs::msg::Image>(
    "/traffic_light_recognition/input/image", rclcpp::SensorDataQoS());
  auto camera_info_pub = tester->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/traffic_light_recognition/input/camera_info", rclcpp::SensorDataQoS());
  tf2_ros::StaticTransformBroadcaster tf_broadcaster(tester);

  map_pub->publish(make_empty_map());

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.header.stamp = tester->now();
  transform.child_frame_id = kCameraFrame;
  transform.transform.rotation.w = 1.0;
  tf_broadcaster.sendTransform(transform);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(tester);

  const auto done = [&]() {
    return capture.got_signals && capture.got_rois && capture.got_camera_info;
  };

  const auto deadline = std::chrono::steady_clock::now() + 60s;
  const rclcpp::Time stamp(20, 0);
  const auto image = make_image(640, 480, stamp);
  const auto camera_info = make_camera_info(640, 480, stamp);
  while (!done() && std::chrono::steady_clock::now() < deadline) {
    image_pub->publish(image);
    camera_info_pub->publish(camera_info);
    exec.spin_some();
    std::this_thread::sleep_for(50ms);
  }
  return capture;
}

// All 3 production output topics fire.
TEST_F(IntegrationTest, ProductionTopicsFire)
{
  if (!data_.available) {
    GTEST_SKIP() << data_.skip_reason;
  }

  // Arrange
  auto node = std::make_shared<tl::TrafficLightRecognitionNode>(make_node_options(data_));

  // Act
  const auto capture = run_node(node);

  // Assert
  EXPECT_TRUE(capture.got_signals);
  EXPECT_TRUE(capture.got_rois);
  EXPECT_TRUE(capture.got_camera_info);
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

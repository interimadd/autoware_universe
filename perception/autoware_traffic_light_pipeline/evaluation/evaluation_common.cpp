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

#include "evaluation_common.hpp"

#include "common/package_config.hpp"
#include "traffic_light_recognition/recognition_config_builder.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/component_test_framework/parameter_loader.hpp>
#include <autoware/image_transport_decompressor/image_decompression.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/map_projection_loader/map_projection_loader.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <tf2/time.h>

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::traffic_light::evaluation
{

// --- yaml helpers -------------------------------------------------------------------------------

YAML::Node require(const YAML::Node & node, const std::string & key)
{
  const auto child = node[key];
  if (!child) {
    throw std::runtime_error("evaluation config: missing required key '" + key + "'");
  }
  return child;
}

std::string expand_user_path(const std::string & path)
{
  if (path.empty() || path[0] != '~') {
    return path;
  }
  const char * home = std::getenv("HOME");
  if (!home) {
    throw std::runtime_error(
      "evaluation config: path '" + path + "' starts with '~' but $HOME is not set");
  }
  return std::string(home) + path.substr(1);
}

std::string require_path(const YAML::Node & node, const std::string & key)
{
  return expand_user_path(require(node, key).as<std::string>());
}

namespace
{

// The whole-image detector ships this remap csv as installed package data, so unlike model_path /
// label_path (which live under $HOME/autoware_data and vary per user) it does not belong in the
// evaluation config at all: leaving roi_remap_path unset (or empty) resolves it from
// autoware_tensorrt_yolox's own share directory, the same file its production launch file defaults
// to via $(find-pkg-share autoware_tensorrt_yolox).
std::string default_roi_remap_path()
{
  return ament_index_cpp::get_package_share_directory("autoware_tensorrt_yolox") +
         "/config/traffic_light_roi_label_remap.csv";
}

}  // namespace

CameraConfig parse_camera(const YAML::Node & node)
{
  CameraConfig camera;
  camera.ns = require(node, "namespace").as<std::string>();
  camera.camera_info_topic = require(node, "camera_info_topic").as<std::string>();
  camera.compressed_image_topic = require(node, "compressed_image_topic").as<std::string>();

  const auto output_topics = require(node, "output_topics");
  camera.traffic_signals_topic = require(output_topics, "traffic_signals").as<std::string>();
  camera.rois_topic = require(output_topics, "rois").as<std::string>();

  // min/max_timestamp_offset are not read here: CameraConfig carries them per camera because each
  // TrafficLightRecognition instance takes its own, but their value is the package config's,
  // filled in by load_evaluation_config().
  return camera;
}

TrafficLightRecognitionConfig parse_recognition(const YAML::Node & node)
{
  // The package config -- the same file the launch file passes the Node -- read with the same rcl
  // yaml parser production uses.
  autoware::component_test_framework::ParameterLoader loader;
  loader.merge_yaml_file(package_config_path(kRecognitionParamFile));

  // The model/label paths are the one group the package config deliberately does not carry: they
  // live under the user's $HOME, so production injects them as launch arguments. Here they come
  // from the evaluation yaml and go in as overrides -- exactly what ParameterLoader::set_override()
  // is for.
  const auto override_path = [&loader](const std::string & name, const std::string & value) {
    loader.set_override(name, rclcpp::ParameterValue(value));
  };

  const auto detector = require(node, "whole_image_detector");
  override_path("whole_image_detector.model_path", require_path(detector, "model_path"));
  override_path("whole_image_detector.label_path", require_path(detector, "label_path"));
  const auto roi_remap_path_node = detector["roi_remap_path"];
  override_path(
    "whole_image_detector.roi_remap_path",
    roi_remap_path_node && !roi_remap_path_node.as<std::string>().empty()
      ? expand_user_path(roi_remap_path_node.as<std::string>())
      : default_roi_remap_path());

  const auto car = require(node, "car_classifier");
  override_path("car_classifier.model_path", require_path(car, "model_path"));
  override_path("car_classifier.label_path", require_path(car, "label_path"));

  const auto pedestrian = require(node, "pedestrian_classifier");
  override_path("pedestrian_classifier.model_path", require_path(pedestrian, "model_path"));
  override_path("pedestrian_classifier.label_path", require_path(pedestrian, "label_path"));

  // The same function TrafficLightRecognitionNode's declare_recognition_config() calls.
  return build_recognition_config(loader);
}

EvaluationConfig load_evaluation_config(
  const std::string & config_path, const std::string & dataset_path)
{
  EvaluationConfig config;
  config.input_bag_path = dataset_path + "/input_bag";
  config.lanelet2_map_path = dataset_path + "/map/lanelet2_map.osm";
  config.map_projector_info_path = dataset_path + "/map/map_projector_info.yaml";

  const auto root = YAML::LoadFile(config_path);
  config.recognition = parse_recognition(require(root, "recognition"));

  for (const auto & camera : require(root, "cameras")) {
    config.cameras.push_back(parse_camera(camera));
    // Every camera gets the package config's window; see CameraConfig.
    config.cameras.back().min_timestamp_offset = config.recognition.min_timestamp_offset;
    config.cameras.back().max_timestamp_offset = config.recognition.max_timestamp_offset;
  }
  if (config.cameras.empty()) {
    throw std::runtime_error("evaluation config: 'cameras' is empty");
  }
  return config;
}

// --- rosbag input -----------------------------------------------------------------------------

namespace
{

template <typename MessageT>
MessageT deserialize(const rosbag2_storage::SerializedBagMessageSharedPtr & bag_message)
{
  rclcpp::SerializedMessage serialized_message(*bag_message->serialized_data);
  rclcpp::Serialization<MessageT> serialization;
  MessageT message;
  serialization.deserialize_message(&serialized_message, &message);
  return message;
}

int64_t stamp_nanoseconds(const std_msgs::msg::Header & header)
{
  return rclcpp::Time(header.stamp).nanoseconds();
}

// Which camera a bag topic belongs to and what it carries, resolved once up front so the single
// pass over the bag does not need to re-derive it.
struct TopicRole
{
  std::size_t camera_index;
  bool is_image;
};

std::unordered_map<std::string, TopicRole> build_topic_roles(
  const std::vector<CameraConfig> & cameras)
{
  std::unordered_map<std::string, TopicRole> roles;
  for (std::size_t index = 0; index < cameras.size(); ++index) {
    const auto & camera = cameras[index];
    roles[camera.compressed_image_topic] = TopicRole{index, true};
    roles[camera.camera_info_topic] = TopicRole{index, false};
  }
  return roles;
}

// One camera's images/camera_infos keyed by header stamp while the bag is being read, so pairing
// does not depend on how the two topics happened to interleave on disk. Images are kept
// compressed, as read -- see Frame.
struct CameraBuffers
{
  std::map<int64_t, sensor_msgs::msg::CompressedImage> images_by_stamp;
  std::map<int64_t, sensor_msgs::msg::CameraInfo> camera_infos_by_stamp;
};

}  // namespace

std::vector<Frame> load_frames(const EvaluationConfig & config)
{
  const auto topic_roles = build_topic_roles(config.cameras);

  rosbag2_cpp::Reader reader;
  reader.open(config.input_bag_path);
  rosbag2_storage::StorageFilter filter;
  for (const auto & [topic, role] : topic_roles) {
    filter.topics.push_back(topic);
  }
  reader.set_filter(filter);

  std::vector<CameraBuffers> buffers(config.cameras.size());
  while (reader.has_next()) {
    const auto bag_message = reader.read_next();
    const auto role_it = topic_roles.find(bag_message->topic_name);
    if (role_it == topic_roles.end()) {
      continue;
    }
    const TopicRole & role = role_it->second;
    auto & camera_buffers = buffers[role.camera_index];

    if (!role.is_image) {
      auto camera_info = deserialize<sensor_msgs::msg::CameraInfo>(bag_message);
      camera_buffers.camera_infos_by_stamp.emplace(
        stamp_nanoseconds(camera_info.header), std::move(camera_info));
    } else {
      auto compressed = deserialize<sensor_msgs::msg::CompressedImage>(bag_message);
      camera_buffers.images_by_stamp.emplace(
        stamp_nanoseconds(compressed.header), std::move(compressed));
    }
  }

  // std::map keeps each camera's messages stamp-sorted, so collecting matched stamps camera by
  // camera into one std::map<(stamp, camera_index)> yields the merged order directly.
  std::map<std::pair<int64_t, std::size_t>, Frame> frames_by_key;
  for (std::size_t index = 0; index < buffers.size(); ++index) {
    for (auto & [stamp, image] : buffers[index].images_by_stamp) {
      auto camera_info_it = buffers[index].camera_infos_by_stamp.find(stamp);
      if (camera_info_it == buffers[index].camera_infos_by_stamp.end()) {
        continue;
      }
      frames_by_key.emplace(
        std::pair{stamp, index}, Frame{index, std::move(image), std::move(camera_info_it->second)});
    }
  }

  std::vector<Frame> frames;
  frames.reserve(frames_by_key.size());
  for (auto & [key, frame] : frames_by_key) {
    frames.push_back(std::move(frame));
  }
  return frames;
}

std::vector<Frame> load_frames_for_camera(const EvaluationConfig & config, std::size_t camera_index)
{
  const auto & camera = config.cameras.at(camera_index);

  rosbag2_cpp::Reader reader;
  reader.open(config.input_bag_path);
  reader.set_filter(
    rosbag2_storage::StorageFilter{{camera.compressed_image_topic, camera.camera_info_topic}});

  CameraBuffers buffers;
  while (reader.has_next()) {
    const auto bag_message = reader.read_next();
    if (bag_message->topic_name == camera.camera_info_topic) {
      auto camera_info = deserialize<sensor_msgs::msg::CameraInfo>(bag_message);
      buffers.camera_infos_by_stamp.emplace(
        stamp_nanoseconds(camera_info.header), std::move(camera_info));
    } else {
      auto compressed_image = deserialize<sensor_msgs::msg::CompressedImage>(bag_message);
      buffers.images_by_stamp.emplace(
        stamp_nanoseconds(compressed_image.header), std::move(compressed_image));
    }
  }

  // buffers.images_by_stamp is already stamp-sorted (std::map), so this preserves ascending order.
  std::vector<Frame> frames;
  frames.reserve(buffers.images_by_stamp.size());
  for (auto & [stamp, image] : buffers.images_by_stamp) {
    auto camera_info_it = buffers.camera_infos_by_stamp.find(stamp);
    if (camera_info_it == buffers.camera_infos_by_stamp.end()) {
      continue;
    }
    frames.push_back(Frame{camera_index, std::move(image), std::move(camera_info_it->second)});
  }
  return frames;
}

std::optional<sensor_msgs::msg::Image> decode_frame_image(const Frame & frame)
{
  const auto & compressed = frame.image;
  auto decoded = autoware::image_transport_decompressor::decompress_image(compressed, "default");
  if (!decoded) {
    std::cerr << "failed to decompress image at " << stamp_nanoseconds(compressed.header) << ": "
              << decoded.error() << std::endl;
    return std::nullopt;
  }
  return std::move(*decoded);
}

std::unique_ptr<tf2::BufferCore> load_transform_buffer(const std::string & bag_path)
{
  auto buffer = std::make_unique<tf2::BufferCore>(tf2::durationFromSec(24 * 60 * 60));

  rosbag2_cpp::Reader reader;
  reader.open(bag_path);
  reader.set_filter(rosbag2_storage::StorageFilter{{"/tf", "/tf_static"}});
  while (reader.has_next()) {
    const auto bag_message = reader.read_next();
    const bool is_static = bag_message->topic_name == "/tf_static";
    const auto message = deserialize<tf2_msgs::msg::TFMessage>(bag_message);
    for (const auto & transform : message.transforms) {
      buffer->setTransform(transform, "traffic_light_pipeline_evaluation", is_static);
    }
  }
  return buffer;
}

autoware_map_msgs::msg::LaneletMapBin load_map(const EvaluationConfig & config)
{
  const auto projector_info =
    autoware::map_projection_loader::load_info_from_yaml(config.map_projector_info_path);
  if (projector_info.projector_type != autoware_map_msgs::msg::MapProjectorInfo::MGRS) {
    throw std::runtime_error("dataset map_projector_info.yaml is not an MGRS projector");
  }
  const auto map =
    autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(config.lanelet2_map_path);
  return autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
}

// --- rosbag output ------------------------------------------------------------------------------

std::string detect_input_bag_storage_id(const std::string & bag_path)
{
  rosbag2_cpp::Reader reader;
  reader.open(bag_path);
  return reader.get_metadata().storage_identifier;
}

void check_only_contains_rosbag_files(const std::string & bag_path)
{
  for (const auto & entry : std::filesystem::directory_iterator(bag_path)) {
    const auto & extension = entry.path().extension();
    if (
      entry.is_directory() ||
      (extension != ".db3" && extension != ".mcap" && extension != ".yaml")) {
      throw std::runtime_error(
        "refusing to overwrite " + bag_path + ": unexpected entry " + entry.path().string() +
        " (expected only .db3 / .mcap / .yaml files -- is --output-bag pointing at the right "
        "directory?)");
    }
  }
}

void remove_output_bag_if_exists(const std::string & output_bag_path)
{
  if (!std::filesystem::exists(output_bag_path)) {
    return;
  }
  if (!std::filesystem::is_directory(output_bag_path)) {
    throw std::runtime_error(
      "refusing to overwrite " + output_bag_path + ": it is not a directory");
  }
  check_only_contains_rosbag_files(output_bag_path);
  std::filesystem::remove_all(output_bag_path);
}

}  // namespace autoware::traffic_light::evaluation

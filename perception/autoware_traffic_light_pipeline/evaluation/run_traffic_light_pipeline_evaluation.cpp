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

// Offline evaluation runner for the full pipeline: TrafficLightRecognition (front-end) followed
// by TrafficLightFusion (back-end), both this package's ROS-free cores.
//
// Given a t4dataset directory it runs the front-end one camera at a time: for each camera it
// loads only that camera's (image, camera_info) pairs out of the dataset's rosbag
// (load_frames_for_camera()), drives a fresh TrafficLightRecognition over them, then discards
// those frames before moving to the next camera -- so memory stays proportional to one camera's
// frames rather than every camera's combined. The front-end has no cross-camera state, so this
// changes nothing about any individual result; the per-camera results are sorted back into
// ascending (stamp, camera_index) order once every camera is done (pass A), and only then fed
// through one TrafficLightFusion instance in that same order (pass B) -- rather than interleaving
// front-end and back-end calls frame by frame the way the Node graph (and component_test's
// run_traffic_light_pipeline) does. TrafficLightFusion is stateful but never reads the clock (see
// traffic_light_fusion.hpp), so replaying the same input sequence in the same order through a
// fresh instance in a second pass produces identical output to interleaving it live -- and
// keeping the two passes separate keeps this file's control flow simple. Finally every result
// (front-end and back-end) is written to an output rosbag under production topic names (pass C).
// No rclcpp::init, no executor, no DDS anywhere in this file.
//
// Reference implementation for the three-pass structure and the back-end config shape:
// autoware_component_test's run_traffic_light_pipeline_main.cpp.
//
// The front-end-only counterpart is run_traffic_light_recognition_evaluation, which shares this
// file's yaml/rosbag-loading plumbing via evaluation_common.{hpp,cpp}.
//
// Usage:
//   run_traffic_light_pipeline_evaluation
//     --config <evaluation config yaml, with a fusion: section>
//     --dataset <t4dataset dir>
//     --output-bag <output bag dir>
//
// The dataset layout is fixed (same convention as the Component Test harness):
//   <dataset>/input_bag
//   <dataset>/map/lanelet2_map.osm
//   <dataset>/map/map_projector_info.yaml

#include "common/package_config.hpp"
#include "evaluation_common.hpp"
#include "traffic_light_fusion/fusion_config_builder.hpp"
#include "traffic_light_fusion/traffic_light_fusion.hpp"
#include "traffic_light_recognition/traffic_light_recognition.hpp"

#include <autoware/component_test_framework/parameter_loader.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_cpp/writer.hpp>

#include <sensor_msgs/msg/camera_info.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace
{
using autoware::traffic_light::build_fusion_config;
using autoware::traffic_light::kFusionParamFile;
using autoware::traffic_light::package_config_path;
using autoware::traffic_light::TrafficLightFusion;
using autoware::traffic_light::TrafficLightFusionConfig;
using autoware::traffic_light::TrafficLightRecognition;
using autoware::traffic_light::TrafficLightRecognitionConfig;
using autoware::traffic_light::TrafficLightRecognitionResult;
using autoware::traffic_light::evaluation::CameraConfig;
using autoware::traffic_light::evaluation::decode_frame_image;
using autoware::traffic_light::evaluation::EvaluationConfig;
using autoware::traffic_light::evaluation::Frame;
using autoware::traffic_light::evaluation::require;

// --- back-end config (yaml) -----------------------------------------------------------------

// The `fusion:` section of the evaluation yaml plus the pipeline-side EvaluationConfig it is
// parsed alongside. The TrafficLightFusionConfig itself is built by build_fusion_config() from
// this package's config/traffic_light_fusion.param.yaml -- the very function and the very file
// TrafficLightFusionNode uses -- so an evaluation always runs the deployed back-end configuration.
// The evaluation yaml's `fusion:` section carries only output_topic, which has no package default
// (it is where this run writes its result).
struct FusionEvaluationConfig
{
  std::string output_topic;
  TrafficLightFusionConfig fusion;
};

FusionEvaluationConfig parse_fusion(const YAML::Node & root)
{
  FusionEvaluationConfig config;
  config.output_topic = require(require(root, "fusion"), "output_topic").as<std::string>();

  // The package config -- the same file launch/traffic_light_fusion.launch.xml passes the Node --
  // read with the same rcl yaml parser and turned into a config by the same function
  // TrafficLightFusionNode's declare_fusion_config() calls.
  autoware::component_test_framework::ParameterLoader loader;
  loader.merge_yaml_file(package_config_path(kFusionParamFile));
  config.fusion = build_fusion_config(loader);

  return config;
}

// --- pass A: front-end --------------------------------------------------------------------------

// One front-end run() result, kept together with the camera it came from and the camera_info it
// was produced from -- the back-end pass needs the latter (fusion.run()'s first argument) without
// re-reading the bag.
struct RecordedFrameResult
{
  std::size_t camera_index;
  sensor_msgs::msg::CameraInfo camera_info;
  TrafficLightRecognitionResult result;
};

// Returns config.recognition with min/max_timestamp_offset overridden from `camera` -- the one
// part of TrafficLightRecognitionConfig that legitimately differs per camera (see CameraConfig's
// own comment in evaluation_common.hpp).
TrafficLightRecognitionConfig recognition_config_for_camera(
  const EvaluationConfig & config, const CameraConfig & camera)
{
  auto recognition_config = config.recognition;
  recognition_config.min_timestamp_offset = camera.min_timestamp_offset;
  recognition_config.max_timestamp_offset = camera.max_timestamp_offset;
  return recognition_config;
}

// Loads the tf buffer, then drives one TrafficLightRecognition per camera, one camera at a time:
// for each camera in turn, builds its recognition instance, loads only that camera's frames from
// the bag (load_frames_for_camera()), runs them all, then lets those frames go out of scope
// before moving on to the next camera -- so at most one camera's worth of frames is held in
// memory at once, unlike buffering every camera's frames up front the way load_frames() does.
// The front-end has no cross-camera state, so this changes nothing about any individual frame's
// result -- but it does mean results come back grouped by camera rather than interleaved by
// stamp, so they are sorted into ascending (stamp, camera_index) order before being returned:
// pass B (run_fusion(), stateful) needs that same order production feeds it results in. Frames
// that fail are logged to stderr and skipped, exactly as the Node drops them.
std::vector<RecordedFrameResult> run_recognition(
  const EvaluationConfig & config, const autoware_map_msgs::msg::LaneletMapBin & map_msg)
{
  const auto tf_buffer =
    autoware::traffic_light::evaluation::load_transform_buffer(config.input_bag_path);

  std::vector<RecordedFrameResult> recorded_results;
  for (std::size_t camera_index = 0; camera_index < config.cameras.size(); ++camera_index) {
    const auto & camera = config.cameras[camera_index];
    TrafficLightRecognition recognition(
      recognition_config_for_camera(config, camera), map_msg, *tf_buffer);

    const auto frames =
      autoware::traffic_light::evaluation::load_frames_for_camera(config, camera_index);
    std::cerr << "loaded " << frames.size() << " frames for camera " << camera.ns << " from "
              << config.input_bag_path << std::endl;

    for (const auto & frame : frames) {
      const auto image = decode_frame_image(frame);
      if (!image) {
        continue;
      }
      const auto result = recognition.run(*image, frame.camera_info);
      if (!result) {
        std::cerr << "camera " << camera.ns << " frame at "
                  << rclcpp::Time(image->header.stamp).nanoseconds()
                  << " failed: " << result.error() << std::endl;
        continue;
      }
      recorded_results.push_back({camera_index, frame.camera_info, *result});
    }
  }

  std::stable_sort(
    recorded_results.begin(), recorded_results.end(),
    [](const RecordedFrameResult & lhs, const RecordedFrameResult & rhs) {
      const auto lhs_stamp = rclcpp::Time(lhs.camera_info.header.stamp).nanoseconds();
      const auto rhs_stamp = rclcpp::Time(rhs.camera_info.header.stamp).nanoseconds();
      return std::tie(lhs_stamp, lhs.camera_index) < std::tie(rhs_stamp, rhs.camera_index);
    });
  return recorded_results;
}

// --- pass B: back-end ----------------------------------------------------------------------------

// One TrafficLightFusion, fed pass A's results in ascending (stamp, camera_index) order --
// matching production's per-trigger arrival order -- required because TrafficLightFusion is
// stateful (see traffic_light_fusion.hpp). Events that fail are logged to stderr and skipped.
std::vector<autoware_perception_msgs::msg::TrafficLightGroupArray> run_fusion(
  const FusionEvaluationConfig & fusion_config,
  const std::vector<RecordedFrameResult> & recorded_frame_results,
  const autoware_map_msgs::msg::LaneletMapBin & map_msg)
{
  TrafficLightFusion fusion(fusion_config.fusion, map_msg);

  std::vector<autoware_perception_msgs::msg::TrafficLightGroupArray> recorded_fusion_results;
  for (const auto & recorded : recorded_frame_results) {
    const auto result = fusion.run(
      recorded.camera_info, recorded.result.selected_rois, recorded.result.merged_signals);
    if (!result) {
      std::cerr << "fusion event at "
                << rclcpp::Time(recorded.camera_info.header.stamp).nanoseconds()
                << " failed: " << result.error() << std::endl;
      continue;
    }
    recorded_fusion_results.push_back(*result);
  }
  return recorded_fusion_results;
}

// --- pass C: rosbag output -----------------------------------------------------------------------

// Writes every front-end and back-end result to `output_bag_path` under the configured production
// topic names. The input image/camera_info topics are not copied over, and neither are the cores'
// intermediate stages. Each message is written at its own header stamp (never wall-clock time),
// so the same dataset always produces the same bag.
void write_to_rosbag(
  const EvaluationConfig & config, const FusionEvaluationConfig & fusion_config,
  const std::string & output_bag_path,
  const std::vector<RecordedFrameResult> & recorded_frame_results,
  const std::vector<autoware_perception_msgs::msg::TrafficLightGroupArray> &
    recorded_fusion_results)
{
  autoware::traffic_light::evaluation::remove_output_bag_if_exists(output_bag_path);

  rosbag2_cpp::Writer writer;
  writer.open(
    {output_bag_path,
     autoware::traffic_light::evaluation::detect_input_bag_storage_id(config.input_bag_path)});

  for (const auto & recorded : recorded_frame_results) {
    const auto & camera = config.cameras[recorded.camera_index];
    const rclcpp::Time stamp(recorded.result.merged_signals.header.stamp);
    writer.write(recorded.result.merged_signals, camera.traffic_signals_topic, stamp);
    writer.write(recorded.result.selected_rois, camera.rois_topic, stamp);
  }
  for (const auto & fusion_result : recorded_fusion_results) {
    writer.write(fusion_result, fusion_config.output_topic, rclcpp::Time(fusion_result.stamp));
  }
}

// --- entry point --------------------------------------------------------------------------------

struct CommandLineArgs
{
  std::string config_path;
  std::string dataset_path;
  std::string output_bag_path;
};

CommandLineArgs parse_args(int argc, char ** argv)
{
  CommandLineArgs args;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--config" && i + 1 < argc) {
      args.config_path = argv[++i];
    } else if (arg == "--dataset" && i + 1 < argc) {
      args.dataset_path = argv[++i];
    } else if (arg == "--output-bag" && i + 1 < argc) {
      args.output_bag_path = argv[++i];
    }
  }
  if (args.config_path.empty() || args.dataset_path.empty() || args.output_bag_path.empty()) {
    throw std::runtime_error(
      "usage: run_traffic_light_pipeline_evaluation --config <path> --dataset <path> "
      "--output-bag <path>");
  }
  return args;
}

// Loads the evaluation config, runs the front-end then the back-end over every camera, and writes
// the results to `output_bag_path`. This is the whole of main()'s work, factored out so it can be
// driven without going through argv (e.g. from tests).
void run_evaluation(
  const std::string & config_path, const std::string & dataset_path,
  const std::string & output_bag_path)
{
  const auto config =
    autoware::traffic_light::evaluation::load_evaluation_config(config_path, dataset_path);
  const auto fusion_config = parse_fusion(YAML::LoadFile(config_path));
  const auto map_msg = autoware::traffic_light::evaluation::load_map(config);

  const auto recorded_frame_results = run_recognition(config, map_msg);
  const auto recorded_fusion_results = run_fusion(fusion_config, recorded_frame_results, map_msg);

  write_to_rosbag(
    config, fusion_config, output_bag_path, recorded_frame_results, recorded_fusion_results);
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto args = parse_args(argc, argv);
    run_evaluation(args.config_path, args.dataset_path, args.output_bag_path);
  } catch (const std::exception & e) {
    std::cerr << "run_traffic_light_pipeline_evaluation failed: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}

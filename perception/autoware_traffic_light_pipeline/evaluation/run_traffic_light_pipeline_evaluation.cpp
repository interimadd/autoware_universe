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

#include "evaluation_common.hpp"
#include "traffic_light_fusion/traffic_light_fusion.hpp"
#include "traffic_light_recognition/traffic_light_recognition.hpp"

#include <rclcpp/serialization.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_filter.hpp>

#include <sensor_msgs/msg/camera_info.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
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
// parsed alongside. `multi_camera_fusion.*` and `arbiter.*` are read from yaml -- the same two
// groups declare_fusion_config() declares as parameters in traffic_light_fusion_node.cpp -- while
// crosswalk_estimator is hardcoded to its production defaults below, mirroring that same function.
struct FusionEvaluationConfig
{
  std::string output_topic;
  TrafficLightFusionConfig fusion;
};

FusionEvaluationConfig parse_fusion(const YAML::Node & root)
{
  FusionEvaluationConfig config;
  const auto fusion_node = require(root, "fusion");
  config.output_topic = require(fusion_node, "output_topic").as<std::string>();

  const auto multi_camera_fusion_node = require(fusion_node, "multi_camera_fusion");
  config.fusion.multi_camera_fusion.message_lifespan =
    require(multi_camera_fusion_node, "message_lifespan").as<double>();
  config.fusion.multi_camera_fusion.prior_log_odds =
    require(multi_camera_fusion_node, "prior_log_odds").as<double>();

  // Fixed values, not parameters: mirrors declare_fusion_config()'s hardcoded
  // signal_consistency_check / crosswalk_estimator defaults (traffic_light_fusion_node.cpp).
  config.fusion.multi_camera_fusion.use_signal_consistency_check = false;
  config.fusion.multi_camera_fusion.publish_partial_matched_signal = false;

  const auto arbiter_node = require(fusion_node, "arbiter");
  config.fusion.arbiter.external_delay_tolerance =
    require(arbiter_node, "external_delay_tolerance").as<double>();
  config.fusion.arbiter.external_time_tolerance =
    require(arbiter_node, "external_time_tolerance").as<double>();
  config.fusion.arbiter.perception_time_tolerance =
    require(arbiter_node, "perception_time_tolerance").as<double>();
  config.fusion.arbiter.enable_signal_matching =
    require(arbiter_node, "enable_signal_matching").as<bool>();

  auto source_priority = require(arbiter_node, "source_priority").as<std::string>();
  if (
    source_priority != "external" && source_priority != "perception" &&
    source_priority != "confidence") {
    std::cerr << "evaluation config: unknown fusion.arbiter.source_priority '" << source_priority
              << "', defaulting to 'confidence'" << std::endl;
    source_priority = "confidence";
  }
  config.fusion.arbiter.source_priority = source_priority;

  config.fusion.crosswalk_estimator.use_last_detect_color = true;
  config.fusion.crosswalk_estimator.use_pedestrian_signal_detect = true;
  config.fusion.crosswalk_estimator.last_detect_color_hold_time = 2.0;
  config.fusion.crosswalk_estimator.flashing_detection.last_colors_hold_time = 1.0;

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

// --- pass A': optional arrival-order override ---------------------------------------------------

// Reads the publish order of `topic` out of `bag_path` and returns each message's own
// `stamp` field, in the order the bag stores them (i.e. the order the recorder received them).
//
// Why this exists: which camera's (camera_info, rois, signals) triple reaches
// multi_camera_fusion first within a cycle is an *input* to the back-end, not something the
// dataset determines. The two cameras run on separate Jetsons, so on a vehicle (and in a
// full-system webauto run) the winner varies cycle by cycle -- measured at 41% / 59% on
// x2 -- while this offline tool necessarily replays in one fixed order. With
// `message_lifespan` > the camera period both orders yield binocular fusion, so the choice
// only matters on the frames where the two cameras disagree; there it changes whether the
// other camera contributes its cycle-N or cycle-N-1 record, which can change the fused colour.
//
// Passing `--arrival-order-bag <full-system result_bag>` replays pass B in that run's real
// arrival order instead of stamp order, which is what makes an exact comparison against a
// specific full-system run possible. It is a cross-validation aid, deliberately NOT the
// default: without it the tool stays fully deterministic and self-contained, which is what a
// component test in CI needs.
std::vector<std::int64_t> load_arrival_order(
  const std::string & bag_path, const std::string & topic)
{
  rosbag2_cpp::Reader reader;
  reader.open({bag_path, autoware::traffic_light::evaluation::detect_input_bag_storage_id(bag_path)});

  rosbag2_storage::StorageFilter filter;
  filter.topics = {topic};
  reader.set_filter(filter);

  rclcpp::Serialization<autoware_perception_msgs::msg::TrafficLightGroupArray> serialization;
  std::vector<std::int64_t> order;
  while (reader.has_next()) {
    const auto bag_message = reader.read_next();
    rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
    autoware_perception_msgs::msg::TrafficLightGroupArray msg;
    serialization.deserialize_message(&serialized, &msg);
    order.push_back(rclcpp::Time(msg.stamp).nanoseconds());
  }
  return order;
}

// Reorders `recorded_frame_results` to follow `arrival_order` (a list of trigger stamps, see
// load_arrival_order()). Results whose stamp does not appear in the reference run keep their
// stamp-ordered position at the end, so a reference bag covering only part of the dataset still
// works. Logs how many results were matched, since a low count means the reference bag does not
// correspond to this dataset.
void apply_arrival_order(
  std::vector<RecordedFrameResult> & recorded_frame_results,
  const std::vector<std::int64_t> & arrival_order)
{
  std::unordered_map<std::int64_t, std::size_t> position_of_stamp;
  for (std::size_t i = 0; i < arrival_order.size(); ++i) {
    // First occurrence wins: a stamp identifies one camera's one cycle, so it should appear once.
    position_of_stamp.emplace(arrival_order[i], i);
  }

  const auto unmatched_position = arrival_order.size();
  std::size_t matched = 0;
  std::vector<std::pair<std::size_t, RecordedFrameResult>> keyed;
  keyed.reserve(recorded_frame_results.size());
  for (const auto & recorded : recorded_frame_results) {
    const auto stamp = rclcpp::Time(recorded.camera_info.header.stamp).nanoseconds();
    const auto it = position_of_stamp.find(stamp);
    if (it != position_of_stamp.end()) {
      ++matched;
      keyed.emplace_back(it->second, recorded);
    } else {
      keyed.emplace_back(unmatched_position, recorded);
    }
  }

  std::stable_sort(
    keyed.begin(), keyed.end(),
    [](const auto & lhs, const auto & rhs) { return lhs.first < rhs.first; });

  for (std::size_t i = 0; i < keyed.size(); ++i) {
    recorded_frame_results[i] = std::move(keyed[i].second);
  }
  std::cerr << "arrival-order override: matched " << matched << " of "
            << recorded_frame_results.size() << " results against " << arrival_order.size()
            << " reference messages" << std::endl;
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
  // Optional; see load_arrival_order().
  std::string arrival_order_bag_path;
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
    } else if (arg == "--arrival-order-bag" && i + 1 < argc) {
      args.arrival_order_bag_path = argv[++i];
    }
  }
  if (args.config_path.empty() || args.dataset_path.empty() || args.output_bag_path.empty()) {
    throw std::runtime_error(
      "usage: run_traffic_light_pipeline_evaluation --config <path> --dataset <path> "
      "--output-bag <path> [--arrival-order-bag <path>]");
  }
  return args;
}

// Loads the evaluation config, runs the front-end then the back-end over every camera, and writes
// the results to `output_bag_path`. This is the whole of main()'s work, factored out so it can be
// driven without going through argv (e.g. from tests).
void run_evaluation(
  const std::string & config_path, const std::string & dataset_path,
  const std::string & output_bag_path, const std::string & arrival_order_bag_path)
{
  const auto config =
    autoware::traffic_light::evaluation::load_evaluation_config(config_path, dataset_path);
  const auto fusion_config = parse_fusion(YAML::LoadFile(config_path));
  const auto map_msg = autoware::traffic_light::evaluation::load_map(config);

  auto recorded_frame_results = run_recognition(config, map_msg);
  if (!arrival_order_bag_path.empty()) {
    apply_arrival_order(
      recorded_frame_results,
      load_arrival_order(
        arrival_order_bag_path, fusion_config.output_topic));
  }
  const auto recorded_fusion_results = run_fusion(fusion_config, recorded_frame_results, map_msg);

  write_to_rosbag(
    config, fusion_config, output_bag_path, recorded_frame_results, recorded_fusion_results);
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto args = parse_args(argc, argv);
    run_evaluation(
      args.config_path, args.dataset_path, args.output_bag_path, args.arrival_order_bag_path);
  } catch (const std::exception & e) {
    std::cerr << "run_traffic_light_pipeline_evaluation failed: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}

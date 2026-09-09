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

// Shared building blocks for this package's offline evaluation executables
// (run_traffic_light_recognition_evaluation and run_traffic_light_pipeline_evaluation): evaluation
// config yaml helpers/parsing, dataset rosbag loading (frames + tf buffer), the lanelet2 map
// loader, and output-bag storage-id detection / overwrite guards. Splitting this out of
// run_traffic_light_recognition_evaluation.cpp (its original home) keeps both executables from
// carrying two copies of code that has nothing to do with which pipeline stage each one drives.

#ifndef PERCEPTION__AUTOWARE_TRAFFIC_LIGHT_PIPELINE__EVALUATION__EVALUATION_COMMON_HPP_
#define PERCEPTION__AUTOWARE_TRAFFIC_LIGHT_PIPELINE__EVALUATION__EVALUATION_COMMON_HPP_

#include "common/config_yaml.hpp"
#include "traffic_light_recognition/traffic_light_recognition.hpp"

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <tf2/buffer_core.h>
#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::traffic_light::evaluation
{

// --- yaml helpers -------------------------------------------------------------------------------

// Returns `node[key]`, throwing if it is missing -- an evaluation config field that has no default
// in the package config (see load_evaluation_config()) is required.
YAML::Node require(const YAML::Node & node, const std::string & key);

// Expands a leading "~" to $HOME, the same convention the shell and ROS2 launch's $(env HOME)
// support, so evaluation config yaml files can reference models under the user's home directory
// without hardcoding an absolute path.
std::string expand_user_path(const std::string & path);

// require(node, key).as<std::string>(), with expand_user_path() applied -- the common case for
// every model_path / label_path field.
std::string require_path(const YAML::Node & node, const std::string & key);

// --- evaluation config (yaml) --------------------------------------------------------------------

// One camera's worth of evaluation configuration: which topics it is read from, which topics its
// front-end results are written to, and the map->camera tf sampling window each camera's
// TrafficLightRecognition instance is built with. The window is per camera because the instances
// are, not because the evaluation yaml can vary it -- load_evaluation_config() fills every camera's
// from the same package config values.
struct CameraConfig
{
  std::string ns;
  std::string camera_info_topic;
  std::string compressed_image_topic;
  std::string traffic_signals_topic;
  std::string rois_topic;
  double min_timestamp_offset = 0.0;
  double max_timestamp_offset = 0.0;
};

// Parses one `cameras[]` entry. `map_based_detector_defaults` is the package config's
// `map_based_detector:` group, used for whichever of the two offsets the entry does not override.
CameraConfig parse_camera(const YAML::Node & node, const YAML::Node & map_based_detector_defaults);

// Fills every TrafficLightRecognitionConfig field except min/max_timestamp_offset, which is per
// camera (see CameraConfig). `node` is the evaluation yaml's `recognition:` section already
// merged over config/traffic_light_recognition.param.yaml (see load_evaluation_config()), so it
// carries the package's thresholds wherever the evaluation yaml stays silent.
TrafficLightRecognitionConfig parse_recognition(const YAML::Node & node);

// The whole evaluation run: N cameras sharing one set of models/thresholds, over one dataset.
struct EvaluationConfig
{
  // Derived from --dataset, not from the yaml (fixed dataset layout, see each executable's own
  // usage comment).
  std::string input_bag_path;
  std::string lanelet2_map_path;
  std::string map_projector_info_path;

  std::vector<CameraConfig> cameras;
  // Every field except min/max_timestamp_offset, which CameraConfig carries per camera.
  TrafficLightRecognitionConfig recognition;
};

// Loads `config_path` and resolves the dataset-derived paths against `dataset_path`.
//
// Every tuned value comes from this package's own config/traffic_light_recognition.param.yaml --
// the same file the launch files feed the Node -- and never from the evaluation yaml, which only
// supplies what is inherently per-run: the camera list, topic names and model/label paths. An
// evaluation therefore always measures the deployed configuration; it cannot state a threshold of
// its own and silently drift from what production uses.
//
// Does not parse a `fusion:` section -- callers that need the back-end config parse it themselves
// from the same root node, reading config/traffic_light_fusion.param.yaml the same way (see
// run_traffic_light_pipeline_evaluation.cpp).
EvaluationConfig load_evaluation_config(
  const std::string & config_path, const std::string & dataset_path);

// --- rosbag input
// ---------------------------------------------------------------------------------

// One exact-stamp matched (image, camera_info) pair, tagged with the camera it came from -- the
// same input unit the Node's message_filters::ExactTime sync hands to run(). `image` is kept
// compressed, as it was read from the bag: decoding every frame up front, for the whole bag, is
// what made load_frames() the dominant memory cost of an evaluation run. Call
// decode_frame_image() instead, right before handing the frame to the pipeline.
struct Frame
{
  std::size_t camera_index;
  sensor_msgs::msg::CompressedImage image;
  sensor_msgs::msg::CameraInfo camera_info;
};

// Reads every configured camera's image/camera_info topics out of `config.input_bag_path` and
// returns the exact-stamp matched frames, in ascending (stamp, camera_index) order. Messages with
// no same-stamp partner on the other topic are dropped -- the same policy
// message_filters::ExactTime enforces in production. Images are not decoded here; see Frame and
// decode_frame_image().
//
// Buffers every camera's frames in memory at once, which is the right tradeoff for callers that
// must replay every camera in one globally stamp-ordered pass
// (run_traffic_light_pipeline_evaluation, whose back-end stage is stateful and needs that order)
// but does not scale as the number of cameras grows. A caller that drives each camera through an
// independent, per-camera-stateless pipeline stage -- as run_traffic_light_recognition_evaluation's
// front-end-only run does -- should use load_frames_for_camera() instead, one camera at a time.
std::vector<Frame> load_frames(const EvaluationConfig & config);

// Reads only `config.cameras[camera_index]`'s image/camera_info topics out of
// `config.input_bag_path` and returns that camera's exact-stamp matched frames, in ascending
// stamp order (every Frame::camera_index is `camera_index`). Same matching policy as load_frames()
// (unmatched messages dropped), but this reads and buffers a single camera's messages at a time --
// call it once per camera, process that camera's frames, then let them go out of scope before
// moving to the next camera, so memory stays proportional to one camera's frame count rather than
// the whole dataset's.
std::vector<Frame> load_frames_for_camera(
  const EvaluationConfig & config, std::size_t camera_index);

// Decodes `frame.image`, returning the plain image the pipeline consumes. Meant to be called right
// before that -- one frame at a time, as it is about to be processed -- rather than while
// load_frames() buffers the whole bag, so at most one decoded image is ever held in memory. Returns
// std::nullopt (after logging to stderr) if the image fails to decompress.
std::optional<sensor_msgs::msg::Image> decode_frame_image(const Frame & frame);

// The Node gets its map->camera transforms from a tf2_ros::TransformListener; offline they all
// come from the dataset bag instead. The cache time is deliberately far longer than
// tf2::BufferCore's 10 s default: the whole bag's transforms must stay resolvable for the whole
// run, since frames are processed after the bag has been fully read.
std::unique_ptr<tf2::BufferCore> load_transform_buffer(const std::string & bag_path);

// Loads `config.lanelet2_map_path` (must be an MGRS-projected map, per
// `config.map_projector_info_path`) as a LaneletMapBin.
autoware_map_msgs::msg::LaneletMapBin load_map(const EvaluationConfig & config);

// --- rosbag output
// ---------------------------------------------------------------------------------

// The output bag is always written in the same storage format as the input bag: open the input
// bag with rosbag2_cpp's auto-detecting Reader and read back whichever storage plugin id it
// detected from the bag's own metadata.yaml.
std::string detect_input_bag_storage_id(const std::string & bag_path);

// Guards remove_output_bag_if_exists() against deleting anything that is not actually a rosbag2
// bag directory: every entry directly inside it must be a rosbag2 storage file (.db3 / .mcap) or
// a yaml file (metadata.yaml). Throws if a stray file is found, so a mistyped --output-bag path
// never silently wipes out an unrelated directory.
void check_only_contains_rosbag_files(const std::string & bag_path);

// Removes `output_bag_path` if it already exists, so a Writer can always create it fresh. Refuses
// to remove anything that does not look like a rosbag2 bag directory (see
// check_only_contains_rosbag_files()), to guard against a mistyped path deleting unrelated data.
void remove_output_bag_if_exists(const std::string & output_bag_path);

}  // namespace autoware::traffic_light::evaluation

#endif  // PERCEPTION__AUTOWARE_TRAFFIC_LIGHT_PIPELINE__EVALUATION__EVALUATION_COMMON_HPP_

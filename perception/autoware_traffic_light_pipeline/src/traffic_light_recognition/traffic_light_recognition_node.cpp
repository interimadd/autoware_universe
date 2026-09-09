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

#include "traffic_light_recognition_node.hpp"

#include "common/node_parameter_source.hpp"
#include "recognition_config_builder.hpp"

#include <memory>
#include <utility>

namespace autoware::traffic_light
{
namespace
{

// Declares this package's ROS 2 parameters on `node` and returns the resulting (flat)
// TrafficLightRecognitionConfig the ROS-free TrafficLightRecognition core consumes. Declaring
// parameters is a Node concern -- the core itself never touches rclcpp -- and NodeParameterSource
// is exactly that concern and nothing else: which parameter name fills which config field lives
// in build_recognition_config() (recognition_config_builder.hpp), shared with the offline
// evaluation tools and this package's tests so the three cannot drift apart.
TrafficLightRecognitionConfig declare_recognition_config(rclcpp::Node * node)
{
  NodeParameterSource source(node);
  return build_recognition_config(source);
}
}  // namespace

TrafficLightRecognitionNode::TrafficLightRecognitionNode(const rclcpp::NodeOptions & node_options)
: Node("traffic_light_recognition", node_options),
  config_(declare_recognition_config(this)),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // Subscribers -----------------------------------------------------------------------------
  vector_map_sub_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrafficLightRecognitionNode::vector_map_callback, this, std::placeholders::_1));
  route_sub_ = create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
    "~/input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrafficLightRecognitionNode::route_callback, this, std::placeholders::_1));

  image_sub_.subscribe(this, "~/input/image", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  camera_info_sub_.subscribe(
    this, "~/input/camera_info", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  sync_ = std::make_unique<Sync>(SyncPolicy(10), image_sub_, camera_info_sub_);
  sync_->registerCallback(
    std::bind(
      &TrafficLightRecognitionNode::sync_callback, this, std::placeholders::_1,
      std::placeholders::_2));

  // Publishers --------------------------------------------------------------------------------
  signals_pub_ = create_publisher<tier4_perception_msgs::msg::TrafficLightArray>(
    "~/output/traffic_signals", rclcpp::QoS{1});
  rois_pub_ = create_publisher<tier4_perception_msgs::msg::TrafficLightRoiArray>(
    "~/output/rois", rclcpp::QoS{1});
  // Publishes camera_info itself (unchanged) so that traffic_light_camera_info_relay can be
  // retired once every consumer moves to this Node's output (plan §4.1).
  camera_info_pub_ =
    create_publisher<sensor_msgs::msg::CameraInfo>("~/output/camera_info", rclcpp::QoS{1});

  if (declare_parameter<bool>("build_only")) {
    build_engines_and_shutdown();
  }
}

void TrafficLightRecognitionNode::build_engines_and_shutdown()
{
  RCLCPP_INFO(get_logger(), "build_only: building TensorRT engines and exiting.");
  try {
    build_engines(config_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "failed to build TensorRT engines: %s", e.what());
  }
  rclcpp::shutdown();
}

void TrafficLightRecognitionNode::vector_map_callback(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
{
  // transient_local + a single publish from the map loader: this fires exactly once (plan §4.3
  // step 2). The TensorRT engines inside `config_` (whole-image detector, car/pedestrian
  // classifiers) are built here too, since TrafficLightRecognition's constructor builds every
  // core it owns.
  recognition_ = std::make_unique<TrafficLightRecognition>(config_, *msg, tf_buffer_);

  if (pending_route_) {
    const auto error = recognition_->set_route(*pending_route_);
    if (error) {
      RCLCPP_ERROR(get_logger(), "%s", error->message.c_str());
    }
    pending_route_.reset();
  }
}

void TrafficLightRecognitionNode::route_callback(
  const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg)
{
  if (!recognition_) {
    // The vector map has not arrived yet: stash the route and apply it once the core is built
    // (plan §4.3 step 3).
    pending_route_ = msg;
    return;
  }
  const auto error = recognition_->set_route(*msg);
  if (error) {
    RCLCPP_ERROR(get_logger(), "%s", error->message.c_str());
  }
}

void TrafficLightRecognitionNode::sync_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
  if (!recognition_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "vector map not received yet: dropping frame");
    return;
  }

  const auto result = recognition_->run(*image_msg, *camera_info_msg);
  if (!result) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "run() failed: %s", result.error().c_str());
    return;
  }

  signals_pub_->publish(result->merged_signals);
  rois_pub_->publish(result->selected_rois);
  camera_info_pub_->publish(*camera_info_msg);
}

}  // namespace autoware::traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::TrafficLightRecognitionNode)

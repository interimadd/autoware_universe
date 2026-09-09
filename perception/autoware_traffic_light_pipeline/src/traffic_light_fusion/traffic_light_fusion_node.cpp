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

#include "traffic_light_fusion_node.hpp"

#include "common/node_parameter_source.hpp"
#include "fusion_config_builder.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::traffic_light
{
namespace
{

// Declares this Node's ROS 2 parameters on `node` and returns the resulting
// TrafficLightFusionConfig the ROS-free TrafficLightFusion core consumes. Which parameter name
// fills which config field lives in build_fusion_config() (fusion_config_builder.hpp), shared with
// run_traffic_light_pipeline_evaluation so the two cannot drift apart; NodeParameterSource is the
// production half of that -- it declares the parameters, which is what keeps them visible to
// launch, `ros2 param` and this package's json schema.
TrafficLightFusionConfig declare_fusion_config(rclcpp::Node * node)
{
  NodeParameterSource source(node);
  return build_fusion_config(source, [node](const std::string & message) {
    RCLCPP_WARN(node->get_logger(), "%s", message.c_str());
  });
}
}  // namespace

TrafficLightFusionNode::TrafficLightFusionNode(const rclcpp::NodeOptions & node_options)
: Node("traffic_light_fusion", node_options), config_(declare_fusion_config(this))
{
  // Subscribers -----------------------------------------------------------------------------
  vector_map_sub_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrafficLightFusionNode::vector_map_callback, this, std::placeholders::_1));

  // Per-camera input topic names are derived from the namespace exactly as MultiCameraFusionNode
  // derives them, so this Node drops into production's topic graph unchanged.
  const auto camera_namespaces = declare_parameter<std::vector<std::string>>("camera_namespaces");
  for (const auto & camera_ns : camera_namespaces) {
    CameraSubscription subscription;
    subscription.camera_info_sub = std::make_unique<message_filters::Subscriber<CameraInfo>>(
      this, camera_ns + "/camera_info", rclcpp::SensorDataQoS().get_rmw_qos_profile());
    subscription.rois_sub = std::make_unique<message_filters::Subscriber<RoiArray>>(
      this, camera_ns + "/detection/rois", rclcpp::QoS{1}.get_rmw_qos_profile());
    subscription.signals_sub = std::make_unique<message_filters::Subscriber<SignalArray>>(
      this, camera_ns + "/classification/traffic_signals", rclcpp::QoS{1}.get_rmw_qos_profile());
    subscription.sync = std::make_unique<Sync>(
      SyncPolicy(10), *subscription.camera_info_sub, *subscription.rois_sub,
      *subscription.signals_sub);
    subscription.sync->registerCallback(
      std::bind(
        &TrafficLightFusionNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3));
    camera_subscriptions_.push_back(std::move(subscription));
  }

  // Publishers --------------------------------------------------------------------------------
  traffic_signals_pub_ =
    create_publisher<TrafficLightGroupArray>("~/output/traffic_signals", rclcpp::QoS{1});
}

void TrafficLightFusionNode::vector_map_callback(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
{
  // transient_local + a single publish from the map loader: this fires exactly once. Rebuilding
  // the core here (rather than calling a setter on it) is what production's three Nodes do too --
  // MultiCameraFusionNode reconstructs its MultiCameraFusion on every map message -- so a second
  // map would reset all three cores' accumulated history, exactly as it does today.
  fusion_ = std::make_unique<TrafficLightFusion>(config_, *msg);
}

void TrafficLightFusionNode::sync_callback(
  const CameraInfo::ConstSharedPtr & camera_info_msg, const RoiArray::ConstSharedPtr & rois_msg,
  const SignalArray::ConstSharedPtr & signals_msg)
{
  if (!fusion_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "vector map not received yet: dropping frame");
    return;
  }

  const auto result = fusion_->run(*camera_info_msg, *rois_msg, *signals_msg);
  if (!result) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "run() failed: %s", result.error().c_str());
    return;
  }

  traffic_signals_pub_->publish(*result);
}

}  // namespace autoware::traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::TrafficLightFusionNode)

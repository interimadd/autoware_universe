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

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::traffic_light
{
namespace
{

// Declares this Node's ROS 2 parameters on `node` and returns the resulting
// TrafficLightFusionConfig the ROS-free TrafficLightFusion core consumes. `multi_camera_fusion.*`
// and `arbiter.*` are exposed as parameters, each under the prefix of the component it belongs to
// (mirroring that component's own Node's parameter names) since one Node now carries what used to
// be three Nodes' worth of configuration and a flat name could collide with a future one.
// crosswalk_estimator's values are hardcoded here instead of declared: no deployment has ever
// needed to change its behavior from its fixed defaults. `source_priority` is normalized here
// exactly as TrafficLightArbiterNode normalizes it, so the core never has to guard against a typo.
//
// `camera_namespaces` is not part of the config struct: it selects which cameras this Node
// subscribes to, which is Node I/O, not core configuration. It is read separately by the Node.
TrafficLightFusionConfig declare_fusion_config(rclcpp::Node * node)
{
  TrafficLightFusionConfig config;

  // These two are the ones MultiCameraFusionNode itself declares and every x2 deployment overrides
  // (see config/traffic_light_fusion.param.yaml for why message_lifespan must exceed the camera
  // period), so they are real parameters rather than hardcoded values.
  config.multi_camera_fusion.message_lifespan =
    node->declare_parameter<double>("multi_camera_fusion.message_lifespan");
  config.multi_camera_fusion.prior_log_odds =
    node->declare_parameter<double>("multi_camera_fusion.prior_log_odds");
  // Not declared: MultiCameraFusionNode declares these two under `signal_consistency_check.*`, but
  // no x2 param file sets either, so both keep the package default (disabled).
  config.multi_camera_fusion.use_signal_consistency_check = false;
  config.multi_camera_fusion.publish_partial_matched_signal = false;
  // lanelet_map_ptr is deliberately left null: TrafficLightFusion's constructor fills it in from
  // the LaneletMapBin the Node receives on ~/input/vector_map.

  config.arbiter.external_delay_tolerance =
    node->declare_parameter<double>("arbiter.external_delay_tolerance");
  config.arbiter.external_time_tolerance =
    node->declare_parameter<double>("arbiter.external_time_tolerance");
  config.arbiter.perception_time_tolerance =
    node->declare_parameter<double>("arbiter.perception_time_tolerance");
  config.arbiter.enable_signal_matching =
    node->declare_parameter<bool>("arbiter.enable_signal_matching");

  // Same normalization TrafficLightArbiterNode does, for the same reason: the core treats every
  // unknown value as "confidence" silently, so the typo is caught (and warned about) here.
  auto source_priority = node->declare_parameter<std::string>("arbiter.source_priority");
  if (
    source_priority != "external" && source_priority != "perception" &&
    source_priority != "confidence") {
    RCLCPP_WARN(
      node->get_logger(), "Unknown arbiter.source_priority '%s', defaulting to 'confidence'",
      source_priority.c_str());
    source_priority = "confidence";
  }
  config.arbiter.source_priority = source_priority;

  // Fixed values, not parameters: these are autoware_crosswalk_traffic_light_estimator's own
  // package defaults, and no deployment has ever needed to change them.
  config.crosswalk_estimator.use_last_detect_color = true;
  config.crosswalk_estimator.use_pedestrian_signal_detect = true;
  config.crosswalk_estimator.last_detect_color_hold_time = 2.0;
  config.crosswalk_estimator.flashing_detection.last_colors_hold_time = 1.0;

  return config;
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

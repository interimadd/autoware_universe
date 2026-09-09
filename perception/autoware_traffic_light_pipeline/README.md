# autoware_traffic_light_pipeline

Single-node compositions of the traffic light recognition pipeline, one per half:

```text
# front-end (前段), src/traffic_light_recognition/ -- one instance per camera
map_based_detector -> whole_image_detector(yolox) -> selector
  -> car_classifier / pedestrian_classifier -> category_merger

# back-end (後段), src/traffic_light_fusion/ -- one instance for all cameras
multi_camera_fusion -> arbiter -> crosswalk_traffic_light_estimator
```

The front-end composes, for one camera, the ROS-free core logic already extracted into
`autoware_tensorrt_yolox`, `autoware_traffic_light_map_based_detector`,
`autoware_traffic_light_selector`, `autoware_traffic_light_classifier`, and
`autoware_traffic_light_category_merger` into a single `rclcpp::Node`, in place of the 5-Node
graph production currently launches per camera. The `fine_detection` path
(`traffic_light_fine_detector` + `traffic_light_occlusion_predictor`) is out of scope.

The back-end composes the ROS-free core logic of
`autoware_traffic_light_multi_camera_fusion`, `autoware_traffic_light_arbiter`, and
`autoware_crosswalk_traffic_light_estimator` into a single `rclcpp::Node`, in place of the 3
Nodes production currently launches once per vehicle. It is a port of
`autoware_traffic_light_component_test`'s `TrafficLightFusionPipeline`.

The two halves are independent Nodes (the back-end is camera-crossing and stateful, the front-end
is per-camera and stateless), so the package ships four libraries -- a ROS-free core plus a Node
adapter for each half:

- `autoware_traffic_light_recognition` -- the ROS-free front-end core (`TrafficLightRecognition`).
- `autoware_traffic_light_recognition_node` -- the front-end Node adapter
  (`TrafficLightRecognitionNode`), registered as an `rclcpp_components` plugin.
- `autoware_traffic_light_fusion` -- the ROS-free back-end core (`TrafficLightFusion`).
- `autoware_traffic_light_fusion_node` -- the back-end Node adapter (`TrafficLightFusionNode`),
  registered as an `rclcpp_components` plugin.

## Front-end Node (`traffic_light_recognition`)

### Input / Output

| Direction  | Topic                      | Type                                                        |
| ---------- | -------------------------- | ----------------------------------------------------------- |
| Subscribed | `~/input/image`            | `sensor_msgs/msg/Image` (already decoded)                   |
| Subscribed | `~/input/camera_info`      | `sensor_msgs/msg/CameraInfo`                                |
| Subscribed | `~/input/vector_map`       | `autoware_map_msgs/msg/LaneletMapBin` (transient_local)     |
| Subscribed | `~/input/route`            | `autoware_planning_msgs/msg/LaneletRoute` (transient_local) |
| Published  | `~/output/traffic_signals` | `tier4_perception_msgs/msg/TrafficLightArray`               |
| Published  | `~/output/rois`            | `tier4_perception_msgs/msg/TrafficLightRoiArray`            |
| Published  | `~/output/camera_info`     | `sensor_msgs/msg/CameraInfo` (republished unchanged)        |

`image` and `camera_info` are synchronized with `message_filters::sync_policies::ExactTime`
(same camera driver, so stamps are expected to match exactly; `approximate_sync` is intentionally
not configurable). `camera_info` is republished unchanged so a `topic_tools relay` node is not
needed downstream.

### Initialization order

The composed core's constructor requires the vector map (`TrafficLightMapBasedDetector`'s
constructor takes a `LaneletMapBin`), so the Node cannot build it at construction time:

1. Constructor: declare parameters, build the config, create the tf buffer/listener and every
   subscriber/publisher.
2. `~/input/vector_map` (transient_local, expected once): build `TrafficLightRecognition`. This
   is also where the whole-image detector's and both classifiers' TensorRT engines get built.
3. `~/input/route`: call `set_route()`. If the map has not arrived yet, the route is held and
   applied once the core is built.
4. Synchronized `(image, camera_info)`: if the core is not yet built, warn (throttled) and drop
   the frame.

This means TensorRT engine construction does not start until the vector map arrives, unlike the
per-stage Nodes it replaces (which each build their own engine in their own constructor). In
`logging_simulator` usage the vector map arrives immediately, so this is not expected to matter in
practice; `build_only` (below) exists for cases where it does.

### `build_only`

When `build_only:=true`, the whole-image detector's and both classifiers' TensorRT engines are
built directly (without constructing `TrafficLightRecognition`, so the vector map is not needed)
and the Node exits. This lets a CI engine-prebuild step target this Node the same way it targets
the existing per-stage Nodes.

## Front-end parameters

See [config/traffic_light_recognition.param.yaml](config/traffic_light_recognition.param.yaml)
and [schema/traffic_light_recognition.schema.json](schema/traffic_light_recognition.schema.json).

Several things are true of this package's parameter surface, all deliberate:

- **`classifier_type` is fixed to CNN.** Both `car_classifier` and `pedestrian_classifier` always
  use `autoware::traffic_light::CNNClassifier`; there is no `classifier_type` parameter (HSV /
  lamp-recognizer backends are not composed here). A future backend would add the parameter back
  at that point rather than exposing a knob with only one valid value today.
- **Semantic-segmentation parameters are not exposed.** The traffic-light yolox model has no
  segmentation head, so `is_roi_overlap_segmentation` / `is_publish_color_mask` /
  `overlap_roi_score_threshold` / the semseg color map are fixed internally rather than declared
  as parameters.
- **Only fp16 models are supported.** `precision` (for both the whole-image detector and both
  classifiers) is fixed to `"fp16"` in source rather than declared as a parameter. The int8-only
  knobs that would otherwise go with it (`calibration_algorithm` / `dla_core_id` /
  `quantize_first_layer` / `quantize_last_layer` / `clip_value` / `calibration_image_list_path`)
  and the dev-only `profile_per_layer` are likewise fixed. Supporting another precision would mean
  reintroducing these as parameters at that point.
- **Classifier input normalization (`mean` / `std`) is fixed.** These are per-channel statistics
  the classifier model was trained with, not a per-deployment tuning knob, and both car and
  pedestrian classifier models use the same ImageNet-style values. They are fixed internally
  rather than declared as parameters; a future model trained with different normalization would
  reintroduce them at that point.
- **`traffic_light_type` is not a parameter.** Which ROIs a classifier instance classifies
  (`CAR_TRAFFIC_LIGHT` vs. `PEDESTRIAN_TRAFFIC_LIGHT`) is fixed by which classifier the value is
  built for, not by anything read from the parameter tree, so a parameter would only ever restate
  that choice.
- **`over_exposure_threshold` / `under_exposure_threshold` live under a single shared
  `classifier` prefix, not `car_classifier` / `pedestrian_classifier`.** `car_classifier` and
  `pedestrian_classifier` classify ROIs cropped from the same camera image at the same timestamp,
  so exposure -- a property of the camera/ISP, not of which traffic light type a given ROI
  happens to be -- is the same for both. Unlike `mean` / `std` (a model property, fixed above) or
  the map_based_detector cutoffs (no established per-deployment override), this one plausibly
  does vary between camera models, so it stays a parameter; it just no longer needs to be
  declared and set identically under two different prefixes. `classifier` is its own top-level
  section (like `whole_image_detector` / `map_based_detector`) purely to keep the config file's
  nesting depth consistent, even though only this one classifier-shaped block exists.
- **Only `map_based_detector.min_timestamp_offset` / `max_timestamp_offset` are parameters.** The
  calibration-error margins (`max_vibration_pitch` / `max_vibration_yaw` / `max_vibration_height`
  / `max_vibration_width` / `max_vibration_depth`) and the range/angle cutoffs
  (`max_detection_range` / `car_traffic_light_max_angle_range` /
  `pedestrian_traffic_light_max_angle_range`) are fixed internally: unlike the timestamp offsets
  (genuinely per-camera, see below), none of these have an established per-vehicle or per-camera
  override in practice.
- **`gpu_id` is not exposed.** The whole-image detector always runs on the default CUDA device;
  this package does not need per-node GPU selection. (The classifier backend
  (`autoware_tensorrt_classifier`) has no `gpu_id` concept at all, always using the default
  device -- this is not an omission on this package's part.)
- **`model_path` / `label_path` / `roi_remap_path` are not in `config/traffic_light_recognition.param.yaml`.**
  They are injected as separate top-level parameters (see
  [launch/traffic_light_recognition.launch.xml](launch/traffic_light_recognition.launch.xml)) so
  the versioned config file never hard-codes a `$HOME/autoware_data`-relative path.

### Where each value lives

Which parameter name fills which field lives in one place:
`build_recognition_config(Source &)`
([recognition_config_builder.hpp](src/traffic_light_recognition/recognition_config_builder.hpp)).
It reads exactly the values listed above (plus the launch-injected paths) into a deliberately flat
`TrafficLightRecognitionConfig`
([traffic_light_recognition.hpp](src/traffic_light_recognition/traffic_light_recognition.hpp)) --
one struct with no nested per-core config types. It never touches a model or label file, and
knows nothing about precision, mean/std, `gpu_id`, or any other fixed value above.

`Source` is whatever supplies values by parameter name, so the same mapping serves both callers:

| Caller                                               | `Source`                                                                                                                                                                                                                              |
| ---------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `TrafficLightRecognitionNode` (production)           | `NodeParameterSource` ([node_parameter_source.hpp](src/common/node_parameter_source.hpp)) -- **declares** each parameter, which is what keeps it visible to launch, `--params-file`/`-p`, `ros2 param` and this package's json schema |
| offline evaluation, `test_traffic_light_recognition` | `autoware::component_test_framework::ParameterLoader` -- reads the same `config/*.param.yaml` with the same rcl yaml parser, with no `rclcpp::Node`                                                                                   |

`test_config_builders_cross_check` pins the two paths to the same result.

Every fixed value, and the label-file reads and nested `TrtYoloXDetectorConfig` /
`TrafficLightMapBasedDetectorConfig` / `CNNConfig` construction that goes with them, live in
`traffic_light_recognition.cpp` instead: `TrafficLightRecognition`'s constructor and the
`build_engines()` free function (used by `build_only`, below) both build those nested configs from
the flat `TrafficLightRecognitionConfig` via the same internal helpers. This keeps the ROS-free
core the single place that knows how the pipeline's cores are actually configured; the Node layer
only ever sees the flat, parameter-sourced config.

### Per-camera deployment

In `autoware_launch`, this package's config is expected to be split into a shared file (this
package's default, camera-independent) plus a small per-camera override file carrying only the
keys that actually differ between cameras (in practice, just
`map_based_detector.min_timestamp_offset`) -- not a full per-camera copy of every key. That split
is `autoware_launch`'s responsibility, not this package's; this package only ships the
camera-independent default.

## Back-end Node (`traffic_light_fusion`)

### Input / Output

| Direction  | Topic                                        | Type                                                    |
| ---------- | -------------------------------------------- | ------------------------------------------------------- |
| Subscribed | `<camera_ns>/camera_info`                    | `sensor_msgs/msg/CameraInfo`                            |
| Subscribed | `<camera_ns>/detection/rois`                 | `tier4_perception_msgs/msg/TrafficLightRoiArray`        |
| Subscribed | `<camera_ns>/classification/traffic_signals` | `tier4_perception_msgs/msg/TrafficLightArray`           |
| Subscribed | `~/input/vector_map`                         | `autoware_map_msgs/msg/LaneletMapBin` (transient_local) |
| Published  | `~/output/traffic_signals`                   | `autoware_perception_msgs/msg/TrafficLightGroupArray`   |

One `(camera_info, rois, traffic_signals)` triple is subscribed and `ExactTime`-synchronized per
entry of the `camera_namespaces` parameter, with the topic names derived from the namespace
exactly as `traffic_light_multi_camera_fusion` derives them -- so the Node drops into production's
topic graph unchanged, and the three topics are exactly what the front-end Node publishes for that
camera. `MultiCameraFusionNode`'s `approximate_sync` parameter is intentionally not carried over:
all three inputs of one camera come from that camera's single frame, so their stamps match
exactly.

Each synchronized triple runs the whole chain: `multi_camera_fusion.fuse()` ->
`arbiter.ingest_perception()` + `arbiter.arbitrate(camera_info.header.stamp)` ->
`crosswalk_traffic_light_estimator.estimate()`, and the estimator's output is published. This
mirrors production's per-trigger callback chain, where each Node publishes and the next one's
subscription fires.

The core never reads the clock: every timestamp it acts on comes from the trigger's own
`camera_info.header.stamp`, so the same input sequence always produces the same output sequence.
The intermediate signals (fusion's, arbiter's) and the diagnostics production's Nodes emit
(unmapped / conflicted / off-map / unregistered ids) are not published: production only ever logs
them as warnings.

**Out of scope: external (V2X) traffic signals.** `TrafficLightArbiterNode`'s
`~/sub/external_traffic_signals` subscription and `TrafficLightArbiter::ingest_external()` have no
counterpart here -- arbitration always runs on perception input alone -- the same way they have
none in the Component Test this is ported from.

### Initialization order

Like the front-end, the composed core's constructor requires the vector map (all three back-end
cores need it), so the Node cannot build it at construction time:

1. Constructor: declare parameters, build the config, create every subscriber/publisher.
2. `~/input/vector_map` (transient_local, expected once): build `TrafficLightFusion`. A second map
   message rebuilds the core, which resets all three cores' accumulated history -- exactly what
   `MultiCameraFusionNode` does today.
3. Synchronized `(camera_info, rois, traffic_signals)`: if the core is not yet built, warn
   (throttled) and drop the frame.

### Back-end parameters

[config/traffic_light_fusion.param.yaml](config/traffic_light_fusion.param.yaml), validated by
[schema/traffic_light_fusion.schema.json](schema/traffic_light_fusion.schema.json). Only
`arbiter.*` is exposed as a parameter, moved under the `arbiter.` prefix (mirroring
`TrafficLightArbiterNode`'s own parameter names) since one Node now carries what used to be three
Nodes' worth of configuration. Its defaults are `autoware_traffic_light_arbiter`'s own defaults.

`multi_camera_fusion.*` and `crosswalk_estimator.*` are not parameters: no deployment has ever
needed to change either component's behavior from its fixed defaults, so
`declare_fusion_config()` hardcodes them (`autoware_traffic_light_multi_camera_fusion`'s and
`autoware_crosswalk_traffic_light_estimator`'s own package defaults, respectively) instead of
declaring them.

Two production parameters are deliberately dropped: `multi_camera_fusion`'s `approximate_sync`
(see above), and the `lanelet_map_ptr` field of `MultiCameraFusionConfig`, which is not a
parameter at all -- `TrafficLightFusion`'s constructor fills it from the received
`LaneletMapBin`, as it does for the other two cores.

`camera_namespaces` (a `string[]`) is not in the config file: which cameras feed the back-end is a
per-deployment fact, injected by
[launch/traffic_light_fusion.launch.xml](launch/traffic_light_fusion.launch.xml), the same way the
front-end's model/label paths are.

`build_fusion_config(Source &)`
([fusion_config_builder.hpp](src/traffic_light_fusion/fusion_config_builder.hpp)) maps those
parameter names to the flat `TrafficLightFusionConfig` the core consumes -- the back-end
counterpart of `build_recognition_config()` above, with the same two `Source`s
(`TrafficLightFusionNode` and `run_traffic_light_pipeline_evaluation`). The one piece of logic it
carries is normalizing an unrecognized `arbiter.source_priority` to `"confidence"` (with a
warning), exactly as `TrafficLightArbiterNode` does -- the core itself would silently do the same,
unwarned.

## Testing

| Test                                         | GPU      | Content                                                                                                               |
| -------------------------------------------- | -------- | --------------------------------------------------------------------------------------------------------------------- |
| `test_traffic_light_recognition`             | required | `TrafficLightRecognition::run()` with an empty map / no route, tf-resolution failure, `set_route()` error propagation |
| `test_traffic_light_recognition_integration` | required | Front-end Node pub/sub: the 3 production output topics fire                                                           |
| `test_config_builders_cross_check`           | no       | `config/*.param.yaml` builds the same config through the Node's parameters and through `ParameterLoader`              |

GPU-gated tests are additionally compiled only when CUDA and TensorRT are detected
(`TRT_AVAIL AND CUDA_AVAIL` in `CMakeLists.txt`, matching `autoware_traffic_light_classifier`'s
convention), and self-skip (`GTEST_SKIP`) at runtime when the ONNX models are not present under
`autoware_data` or no usable GPU is found, so the same binary behaves correctly whether or not the
environment is fully provisioned.

## Offline evaluation

`evaluation/` ships `run_traffic_light_recognition_evaluation`, a ROS-runtime-free executable that
takes a t4dataset path, drives `TrafficLightRecognition::run()` over every (image, camera_info)
frame in the dataset's rosbag, and writes the results to an output rosbag under production topic
names. It is a simplified port of `autoware_traffic_light_component_test`'s
`run_traffic_light_pipeline`, restricted to this package's front-end core. See
[evaluation/README.md](evaluation/README.md).

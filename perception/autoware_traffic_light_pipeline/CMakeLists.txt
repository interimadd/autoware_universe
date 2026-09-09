cmake_minimum_required(VERSION 3.14)
project(autoware_traffic_light_pipeline)

find_package(autoware_cmake REQUIRED)
autoware_package()

# This package hosts both halves of the traffic light pipeline, each in its own src/ subdirectory
# and its own pair of libraries (ROS-free core + Node adapter):
#   src/traffic_light_recognition/ -- the front-end (前段): map_based_detector ->
#     whole_image_detector(yolox) -> selector -> car/pedestrian classifier -> category_merger
#   src/traffic_light_fusion/      -- the back-end (後段): multi_camera_fusion -> arbiter ->
#     crosswalk_traffic_light_estimator
# The library names are the component names rather than ${PROJECT_NAME}, since ${PROJECT_NAME}
# now covers two independent compositions.

# --- shared config helpers ---------------------------------------------------------------------

# Locating this package's installed config/*.param.yaml (src/common/package_config.hpp). The
# param-name -> config-field mapping itself lives in the header-only
# src/traffic_light_recognition/recognition_config_builder.hpp and
# src/traffic_light_fusion/fusion_config_builder.hpp, shared by the Nodes (via
# src/common/node_parameter_source.hpp, which declares the parameters) and by the ROS-free callers
# (via autoware_component_test_framework's ParameterLoader, which reads the same file with the same
# rcl yaml parser).
find_package(yaml-cpp REQUIRED)
ament_auto_add_library(autoware_traffic_light_pipeline_config SHARED
  src/common/package_config.cpp
)
target_include_directories(autoware_traffic_light_pipeline_config PUBLIC src)

# --- front-end (traffic_light_recognition) ---------------------------------------------------

# ROS 非依存の合成コア
ament_auto_add_library(autoware_traffic_light_recognition SHARED
  src/traffic_light_recognition/traffic_light_recognition.cpp
)

# Node アダプタ（param 変換を含む）
ament_auto_add_library(autoware_traffic_light_recognition_node SHARED
  src/traffic_light_recognition/traffic_light_recognition_node.cpp
)
target_include_directories(autoware_traffic_light_recognition_node PRIVATE src)
target_link_libraries(autoware_traffic_light_recognition_node autoware_traffic_light_recognition)

rclcpp_components_register_node(autoware_traffic_light_recognition_node
  PLUGIN "autoware::traffic_light::TrafficLightRecognitionNode"
  EXECUTABLE traffic_light_recognition_node)

# --- back-end (traffic_light_fusion) ----------------------------------------------------------

# ROS 非依存の合成コア
ament_auto_add_library(autoware_traffic_light_fusion SHARED
  src/traffic_light_fusion/traffic_light_fusion.cpp
)

# Node アダプタ（param 変換を含む）
ament_auto_add_library(autoware_traffic_light_fusion_node SHARED
  src/traffic_light_fusion/traffic_light_fusion_node.cpp
)
target_include_directories(autoware_traffic_light_fusion_node PRIVATE src)
target_link_libraries(autoware_traffic_light_fusion_node autoware_traffic_light_fusion)

rclcpp_components_register_node(autoware_traffic_light_fusion_node
  PLUGIN "autoware::traffic_light::TrafficLightFusionNode"
  EXECUTABLE traffic_light_fusion_node)

# --- offline evaluation tools -------------------------------------------------------------------

# t4dataset を入力に前段（run_traffic_light_recognition_evaluation）／前段+後段
# （run_traffic_light_pipeline_evaluation）を回し、結果を rosbag に書き出すオフライン評価ツール
# （evaluation/README.md）。ROS ランタイム（rclcpp::init / executor / DDS）は使わず、コアライブラリ
# と rosbag2 だけに依存する。yaml/rosbag 読み込みの共通部分は evaluation_common に切り出し、両方の
# 実行ファイルで共有する。
ament_auto_add_library(evaluation_common SHARED
  evaluation/evaluation_common.cpp
)
target_include_directories(evaluation_common PUBLIC src)
target_link_libraries(evaluation_common
  autoware_traffic_light_recognition autoware_traffic_light_pipeline_config yaml-cpp)

ament_auto_add_executable(run_traffic_light_recognition_evaluation
  evaluation/run_traffic_light_recognition_evaluation.cpp
)
target_link_libraries(run_traffic_light_recognition_evaluation evaluation_common)

ament_auto_add_executable(run_traffic_light_pipeline_evaluation
  evaluation/run_traffic_light_pipeline_evaluation.cpp
)
target_include_directories(run_traffic_light_pipeline_evaluation PRIVATE src)
target_link_libraries(run_traffic_light_pipeline_evaluation
  evaluation_common autoware_traffic_light_fusion yaml-cpp)

install(DIRECTORY evaluation/config DESTINATION share/${PROJECT_NAME}/evaluation)

# Phase 2 (evaluate_result_implementation_plan.md): Python accuracy evaluation script, plus the
# sibling modules it imports (criteria.py / traffic_light_label.py). All three are installed flat
# into lib/${PROJECT_NAME} so `ros2 run` finds the entry point and its own sys.path insertion (see
# evaluate_traffic_light_recognition.py) finds the siblings next to it.
install(PROGRAMS
  evaluation/scripts/evaluate_traffic_light_recognition.py
  evaluation/scripts/criteria.py
  evaluation/scripts/traffic_light_label.py
  evaluation/scripts/run_traffic_light_test_case_evaluation.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # GPU-gated tests (the front-end core builds TensorRT engines in its constructor). Follows
  # autoware_traffic_light_classifier's convention: build only when CUDA/TensorRT are available,
  # and additionally self-skip (GTEST_SKIP) at runtime when the model files or a usable GPU are
  # not found, so the same binary behaves correctly whether or not autoware_data is populated.
  # The back-end tests need neither: none of its three cores touch TensorRT.
  option(CUDA_AVAIL "CUDA available" OFF)
  find_package(CUDA)
  if(CUDA_FOUND)
    set(CUDA_AVAIL ON)
  endif()
  option(TRT_AVAIL "TensorRT available" OFF)
  find_library(NVINFER NAMES nvinfer)
  find_library(NVONNXPARSER nvonnxparser)
  if(NVINFER AND NVONNXPARSER)
    set(TRT_AVAIL ON)
  endif()

  # Cross-check that the Nodes' parameter path and the ROS-free ParameterLoader path build the
  # same config from the same config/*.param.yaml (see the test's own header comment). Needs
  # neither a GPU nor a model: it only builds config structs.
  ament_add_ros_isolated_gtest(test_config_builders_cross_check
    test/test_config_builders_cross_check.cpp
  )
  target_include_directories(test_config_builders_cross_check PRIVATE src)
  target_link_libraries(test_config_builders_cross_check
    autoware_traffic_light_recognition
    autoware_traffic_light_fusion
    autoware_traffic_light_pipeline_config)

  # ROS-free unit tests for the composed front-end core. GPU-gated (see comment above): the core's
  # constructor builds the whole-image detector's and both classifiers' TensorRT engines.
  if(TRT_AVAIL AND CUDA_AVAIL)
    ament_auto_add_gtest(test_traffic_light_recognition
      test/traffic_light_recognition/test_traffic_light_recognition.cpp
      TIMEOUT 600
    )
    target_link_libraries(test_traffic_light_recognition
      autoware_traffic_light_recognition autoware_traffic_light_pipeline_config)
  endif()

  # Node pub/sub integration test. GPU-gated for the same reason as test_traffic_light_recognition.
  if(TRT_AVAIL AND CUDA_AVAIL)
    ament_add_ros_isolated_gtest(test_traffic_light_recognition_integration
      test/traffic_light_recognition/test_traffic_light_recognition_integration.cpp
      TIMEOUT 600
    )
    target_include_directories(test_traffic_light_recognition_integration PRIVATE src)
    target_link_libraries(test_traffic_light_recognition_integration
      autoware_traffic_light_recognition_node autoware_traffic_light_pipeline_config)
  endif()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  rviz
)

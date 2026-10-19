cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_follower_node)

find_package(autoware_cmake REQUIRED)
autoware_package()

# The CMake target keeps its name, so that the executable and the topic paths that
# carry the node name do not change.
set(CONTROLLER_NODE controller_node)
# Core logic: no node, no topics, no parameters. An offline evaluation links only this.
set(TRAJECTORY_FOLLOWER trajectory_follower)
ament_auto_add_library(${TRAJECTORY_FOLLOWER} SHARED
  src/trajectory_follower.cpp
  src/trajectory_follower_config.cpp
  src/controller_factory.cpp
)

ament_auto_add_library(${CONTROLLER_NODE} SHARED
  src/trajectory_follower_node.cpp
  src/create_config.cpp
  src/ros_log_writer.cpp
)
target_link_libraries(${CONTROLLER_NODE} ${TRAJECTORY_FOLLOWER})

autoware_agnocast_wrapper_register_node(${CONTROLLER_NODE}
  PLUGIN "autoware::motion::control::trajectory_follower_node::TrajectoryFollowerNode"
  EXECUTABLE ${CONTROLLER_NODE}_exe
  AGNOCAST_EXECUTOR CallbackIsolatedAgnocastExecutor
  ROS2_EXECUTOR SingleThreadedExecutor
)

# simple trajectory follower
set(SIMPLE_TRAJECTORY_FOLLOWER_NODE simple_trajectory_follower)
ament_auto_add_library(${SIMPLE_TRAJECTORY_FOLLOWER_NODE} SHARED
  include/autoware/trajectory_follower_node/simple_trajectory_follower.hpp
  src/simple_trajectory_follower.cpp
)

autoware_agnocast_wrapper_register_node(${SIMPLE_TRAJECTORY_FOLLOWER_NODE}
  PLUGIN "simple_trajectory_follower::SimpleTrajectoryFollower"
  EXECUTABLE ${SIMPLE_TRAJECTORY_FOLLOWER_NODE}_exe
  AGNOCAST_EXECUTOR CallbackIsolatedAgnocastExecutor
  ROS2_EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  # Unit test of the core logic. It starts no ROS 2 context.
  ament_add_gtest(test_trajectory_follower
    test/test_trajectory_follower.cpp
  )
  target_link_libraries(test_trajectory_follower ${TRAJECTORY_FOLLOWER})

  set(TRAJECTORY_FOLLOWER_NODES_TEST test_trajectory_follower_node)
  ament_add_ros_isolated_gtest(${TRAJECTORY_FOLLOWER_NODES_TEST}
    test/trajectory_follower_test_utils.hpp
    test/test_trajectory_follower_node.cpp
  )
  ament_target_dependencies(${TRAJECTORY_FOLLOWER_NODES_TEST} autoware_fake_test_node ament_index_cpp)
  target_link_libraries(
    ${TRAJECTORY_FOLLOWER_NODES_TEST} ${CONTROLLER_NODE})

  # Temporary disabled, tracked by:
  # https://github.com/autowarefoundation/autoware_universe/issues/7733
#  find_package(autoware_testing REQUIRED)
#  # smoke test for MPC controller
#  add_smoke_test(${PROJECT_NAME} ${CONTROLLER_NODE}_exe
#    PARAM_FILENAMES "lateral/mpc.param.yaml longitudinal/pid.param.yaml
#trajectory_follower_node.param.yaml"
#    TEST_PARAM_FILENAMES "test_controller_mpc.param.yaml test_controller_pid.param.yaml
#test_vehicle_info.param.yaml test_nearest_search.param.yaml"
#    TARGET_INFIX "mpc"
#  )
#  # smoke test for pure pursuit controller
#  add_smoke_test(${PROJECT_NAME} ${CONTROLLER_NODE}_exe
#    PARAM_FILENAMES "lateral/pure_pursuit.param.yaml longitudinal/pid.param.yaml
#trajectory_follower_node.param.yaml"
#    TEST_PARAM_FILENAMES "test_controller_pure_pursuit.param.yaml test_controller_pid.param.yaml
#test_vehicle_info.param.yaml test_nearest_search.param.yaml"
#    TARGET_INFIX "pure_pursuit"
#  )

endif()

ament_auto_package(
  INSTALL_TO_SHARE
  param
  test
  launch
  config
)

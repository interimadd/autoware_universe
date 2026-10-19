cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_follower_base)

find_package(autoware_cmake REQUIRED)
autoware_package()

# This package only defines the types and the interfaces the controllers share.
ament_auto_package()

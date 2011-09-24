cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDeb)

rosbuild_init()
rosbuild_genmsg()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_add_boost_directories()

rosbuild_add_executable(teleop_source_ros
  src/teleop_source_ros.cpp
  src/teleop_source.cpp
  src/teleop_source_joystick.cpp
  src/teleop_source_keyboard.cpp
)

rosbuild_link_boost(teleop_source_ros thread)

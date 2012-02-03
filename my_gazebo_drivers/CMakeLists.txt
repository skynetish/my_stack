cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
rosbuild_add_boost_directories()

rosbuild_add_executable(gazebo_ros_diffdrive src/gazebo_ros_diffdrive.cpp)
rosbuild_link_boost(gazebo_ros_diffdrive thread)

rosbuild_add_library(gazebo_ros_diffdrive_plugin src/gazebo_ros_diffdrive_plugin.cpp)
rosbuild_link_boost(gazebo_ros_diffdrive_plugin thread)
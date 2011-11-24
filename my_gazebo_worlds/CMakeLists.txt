cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()

#######################################
# Process certain xacro files
#######################################
# Find the xacro package and include the xacro cmake macro file
# THIS IS DEPRECATED, USE THIS INSTEAD WHEN XACRO IS FIXED: rosbuild_include(xacro xacro)
rosbuild_find_ros_package(xacro)
include(${xacro_PACKAGE_PATH}/cmake/xacro.cmake)

# Declare variable containing all input files
file(GLOB_RECURSE input_files ${CMAKE_CURRENT_SOURCE_DIR}/worlds/*.world.xacro)

# Declare variable containing all output files
set(output_files "")

# For each input file
foreach(input_file ${input_files})

  # Declare variable containing output filename
  string(REPLACE ".xacro" "" output_file ${input_file})

  # Status message
  message("Converting file ${input_file} to ${output_file}")

  # Process with xacro
  xacro_add_xacro_file(${input_file} ${output_file})

  # Add to list of output files
  set(output_files ${output_files} ${output_file})

endforeach(input_file)

# Add target to ensure that the output files are updated as needed
add_custom_target(output_files_target ALL DEPENDS ${output_files})

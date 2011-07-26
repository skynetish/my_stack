cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Find the xacro package and include the xacro cmake macro file
rosbuild_find_ros_package(xacro)
include(${xacro_PACKAGE_PATH}/cmake/xacro.cmake)
# THE ABOVE IS DEPRECATED, USE THIS INSTEAD WHEN XACRO IS FIXED:
# rosbuild_include(xacro xacro)

# Declare variable containing all (input) world.xacro files
file(GLOB world_xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/worlds/*.world.xacro)

# Declare variable containing all (output) world files
set(world_files "")

# For each input file
foreach(input_file ${world_xacro_files})

  # Get basepath and basename without extension
  get_filename_component(basepath ${input_file} PATH)
  get_filename_component(basename ${input_file} NAME_WE)

  # Declare variable containing output filename
  set(output_file "${basepath}/${basename}.world")

  # Status message
  message("Converting file ${input_file} to ${output_file}")

  # Process with xacro
  xacro_add_xacro_file(${input_file} ${output_file})

  # Add to list of output files
  set(world_files ${world_files} ${output_file})

endforeach(file)

# Add target to ensure that the world files are properly updated
add_custom_target(world_files_target ALL DEPENDS ${world_files})

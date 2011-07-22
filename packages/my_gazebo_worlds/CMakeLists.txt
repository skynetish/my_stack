cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})





# Load the xacro package
rosbuild_find_ros_package(xacro)
include(${xacro_PACKAGE_PATH}/cmake/xacro.cmake)

# Variable containing input world.xacro files
file(GLOB world_xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/worlds/*.world.xacro)

# Variable containing output world files
set(world_files "")

# For each world.xacro file
foreach(it ${world_xacro_files})

  # Get path and basename without extension
  get_filename_component(basepath ${it} PATH)
  get_filename_component(basename ${it} NAME_WE)

  # Set output filename
  set(expanded_file "${basepath}/${basename}.world")

  message("Converting file ${it} to ${expanded_file}")

  # Process with xacro
  xacro_add_xacro_file(${it} ${expanded_file})

  # Add to list of output world files
  set(world_files ${world_files} ${expanded_file})

endforeach(it)

# Add target to ensure that the world files are created
add_custom_target(media_files ALL DEPENDS ${world_files})
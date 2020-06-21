cmake_minimum_required(VERSION 2.8.3)
project(pcl_ros_tutorials)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl_conversions
  sensor_msgs
)

catkin_package(INCLUDE_DIRS
  include
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(passthrough_node src/passthrough_node.cpp)
target_link_libraries(passthrough_node ${catkin_LIBRARIES})

add_executable(voxel_grid_filter_node src/voxel_grid_filter_node.cpp)
target_link_libraries(voxel_grid_filter_node ${catkin_LIBRARIES})

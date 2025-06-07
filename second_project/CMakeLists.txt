
cmake_minimum_required(VERSION 3.0.2)
project(second_project)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  laser_geometry
  message_filters
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  laser_geometry
  stage_ros
  map_server
  amcl
  move_base_msgs
  actionlib
)
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs laser_geometry message_filters
)
include_directories(${catkin_INCLUDE_DIRS})
add_executable(scan_merger src/scan_merger.cpp)
target_link_libraries(scan_merger ${catkin_LIBRARIES})
add_executable(odom_to_tf src/odom_to_tf.cpp)
target_link_libraries(odom_to_tf ${catkin_LIBRARIES})
add_executable(odom_filter src/odom_filter.cpp)
target_link_libraries(odom_filter ${catkin_LIBRARIES})
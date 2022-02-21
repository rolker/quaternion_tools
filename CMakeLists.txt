cmake_minimum_required(VERSION 2.8.3)
project(quaternion_tools)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  ros_type_introspection
  tf2
  topic_tools
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES udp_bridge
  CATKIN_DEPENDS geometry_msgs roscpp ros_type_introspection tf2 topic_tools
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(quat_to_rpy src/quat_to_rpy.cpp)
target_link_libraries(quat_to_rpy ${catkin_LIBRARIES})

add_executable(rpy_to_hpr_degrees src/rpy_to_hpr_degrees.cpp)
target_link_libraries(rpy_to_hpr_degrees ${catkin_LIBRARIES})

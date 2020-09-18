cmake_minimum_required(VERSION 3.0.2)
project(rviz-basic-markers)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  visualization_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rviz-basic-markers
#  CATKIN_DEPENDS roscpp rospy visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(basic_markers_node src/basic_markers_node src/basic_markers.cpp)
target_link_libraries(basic_markers_node ${catkin_LIBRARIES})
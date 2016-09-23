cmake_minimum_required(VERSION 2.8.3)
project(elikos_tracking)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  elikos_ros
  geometry_msgs
  image_transport
  roscpp
  std_msgs
  tf
)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES elikos_tracking
 CATKIN_DEPENDS cv_bridge elikos_ros geometry_msgs image_transport roscpp std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
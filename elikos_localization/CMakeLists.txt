cmake_minimum_required(VERSION 2.8.3)
project(elikos_localization)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  image_transport
  roscpp
  std_msgs
  sensor_msgs
  tf2
  eigen_conversions
  pcl_ros
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS image_transport cv_bridge geometry_msgs nav_msgs message_runtime roscpp sensor_msgs std_msgs tf2 pcl_ros
)


add_subdirectory(src/localization)



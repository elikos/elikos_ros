cmake_minimum_required(VERSION 2.8.3)
project(elikos_preprocessing)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  nodelet
  cv_bridge
  geometry_msgs
  nav_msgs
  elikos_ros
  image_transport
  roscpp
  std_msgs
  sensor_msgs
  tf2
  eigen_conversions
  pcl_ros
)
find_package(Qt5Gui)
find_package(Qt5Core)
find_package(Qt5Widgets)
find_package(Qt5Test)
find_package(Qt5Concurrent)
find_package(Qt5OpenGL)
find_package(OpenCV 2 REQUIRED)

catkin_package(
#   INCLUDE_DIRS include
  LIBRARIES elikos_preprocessing
  CATKIN_DEPENDS image_transport cv_bridge geometry_msgs nav_msgs message_runtime roscpp sensor_msgs std_msgs tf2 pcl_ros
#   DEPENDS system_lib
)

include_directories(
	${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(elikos_preprocessing
    src/Main.cpp
    src/MessageHandler.cpp
    src/MessageHandler.h
    src/PreProcessing.cpp
    src/PreProcessing.h)

target_link_libraries(elikos_preprocessing
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(elikos_preprocessing ${catkin_EXPORTED_LIBRARIES})
endif()


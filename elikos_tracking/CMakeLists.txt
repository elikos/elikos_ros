cmake_minimum_required(VERSION 2.8.3)
project(elikos_tracking)
set(CMAKE_CXX_FLAGS "-std=c++11 -I/usr/include/eigen3 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  elikos_ros
  image_transport
  geometry_msgs
  sensor_msgs
  std_msgs
  tf
  message_generation
)
find_package(Qt5Gui)
find_package(Qt5Core)
find_package(Qt5Widgets)
find_package(Qt5Test)
find_package(Qt5Concurrent)
find_package(Qt5OpenGL)
find_package(OpenCV 3 REQUIRED)


catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES elikos_detection
	CATKIN_DEPENDS roscpp cv_bridge elikos_ros geometry_msgs std_msgs tf
#   DEPENDS system_lib
)

include_directories(
	${catkin_INCLUDE_DIRS}
  #src/elikos_detection/TargetDetection/CVWrapper
  ${OpenCV_INCLUDE_DIRS}
)

add_executable( elikos_tracking
    src/elikos_tracking/elikos_tracking.cpp
    src/elikos_tracking/modele/robot.cpp
    src/elikos_tracking/TrackingHandler.cpp
   )

target_link_libraries(elikos_tracking
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

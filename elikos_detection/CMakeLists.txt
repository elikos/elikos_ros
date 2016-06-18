cmake_minimum_required(VERSION 2.8.3)
project(elikos_detection)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  elikos_ros
  image_transport
  sensor_msgs
  std_msgs
)
find_package(OpenCV REQUIRED)

catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES elikos_detection
	CATKIN_DEPENDS roscpp cv_bridge elikos_ros image_transport sensor_msgs std_msgs
#   DEPENDS system_lib
)

include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable( elikos_detection
    src/detection/dnt.cpp
    src/detection/MessageHandler.h
    src/detection/MessageHandler.cpp
	src/detection/BlobDetection/RobotDesc.h
	src/detection/BlobDetection/RobotDesc.cpp
	src/detection/BlobDetection/Color.cpp
	src/detection/BlobDetection/Color.h
	src/detection/BlobDetection/GreenColor.cpp
	src/detection/BlobDetection/GreenColor.h
	src/detection/BlobDetection/RedColor.cpp
	src/detection/BlobDetection/RedColor.h
	src/detection/BlobDetection/WhiteColor.cpp
	src/detection/BlobDetection/WhiteColor.h
    src/detection/BlobDetection/RobotDetection.cpp
    src/detection/BlobDetection/RobotDetection.h
    src/detection/BlobDetection/BlobTracking.h
    src/detection/BlobDetection/BlobTracking.cpp)

target_link_libraries(elikos_detection
  ${catkin_LIBRARIES}
)

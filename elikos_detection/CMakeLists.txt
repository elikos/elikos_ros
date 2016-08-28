cmake_minimum_required(VERSION 2.8.3)
project(elikos_detection)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  elikos_ros
  image_transport
  geometry_msgs
  sensor_msgs
  std_msgs
  tf
)
find_package(OpenCV REQUIRED)

catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES elikos_detection
	CATKIN_DEPENDS roscpp cv_bridge elikos_ros image_transport sensor_msgs geometry_msgs std_msgs tf
#   DEPENDS system_lib
)

include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable( elikos_detection
    src/elikos_detection/elikos_detection.cpp
    src/elikos_detection/MessageHandler.h
    src/elikos_detection/MessageHandler.cpp
    src/elikos_detection/TargetDetection/RobotDesc.h
    src/elikos_detection/TargetDetection/RobotDesc.cpp
    src/elikos_detection/TargetDetection/Color.cpp
    src/elikos_detection/TargetDetection/Color.h
    src/elikos_detection/TargetDetection/GreenColor.cpp
    src/elikos_detection/TargetDetection/GreenColor.h
    src/elikos_detection/TargetDetection/RedColor.cpp
    src/elikos_detection/TargetDetection/RedColor.h
    src/elikos_detection/TargetDetection/WhiteColor.cpp
    src/elikos_detection/TargetDetection/WhiteColor.h
    src/elikos_detection/TargetDetection/BlobDetection.cpp
    src/elikos_detection/TargetDetection/BlobDetection.h
    src/elikos_detection/TargetDetection/TargetDetection.h
    src/elikos_detection/TargetDetection/TargetDetection.cpp)

target_link_libraries(elikos_detection
  ${catkin_LIBRARIES}
)

add_executable( elikos_transformation
   src/elikos_transformation/elikos_transformation.cpp
   src/elikos_transformation/MessageHandler.h
   src/elikos_transformation/MessageHandler.cpp
   src/elikos_transformation/TransformationUnit.h
   src/elikos_transformation/TransformationUnit.cpp)

target_link_libraries(elikos_transformation
  ${catkin_LIBRARIES}
)
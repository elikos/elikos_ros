cmake_minimum_required(VERSION 2.8.3)
project(elikos_control)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
   message_generation
   roscpp
   tf
   std_msgs
   sensor_msgs
   mavros_msgs
   sensor_msgs
   geometry_msgs
   pluginlib
   cmake_modules
   std_srvs
   cv_bridge
)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES elikos_tracking
 CATKIN_DEPENDS elikos_ros roscpp std_msgs tf cv_bridge
#  DEPENDS system_lib
CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable( elikos_control
    src/elikos_control/MessageHandler.h
    src/elikos_control/MessageHandler.cpp
    src/elikos_control/elikos_control.cpp
)


target_link_libraries( elikos_control
  ${catkin_LIBRARIES}
)

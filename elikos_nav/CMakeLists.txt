cmake_minimum_required(VERSION 2.8.3)
project(elikos_nav)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
   message_generation
   roscpp
   tf
   std_msgs
   genmsg
   sensor_msgs
   cv_bridge
   image_transport
   mavros_msgs
   sensor_msgs
   cv_bridge
   geometry_msgs
   moveit_core
   moveit_ros_planning
   moveit_ros_planning_interface
   pluginlib
   cmake_modules
)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES elikos_tracking
 CATKIN_DEPENDS cv_bridge elikos_ros geometry_msgs image_transport roscpp std_msgs tf
#  DEPENDS system_lib
CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable( elikos_moveit_move_group
    src/elikos_moveit_move_group/moveit_move_group.cpp
    src/elikos_moveit_move_group/MessageHandler_moveit.h
    src/elikos_moveit_move_group/MessageHandler_moveit.cpp)


add_dependencies( elikos_moveit_move_group
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
)

target_link_libraries(elikos_moveit_move_group
  ${catkin_LIBRARIES}
)

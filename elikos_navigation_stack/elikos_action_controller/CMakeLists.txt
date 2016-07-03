cmake_minimum_required(VERSION 2.8.3)
project(elikos_action_controller)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread)

include_directories(SYSTEM "msg_gen/cpp/include")

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  actionlib
  roscpp
  trajectory_msgs
  actionlib_msgs
  std_msgs
  message_generation
  tf
)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_message_files(
  FILES
  MultiDofFollowJointTrajectoryFeedback.msg
  MultiDofFollowJointTrajectoryGoal.msg
  MultiDofFollowJointTrajectoryResult.msg
  MultiDofFollowJointTrajectoryActionFeedback.msg
  MultiDofFollowJointTrajectoryActionGoal.msg
  MultiDofFollowJointTrajectoryActionResult.msg
  MultiDofFollowJointTrajectoryAction.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  trajectory_msgs
  actionlib_msgs
  elikos_action_controller
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  DEPENDS
  CATKIN_DEPENDS message_runtime
  moveit_core
  tf
)

include_directories(include)

# add_library(action_controller src/actionController.cpp)
add_executable(elikos_action_controller src/elikos_actionController.cpp)
target_link_libraries(elikos_action_controller ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})


#install(TARGETS elikos_action_controller LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

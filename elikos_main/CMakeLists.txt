
cmake_minimum_required(VERSION 2.8.3)
project(elikos_main)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package( catkin REQUIRED COMPONENTS
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
  std_srvs
  trajectory_msgs
  elikos_msgs
)
find_package(OpenCV REQUIRED)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################
## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   RobotPos.msg
#   RobotsPos.msg
#   RobotRaw.msg
#   RobotRawArray.msg
#   TargetRobot.msg
#   TargetRobotArray.msg
#   TrajectoryCmd.msg
#   AICmd.msg
#   Matrix3.msg
#   StampedMatrix3.msg
#   Intersection.msg
#   IntersectionArray.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
#   mavros_msgs
#   sensor_msgs
#   trajectory_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS message_runtime
LIBRARIES elikos_lib
CATKIN_DEPENDS std_msgs std_srvs mavros_msgs sensor_msgs geometry_msgs roscpp tf elikos_msgs
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories( include ${catkin_INCLUDE_DIRS} )

## Declare a cpp library
add_library( elikos_lib
  src/elikos_lib/pid.cpp
  src/elikos_lib/RCReceiver.cpp
include/elikos_lib/PositionHelper.h)



## Declare a cpp executable
# add_executable(elikos_node src/elikos_node.cpp)

#add_executable( robotsim
#  src/simulation/simulation.cpp
#  src/simulation/Robot.cpp
#  src/simulation/TargetRobot.cpp
#  src/simulation/ObstacleRobot.cpp
#  src/simulation/MAV.cpp)

#add_executable( pose_offset
#    src/pose_offset/pose_offset.cpp
#    src/pose_offset/PoseOffsetNode.cpp
#    src/pose_offset/McptamCtrl.cpp
#    include/pose_offset/SvoCtrl.h
#    include/pose_offset/McptamCtrl.h
#    include/pose_offset/VoCtrl.h
#    src/pose_offset/SvoCtrl.cpp
#)


#add_executable( dnt
#    src/dnt/dnt.cpp
#    src/dnt/MessageHandler.h
#    src/dnt/MessageHandler.cpp
#	src/dnt/BlobDetection/RobotDesc.h
#	src/dnt/BlobDetection/RobotDesc.cpp
#	src/dnt/BlobDetection/Color.cpp
#	src/dnt/BlobDetection/Color.h
#	src/dnt/BlobDetection/GreenColor.cpp
#	src/dnt/BlobDetection/GreenColor.h
#	src/dnt/BlobDetection/RedColor.cpp
#	src/dnt/BlobDetection/RedColor.h
#	src/dnt/BlobDetection/WhiteColor.cpp
#	src/dnt/BlobDetection/WhiteColor.h
#    src/dnt/BlobDetection/RobotDetection.cpp
#    src/dnt/BlobDetection/RobotDetection.h
#    src/dnt/BlobDetection/BlobTracking.h
#    src/dnt/BlobDetection/BlobTracking.cpp)

#add_executable( tracking_abs
#    src/tracking_abs/tracking_abs.cpp
#    src/tracking_abs/MessageHandler_abs.h
#    src/tracking_abs/MessageHandler_abs.cpp
#    src/tracking_abs/TransformationUnit.h
#    src/tracking_abs/TransformationUnit.cpp)

add_executable( elikos_origin_init
  src/elikos_origin_init/elikos_origin_init.cpp
)

#add_subdirectory(src/ai)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(robotsim elikos_lib)

# add_dependencies(elikos_lib
#     mavros_msgs
#     sensor_msgs
# )



## Specify libraries to link a library or executable target against
#target_link_libraries(elikos_node
#  ${catkin_LIBRARIES}
#)

target_link_libraries( elikos_lib ${catkin_LIBRARIES})
#target_link_libraries( robotsim elikos_lib ${catkin_LIBRARIES})
#target_link_libraries( dnt ${catkin_LIBRARIES} ${OpenCV_LIBS})
#target_link_libraries( tracking_abs ${catkin_LIBRARIES})
target_link_libraries(elikos_origin_init
  ${catkin_LIBRARIES}
)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS elikos elikos_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_elikos.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

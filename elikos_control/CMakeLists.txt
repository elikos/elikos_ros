cmake_minimum_required(VERSION 2.8.3)
project(elikos_control)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
   elikos_main
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
  INCLUDE_DIRS include
#  LIBRARIES elikos_tracking
 CATKIN_DEPENDS elikos_main roscpp std_msgs tf cv_bridge
#  DEPENDS system_lib
CATKIN_DEPENDS
)


install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

include_directories(
    src/elikos_control/cmd
    ${catkin_INCLUDE_DIRS}
)

add_executable( elikos_control
    src/elikos_control/cmd/CmdTravel.h
    src/elikos_control/cmd/CmdTravel.cpp
    src/elikos_control/cmd/CmdTravel.h
    src/elikos_control/cmd/CmdTravel.cpp
    src/elikos_control/cmd/CmdFrontInteraction.h
    src/elikos_control/cmd/CmdFrontInteraction.cpp
    src/elikos_control/cmd/CmdLanding.h
    src/elikos_control/cmd/CmdLanding.cpp
    src/elikos_control/cmd/CmdStandBy.h
    src/elikos_control/cmd/CmdStandBy.cpp
    src/elikos_control/cmd/CmdTopInteraction.h
    src/elikos_control/cmd/CmdTopInteraction.cpp
    src/elikos_control/cmd/CmdAbs.h
    src/elikos_control/cmd/CmdAbs.cpp
    src/elikos_control/cmd/CmdTakeOff.cpp
    src/elikos_control/cmd/CmdTakeOff.cpp
    src/elikos_control/MessageHandler.h
    src/elikos_control/MessageHandler.cpp
    src/elikos_control/elikos_control.cpp
    src/elikos_control/CmdExecutor.h
    src/elikos_control/CmdExecutor.cpp
    src/../include/CmdDefines.h
)


target_link_libraries( elikos_control
  ${catkin_LIBRARIES}
)

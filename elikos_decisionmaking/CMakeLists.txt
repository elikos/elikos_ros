cmake_minimum_required(VERSION 2.8.3)
project(elikos_decisionmaking)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  elikos_main
  geometry_msgs
  roscpp
  std_msgs
  tf
  elikos_control
)

catkin_package(
#	INCLUDE_DIRS include
#	LIBRARIES elikos_decisionmaking
	CATKIN_DEPENDS elikos_main geometry_msgs roscpp std_msgs tf elikos_control
#	DEPENDS system_lib
)

include_directories(
    src/ai
    src/emulator)

add_subdirectory(src/ai)
add_subdirectory(src/emulator)

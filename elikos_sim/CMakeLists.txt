cmake_minimum_required(VERSION 2.8.3)
project(elikos_sim)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  elikos_ros
  geometry_msgs
  roscpp
  std_msgs
  tf
)

catkin_package(
#	INCLUDE_DIRS include
#	LIBRARIES elikos_ai
	CATKIN_DEPENDS elikos_ros geometry_msgs roscpp std_msgs tf
#	DEPENDS system_lib
)

include_directories(
    ../elikos_ros/src/
    ${catkin_INCLUDE_DIRS}
)

add_executable(elikos_sim
    src/simulation/GameManager.cpp
    src/simulation/GameManager.hpp
    src/simulation/MAV.cpp
    src/simulation/MAV.h
    src/simulation/ObstacleRobot.cpp
    src/simulation/ObstacleRobot.hpp
    src/simulation/Robot.cpp
    src/simulation/Robot.hpp
    src/simulation/simulation.cpp
    src/simulation/Simulation.hpp
    src/simulation/TargetRobot.cpp
    src/simulation/TargetRobot.hpp)

target_link_libraries(elikos_sim
  ${catkin_LIBRARIES}
)

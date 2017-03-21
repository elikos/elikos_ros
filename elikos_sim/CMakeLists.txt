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
    src/rviz_sim/GameManager.cpp
    src/rviz_sim/GameManager.hpp
    src/rviz_sim/MAV.cpp
    src/rviz_sim/MAV.h
    src/rviz_sim/ObstacleRobot.cpp
    src/rviz_sim/ObstacleRobot.hpp
    src/rviz_sim/Robot.cpp
    src/rviz_sim/Robot.hpp
    src/rviz_sim/simulation.cpp
    src/rviz_sim/Simulation.hpp
    src/rviz_sim/TargetRobot.cpp
    src/rviz_sim/TargetRobot.hpp)

target_link_libraries(elikos_sim
  ${catkin_LIBRARIES}
)

add_executable(gazebo_tf_fix
    src/gazebo_sim/gazebo_tf_fix/gazebo_tf_fix.cpp)

target_link_libraries(gazebo_tf_fix
  ${catkin_LIBRARIES}
)

add_executable(robots_sim
    src/gazebo_sim/robots_sim/Obstacle.h
    src/gazebo_sim/robots_sim/Obstacle.cpp
    src/gazebo_sim/robots_sim/Robot.h
    src/gazebo_sim/robots_sim/Robot.cpp
    src/gazebo_sim/robots_sim/robots_sim.cpp)

target_link_libraries(robots_sim
  ${catkin_LIBRARIES} 
)

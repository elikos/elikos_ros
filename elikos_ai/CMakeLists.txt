cmake_minimum_required(VERSION 2.8.3)
project(elikos_ai)
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
  	${catkin_INCLUDE_DIRS}
  	src/ai/StateMachine
    src/ai/Strategy
	src/ai/Robot
	src/ai
)

set(STATE_MACHINE_SRC
    src/ai/StateMachine/StateMachine.h
    src/ai/StateMachine/StateMachine.cpp
    src/ai/StateMachine/State.h
    src/ai/StateMachine/State.cpp)

set(STRATEGY_SRC
    src/ai/Strategy/Strategy.h
    src/ai/Strategy/Strategy.cpp
    src/ai/Strategy/FollowClosestTarget.cpp
    src/ai/Strategy/FollowClosestTarget.h)

set(ROBOT_SRC
    src/ai/Robot/Robot.h
    src/ai/Robot/Robot.cpp
    src/ai/Robot/ObstacleRobot.h
    src/ai/Robot/ObstacleRobot.cpp
    src/ai/Robot/QuadRobot.h
    src/ai/Robot/QuadRobot.cpp
    src/ai/Robot/TargetRobot.h
    src/ai/Robot/TargetRobot.cpp
    src/ai/Robot/RobotTypes.h)

add_executable(elikos_ai
    src/ai/main.cpp
    src/ai/CmdLineParser.h
    src/ai/CmdLineParser.cpp
    src/ai/Agent.cpp
    src/ai/Agent.h
    src/ai/MessageHandler.h
    src/ai/MessageHandler.cpp
        src/ai/MessageEmulator.h
        src/ai/MessageEmulator.cpp
    ${STATE_MACHINE_SRC}
    ${STRATEGY_SRC}
    ${ROBOT_SRC})

target_link_libraries(elikos_ai
  ${catkin_LIBRARIES}
)

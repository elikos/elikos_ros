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
    src/ai
  	src/ai/StateMachine
	src/ai/Robot
    src/ai/Consideration)

set(STATE_MACHINE_SRC
    src/ai/StateMachine/StateMachine.h
    src/ai/StateMachine/StateMachine.cpp
    src/ai/StateMachine/AbstractState.h
    src/ai/StateMachine/AbstractState.cpp
    src/ai/StateMachine/MovementState.cpp
    src/ai/StateMachine/MovementState.h)

set(ROBOT_SRC
    src/ai/Robot/Robot.h
    src/ai/Robot/Robot.cpp
    src/ai/Robot/QuadRobot.h
    src/ai/Robot/QuadRobot.cpp
    src/ai/Robot/TargetRobot.h
    src/ai/Robot/TargetRobot.cpp
    src/ai/Robot/RobotTypes.h)

set(CONSIDERATION_SRC
    src/ai/Consideration/ConsiderationPipeline.cpp
    src/ai/Consideration/ConsiderationPipeline.h
    src/ai/Consideration/AbstractConsideration.cpp
    src/ai/Consideration/AbstractConsideration.h
    src/ai/Consideration/QuadDistance.cpp
    src/ai/Consideration/QuadDistance.h
    src/ai/Consideration/RedLineDistance.cpp
    src/ai/Consideration/RedLineDistance.h
    src/ai/Consideration/ConsiderationTypes.h
    src/ai/StateMachine/ObservationState.cpp
    src/ai/StateMachine/ObservationState.h
    src/ai/StateMachine/InteractionState.cpp
    src/ai/StateMachine/InteractionState.h)

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
    ${CONSIDERATION_SRC}
    ${STATE_MACHINE_SRC}
    ${ROBOT_SRC})

target_link_libraries(elikos_ai
  ${catkin_LIBRARIES})

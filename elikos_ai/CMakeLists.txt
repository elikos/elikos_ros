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
    src/ai/Context
    src/ai/Consideration
	src/ai/Robot
	src/ai/Util
	src/ai/Behavior)

set(BEHAVIOR_SRC
    src/ai/Behavior/AbstractBehavior.cpp
    src/ai/Behavior/AbstractBehavior.h
    src/ai/Behavior/AggressiveBehavior.cpp
    src/ai/Behavior/AggressiveBehavior.h
    src/ai/Behavior/PreventiveBehavior.cpp
    src/ai/Behavior/PreventiveBehavior.h)

set(STATE_MACHINE_SRC
    src/ai/StateMachine/StateMachine.h
    src/ai/StateMachine/StateMachine.cpp
    src/ai/StateMachine/AbstractState.h
    src/ai/StateMachine/AbstractState.cpp
    src/ai/StateMachine/MovementState.cpp
    src/ai/StateMachine/MovementState.h
    src/ai/StateMachine/ObservationState.cpp
    src/ai/StateMachine/ObservationState.h
    src/ai/StateMachine/InteractionState.cpp
    src/ai/StateMachine/InteractionState.h
    src/ai/StateMachine/Command.cpp
    src/ai/StateMachine/Command.h)

set(CONTEXT_SRC
    src/ai/Context/AbstractLine.cpp
    src/ai/Context/AbstractLine.h
	src/ai/Context/AbstractArena.h
	src/ai/Context/AbstractArena.cpp
    src/ai/Context/ArenaA.cpp
    src/ai/Context/ArenaA.h
    src/ai/Context/ArenaB.cpp
    src/ai/Context/ArenaB.h
    src/ai/Context/RedLine.cpp
    src/ai/Context/RedLine.h
    src/ai/Context/WhiteLine.cpp
    src/ai/Context/WhiteLine.h
    src/ai/Context/GreenLine.cpp
    src/ai/Context/GreenLine.h
    src/ai/Context/TargetOrientationEvaluation.cpp
    src/ai/Context/TargetOrientationEvaluation.h
    src/ai/Context/ContextTypes.h)

set(ROBOT_SRC
    src/ai/Robot/AbstractRobot.h
    src/ai/Robot/AbstractRobot.cpp
    src/ai/Robot/QuadRobot.h
    src/ai/Robot/QuadRobot.cpp
    src/ai/Robot/TargetRobot.h
    src/ai/Robot/TargetRobot.cpp
    src/ai/Robot/RobotTypes.h)

set(CONSIDERATION_SRC
    src/ai/Consideration/PriorityEvaluationPipeline.cpp
    src/ai/Consideration/PriorityEvaluationPipeline.h
    src/ai/Consideration/AbstractConsideration.cpp
    src/ai/Consideration/AbstractConsideration.h
    src/ai/Consideration/QuadDistance.cpp
    src/ai/Consideration/QuadDistance.h
    src/ai/Consideration/TargetDestination.cpp
    src/ai/Consideration/TargetDestination.h
    src/ai/Consideration/ConsiderationTypes.h
    src/ai/Consideration/ClusterSize.cpp
    src/ai/Consideration/ClusterSize.h)

set(UTIL_SRC
    src/ai/Util/Line.cpp
    src/ai/Util/Line.h
    src/ai/Util/Segment.cpp
    src/ai/Util/Segment.h)

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
    ${CONTEXT_SRC}
    ${ROBOT_SRC}
	${UTIL_SRC}
	${BEHAVIOR_SRC})

target_link_libraries(elikos_ai
  ${catkin_LIBRARIES})



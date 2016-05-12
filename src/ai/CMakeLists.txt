include_directories(StateMachine)
include_directories(Robot)

add_executable( ai
    ai.cpp
    CmdLineParser.h
    CmdLineParser.cpp
    Agent.cpp
    Agent.h
    MessageHandler.h
    MessageHandler.cpp
    StateMachine/StateMachine.h
    StateMachine/StateMachine.cpp
    StateMachine/AbstractState.h
    StateMachine/AbstractState.cpp
    Robot/Robot.h
    Robot/Robot.cpp
    Robot/ObstacleRobot.h
    Robot/ObstacleRobot.cpp
    Robot/QuadRobot.h
    Robot/QuadRobot.cpp
    Robot/TargetRobot.h
    Robot/TargetRobot.cpp
    Robot/RobotTypes.h)

target_link_libraries( ai ${catkin_LIBRARIES})

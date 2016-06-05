include(Robot/CMakeLists.txt)
include(StateMachine/CMakeLists.txt)
include(Strategy/CMakeLists.txt)

add_executable( ai
    main.cpp
    CmdLineParser.h
    CmdLineParser.cpp
    Agent.cpp
    Agent.h
    MessageHandler.h
    MessageHandler.cpp
    MsgEmulator.h
    MsgEmulator.cpp
    ${STATE_MACHINE_SRC}
    ${STRATEGY_SRC}
    ${ROBOT_SRC})

target_link_libraries( ai ${catkin_LIBRARIES})

add_dependencies(ai
    elikos_ros_generate_messages_cpp)

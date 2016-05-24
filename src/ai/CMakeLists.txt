include(Robot/CMakeLists.txt)
include(StateMachine/CMakeLists.txt)
include(Strategy/CMakeLists.txt)
include(Translator/CMakeLists.txt)

add_executable( ai
    main.cpp
    CmdLineParser.h
    CmdLineParser.cpp
    Agent.cpp
    Agent.h
    MessageHandler.h
    MessageHandler.cpp
    ${STATE_MACHINE_SRC}
    ${STRATEGY_SRC}
    ${ROBOT_SRC})

target_link_libraries( ai ${catkin_LIBRARIES})

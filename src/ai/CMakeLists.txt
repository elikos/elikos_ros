
include_directories(StateMachine)

add_executable( ai
    ai.cpp
    Agent.cpp
    Agent.h
    MessageHandler.h
    MessageHandler.cpp
    StateMachine/StateMachine.h
    StateMachine/StateMachine.cpp
    StateMachine/AbstractState.h
    StateMachine/AbstractState.cpp)

target_link_libraries( ai ${catkin_LIBRARIES})

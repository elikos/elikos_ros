include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(emulator
    Emulator.cpp
    MessageEmulator.h
    MessageEmulator.cpp)

target_link_libraries(emulator
    ${catkin_LIBRARIES})

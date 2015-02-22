cmake_minimum_required(VERSION 2.8.4)
project(MultipleTracking_Tutorial)
find_package( OpenCV REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES tracking.cpp RobotDesc.cpp RobotDesc.h)
add_executable(tracking ${SOURCE_FILES})
target_link_libraries( tracking ${OpenCV_LIBS} )

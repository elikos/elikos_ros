include_directories(Translator)

add_executable(tf_translater
    Translator/main.cpp
    Translator/Translator.h
    Translator/Translator.cpp)

target_link_libraries( tf_translater ${catkin_LIBRARIES})

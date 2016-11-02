add_executable(elikos_localization
    Localizaton.cpp
    MessageHandler.h
    MessageHandler.cpp
    ImageProcessor.cpp
    ImageProcessor.h
    context/Line.cpp
    context/Line.h
    context/Arena.cpp
    context/Arena.h
    context/Intersection.cpp
    context/Intersection.h
    context/LineGroup.cpp
    context/LineGroup.h
    algorithm/DBSCAN.cpp 
    algorithm/DBSCAN.h
    algorithm/PerspectiveRANSAC.h
    algorithm/PerspectiveRANSAC.cpp
    context/Grid.cpp 
    context/Grid.h)


include_directories(${catkin_INCLUDE_DIRS}
                    ${OpenCV_LIBS}
                    context
                    algorithm)

target_link_libraries(elikos_localization ${catkin_LIBRARIES} ${OpenCV_LIBS})


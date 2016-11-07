

add_executable(elikos_localization
    Localizaton.cpp
    MessageHandler.h
    MessageHandler.cpp
    context/Line.cpp
    context/Line.h
    context/LineGroup.cpp
    context/LineGroup.h)

add_library(arena-detection 
    arena-detection/ImageProcessor.h 
    arena-detection/ImageProcessor.cpp 
    arena-detection/PerspectiveTransform.h 
    arena-detection/PerspectiveTransform.cpp
    arena-detection/algorithm/DBSCAN.cpp
    arena-detection/algorithm/DBSCAN.h
    arena-detection/algorithm/RANSAC.h
    arena-detection/algorithm/RANSAC.cpp)

include_directories(${catkin_INCLUDE_DIRS}
                    ${OpenCV_LIBS}
                    context
                    arena-detection
                    arena-detection/algorithm)

target_link_libraries(elikos_localization 
    arena-detection
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBS})


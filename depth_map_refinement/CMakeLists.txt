cmake_minimum_required(VERSION 2.8)

PROJECT(DepthMapRefinement)

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/OpenCV/") 
#set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-msse -msse2 -msse3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3 -msse4")

find_package(OpenCV REQUIRED)

InCLUDE_DIRECTORIES(include 
    ${OPENCV_INCLUDE_DIR} 
) 

AUX_SOURCE_DIRECTORY(src DIR_SRCS)

set(src_path src/alphaBlend.cpp 
    src/bilateralFilter.cpp 
    src/depthEval.cpp 
    src/depthFilter.cpp 
    src/guidedFilter.cpp 
    src/jointBilateralFilter.cpp 
    src/jointNearest.cpp 
    src/minmaxFilter.cpp 
    src/trilateralFilter.cpp 
    src/util.cpp 
    src/split.cpp
)
#src/viewSynthesis.cpp 


add_library(algorithm_lib ${src_path} )
target_link_libraries(algorithm_lib ${OpenCV_LIBRARIES})

add_executable(${PROJECT_NAME} src/main.cpp)


target_link_libraries(${PROJECT_NAME} 
    algorithm_lib
    ${OpenCV_LIBRARIES}
)



cmake_minimum_required(VERSION 2.8.12)
project(catch)

# Set c++11 cmake flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3 -msse4  -std=c++11 -fext-numeric-literals")#

LINK_DIRECTORIES("/opt/DahuaTech/MVviewer/lib")

FIND_PACKAGE(freenect2 REQUIRED)
include_directories( ${freenect2_INCLUDE_DIRS} )
link_directories( ${freenect2_LIBRARY_DIRS} )


# Find rquired packeges
find_package(OpenCV 3.4.0 REQUIRED)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME} 
src/main.cpp 
src/streamRetrieve.cpp
src/frameCapture.cpp
src/Timer.cpp
src/frameCapture.cpp
src/depthRegistration.cpp
)

target_link_libraries(${PROJECT_NAME} ${freenect2_LIBRARIES} )
add_library(depth_refinement_lib 
depth_map_refinement/bilateralFilter.cpp
depth_map_refinement/depthFilter.cpp
depth_map_refinement/jointBilateralFilter.cpp
depth_map_refinement/jointNearest.cpp
depth_map_refinement/trilateralFilter.cpp
depth_map_refinement/split.cpp
)
target_link_libraries(${PROJECT_NAME} depth_refinement_lib)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})


# Enable OPENCV 
add_definitions(-DOPENCV)

target_link_libraries(${PROJECT_NAME}    
${OpenCV_LIBRARIES}
${catkin_LIBRARIES}
${OpenCV_LIBS}
)

FIND_PACKAGE(OpenCL)
include_directories( ${OpenCL_INCLUDE_DIRS} )
target_link_libraries(${PROJECT_NAME} ${OpenCL_LIBRARIES})
 
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES})

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
target_link_libraries( ${PROJECT_NAME} ${Eigen_LIBRARIES} )
add_definitions(-DEigen_DONT_VECTORIZE)
add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)


target_link_libraries(${PROJECT_NAME}  MVSDK)

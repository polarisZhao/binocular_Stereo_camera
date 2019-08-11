cmake_minimum_required(VERSION 3.6)
project(pcltest)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# opencv
FIND_PACKAGE( OpenCV REQUIRED )
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )

# pcl
find_package(PCL 1.6 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcltest pcltest.cpp)
target_link_libraries( pcltest ${OpenCV_LIBS} )
target_link_libraries( pcltest ${PCL_LIBRARIES} )

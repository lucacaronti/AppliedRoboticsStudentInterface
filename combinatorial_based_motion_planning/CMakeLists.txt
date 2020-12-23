cmake_minimum_required(VERSION 2.8.3)
project(voronoi)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)

## Specify additional locations of header files
add_library(voronoi_libraries STATIC
   src/voronoi.cpp
)

target_include_directories(voronoi_libraries PUBLIC inc)

add_executable(voronoi 
   src/voronoi.cpp
)
target_link_libraries(voronoi
  voronoi_libraries
  ${OpenCV_LIBRARIES}
)

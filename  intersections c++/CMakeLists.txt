cmake_minimum_required(VERSION 2.8.3)
project(intersections)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)

## Specify additional locations of header files
add_library(intersections_libraries STATIC
   intersections.cpp
)

target_include_directories(intersections_libraries PUBLIC inc)

add_executable(intersections 
	intersections.cpp
)
target_link_libraries(intersections
  intersections_libraries
  ${OpenCV_LIBRARIES}
)

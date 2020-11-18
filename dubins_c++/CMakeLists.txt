cmake_minimum_required(VERSION 2.8.3)
project(dubins)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)

## Specify additional locations of header files
add_library(dubins_libraries STATIC
   dubins.cpp
   primitives.cpp
   inc/CvPlot/cvplot.h
)

target_include_directories(dubins_libraries PUBLIC inc)

add_executable(dubins 
	dubins.cpp
)
target_link_libraries(dubins
  dubins_libraries
  ${OpenCV_LIBRARIES}
)

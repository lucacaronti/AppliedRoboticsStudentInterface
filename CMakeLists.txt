cmake_minimum_required(VERSION 2.8.3)
project(student_proect)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED )
find_package(project_interface REQUIRED )

## Specify additional locations of header files
include_directories(
 include 
 ${OpenCV_INCLUDE_DIRS}
 ${project_interface_INCLUDE_DIRS}
)

## CREATE EXECUTABLE LIBRARY
add_library(student SHARED
   src/student_interface.cpp
)

target_link_libraries(student
  ${OpenCV_LIBRARIES}  
)

## COMPILE DEMO EXECUTABLE
add_executable(demo_dilatation_erosion
  src/demo_shape_detection/demo_dilatation_erosion.cpp
)
add_executable(demo_hsv_filter
  src/demo_shape_detection/demo_hsv_filter.cpp
)
add_executable(demo_smoothing
  src/demo_shape_detection/demo_smoothing.cpp
)
add_executable(demo_filters
  src/demo_shape_detection/demo_filters.cpp
)
add_executable(demo_rgb_filter
  src/demo_shape_detection/demo_rgb_filter.cpp
)
add_executable(demo_full_example
  src/demo_shape_detection/demo_full_example.cpp
)
add_executable(demo_simple_shape_detection
  src/demo_shape_detection/demo_simple_shape_detection.cpp
)

target_link_libraries(demo_dilatation_erosion
  ${OpenCV_LIBRARIES}
)
target_link_libraries(demo_hsv_filter
  ${OpenCV_LIBRARIES}
)
target_link_libraries(demo_smoothing
  ${OpenCV_LIBRARIES}
)
target_link_libraries(demo_filters
  ${OpenCV_LIBRARIES}
)
target_link_libraries(demo_rgb_filter
  ${OpenCV_LIBRARIES}
)
target_link_libraries(demo_full_example
  ${OpenCV_LIBRARIES}
)
target_link_libraries(demo_simple_shape_detection
  ${OpenCV_LIBRARIES}
)

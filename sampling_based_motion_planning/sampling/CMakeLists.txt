# cmake_minimum_required(VERSION 2.8.3)
cmake_minimum_required(VERSION 3.19)
project(sampling)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)

add_executable(sampling 
	sampling.cpp
)

target_link_libraries(sampling
	${OpenCV_LIBRARIES}
)
# cmake_minimum_required(VERSION 2.8.3)
cmake_minimum_required(VERSION 3.5.1)
project(sbmp)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Only if there is an error:
# add_compile_options(-I/usr/include/opencv4/)
add_compile_options(-I../../dubins_c++/inc/)

find_package(OpenCV REQUIRED)


## Specify additional locations of header files
add_library(sbmp_libraries STATIC
	src/sbmp.cpp	
	src/sampling.cpp
	src/hammersley.cpp
	src/halton.cpp
	src/dijkstra.cpp
)

target_include_directories(sbmp_libraries PUBLIC inc)

add_executable(sbmp 
	src/sbmp.cpp
)

target_link_libraries(sbmp
	${OpenCV_LIBRARIES}
	sbmp_libraries
)
# cmake_minimum_required(VERSION 2.8.3)
cmake_minimum_required(VERSION 3.5.1)
project(sbmp)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -Wall)

# Only if there is an error:
# add_compile_options(-I/usr/include/opencv4/)
add_compile_options(-I../../dubins_c++/inc/)

find_package(OpenCV REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)


## Specify additional locations of header files
add_library(sbmp_libraries STATIC
	src/sbmp.cpp	
	src/sampling.cpp
	src/dubins.cpp
	src/dubins_primitives.cpp
	src/intersections.cpp
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
	pthread
)
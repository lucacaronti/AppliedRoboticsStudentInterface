# cmake_minimum_required(VERSION 2.8.3)
cmake_minimum_required(VERSION 3.5.1)
project(sampling)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)


## Specify additional locations of header files
add_library(sampling_libraries STATIC
	src/sampling.cpp
	src/hammersley.cpp
	src/halton.cpp
	# inc/CvPlot/cvplot.h
	../../dubins_c++/inc/CvPlot/cvplot.h
)

target_include_directories(sampling_libraries PUBLIC inc)

add_executable(sampling 
	src/sampling.cpp
)

target_link_libraries(sampling
	sampling_libraries
	${OpenCV_LIBRARIES}
)
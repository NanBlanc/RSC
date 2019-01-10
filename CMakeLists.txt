cmake_minimum_required (VERSION 3.0)
project (RSC)

# The version number.
set (Tutorial_VERSION_MAJOR 0)
set (Tutorial_VERSION_MINOR 1)

set(OpenCV_DIR "C:/programmation/lib/opencv/build")
find_package(OpenCV REQUIRED core)

set(PCL_DIR "C:/programmation/lib/PCL 1.8.1/cmake")
find_package(PCL 1.8 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS}
./)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_library(../
# add the executable
add_executable (RSC 
main.cpp
./fonctions.cpp)

# add libs 
target_link_libraries (RSC 
${OpenCV_LIBS} 
${PCL_COMMON_LIBRARIES} 
${PCL_IO_LIBRARIES})

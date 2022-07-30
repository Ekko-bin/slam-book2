cmake_minimum_required(VERSION 2.8)
set(CMAKE_BUILD_TYPE DEBUG)
find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(pose3d-2d pose3d-2d.cpp)
target_link_libraries(pose3d-2d ${OpenCV_LIBS})


cmake_minimum_required(VERSION 3.0)
project(jiabinorb)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS}
                                           ${PROJECT_SOURCE_DIR}/include)

add_executable( ${PROJECT_NAME}  main/main.cpp
                                                                               src/orb_detect.cpp)
                                                                               
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
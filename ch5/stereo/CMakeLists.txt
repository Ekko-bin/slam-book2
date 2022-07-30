find_package(Pangolin REQUIRED)


include_directories("/usr/include/eigen3")
include_directories("usr/icnlude/eigen3/Eigen")
add_executable(stereoVision stereoVision.cpp)
target_link_libraries(stereoVision ${OpenCV_LIBS} ${Pangolin_LIBRARIES})
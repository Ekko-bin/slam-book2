cmake_minimum_required( VERSION 2.8 )
project( geometry )

# 添加Eigen头文件
include_directories(
        ${OpenCV_INCLUDE_DIRS}
        "/usr/include/eigen3/"
)

find_package(OpenCV 3 REQUIRED)

add_executable(eigenGeometry eigenGeometry.cpp)
add_executable(demoG demoG.cpp)
add_executable(demoG1 demoG1.cpp)
add_executable(tezt_wechat tezt_wechat.cpp)
target_link_libraries(tezt_wechat ${OpenCV_LIBS})

add_executable(testenlurangle testenlurangle.cpp)
target_link_libraries(testenlurangle ${OpenCV_LIBS})


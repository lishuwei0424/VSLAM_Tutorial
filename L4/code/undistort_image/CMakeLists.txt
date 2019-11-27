cmake_minimum_required(VERSION 3.5)
project(undistort_image)

set(CMAKE_CXX_STANDARD 14)

set(APP_SOURCE   undistort_image.cpp)

#opencv3
find_package(OpenCV REQUIRED)

#包含头文件
include_directories(${OpenCV_DIRS})

add_executable(undistort_image undistort_image.cpp)
target_link_libraries(undistort_image ${OpenCV_LIBS})
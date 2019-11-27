cmake_minimum_required(VERSION 3.5)
project(gaussnewton)

set(CMAKE_CXX_STANDARD 14)

#opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_DIRS})
#Eigen
include_directories("/usr/include/eigen3")

add_executable(gaussnewton gaussnewton.cpp)
target_link_libraries(gaussnewton ${OpenCV_LIBS})

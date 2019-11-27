cmake_minimum_required(VERSION 3.5)
project(disparity)

set(CMAKE_CXX_STANDARD 14)

find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS})
include_directories($OpenCV_DIRS})

add_executable(disparity disparity.cpp)
target_link_libraries(disparity   ${Pangolin_LIBRARIES} ${OpenCV_LIBS})
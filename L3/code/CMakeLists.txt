# writed by zhang ning 2018/10/22
cmake_minimum_required( VERSION 2.8 )

project(draw_trajectory)

set( CMAKE_BUILD_TYPE "Debug" )

set( CMAKE_CXX_FLAGS "-std=c++11 -O3" ) 



find_package( Sophus REQUIRED)
find_package( Pangolin REQUIRED)


include_directories( "/usr/include/eigen3" )
include_directories( ${Sophus_INCLUDE_DIRS} )
include_directories( ${Pangolin_INCLUDE_DIRS} )

add_executable( draw_trajectory draw_trajectory.cpp)

target_link_libraries( draw_trajectory ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} )


add_executable( trajectory_error trajectory_error.cpp)

target_link_libraries( trajectory_error ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} )




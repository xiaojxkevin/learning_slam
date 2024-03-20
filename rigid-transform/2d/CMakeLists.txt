cmake_minimum_required(VERSION 3.22)
project( 2d-transformation )

set( CMAKE_CXX_FLAGS "-std=c++17" )

find_package( Sophus REQUIRED )
include_directories( 
    ${Sophus_INCLUDE_DIRS}
    "/usr/include/eigen3"
)

add_executable( se2 se2.cc )

target_link_libraries(se2 fmt)
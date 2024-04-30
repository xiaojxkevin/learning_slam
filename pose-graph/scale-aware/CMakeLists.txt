cmake_minimum_required(VERSION 3.22)
project( scale_drift )

set( CMAKE_CXX_FLAGS "-std=c++17" )

# 添加cmake模块以使用ceres库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# 寻找G2O
find_package( G2O REQUIRED )
include_directories( 
    ${G2O_INCLUDE_DIRS}
    "/usr/include/eigen3"
)

add_executable( scale_drift scale_drift.cc )
target_link_libraries( scale_drift 
                    g2o_core g2o_stuff 
                    fmt )

add_executable( scale_jump scale_jump.cc )
target_link_libraries( scale_jump
                    g2o_core g2o_stuff 
                    fmt)

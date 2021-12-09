cmake_minimum_required(VERSION 3.5)
project(time_sync)
set(CMAKE_CXX_STANDARD 11)

#ros
find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        geometry_msgs
        )
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(include)

add_library(time_sync src/synchronizer.cpp)
add_executable(time_sync_demo demo.cpp)
target_link_libraries(time_sync_demo PRIVATE ${roscpp_LIBRARIES} time_sync)


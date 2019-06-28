cmake_minimum_required(VERSION 2.8.3)
project(new_feature_tracker)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        roscpp
        sensor_msgs
        OpenCV
)

add_subdirectory(src/edge_finder)

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        src/edge_finder
)

add_executable(new_feature_tracker src/new_feature_tracker_node.cpp)

target_link_libraries(new_feature_tracker
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        edge_finder
        /usr/lib/x86_64-linux-gnu/libv4l2.so.0
        /usr/lib/lapack/liblapack.so
        )

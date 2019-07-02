cmake_minimum_required(VERSION 2.8.3)
project(new_feature_tracker)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
        OpenCV
        cv_bridge
        roscpp
        sensor_msgs
        camera_model
        std_msgs
)

add_subdirectory(src/edge_finder)

catkin_package()

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        src/edge_finder
)

add_executable(new_feature_tracker
        src/new_feature_tracker_node.cpp
        src/parameters.cpp
        src/parameters.h
        src/new_feature_tracker.cpp
        src/new_feature_tracker.h
        src/edge_points_extractor.cpp
        src/edge_points_extractor.h
        )

target_link_libraries(new_feature_tracker
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        edge_finder

        /usr/lib/x86_64-linux-gnu/libv4l2.so.0 #这两个so库是和TooN库相关的，在边缘检测算法中需要用到
        /usr/lib/x86_64-linux-gnu/libX11.so
        /usr/lib/lapack/liblapack.so
        )

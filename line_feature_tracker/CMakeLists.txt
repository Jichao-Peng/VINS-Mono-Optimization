cmake_minimum_required(VERSION 2.8.3)
project(line_feature_tracker)

find_package(catkin REQUIRED COMPONENTS
        OpenCV
        cv_bridge
        roscpp
        sensor_msgs
        std_msgs
        Eigen3
)

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/line_descriptor/include/
)

add_executable(line_feature_tracker
        src/feature_traker_node.cpp
        src/config.cpp
        src/auxiliar.cpp)

target_link_libraries(line_feature_tracker
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${PROJECT_SOURCE_DIR}/line_descriptor/lib/liblinedesc.so)



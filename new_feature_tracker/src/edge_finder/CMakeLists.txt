cmake_minimum_required(VERSION 2.8.3)

include_directories(UtilLib VideoLib)

add_library(edge_finder
        edge_finder.cpp
        sspace.cpp
        iigauss.cpp
        iimage.cpp
        VideoLib/video_io.cpp
        )
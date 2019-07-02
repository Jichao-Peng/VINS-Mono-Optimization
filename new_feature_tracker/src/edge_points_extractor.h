//
// Created by leo on 19-6-30.
//

#ifndef SRC_EDGE_POINTS_EXTRACTOR_H
#define SRC_EDGE_POINTS_EXTRACTOR_H

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include "parameters.h"

#include "edge_finder.h"
#ifdef Bool //因为在edge_finder调用的底层出现了Bool的define，所以需要Bool，好坑啊...
    #undef Bool
#endif
#include <sensor_msgs/Image.h>


using namespace std;

class EdgePointsExtrator {
public:
    EdgePointsExtrator();

    bool extractEdgePoints(const sensor_msgs::ImageConstPtr &imgMsg, vector<cv::Point2f>& edge_points);

private:
    bool imgMsg2Rebvo(const sensor_msgs::ImageConstPtr &imgMsg,
                      shared_ptr<rebvo::Image<rebvo::RGB24Pixel> > &imgRebvo);


    rebvo::cam_model* camModel;
    rebvo::edge_finder* edgeFinder;
    rebvo::sspace* rebvoSpace;
};


#endif //SRC_EDGE_POINTS_EXTRACTOR_H

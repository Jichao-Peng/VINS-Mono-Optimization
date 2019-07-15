//
// Created by leo on 19-7-15.
//

#ifndef SRC_LINE_FEATURE_MANAGER_H
#define SRC_LINE_FEATURE_MANAGER_H

#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <ros/console.h>
#include <ros/assert.h>

#include "parameters.h"
using namespace std;

class LineFeaturePerFrame
{
public:
    LineFeaturePerFrame(const Eigen::Matrix<double, 4, 1> &line, double td)
    {
        start_point.x() = line(0);
        start_point.y() = line(1);
        end_point.x() = line(2);
        end_point.y() = line(3);
    }
    Vector2d start_point;
    Vector2d end_point;
};

class LineFeaturePerId
{
public:
    LineFeaturePerId(int _line_feature_id, int _start_frame)
    :line_feature_id(_line_feature_id), start_frame(_start_frame)
    {
    }

    vector<LineFeaturePerFrame> line_feature_per_frame;
    int start_frame;
    int line_feature_id;
};

class LineFeatureManager
{
public:
    list<LineFeaturePerId> line_feature;
    void addFeature(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 4, 1>>>> &line_image, double td);
};


#endif //SRC_LINE_FEATURE_MANAGER_H

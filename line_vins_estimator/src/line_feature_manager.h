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
        pts_s.x() = line(0);
        pts_s.y() = line(1);
        pts_s.z() = 1;
        pts_e.x() = line(2);
        pts_e.y() = line(3);
        pts_e.z() = 1;
    }
    Vector3d pts_s;
    Vector3d pts_e;
};

class LineFeaturePerId
{
public:
    LineFeaturePerId(int _line_feature_id, int _start_frame)
    :start_frame(_start_frame), line_feature_id(_line_feature_id)
    {
    }

    int endFrame()
    {
        return start_frame + line_feature_per_frame.size() - 1;
    }

    vector<LineFeaturePerFrame> line_feature_per_frame;
    int start_frame;
    int line_feature_id;

    Vector3d Lw_n;//这两个是空间直线世界坐标系下的普吕克坐标
    Vector3d Lw_d;
};

class LineFeatureManager
{
public:
    list<LineFeaturePerId> line_feature;
    void addFeature(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 4, 1>>>> &line_image, double td);
    void removeBack();
    void removeFront(int frame_count);
    //TODO：实现直线三角化函数
    void line_triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);
};


#endif //SRC_LINE_FEATURE_MANAGER_H

//
// Created by leo on 19-7-15.
//

#ifndef SRC_LINE_FEATURE_MANAGER_H
#define SRC_LINE_FEATURE_MANAGER_H

#include <eigen3/Eigen/Dense>
#include "utility/utility.h"
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

    vector<double> line;
    int used_num;
};

class LineFeatureManager
{
public:
    LineFeatureManager(Matrix3d _Rs[]);
    void setRic(Matrix3d _ric[]);
    void clearState();
    void removeFailures();

    list<LineFeaturePerId> line_feature;
    void addFeature(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 4, 1>>>> &line_image, double td);
    void removeBack();
    void removeFront(int frame_count);
    //TODO：实现直线三角化函数
    void line_triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);
    vector<vector<double>> getLineVector();
    int getFeatureCount();
    void setLineFeature(vector<vector<double>> lineVector);

    const Matrix3d *Rs;
    Matrix3d ric[NUM_OF_CAM];
};


#endif //SRC_LINE_FEATURE_MANAGER_H

//
// Created by leo on 19-6-29.
//

#ifndef SRC_NEW_FEATURE_TRACKER_H
#define SRC_NEW_FEATURE_TRACKER_H

#pragma once

#include <queue>
#include <cstdio>
#include <csignal>
#include <iostream>
#include <execinfo.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>

#include "edge_points_extractor.h"
#ifdef Success //因为在edge_finder调用的底层出现了Success的define，所以需要undef，好坑啊...
    #undef Success
#endif
#include <eigen3/Eigen/Dense>
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"


using namespace std;
using namespace camodocal;
using namespace Eigen;

class FeatureTracker {
public:
    FeatureTracker();

    void readIntrinsicParameter(const string &calib_file);

    void readImage(sensor_msgs::ImageConstPtr img_msg);

    bool inBorder(const cv::Point2f &pt);

    void rejectWithF();

    void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);

    void reduceVector(vector<int> &v, vector<uchar> status);

    bool updateID(unsigned int i);

    void addPoints();

    void undistortedPoints();

    camodocal::CameraPtr m_camera; // VINS自带的相机模型

    cv::Mat fisheye_mask; // 鱼眼相机mask，用来去除边缘噪点

    vector<cv::Point2f> edge_pts; //提取的边缘点
    double prev_time, cur_time;//时间
    vector<cv::Point2f> pts_velocity;//当前帧相对前一帧特征点在归一化平明上的移动速度
    vector<cv::Point2f> prev_un_pts, cur_un_pts;//归一化平面上消除畸变的坐标
    map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map;//对应的id和像素位置
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts; // 这三个分别是前一帧，当前帧和后一帧的关键点
    cv::Mat prev_img, cur_img, forw_img;//图像

    int n_id; // 相当于一个计数器

    vector<int> ids;
    vector<int> track_cnt;

    EdgePointsExtrator* edge_points_extrator;
};


#endif //SRC_NEW_FEATURE_TRACKER_H

//
// Created by leo on 19-6-29.
//

#include "new_feature_tracker.h"

FeatureTracker::FeatureTracker()
{
}


//最主要函数，读取图片并进行处理
void FeatureTracker::readImage(sensor_msgs::ImageConstPtr img_msg)
{
    cur_time = img_msg->header.stamp.toSec();
    cv::Mat img_show = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;

    if (forw_img.empty())
    {
        //如果当前帧的图像数据forw_img为空，说明当前是第一次读入图像数据
        prev_img = cur_img = forw_img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;;
    }
    else
    {
        //否则，说明之前就已经有图像读入
        forw_img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;;
    }

    //此时forw_pts还保存的是上一帧图像中的特征点，所以把它清除
    forw_pts.clear();

    if(cur_pts.size()>0)//第一张图片进来的时候cur_pts的size肯定为0
    {
        //下面会进行tracking
        vector<uchar> status;
        vector<float> err;
        cout<<"the size of points BEFORE calculate optical flow: "<<cur_pts.size()<<endl;
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

        //将位于图像边界外的点标记为0
        for (int i = 0; i < int(forw_pts.size()); i++)
            if (status[i] && !inBorder(forw_pts[i]))
                status[i] = 0;

        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(cur_un_pts, status);
        reduceVector(track_cnt, status);
        cout<<"the size of points AFTER calculate optical flow: "<<forw_pts.size()<<endl<<endl;
    }

    //数值n代表被追踪的次数，数值越大，说明被追踪的就越久
    for (auto &n : track_cnt)
    {
        n++;
    }

    if(PUB_THIS_FRAME)//程序目前是控制在10帧左右，不PUB的话就只进行Tracking
    {
        //TODO:源代码在这里进行了点的筛选，算法可以改进
        cout<<"the size of points BEFORE reject with F: "<<forw_pts.size()<<endl;
        rejectWithF();
        cout<<"the size of points AFTER reject with F: "<<forw_pts.size()<<endl<<endl;

        int n_max_cnt = MAX_CNT- static_cast<int>(forw_pts.size());//如果有达到最大点数就不需要再提取新的关键点
        cout<<n_max_cnt<<endl<<endl;
        if(n_max_cnt>0)
        {
            edge_points_extrator->extractEdgePoints(img_msg, edge_pts);
            cout<<"abstract "<<edge_pts.size()<<" edge points!!!"<<endl<<endl;
        } else
        {
            edge_pts.clear();
        }
    }
    else
    {
        edge_pts.clear();
    }

    //下面画的是边界点
//    for(auto p:edge_pts)
//    {
//        cv::circle(img_show, p, 1, cv::Scalar(255,0,0), 1);
//    }
//    cv::imshow("image",img_show);
//    cv::waitKey(1);


    //将刚刚提取的边界点添加到flow_pts中去
    cout<<"the size of points BEFORE add points: "<<forw_pts.size()<<endl;
    addPoints();
    cout<<"the size of points AFTER add points: "<<forw_pts.size()<<endl<<endl;

    cout<<"--------------------------------------------------------"<<endl;

    //把当前帧的数据cur_img、cur_pts赋给当前帧prev_img、prev_pts
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;

    //把下一帧的数据forw_img、forw_pts赋给当前帧cur_img、cur_pts
    cur_img = forw_img;
    cur_pts = forw_pts;

    //根据不同的相机模型去畸变矫正和转换到归一化坐标系上，计算速度
    undistortedPoints();
    prev_time = cur_time;
}








//读取参数
void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
    edge_points_extrator = new EdgePointsExtrator;//需要在读完参数之后再进行初始化
}


// 利用基础矩阵删除部分特征点
void FeatureTracker::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {

            Eigen::Vector3d tmp_p;
            //根据不同的相机模型将二维坐标转换到三维坐标
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            //转换为归一化像素坐标
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        //调用cv::findFundamentalMat对un_cur_pts和un_forw_pts计算F矩阵
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
    }
}

//判断跟踪的特征点是否在图像边界内
bool FeatureTracker::inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    //cvRound()：返回跟参数最接近的整数值，即四舍五入；
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

// 去除无法跟踪的特征点
void FeatureTracker::reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

// 去除无法追踪到的特征点
void FeatureTracker::reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

// 更新特征点id
bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}


// 添将新检测到的特征点n_pts
void FeatureTracker::addPoints()
{
    for (auto &p : edge_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);//新提取的特征点id初始化为-1
        track_cnt.push_back(1);//新提取的特征点被跟踪的次数初始化为1
    }
}

//对角点图像坐标进行去畸变矫正，转换到归一化坐标系上，并计算每个角点的速度。
void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();
    cur_un_pts_map.clear();

    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;

        //根据不同的相机模型将图像像素坐标转移到归一化平面上的坐标
        m_camera->liftProjective(a, b);

        //再延伸到深度归一化平面上
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
    }

    // 计算每个特征点的速度到pts_velocity
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            if (ids[i] != -1)//新提取的特征点前一帧没有特征点，因此不计算速度或者说速度为零
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                if (it != prev_un_pts_map.end())
                {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    prev_un_pts_map = cur_un_pts_map;
}





//
// Created by leo on 19-6-28.
//

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>

#include "parameters.h"
#include "new_feature_tracker.h"

using namespace std;


//全局变量放开头..写C的感觉...
double first_image_time = 0, last_image_time = 0;
int pub_count = 1;
bool first_image_flag = true;
bool init_pub = 0;

ros::Publisher pub_img,pub_match,pub_restart;
FeatureTracker trackerData[NUM_OF_CAM];

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    // 判断是否是第一帧,主要是记录了一下时间戳
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = img_msg->header.stamp.toSec();//记录图像帧的时间
        last_image_time = img_msg->header.stamp.toSec();//上一帧的时间戳
        return;
    }

    // 通过判断时间间隔，有问题则restart
    if (img_msg->header.stamp.toSec() - last_image_time > 1.0 || img_msg->header.stamp.toSec() < last_image_time)//时间戳断开了就立马重启
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true;
        last_image_time = 0;
        pub_count = 1;
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }
    last_image_time = img_msg->header.stamp.toSec();

    // 发布频率控制，并不是每读入一帧图像，就要发布特征点，判断间隔时间内的发布次数
    if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // 时间间隔内的发布频率十分接近设定频率时，更新时间间隔起始时刻，并将数据发布次数置0
        if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = img_msg->header.stamp.toSec();
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    for (int i = 0; i < NUM_OF_CAM; i++) {
        ROS_DEBUG("processing camera %d", i);
        if (i != 1 || !STEREO_TRACK)//单目，我们用的相机就是单目的
        {
            trackerData[i].readImage(img_msg);
        }
//        else//双目
//        {
//            trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
//        }
    }

    //更新全局ID
    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }

    if (SHOW_TRACK)
    {
        cv_bridge::CvImageConstPtr img_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat show_img = img_ptr->image;
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            cv::Mat tmp_img = show_img.rowRange(i * ROW, (i + 1) * ROW);

            //显示追踪状态，越红越好，越蓝越不行
            for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
            {
                double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                //draw speed line

                Vector2d tmp_cur_un_pts (trackerData[i].cur_un_pts[j].x, trackerData[i].cur_un_pts[j].y);
                Vector2d tmp_pts_velocity (trackerData[i].pts_velocity[j].x, trackerData[i].pts_velocity[j].y);
                Vector3d tmp_prev_un_pts;
                tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                tmp_prev_un_pts.z() = 1;
                Vector2d tmp_prev_uv;
                trackerData[i].m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                cv::line(tmp_img, trackerData[i].cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);
            }
        }
        cv::imshow("vis", show_img);
        cv::waitKey(5);
        //pub_match.publish(img_ptr->toImageMsg());
    }
}




int main(int argc, char **argv) {
    //ros初始化和设置句柄
    ros::init(argc, argv, "new_feature_tracker");
    ros::NodeHandle n("~");

    //设置logger的级别。 只有级别大于或等于level的日志记录消息才会得到处理。
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    //读取config中的一些配置参数,确定相机和IMU的外参是否准确，不准确可以自己标定，确定IMU和时间是否同步，不同步可以设置补偿时间
    readParameters(n);

    //读取每个相机实例读取对应的相机内参
    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

    //判断是否加入鱼眼mask来去除边缘噪声
    if(FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                ROS_INFO("load mask fail");
                ROS_BREAK();
            }
            else
                ROS_INFO("load mask success");
        }
    }

    //订阅话题IMAGE_TOPIC(/cam0/image_raw),执行回调函数
    ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);

    //发布feature，实例feature_points，跟踪的特征点，给后端优化用
    pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
    //发布feature_img，实例ptr，跟踪的特征点图，给RVIZ用和调试用
    pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);
    //发布restart
    pub_restart = n.advertise<std_msgs::Bool>("restart",1000);

    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */
    ros::spin();
    return 0;
}
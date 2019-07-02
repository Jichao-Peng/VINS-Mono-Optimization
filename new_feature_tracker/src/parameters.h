#pragma once
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

extern int ROW;//图像宽度
extern int COL;//图像高度
extern int FOCAL_LENGTH;//焦距
const int NUM_OF_CAM = 1;//相机的个数


extern std::string IMAGE_TOPIC;//图像的ROS TOPIC
extern std::string IMU_TOPIC;//IMU的ROS TOPIC
extern std::string FISHEYE_MASK;//鱼眼相机mask图的位置
extern std::vector<std::string> CAM_NAMES;//相机参数配置文件名
extern int MAX_CNT;//特征点最大个数
extern int MIN_DIST;//特征点之间的最小间隔
extern int WINDOW_SIZE;
extern int FREQ;//控制图像光流跟踪的频率
extern double F_THRESHOLD;//ransac算法的门限
extern int SHOW_TRACK;//是否发布跟踪点的图像
extern int STEREO_TRACK;//双目跟踪则为1
extern int EQUALIZE;//如果光太亮或太暗则为1，进行直方图均衡化
extern int FISHEYE;//如果是鱼眼相机则为1
extern bool PUB_THIS_FRAME;//是否需要发布特征点

void readParameters(ros::NodeHandle &n);

//---------------------------------------我是一条没睡够的分割线---------------------------------------
//下面是新添加的参数，为了和原来的参数发生冲突，全部以NEW_前缀开头
//构建高斯图像用的参数
extern double NEW_SIGMA_0;
extern double NEW_SIGMA_K;

//边缘检测用的参数
extern int NEW_DETECTOR_PLANE_FIT_SIZE;
extern double NEW_DETECTOR_POS_NEG_THRESH;
extern double NEW_DETECTOR_DOG_THRESH;
extern double NEW_DETECTOR_AUTO_GAIN;
extern double NEW_DETECTOR_MAX_THRESH;
extern double NEW_DETECTOR_MIN_THRESH;
extern double NEW_DETECTOR_THRESH;
extern int NEW_MAX_POINT;
extern int NEW_REFERENCE_POINT;

extern double NEW_FX;
extern double NEW_FY;
extern double NEW_CX;
extern double NEW_CY;

extern int NEW_EDGE_THRESH;



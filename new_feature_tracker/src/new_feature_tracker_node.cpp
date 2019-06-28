//
// Created by leo on 19-6-28.
//

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include "edge_finder.h"

using namespace std;

bool imgMsg2Rebvo(const sensor_msgs::ImageConstPtr &imgMsg,
                  shared_ptr<rebvo::Image<rebvo::RGB24Pixel> > &imgRebvo) {

    if (imgMsg->width != imgRebvo->Size().w
        || imgMsg->height != imgRebvo->Size().h) {
        return false;
    }

    if (imgMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0) {

        for (int y = 0; y < imgRebvo->Size().h; ++y) {
            for (int x = 0; x < imgRebvo->Size().w; ++x) {

                (*imgRebvo)(x, y).pix.r = imgMsg->data[imgMsg->step * y + x];
                (*imgRebvo)(x, y).pix.g = imgMsg->data[imgMsg->step * y + x];
                (*imgRebvo)(x, y).pix.b = imgMsg->data[imgMsg->step * y + x];
            }
        }

    } else if (imgMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) {

        for (int y = 0; y < imgRebvo->Size().h; ++y) {
            for (int x = 0; x < imgRebvo->Size().w; ++x) {

                (*imgRebvo)(x, y).pix.r = imgMsg->data[imgMsg->step * y + x * 3 + 0];
                (*imgRebvo)(x, y).pix.g = imgMsg->data[imgMsg->step * y + x * 3 + 1];
                (*imgRebvo)(x, y).pix.b = imgMsg->data[imgMsg->step * y + x * 3 + 2];
            }
        }

    } else if (imgMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0) {

        for (int y = 0; y < imgRebvo->Size().h; ++y) {
            for (int x = 0; x < imgRebvo->Size().w; ++x) {

                (*imgRebvo)(x, y).pix.b = imgMsg->data[imgMsg->step * y + x * 3 + 0];
                (*imgRebvo)(x, y).pix.g = imgMsg->data[imgMsg->step * y + x * 3 + 1];
                (*imgRebvo)(x, y).pix.r = imgMsg->data[imgMsg->step * y + x * 3 + 2];
            }
        }

    }else {
        return false;
    }

    return true;
}


//构建高斯图像的参数
double sigma0 = 3.56359;
double sigmaK = 1.2599;
//检测的参数
int detectorPlaneFitSize = 2;
double detectorPosNegThresh = 0.4;
double detectorDoGThresh = 0.095259868922420;
int maxPoint = 16000;
double detectThresh = 0.01;
int keyLineNum = 0;
int referencePoints = 12000;
double detectorAutoGain = 5e-7;
double detectorMaxThresh = 0.5;
double detectorMinThresh = 0.005;
//相机参数
float fx = 535.4;
float fy = 539.2;
float cx = 320.1;
float cy = 247.6;
double kc2 = 0;
double kc4 = 0;
double kc6 = 0;
double p1 = 0;
double p2 = 0;
unsigned int rows = 480;
unsigned int cols = 640;

rebvo::cam_model* camModel;
rebvo::edge_finder* edgeFinder;
rebvo::sspace* rebvoSpace;
void img_callback(const sensor_msgs::ImageConstPtr &img_msg) {
    //先将img_msg格式转成Image<rebvo::RGB24Pixel>格式
    rebvo::Size2D imageSize = {640,480};
    shared_ptr<rebvo::Image<rebvo::RGB24Pixel>> image = make_shared<rebvo::Image<rebvo::RGB24Pixel>>(imageSize);
    if(imgMsg2Rebvo(img_msg, image)) {

        //再将Image<rebvo::RGB24Pixel>格式转成Image<float>格式
        shared_ptr<rebvo::Image<float>> imageF = make_shared<rebvo::Image<float>>(imageSize);
        rebvo::Image<float>::ConvertRGB2BW(*imageF, *image);

        //计算高斯图像
        rebvoSpace->build(*imageF);
        edgeFinder->detect(rebvoSpace,
                detectorPlaneFitSize,
                detectorPosNegThresh,
                detectorDoGThresh,
                maxPoint,
                detectThresh,
                keyLineNum,
                referencePoints,
                detectorAutoGain,
                detectorMaxThresh,
                detectorMinThresh);

        cout<<keyLineNum<<endl;
    }
    else
    {
        cout<<"input image error"<<endl;
    }

    cv::Mat mat = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;

    for(int i = 0; i<edgeFinder->kn; i++)
    {
        cv::circle(mat, cv::Point(edgeFinder->kl[i].c_p.x,edgeFinder->kl[i].c_p.y), 2, cv::Scalar(255,0,0), -1);
    }

    cv::imshow("image",mat);
    cout<<mat.rows<<mat.cols<<endl;
    cv::waitKey(1);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "new_feature_tracker");
    ros::NodeHandle n("~");

    rebvo::cam_model::rad_tan_distortion distortion = {kc2, kc4, kc6, p1, p2};
    camModel = new rebvo::cam_model({cx,cy},{fx,fy},distortion,{cols,rows});
    edgeFinder = new rebvo::edge_finder(*camModel,255*3);
    rebvoSpace = new rebvo::sspace(sigma0, sigmaK, {cols,rows}, 3);
    cv::namedWindow("image");

    ros::Subscriber sub_img = n.subscribe("/camera/rgb/image_color", 100, img_callback);

    ros::spin();
}
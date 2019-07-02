//
// Created by leo on 19-6-30.
//

#include "edge_points_extractor.h"

EdgePointsExtrator::EdgePointsExtrator()
{
    camModel = new rebvo::cam_model({(float)NEW_CX,(float)NEW_CY},{(float)NEW_FX,(float)NEW_FY},{0,0,0,0,0},{(unsigned int)COL,(unsigned int)ROW});
    edgeFinder = new rebvo::edge_finder(*camModel,255*3);
    rebvoSpace = new rebvo::sspace(NEW_SIGMA_0, NEW_SIGMA_K, {(unsigned int)COL,(unsigned int)ROW}, 3);
}


bool EdgePointsExtrator::imgMsg2Rebvo(const sensor_msgs::ImageConstPtr &imgMsg,
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


bool EdgePointsExtrator::extractEdgePoints(const sensor_msgs::ImageConstPtr &imgMsg,
                                           vector<cv::Point2f> &edgePoints) {
    rebvo::Size2D imageSize = {(unsigned int) COL, (unsigned int) ROW};
    shared_ptr<rebvo::Image<rebvo::RGB24Pixel>> image = make_shared<rebvo::Image<rebvo::RGB24Pixel>>(imageSize);
    if (imgMsg2Rebvo(imgMsg, image)) {

        //再将Image<rebvo::RGB24Pixel>格式转成Image<float>格式
        shared_ptr<rebvo::Image<float>> imageF = make_shared<rebvo::Image<float>>(imageSize);
        rebvo::Image<float>::ConvertRGB2BW(*imageF, *image);

        int keylineNum = 0;
        //计算高斯图像
        rebvoSpace->build(*imageF);
        edgeFinder->detect(rebvoSpace,
                           NEW_DETECTOR_PLANE_FIT_SIZE,
                           NEW_DETECTOR_POS_NEG_THRESH,
                           NEW_DETECTOR_DOG_THRESH,
                           NEW_MAX_POINT,
                           NEW_DETECTOR_THRESH,
                           keylineNum,
                           NEW_REFERENCE_POINT,
                           NEW_DETECTOR_AUTO_GAIN,
                           NEW_DETECTOR_MAX_THRESH,
                           NEW_DETECTOR_MIN_THRESH);
        edgePoints.clear();

        for (int i = 0; i < keylineNum; i++) {
            if(edgeFinder->kl[i].c_p.x>NEW_EDGE_THRESH
            && edgeFinder->kl[i].c_p.y>NEW_EDGE_THRESH
            && edgeFinder->kl[i].c_p.x<COL-NEW_EDGE_THRESH
            && edgeFinder->kl[i].c_p.y<ROW-NEW_EDGE_THRESH)//边缘区域的点不稳定，去除掉
                edgePoints.emplace_back(cv::Point2f(edgeFinder->kl[i].c_p.x, edgeFinder->kl[i].c_p.y));
        }
        return true;
    }
}
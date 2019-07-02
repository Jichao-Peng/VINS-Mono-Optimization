#include "parameters.h"

std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;

//---------------------------------------我是一条没睡够的分割线---------------------------------------
//下面是新添加的参数，为了避免和原来的参数发生冲突，全部以NEW_前缀开头
//构建高斯图像用的参数
double NEW_SIGMA_0;
double NEW_SIGMA_K;

//边缘检测用的参数
int NEW_DETECTOR_PLANE_FIT_SIZE;
double NEW_DETECTOR_POS_NEG_THRESH;
double NEW_DETECTOR_DOG_THRESH;
double NEW_DETECTOR_AUTO_GAIN;
double NEW_DETECTOR_MAX_THRESH;
double NEW_DETECTOR_MIN_THRESH;
double NEW_DETECTOR_THRESH;
int NEW_MAX_POINT;
int NEW_REFERENCE_POINT;

double NEW_FX;
double NEW_FY;
double NEW_CX;
double NEW_CY;

int NEW_EDGE_THRESH;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    //config_file = readParam<std::string>(n, "config_file"); //最后跑的时候这里要改回来
    config_file = "/home/leo/catkin_ws2/src/VINS-Mono-Learning/config/my_config/my_config.yaml";
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    //std::string VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder"); //最后跑的时候这里要改回来
    std::string VINS_FOLDER_PATH = "/home/leo/catkin_ws2/src/VINS-Mono-Learning";

    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["imu_topic"] >> IMU_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];
    FISHEYE = fsSettings["fisheye"];
    if (FISHEYE == 1)
        FISHEYE_MASK = VINS_FOLDER_PATH + "config/fisheye_mask.jpg";
    CAM_NAMES.push_back(config_file);

    WINDOW_SIZE = 20;
    STEREO_TRACK = false;
    FOCAL_LENGTH = 460;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    //---------------------------------------我是一条没睡够的分割线---------------------------------------
    //下面是新添加的参数，为了和原来的参数发生冲突，全部以NEW_前缀开头
    //构建高斯图像用的参数
    NEW_SIGMA_0 = fsSettings["new_sigma0"];
    NEW_SIGMA_K = fsSettings["new_sigmaK"];

    //边缘检测用的参数
    NEW_DETECTOR_PLANE_FIT_SIZE = fsSettings["new_detector_plane_fit_size"];
    NEW_DETECTOR_POS_NEG_THRESH = fsSettings["new_detector_pos_neg_thresh"];
    NEW_DETECTOR_DOG_THRESH = fsSettings["new_detector_dog_thresh"];
    NEW_DETECTOR_AUTO_GAIN = fsSettings["new_detector_auto_gain"];
    NEW_DETECTOR_MAX_THRESH = fsSettings["new_detector_max_thresh"];
    NEW_DETECTOR_MIN_THRESH = fsSettings["new_detector_min_thresh"];
    NEW_DETECTOR_THRESH = fsSettings["new_detector_thresh"];
    NEW_MAX_POINT = fsSettings["new_max_point"];
    NEW_REFERENCE_POINT = fsSettings["new_reference_point"];

    cv::FileNode NEW_PROJECTOR_PARAMETERS = fsSettings["projection_parameters"];
    NEW_FX = static_cast<double>(NEW_PROJECTOR_PARAMETERS["fx"]);
    NEW_FY = static_cast<double>(NEW_PROJECTOR_PARAMETERS["fy"]);
    NEW_CX = static_cast<double>(NEW_PROJECTOR_PARAMETERS["cx"]);
    NEW_CY = static_cast<double>(NEW_PROJECTOR_PARAMETERS["cy"]);

    NEW_EDGE_THRESH = 10;
    fsSettings.release();
}

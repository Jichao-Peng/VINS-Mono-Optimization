#include "feature_tracker.h"

//FeatureTracker的static成员变量n_id初始化为0
int FeatureTracker::n_id = 0;
int FeatureTracker::n_line_id = 0;

//判断跟踪的特征点是否在图像边界内
bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    //cvRound()：返回跟参数最接近的整数值，即四舍五入；
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

bool inBorder(const cv::line_descriptor::KeyLine &line)
{
    const int BORDER_SIZE = 2;
    int start_x = cvRound(line.startPointX);
    int start_y = cvRound(line.startPointY);
    int end_x = cvRound(line.endPointX);
    int end_y = cvRound(line.endPointY);
    return (BORDER_SIZE <= start_x && start_x < COL - BORDER_SIZE && BORDER_SIZE <= start_y && start_y < ROW - BORDER_SIZE) ||
            (BORDER_SIZE <= end_x && end_x < COL - BORDER_SIZE && BORDER_SIZE <= end_y && end_y < ROW - BORDER_SIZE);
}

//去除无法跟踪的特征点
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

//去除无法追踪到的特征点
void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

//空的构造函数
FeatureTracker::FeatureTracker()
{
}

/**
 * @brief   对跟踪点进行排序并去除密集点
 * @Description 对跟踪到的特征点，按照被追踪到的次数排序并依次选点
 *              使用mask进行类似非极大抑制，半径为30，去掉密集点，使特征点分布均匀            
 * @return      void
*/
void FeatureTracker::setMask()
{
    if(FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    
    // prefer to keep features that are tracked for long time
    //构造(cnt，pts，id)序列
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

    //对光流跟踪到的特征点forw_pts，按照被跟踪到的次数cnt从大到小排序
    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    //清空cnt，pts，id并重新存入
    forw_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            //当前特征点位置对应的mask值为255，则保留当前特征点，将对应的特征点位置pts，id，被追踪次数cnt分别存入
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);

            //在mask中将当前特征点周围半径为MIN_DIST的区域设置为0，后面不再选取该区域内的点（使跟踪点不集中在一个区域上）
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

//添将新检测到的特征点n_pts
void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);//新提取的特征点id初始化为-1
        track_cnt.push_back(1);//新提取的特征点被跟踪的次数初始化为1
    }
}

//读入图片进行处理
void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)
{
    cv::Mat img;
    TicToc t_r;
    cur_time = _cur_time;

    //如果EQUALIZE=1，表示太亮或太暗
    if (EQUALIZE)//判断是否进行直方图均衡化处理
    {
        //自适应直方图均衡
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
        ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
    {
        img = _img;
    }

    if (forw_img.empty())
    {
        cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }
    forw_pts.clear();//此时forw_pts还保存的是上一帧图像中的特征点，所以把它清除


    //-------------------------------------上面是预处理----------------------------------------


    if (cur_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;

        //调用cv::calcOpticalFlowPyrLK()对前一帧的特征点cur_pts进行LK金字塔光流跟踪，得到forw_pts
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

        //将位于图像边界外的点标记为0
        for (int i = 0; i < int(forw_pts.size()); i++)
            if (status[i] && !inBorder(forw_pts[i]))
                status[i] = 0;

        //根据status,把跟踪失败的点剔除
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(cur_un_pts, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
    }

    //光流追踪成功,特征点被成功跟踪的次数就加1,数值代表被追踪的次数，数值越大，说明被追踪的就越久
    for (auto &n : track_cnt)
        n++;

    //PUB_THIS_FRAME=1 需要发布特征点
    if (PUB_THIS_FRAME)
    {
        //通过基本矩阵剔除outliers
        rejectWithF();

        ROS_DEBUG("set mask begins");
        TicToc t_m;

        setMask();//保证相邻的特征点之间要相隔30个像素,设置mask
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;

        //计算是否需要提取新的特征点
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != forw_img.size())
                cout << "wrong size " << endl;

            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);
        }
        else
            n_pts.clear();
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        ROS_DEBUG("add feature begins");
        TicToc t_a;

        //添将新检测到的特征点n_pts添加到forw_pts中，id初始化-1,track_cnt初始化为1.
        addPoints();

        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }


    //-------------------------------------上面是和点相关的数据处理----------------------------------------


    //不管怎样，都要检测下一帧的线段,然后进行匹配，相当于追踪
    m_line_feature.detectLineFeatures(forw_img, forw_lines, forw_ldesc, forw_min_length);

    if (cur_lines.size() != 0 && !cur_ldesc.empty())
    {
        vector<int> matches;
        m_line_feature.matchLineFeatures(cur_ldesc, forw_ldesc, matches);//matches的下标为前一帧匹配线，matches的下标对应的值为后一帧的匹配线，没有匹配的值为-1

        //对线特征进行过滤
        for(int i = 0; i<cur_lines.size(); i++)
        {
            if(!inBorder(cur_lines[i]))//如果上一帧的线端点都位于图片的边界上，该匹配作废
            {
                matches[i] = -1;
            }

            //中点相差60个像素的直线剔除
            if(!m_line_feature.judgeMidPoint(cur_lines[i], forw_lines[matches[i]]))
            {
                matches[i] = -1;
            }

            //角度相差30度的直线剔除
            if(!m_line_feature.judgeAngle(cur_lines[i], forw_lines[matches[i]]))
            {
                matches[i] = -1;
            }
        }

        vector<int> line_ids_temp(forw_lines.size(),-1);
        vector<int> line_track_cnt_temp(forw_lines.size(),0);
        for(int i = 0; i<matches.size(); i++)
        {
            if(matches[i] != -1)
            {
                line_ids_temp[matches[i]] = line_ids[i];//把上一帧的id赋值给这一帧的id
                line_track_cnt_temp[matches[i]] = line_track_cnt[i];//把上一阵的追踪数量赋值给这一帧的id
            }
        }
        for(int i = 0; i<line_ids_temp.size(); i++)
        {
            line_track_cnt_temp[i]++;
            if(line_ids_temp[i] == -1)//如果上一帧没有匹配到的
                line_ids_temp[i] = n_line_id;
            n_line_id++;
        }
        line_ids = line_ids_temp;
        line_track_cnt =  line_track_cnt_temp;
    }
    else
    {
        line_ids.resize(forw_lines.size(),-1);
        line_track_cnt.resize((forw_lines.size()),0);
        for(int i = 0; i<forw_lines.size(); i++)
        {
            line_track_cnt[i]++;
            if(line_ids[i] == -1)
                line_ids[i] = n_line_id;
            n_line_id++;
        }
    }

//    cout<<endl;
//    for(auto id:line_ids)
//        cout<<setw(5)<<id;
//    cout<<endl;
//    for(auto cnt:line_track_cnt)
//        cout<<setw(5)<<cnt;
//    cout<<endl;


    //-------------------------------------上面是和线相关的数据处理----------------------------------------


    //当下一帧图像到来时，当前帧数据就成为了上一帧发布的数据

    //把当前帧的数据forw_img、forw_pts赋给上一帧cur_img、cur_pts
    cur_img = forw_img;
    cur_pts = forw_pts;

    cur_lines = forw_lines;
    cur_ldesc = forw_ldesc.clone();

    //根据不同的相机模型去畸变矫正和转换到归一化坐标系上，计算速度
    undistortedPoints();
    prev_time = cur_time;
}

/**
 * @brief   通过F矩阵去除outliers
 * @Description 将图像坐标转换为归一化坐标
 *              cv::findFundamentalMat()计算F矩阵
 *              reduceVector()去除outliers 
 * @return      void
*/
void FeatureTracker::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");//才哟ing的是ransac的方法
        TicToc t_f;

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
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

//更新特征点id
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

//读取相机内参
void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}


//显示去畸变矫正后的特征点  name为图像帧名称
void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}


//对角点图像坐标进行去畸变矫正，转换到归一化坐标系上，并计算每个角点的速度。                       
void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();
    cur_un_pts_map.clear();
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());

    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;

        //根据不同的相机模型将二维坐标转换到三维坐标
        m_camera->liftProjective(a, b);

        //再延伸到深度归一化平面上
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        //printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }

    // 计算每个特征点的速度到pts_velocity
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            if (ids[i] != -1)
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


void FeatureTracker::undistortedline(Vector3d &un_pts_s, Vector3d &un_pts_e, cv::line_descriptor::KeyLine line)
{
    Vector2d pts_s, pts_e;
    pts_s<<line.startPointX, line.startPointY;
    pts_e<<line.endPointX, line.endPointY;
    m_camera->liftProjective(pts_s, un_pts_s);
    m_camera->liftProjective(pts_e, un_pts_e);
}
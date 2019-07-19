//
// Created by leo on 19-7-15.
//

#include "line_feature_manager.h"

LineFeatureManager::LineFeatureManager(Eigen::Matrix<double, 3, 3> *_Rs) : Rs(_Rs)
{
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric[i].setIdentity();
}

void LineFeatureManager::setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}

void LineFeatureManager::clearState()
{};

//TODO:后端优化求解失败的时候应该移除掉失败线
void LineFeatureManager::removeFailures()
{
    for (auto it = line_feature.begin(), it_next = line_feature.begin();
         it != line_feature.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == 2)
            line_feature.erase(it);
    }
}

void LineFeatureManager::addFeature(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 4, 1>>>> &line_image, double td)
{
    ROS_DEBUG("input line feature: %d", (int) line_image.size());
    ROS_DEBUG("num of line feature: %d", getFeatureCount());
    for (auto &id_lines : line_image)//遍历这一帧里面所有的线段
    {
        LineFeaturePerFrame line_f_per_fra(id_lines.second[0].second, td);

        int line_feature_id = id_lines.first;
        auto it = find_if(line_feature.begin(), line_feature.end(), [line_feature_id](const LineFeaturePerId &it) {
            return it.line_feature_id == line_feature_id;
        });//在line_feature里面寻找是否有相同id的线特征存在

        if (it == line_feature.end())//如果不存在则新建一个id的线特，并将这个特征在当前帧的观测添加进去
        {
            line_feature.push_back(LineFeaturePerId(line_feature_id, frame_count));
            line_feature.back().line_feature_per_frame.push_back(line_f_per_fra);
        }
        else if (it->line_feature_id == line_feature_id)//如果存在则在相应id的线特征中添加当前帧的观测
        {
            it->line_feature_per_frame.push_back(line_f_per_fra);
        }
    }
}

//边缘化最老帧相关的特征点
void LineFeatureManager::removeBack()
{
    for (auto it = line_feature.begin(), it_next = line_feature.begin();
         it != line_feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->line_feature_per_frame.erase(it->line_feature_per_frame.begin());
            if (it->line_feature_per_frame.size() == 0)
                line_feature.erase(it);
        }
    }
}


//边缘化次新帧相关的特征点
void LineFeatureManager::removeFront(int frame_count)
{
    for (auto it = line_feature.begin(), it_next = line_feature.begin();
         it != line_feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count)
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;//从起始帧到次新帧的位置
            //如果次新帧之前已经跟踪结束就什么都不做
            if (it->endFrame() < frame_count - 1)
                continue;

            //如果次新帧仍然被跟踪，则删除feature_per_frame中次新帧对应的FeaturePerFrame
            it->line_feature_per_frame.erase(it->line_feature_per_frame.begin() + j);
            if (it->line_feature_per_frame.size() == 0)
                line_feature.erase(it);
        }
    }
}

//返回当前线特征的数量
int LineFeatureManager::getFeatureCount()
{
    int cnt = 0;
    for (auto &it : line_feature)
    {

        it.used_num = it.line_feature_per_frame.size();

        if (it.used_num >= 2 && it.start_frame < WINDOW_SIZE - 2)//要求线特征的被观察的数量大于2,也就是有用的线特征
        {
            cnt++;
        }
    }
    return cnt;
}

//返回当前所有的世界坐标系下线特征
vector<vector<double>> LineFeatureManager::getLineVector()
{
    vector<vector<double>> lineVector;
    for (auto &it_per_id : line_feature)
    {
        it_per_id.used_num = it_per_id.line_feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))//同理要求线特征的被观察的数量大于2,也就是有用的线特征
            continue;
        //cout<<it_per_id.line.size()<<endl;
        lineVector.push_back(it_per_id.line);
    }
    return lineVector;
}


void LineFeatureManager::setLineFeature(vector<vector<double>> lineVector)
{
    int feature_index = -1;
    for (auto &it_per_id : line_feature)
    {
        it_per_id.used_num = it_per_id.line_feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        it_per_id.line = lineVector[++feature_index];

        //TODO:如何仿照点一样判断是否求解成功，并移除失败线
        Eigen::Vector3d n, d;
        Utility::cvtOrthonormalToPlucker(it_per_id.line, n, d);
        Eigen::Vector3d dis = d.cross(n);//或者反过来，总之有一个是对的
        if (dis.z() < 0)
        {
            it_per_id.solve_flag = 2;//失败估计
        }
        else
        {
            it_per_id.solve_flag = 1;
        }
    }
}

void LineFeatureManager::line_triangulate(Eigen::Matrix<double, 3, 1> *Ps, Vector3d *tic, Matrix3d *ric)
{
    for (auto &it_per_id : line_feature)
    {
        it_per_id.used_num = it_per_id.line_feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        if (!it_per_id.line.empty())
            continue;

        int s_f = it_per_id.start_frame;
        int e_f = it_per_id.start_frame + it_per_id.used_num - 1;

        Eigen::Vector3d t0 = Ps[s_f] + Rs[s_f]*tic[0];
        Eigen::Matrix3d R0 = Rs[s_f]*ric[0];

        Eigen::Vector3d t1 = Ps[e_f] + Rs[e_f]*tic[0];//第R_1和t_1是第j帧在世界坐标系下的位姿
        Eigen::Matrix3d R1 = Rs[e_f]*ric[0];

        //计算起始帧上直线构成的平面
        Vector3d pi_xyz_0 = it_per_id.line_feature_per_frame[0].pts_s.cross(it_per_id.line_feature_per_frame[0].pts_e);//起始点与终止点叉乘
        double pi_w_0 = pi_xyz_0.dot(t0);//pi_xyz和相机中心点成

        //计算结束帧上直线构成的平面
        Vector3d pi_xyz_1 = it_per_id.line_feature_per_frame.back().pts_s.cross(it_per_id.line_feature_per_frame.back().pts_e);//起始点与终止点叉乘
        double pi_w_1 = pi_xyz_1.dot(t1);//pi_xyz和相机中心点成
//        cout<<pi_xyz_0<<" "<<pi_xyz_1<<endl;

        Vector4d pi_0, pi_1;
        pi_0 << pi_xyz_0.x(), pi_xyz_0.y(), pi_xyz_0.z(), pi_w_0;//构建前后两帧pi平面
        pi_1 << pi_xyz_1.x(), pi_xyz_1.y(), pi_xyz_1.z(), pi_w_1;

        Matrix4d matrix_pu = pi_0*pi_1.transpose() - pi_1*pi_0.transpose();
//        cout<<matrix_pu<<endl<<endl;


        Vector3d pu_n, pu_d;
        pu_n = matrix_pu.block<3, 1>(0, 3);
        pu_d << -matrix_pu(1, 2), matrix_pu(0, 2), -matrix_pu(0, 1);

//        cout<<pu_n<<" "<<pu_d<<endl;
        it_per_id.line.resize(5);
        Utility::cvtPluckerToOrthonormal(pu_n, pu_d, it_per_id.line);
//        cout<<it_per_id.line[0]<<" "<<it_per_id.line[1]<<" "<<it_per_id.line[2]<<" "<<it_per_id.line[3]<<" "<<it_per_id.line[4]<<" "<<endl;

        //TODO:在点的三角化中会对求解结果有一个限制
    }
}








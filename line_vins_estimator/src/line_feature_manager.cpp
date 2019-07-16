//
// Created by leo on 19-7-15.
//

#include "line_feature_manager.h"


void LineFeatureManager::addFeature(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 4, 1>>>> &line_image, double td)
{
    for(auto &id_lines : line_image)//遍历这一帧里面所有的线段
    {
        LineFeaturePerFrame line_f_per_fra(id_lines.second[0].second, td);

        int line_feature_id = id_lines.first;
        auto it = find_if(line_feature.begin(), line_feature.end(), [line_feature_id](const LineFeaturePerId &it)
        {
            return it.line_feature_id == line_feature_id;
        });//在line_feature里面寻找是否有相同id的线特征存在

        if(it == line_feature.end())//如果不存在则新建一个id的线特，并将这个特征在当前帧的观测添加进去
        {
            line_feature.push_back(LineFeaturePerId(line_feature_id, frame_count));
            line_feature.back().line_feature_per_frame.push_back(line_f_per_fra);
        }
        else if(it->line_feature_id == line_feature_id)//如果存在则在相应id的线特征中添加当前帧的观测
        {
            it->line_feature_per_frame.push_back(line_f_per_fra);
        }
    }
}

//边缘化最老帧相关的特征点
void LineFeatureManager::removeBack()
{
    for(auto it = line_feature.begin(), it_next = line_feature.begin();
    it != line_feature.end(); it = it_next)
    {
        it_next++;

        if(it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->line_feature_per_frame.erase(it->line_feature_per_frame.begin());
            if(it->line_feature_per_frame.size() == 0)
                line_feature.erase(it);
        }
    }
}


//边缘化次新帧相关的特征点
void LineFeatureManager::removeFront(int frame_count)
{
    for(auto it = line_feature.begin(), it_next = line_feature.begin();
    it != line_feature.end(); it = it_next)
    {
        it_next++;

        if(it->start_frame == frame_count)
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;//从起始帧到次新帧的位置
            //如果次新帧之前已经跟踪结束就什么都不做
            if(it->endFrame() < frame_count - 1)
                continue;

            //如果次新帧仍然被跟踪，则删除feature_per_frame中次新帧对应的FeaturePerFrame
            it->line_feature_per_frame.erase(it->line_feature_per_frame.begin() + j);
            if(it->line_feature_per_frame.size() == 0)
                line_feature.erase(it);
        }
    }
}
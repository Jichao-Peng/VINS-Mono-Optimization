//
// Created by lisilin on 19-7-16.
//

#pragma once

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../parameters.h"

/// \brief 直线残差的投影计算
///
/// ceres::SizedCostFunction<2, 7, 5>说明：
/// 2: 残差维度
/// 7： i帧imu pose(P_wbi, R_wbi)
/// 5： i帧直线在World坐标系中参数[q1,q2,q3,q4,phi]前4个表示U matrix, 最后一个表示W matrix
class LineProjectionFactor : public ceres::SizedCostFunction<2, 7, 5> {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    /// \brief 后端优化需要传入的直线观测值和投影值
    ///
    /// 得到的点是在image plane下，残差也是在image plane下完成的
    /// \param _Lw_n 世界坐标系下的直线普吕克坐标n
    /// \param _Lw_d 世界坐标系下的直线普吕克坐标d
    /// \param _pts_s image plane的直线起止点
    /// \param _pts_e
    /// \param _para_Ex_Pose imu和camera外参，这个地方一定要用指针，只有这样才能保证，其他地方优化这个变量的时候，该处能够使用最新的外参
    LineProjectionFactor(const Eigen::Vector3d &_pts_s, const Eigen::Vector3d &_pts_e,
                         double *_para_Ex_Pose);

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    void check(double **parameters);

    /// \brief
    ///
    /// \param R_wc
    /// \param t_wc
    /// \param mode 0 从camera到world
    ///             1 从world到camera
    void projectLine(const Eigen::Matrix3d &R_wc, const Eigen::Vector3d &t_wc, int mode = 0);

    Eigen::Vector3d Lw_n;
    Eigen::Vector3d Lw_d;
    Eigen::Vector3d pts_s, pts_e;
    double *para_Ex_Pose;


    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};

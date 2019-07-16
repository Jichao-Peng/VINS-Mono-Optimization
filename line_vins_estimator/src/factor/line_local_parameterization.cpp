//
// Created by lisilin on 19-7-16.
//

#include "line_local_parameterization.h"

//q1, q2, q3, q4, phi
bool LineLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Quaterniond> _q(x);//U matrix
    const double _phi = x[4]; //W matrix

    Eigen::Quaterniond d_q = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta));//前三个是U matrix增量
    const double d_phi = delta[4]; //W matrix

    //修改q,phi的值相当于修改了x_plus_delta
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta);//U matrix
    double &phi = x_plus_delta[4]; //W matrix

    //update U matrix
    q = (_q*d_q).normalized();

    //update W matrix
    Eigen::Matrix3d _W;
    _W << cos(_phi), -sin(_phi), 0,
            sin(_phi), cos(_phi), 0,
            0, 0, 1;//phi初始值

    Eigen::Vector3d d_W(0, 0, d_phi);//phi更新量

    Eigen::Quaterniond _qW(_W);
    Eigen::Quaterniond qW = (_qW*Utility::deltaQ(d_W)).normalized();
    Eigen::Matrix3d rW = qW.toRotationMatrix();
    phi = atan2(rW(1, 0), rW(0, 0));

    return true;
}

//TODO: 这个地方不知对错！！
bool LineLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    /*构建如下矩阵
    1 0 0 0
    0 1 0 0
    0 0 1 0
    0 0 0 0
    0 0 0 1*/
    Eigen::Map<Eigen::Matrix<double, 5, 4, Eigen::RowMajor>> j(jacobian);
    j.topRows<3>().setIdentity();
    j.block<1, 4>(3, 0).setZero();
    j.bottomRows<1>() = Eigen::Vector4d(0, 0, 0, 1);;

    return true;
}

//
// Created by lisilin on 19-7-16.
//

#pragma once

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "../utility/utility.h"


/// \brief 直线参数的更新
///
/// 传进去5个参数，两对点，事实上是4个DOF
class LineLocalParameterization : public ceres::LocalParameterization
{
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    virtual int GlobalSize() const { return 5; };
    virtual int LocalSize() const { return 4; };
};

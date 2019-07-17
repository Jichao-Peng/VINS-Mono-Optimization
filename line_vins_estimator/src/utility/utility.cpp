#include "utility.h"
#include <cmath>

Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d &g)
{
    Eigen::Matrix3d R0;
    Eigen::Vector3d ng1 = g.normalized();
    Eigen::Vector3d ng2{0, 0, 1.0};
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0})*R0;
    // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
    return R0;
}

void Utility::cvtPluckerToOrthonormal(const Eigen::Vector3d &n, const Eigen::Vector3d &d, double *orth)
{
    //get U matrix
    Eigen::Matrix3d U;
    U.col(0) = n.normalized();
    U.col(1) = d.normalized();
    U.col(2) = n.cross(d).normalized();
    Eigen::Quaterniond qu(U);
    qu.normalize();
    orth[0] = qu.x();
    orth[1] = qu.y();
    orth[2] = qu.z();
    orth[3] = qu.w();

    //get W matrix
    double phi = atan2(d.norm(), n.norm());
    orth[4] = phi;
}

void Utility::cvtOrthonormalToPlucker(double*orth, Eigen::Vector3d &n, Eigen::Vector3d &d)
{
    //使用Map对应的初始化位置如下
    //orth: [1 2 3 4]
    //qu:    x y z w
    Eigen::Quaterniond qu(orth[3], orth[0], orth[1], orth[2]);
    double phi = orth[4];

    Eigen::Matrix3d U = qu.toRotationMatrix();
    n = U.col(0)*cos(phi);
    d = U.col(1)*sin(phi);
}

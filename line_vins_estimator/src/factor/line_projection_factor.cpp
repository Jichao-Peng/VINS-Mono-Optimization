//
// Created by lisilin on 19-7-16.
//

#include "line_projection_factor.h"

Eigen::Matrix2d LineProjectionFactor::sqrt_info;
double LineProjectionFactor::sum_t;

LineProjectionFactor::LineProjectionFactor(const Eigen::Vector3d &_pts_s, const Eigen::Vector3d &_pts_e, double *_para_Ex_Pose)
        : pts_s(_pts_s), pts_e(_pts_e), para_Ex_Pose(_para_Ex_Pose)
{
//#ifdef UNIT_SPHERE_ERROR
//    Eigen::Vector3d b1, b2;
//    Eigen::Vector3d a = pts_j.normalized();
//    Eigen::Vector3d tmp(0, 0, 1);
//    if(a == tmp)
//        tmp << 1, 0, 0;
//    b1 = (tmp - a * (a.transpose() * tmp)).normalized();
//    b2 = a.cross(b1);
//    tangent_base.block<1, 3>(0, 0) = b1.transpose();
//    tangent_base.block<1, 3>(1, 0) = b2.transpose();
//#endif
};


bool LineProjectionFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    TicToc tic_toc;

    // i 帧imu的pose P_wb, Q_wb
    Eigen::Vector3d P(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Q(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);//注意初始化顺序w,x,y,z
    // i 帧直线在世界坐标系下参数 U and W，注意eigen Quaternion初始化顺序(w,x,y,z)
    double orth[5];
    for (int i = 0; i < 5; ++i)
    {
        orth[i] = parameters[1][i];
    }
    Eigen::Quaterniond qW(parameters[1][3], parameters[1][0], parameters[1][1], parameters[1][2]);
    double phi = parameters[1][4];

    Eigen::Vector3d Lw_n, Lw_d, Lc_n, Lc_d;//Lw是世界坐标系下直线的pluker坐标
    Eigen::Matrix<double, 6, 1> Lw, Lc;//Lc是归一化相机，平面直线的pluker坐标
    Utility::cvtOrthonormalToPlucker(orth, Lw_n, Lw_d);
    Lw.head(3) = Lw_n;
    Lw.tail(3) = Lw_d;

    //得到相机和imu的外参数
    Eigen::Map<Eigen::Vector3d> t_bc(para_Ex_Pose);
    Eigen::Map<Eigen::Quaterniond> Q_bc(para_Ex_Pose + 3);
    Eigen::Matrix3d R_bc(Q_bc);
    //得到body和world变换矩阵
    Eigen::Matrix3d R_wb(Q);
    Eigen::Vector3d t_wb(P);
    //get R_wc
    Eigen::Matrix3d R_wc = R_wb*R_bc;
    Eigen::Vector3d t_wc = R_wb*t_bc + t_wb;

    //project line in world to line in camera normalized plane
    Eigen::Matrix<double, 6, 6> line_T_cw = Utility::getProjectLineTransform(R_wc, t_wc, Utility::WorldToCamera);
    Lc = line_T_cw*Lw;
    Lc_n = Lc.head(3);//归一化相机平面
    Lc_d = Lc.tail(3);
    Eigen::Vector3d line_un = Lc_n;

    //计算残差
    Eigen::Map<Eigen::Vector2d> residual(residuals);
#ifdef UNIT_SPHERE_ERROR
    residual =  tangent_base * (pts_camera_j.normalized() - pts_j.normalized());
#else
    residual = Eigen::Vector2d(pts_s.dot(line_un), pts_e.dot(line_un))/line_un.head(2).norm();
#endif
    residual = sqrt_info*residual;

    //计算jacobians
    if (jacobians)
    {
        Eigen::Matrix<double, 2, 6> reduce;
        Eigen::Matrix<double, 2, 3> reduce_1;
        Eigen::Matrix<double, 3, 6> reduce_2;
        double l1 = line_un(0);
        double l2 = line_un(1);
        double l3 = line_un(2);
        double u1 = pts_s(0);
        double v1 = pts_s(1);
        double u2 = pts_e(0);
        double v2 = pts_e(1);

        double l1l2_23 = line_un.head(2).norm()*(l1*l1 + l2*l2);
#ifdef UNIT_SPHERE_ERROR
        double norm = pts_camera_j.norm();
        Eigen::Matrix3d norm_jaco;
        double x1, x2, x3;
        x1 = pts_camera_j(0);
        x2 = pts_camera_j(1);
        x3 = pts_camera_j(2);
        norm_jaco << 1.0 / norm - x1 * x1 / pow(norm, 3), - x1 * x2 / pow(norm, 3),            - x1 * x3 / pow(norm, 3),
                     - x1 * x2 / pow(norm, 3),            1.0 / norm - x2 * x2 / pow(norm, 3), - x2 * x3 / pow(norm, 3),
                     - x1 * x3 / pow(norm, 3),            - x2 * x3 / pow(norm, 3),            1.0 / norm - x3 * x3 / pow(norm, 3);
        reduce = tangent_base * norm_jaco;
#else   //谢论文
        reduce_1 << (u1*l2*l2 - l1*l2*v1 - l1*l3)/l1l2_23, (v1*l1*l1 - l1*l2*u1 - l2*l3)/l1l2_23, 1/line_un.head(2).norm(),
                (u2*l2*l2 - l1*l2*v2 - l1*l3)/l1l2_23, (v2*l1*l1 - l1*l1*v2 - l2*l3)/l1l2_23, 1/line_un.head(2).norm();
        reduce_2.setZero();
        reduce_2.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        reduce = reduce_1*reduce_2;
#endif
        reduce = sqrt_info*reduce;

        if (jacobians[0])
        {
            //准备工作
            Eigen::Matrix3d R_cb = R_bc.transpose();
            Eigen::Matrix<double, 6, 6> line_T_cb;
            line_T_cb.block<3, 3>(0, 0) = R_cb;
            line_T_cb.block<3, 3>(0, 3) = Utility::skewSymmetric(-R_cb*t_bc)*R_cb;
            line_T_cb.block<3, 3>(3, 3) = R_cb;

            Eigen::Matrix<double, 6, 3> part1, part2;
            part1.setZero();
            part1.block<3, 3>(0, 0) = R_wb.transpose()*Utility::skewSymmetric(Lw_d);
            part2.block<3, 3>(0, 0) = Utility::skewSymmetric(R_wb.transpose()*(Lw_n + Utility::skewSymmetric(Lw_d)*t_wb));
            part2.block<3, 3>(3, 0) = Utility::skewSymmetric(R_wb.transpose()*Lw_d);


            //之所以用RowMajor是为了按行去填充Eigen形式的Jacobian矩阵
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

            Eigen::Matrix<double, 6, 6> jaco_i;
            jaco_i.leftCols<3>() = line_T_cb*part1;
            jaco_i.rightCols<3>() = line_T_cb*part2;

            jacobian_pose_i.leftCols<6>() = reduce*jaco_i;
            jacobian_pose_i.rightCols<1>().setZero();//7是占位，其实最后一个为0
        }

        if (jacobians[1])
        {
            //准备工作
            double w1 = cos(phi);
            double w2 = sin(phi);
            Eigen::Vector3d U1 = Lw_n.normalized();
            Eigen::Vector3d U2 = Lw_d.normalized();
            Eigen::Vector3d U3 = Lw_n.cross(Lw_d).normalized();


            Eigen::Map<Eigen::Matrix<double, 2, 5, Eigen::RowMajor>> jacobian_line(jacobians[1]);

            Eigen::Matrix<double, 6, 5> jaco_i;
            jaco_i.setZero();
            jaco_i.block<3, 1>(3, 0) = w2*U3;
            jaco_i.block<3, 1>(0, 1) = -w1*U3;
            jaco_i.block<3, 1>(0, 2) = w1*U2;
            jaco_i.block<3, 1>(3, 2) = -w2*U1;
            jaco_i.block<3, 1>(0, 4) = -w2*U1;
            jaco_i.block<3, 1>(3, 4) = w1*U2;

            jacobian_line = reduce*jaco_i;
        }
//        if (jacobians[2])
//        {
//            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[2]);
//            Eigen::Matrix<double, 3, 6> jaco_ex;
//            jaco_ex.leftCols<3>() = ric.transpose()*(Rj.transpose()*Ri - Eigen::Matrix3d::Identity());
//            Eigen::Matrix3d tmp_r = ric.transpose()*Rj.transpose()*Ri*ric;
//            jaco_ex.rightCols<3>() = -tmp_r*Utility::skewSymmetric(pts_camera_i) + Utility::skewSymmetric(tmp_r*pts_camera_i) +
//                                     Utility::skewSymmetric(ric.transpose()*(Rj.transpose()*(Ri*tic + Pi - Pj) - tic));
//            jacobian_ex_pose.leftCols<6>() = reduce*jaco_ex;
//            jacobian_ex_pose.rightCols<1>().setZero();
//        }
//        if (jacobians[3])
//        {
//            Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[3]);
//#if 1
//            jacobian_feature = reduce*ric.transpose()*Rj.transpose()*Ri*ric*pts_i*-1.0/(inv_dep_i*inv_dep_i);
//#else
//            jacobian_feature = reduce * ric.transpose() * Rj.transpose() * Ri * ric * pts_i;
//#endif
//        }
    }
    sum_t += tic_toc.toc();

    return true;
}

void LineProjectionFactor::check(double **parameters)
{
//    double *res = new double[15];
//    double **jaco = new double *[4];
//    jaco[0] = new double[2*7];
//    jaco[1] = new double[2*7];
//    jaco[2] = new double[2*7];
//    jaco[3] = new double[2*1];
//    Evaluate(parameters, res, jaco);
//    puts("check begins");
//
//    puts("my");
//
//    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 1>>(res).transpose() << std::endl
//              << std::endl;
//    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[0]) << std::endl
//              << std::endl;
//    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[1]) << std::endl
//              << std::endl;
//    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[2]) << std::endl
//              << std::endl;
//    std::cout << Eigen::Map<Eigen::Vector2d>(jaco[3]) << std::endl
//              << std::endl;
//
//    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
//    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
//
//    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
//    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
//
//    Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
//    Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
//    double inv_dep_i = parameters[3][0];
//
//    Eigen::Vector3d pts_camera_i = pts_i/inv_dep_i;
//    Eigen::Vector3d pts_imu_i = qic*pts_camera_i + tic;
//    Eigen::Vector3d pts_w = Qi*pts_imu_i + Pi;
//    Eigen::Vector3d pts_imu_j = Qj.inverse()*(pts_w - Pj);
//    Eigen::Vector3d pts_camera_j = qic.inverse()*(pts_imu_j - tic);
//
//
//    Eigen::Vector2d residual;
//#ifdef UNIT_SPHERE_ERROR
//    residual =  tangent_base * (pts_camera_j.normalized() - pts_j.normalized());
//#else
//    double dep_j = pts_camera_j.z();
//    residual = (pts_camera_j/dep_j).head<2>() - pts_j.head<2>();
//#endif
//    residual = sqrt_info*residual;
//
//    puts("num");
//    std::cout << residual.transpose() << std::endl;
//
//    const double eps = 1e-6;
//    Eigen::Matrix<double, 2, 19> num_jacobian;
//    for (int k = 0; k < 19; k++)
//    {
//        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
//        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
//
//        Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
//        Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
//
//        Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
//        Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
//        double inv_dep_i = parameters[3][0];
//
//        int a = k/3, b = k%3;
//        Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2)*eps;
//
//        if (a == 0)
//            Pi += delta;
//        else if (a == 1)
//            Qi = Qi*Utility::deltaQ(delta);
//        else if (a == 2)
//            Pj += delta;
//        else if (a == 3)
//            Qj = Qj*Utility::deltaQ(delta);
//        else if (a == 4)
//            tic += delta;
//        else if (a == 5)
//            qic = qic*Utility::deltaQ(delta);
//        else if (a == 6)
//            inv_dep_i += delta.x();
//
//        Eigen::Vector3d pts_camera_i = pts_i/inv_dep_i;
//        Eigen::Vector3d pts_imu_i = qic*pts_camera_i + tic;
//        Eigen::Vector3d pts_w = Qi*pts_imu_i + Pi;
//        Eigen::Vector3d pts_imu_j = Qj.inverse()*(pts_w - Pj);
//        Eigen::Vector3d pts_camera_j = qic.inverse()*(pts_imu_j - tic);
//
//        Eigen::Vector2d tmp_residual;
//#ifdef UNIT_SPHERE_ERROR
//        tmp_residual =  tangent_base * (pts_camera_j.normalized() - pts_j.normalized());
//#else
//        double dep_j = pts_camera_j.z();
//        tmp_residual = (pts_camera_j/dep_j).head<2>() - pts_j.head<2>();
//#endif
//        tmp_residual = sqrt_info*tmp_residual;
//        num_jacobian.col(k) = (tmp_residual - residual)/eps;
//    }
//    std::cout << num_jacobian << std::endl;
}

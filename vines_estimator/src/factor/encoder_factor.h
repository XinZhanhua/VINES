#pragma once
#include <ros/assert.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../parameters.h"

#include <ceres/ceres.h>

class EncoderFactor : public ceres::SizedCostFunction<6, 7, 7>
{
  public:
    EncoderFactor() = delete;
    EncoderFactor(double _angle_i, double _angle_j) : angle_i(_angle_i), angle_j(_angle_j)
    {
    }
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        Eigen::Vector3d tic_i(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond qic_i(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Vector3d tic_j(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond qic_j(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

        Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
        double derta_theta = angle_j - angle_i;
        double k = 1 / sqrt(1 + derta_theta * derta_theta / 4);
        Eigen::Vector3d xyz = 1/2 * derta_theta * k * axis[0];
        Eigen::Quaterniond derta_q(k, xyz.x(), xyz.y(), xyz.z());
        residual.block<3, 1>(0, 0) = tic_j - tic_i + Utility::Rodrigues(axis[0], angle_i) * derta_theta * Utility::skewSymmetric(TEC[0]) * axis[0];
        residual.block<3, 1>(3, 0) = 2 * (qic_j * qic_i.inverse() * derta_q.inverse()).vec();

        if (jacobians)
        {

            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_ex_pose_i(jacobians[0]);
                jacobian_ex_pose_i.setZero();

                jacobian_ex_pose_i.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
                jacobian_ex_pose_i.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();
            }

            if (jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_ex_pose_j(jacobians[1]);
                jacobian_ex_pose_j.setZero();

                jacobian_ex_pose_j.block<3, 3>(0, 3) = -(Utility::Qleft(derta_q * qic_i) * Utility::Qright(qic_j)).bottomRightCorner<3, 3>();
                jacobian_ex_pose_j.block<3, 3>(3, 3) = (Utility::Qleft(derta_q * qic_i) * Utility::Qright(qic_j)).bottomRightCorner<3, 3>();
            }
        }
    }
    double angle_i, angle_j;
};


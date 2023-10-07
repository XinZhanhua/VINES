#pragma once

#include <cmath>
#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../parameters.h"

class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1>
{
  public:
    ProjectionFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j);
    ProjectionFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j, double _angle_i, double _angle_j);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector3d pts_i, pts_j;
    double angle_i, angle_j;
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
    bool isrotate;
};

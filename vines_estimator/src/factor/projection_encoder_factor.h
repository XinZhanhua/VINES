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

class ProjectionEncoderFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1, 1, 1>
{
  public:
    ProjectionEncoderFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector3d pts_i, pts_j;
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};

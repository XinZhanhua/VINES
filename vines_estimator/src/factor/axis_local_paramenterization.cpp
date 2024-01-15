#include "axis_local_paramenterization.h"

bool AxisLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    // Assuming the input vectors are already normalized unit vectors
    Eigen::Map<const Eigen::Vector3d> v(x);
    Eigen::Map<const Eigen::Vector2d> derta_v(delta);
    Eigen::Vector3d temp = Utility::Bx(v) * derta_v;
    Eigen::AngleAxisd rotation_temp(temp.norm(), temp.normalized());
    Eigen::Matrix3d derta_rotation = rotation_temp.toRotationMatrix();

    Eigen::Map<Eigen::Vector3d> result(x_plus_delta);
    result = derta_rotation * v;

    return true;
}
bool AxisLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 3, 2, Eigen::RowMajor>> j(jacobian);
    j.topRows<2>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
}

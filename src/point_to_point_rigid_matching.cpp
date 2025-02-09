#include "point_to_point_rigid_matching.h"
#include "closest_rotation.h"
#include <iostream>

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
    double c = 1.0 / (double) X.rows();
    Eigen::RowVector3d p_bar = c * P.colwise().sum();
    Eigen::RowVector3d x_bar = c * X.colwise().sum();

    Eigen::Matrix3d M = (P.rowwise() - p_bar).transpose() * (X.rowwise() - x_bar);
    closest_rotation(M, R);
    t = p_bar.transpose() - R * x_bar.transpose();
}


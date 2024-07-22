#include "point_to_point_rigid_matching.h"
#include "closest_rotation.h"

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
    double N = X.rows();
    Eigen::RowVector3d x_bar, p_bar;
    Eigen::MatrixXd M;

    x_bar = (1/N) * X.colwise().sum();
    p_bar = (1/N) * P.colwise().sum();

    M = (P.rowwise() - p_bar).transpose() * (X.rowwise() - x_bar);
    closest_rotation(M, R);
    t = p_bar.transpose() - R * x_bar.transpose();
}


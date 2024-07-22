#include "point_to_plane_rigid_matching.h"

void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
    Eigen::MatrixXd Y, PX;
    Eigen::Vector3d b;
    Eigen::Matrix3d A;
    A.setZero();
    b.setZero();

    Y << X.rowwise().cross(N), N;
    PX = P - X;

    for (int idx = 0; idx < Y.rows(); idx++) {
        A += Y.row(idx).transpose() * Y.row(idx);
        b += Y.row(idx).transpose() * N.row(idx) * PX.row(idx);
    }

    auto u = A.inverse() * b;

  // Replace with your code
  R << 1, -u(2), u(1), u(2), 1, -u(0), -u(1), u(0), 1;
  t << u(3), u(4), u(5);
}

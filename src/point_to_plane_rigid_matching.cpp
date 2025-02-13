#include "point_to_plane_rigid_matching.h"
#include <Eigen/LU>

void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
    Eigen::MatrixXd Y(X.rows(),6), B(X.rows(), 3), b(6,1);
    Eigen::RowVector3d v, w;
    Eigen::MatrixXd A(6,6);
    A.setZero();
    B.setZero();
    b.setZero();

    for (int idx = 0; idx < X.rows(); idx++) {
        w << N(idx, 0), N(idx, 1), N(idx, 2);
        v << X(idx, 0), X(idx, 1), X(idx, 2);
        v = v.cross(w);
        Y.row(idx) << v(0), v(1), v(2), w(0), w(1), w(2);
    }

    b = Y.transpose() * ((P - X) * N.transpose()).diagonal(); // 6 x n * (n x 1)
    A = Y.transpose() * Y;

    auto u = A.inverse() * b;

     R << 1, -u(5), u(4),
         u(5), 1, -u(3),
         -u(4), u(3), 1;
     //R = R.inverse();
    t << u(0), u(1), u(2);
}

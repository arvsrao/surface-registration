#include "closest_rotation.h"
#include <Eigen/SVD>
#include <Eigen/LU>

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3d O;
    svd.computeU();
    svd.computeV();

    O << 1,0,0, 0,1,0, 0,0,(svd.matrixU() * svd.matrixV().transpose()).determinant();
    R = svd.matrixU() * O * svd.matrixV().transpose();
}

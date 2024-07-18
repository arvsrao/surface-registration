#include "closest_rotation.h"
#include <Eigen/SVD>

using Eigen::DecompositionOptions::ComputeThinU;
using Eigen::DecompositionOptions::ComputeThinV;

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, ComputeThinU | ComputeThinV);
    Eigen::Matrix3d O;
    O << 1,0,0, 0,1,0, 0,0,(svd.matrixU() * svd.matrixV().transpose()).determinant();
    R = svd.matrixU() * O * svd.matrixV().transpose();
}

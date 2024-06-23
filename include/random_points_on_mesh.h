#ifndef RANDOM_POINTS_ON_MESH_H
#define RANDOM_POINTS_ON_MESH_H
#include <Eigen/Core>

/** Binary search of a vector.
 *
 * @param t a randomly generated fraction of the total area of all faces.
 * @param C the vector (Nx1) to be searched.
 * @return an greatest index, idx, where t > C[idx].
 *
 *  Expectations:
 *  1. C, the cumulative areas matrix, is a single column vector
 *  2. C is sorted in ascending order.
 *  3. 't' can not be more than total surface area of the triangle mesh.
 *  4. C is expected to be padded with a zero.
 */
 int binary_search_vector(double t, Eigen::MatrixXd& C);

// RANDOM_POINTS_ON_MESH Randomly sample a mesh (V,F) n times.
//
// Inputs:
//   n  number of samples
//   V  #V by dim list of mesh vertex positions
//   F  #F by 3 list of mesh triangle indices
// Outputs:
//   X  n by 3 list of random points on (V,F)
//
void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X);
#endif

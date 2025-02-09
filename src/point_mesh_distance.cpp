#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include "igl/per_face_normals.h"

void point_mesh_distance(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  Eigen::VectorXd & D,
  Eigen::MatrixXd & P,
  Eigen::MatrixXd & N)
{
    // make normals
    igl::per_face_normals(VY, FY, N);

    P.resize(X.rows(), X.cols());
    D.resize(X.rows());
    D.setConstant(std::numeric_limits<double>::max());

    double d = 0;
    Eigen::RowVector3d p, x;

    // Loop through sample points in X
    for (int rowIdx = 0; rowIdx < X.rows(); rowIdx++)
    {
        x = X.row(rowIdx);
        // Loop through faces of Y
        for (int j = 0; j < FY.rows(); j++)
        {
            point_triangle_distance(
                    x,
                    VY.row(FY(j,0)),
                    VY.row(FY(j,1)),
                    VY.row(FY(j,2)),
                    N.row(j),
                    d,
                    p);

            if (D(rowIdx) > d) {
                D(rowIdx) = d;
                P.row(rowIdx) = p;
            }
        }
    }
}

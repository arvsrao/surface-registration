#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include "igl/per_face_normals.h"
#include <iostream>

void point_mesh_distance(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  Eigen::VectorXd & D,
  Eigen::MatrixXd & P,
  Eigen::MatrixXd & N)
{
    Eigen::MatrixXi FY_perm;
    FY_perm.resizeLike(FY);

    P.resizeLike(X);
    N.resizeLike(X);
    D.resize(X.rows());
    D.setZero();

    double d = 0;
    int faceIdx = 0;
    Eigen::RowVector3d p, a, b, c, n, x;

    for (int i = 0; i < X.rows(); i++)
    {
        x = X.row(i);
        for (int j = 0; j < FY.rows(); j++)
        {
            point_triangle_distance(
                    x,
                    VY.row(FY.row(j).x()),
                    VY.row(FY.row(j).y()),
                    VY.row(FY.row(j).z()),
                    d,
                    p);

            if (D(i) == 0 || D(i) > d)
            {
                D(i) = d;
                P.row(i)   = p;
                faceIdx    = j;
            }
        }

        FY_perm.row(i).x() = FY.row(faceIdx).x();
        FY_perm.row(i).y() = FY.row(faceIdx).y();
        FY_perm.row(i).z() = FY.row(faceIdx).z();
        // a = VY.row(FY.row(faceIdx).x());
        // b = VY.row(FY.row(faceIdx).y());
        // c = VY.row(FY.row(faceIdx).z());
        // n = (b - a).cross(c - a); // clockwise orientation
    }

   igl::per_face_normals(VY, FY_perm, N);
}

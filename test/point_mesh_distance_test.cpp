#include <catch2/catch_test_macros.hpp>
#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include <Eigen/Core>

void choose_closest_face()
{
    Eigen::MatrixXd X(4,3), V(5,3), P, N;
    Eigen::MatrixXi F(4,3);
    Eigen::VectorXd D;

    V << 2,0,0, 0,2,0, -2,0,0, 0,-2,0, 0,0,2;
    F << 0,1,4, 0,3,4, 2,3,4, 2,1,4;
    X << 5,5,5, -5,-5,5, -5,5,5, 5,-5,5;

    point_mesh_distance(X, V, F, D, P, N);

    auto bcc0 = BarycentricCoordinates(P.row(0), V.row(0), V.row(1), V.row(4));
    assert(P.row(0) == bcc0.project_to_triangle());

    auto bcc1 = BarycentricCoordinates(P.row(1), V.row(2), V.row(3), V.row(4));
    assert(P.row(1) == bcc1.project_to_triangle());

    auto bcc2 = BarycentricCoordinates(P.row(2), V.row(2), V.row(1), V.row(4));
    assert(P.row(2) == bcc2.project_to_triangle());

    auto bcc3 = BarycentricCoordinates(P.row(3), V.row(0), V.row(3), V.row(4));
    assert(P.row(3) == bcc3.project_to_triangle());
}

int main()
{
    choose_closest_face();
}
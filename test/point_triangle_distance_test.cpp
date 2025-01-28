#include <iostream>
#include <catch2/catch_test_macros.hpp>
#include <point_triangle_distance.h>
#include <Eigen/Core>

/** project point in the plane of a given triangle to nearest point on the triangle. */
void project_to_triangle_test()
{
    // define triangle in Y-Z plane.
    Eigen::RowVector3d a, b, c, p1, p2, p3, q;
    a << 0,0,0;
    b << 0,1,1;
    c << 0,2,0;

    // A point above and perpendicular to edge AB should be projected to a point in AB.
    p1 << 0,0,1;
    auto bcc1 = BarycentricCoordinates(p1,a,b,c);
    q = bcc1.project_to_triangle();
    assert(p1 != q);
    assert(q.x() == 0);
    assert(q.y() == 0.5);
    assert(q.z() == 0.5);

    // a point properly inside the triangle should be projected to itself.
    p2 << 0, 1, 0.5;
    auto bcc2 = BarycentricCoordinates(p2,a,b,c);
    assert(p2 == bcc2.project_to_triangle());

    // A point directly above 'b' should be projected to 'b'.
    p3 << 0, 1, 5;
    auto bcc3 = BarycentricCoordinates(p3,a,b,c);
    assert(b == bcc3.project_to_triangle());
}

/** project points lying outside the plane of triangle to the triangle. */
void point_triangle_test()
{
    double d       = 0;
    double EPSILON = 0.000001;

    Eigen::RowVector3d a, b, c, p, x, q;
    a << 2, 0, 0;
    b << 0, 2, 0;
    c << 0, 0, 2;

    // A point under a triangle with vertices on all positive axes should be projected to the middle of the triangle.
    q << 0.666667, 0.666667, 0.666667;
    x << -1, -1, -1;
    point_triangle_distance(x, a, b, c, d, p);
    assert((p - q).norm() < EPSILON);

    // should project to a point inside the triangle, but off center.
    x += + 0.1 * Eigen::RowVector3d(-1,-1,2);
    q << 0.566667, 0.566667, 0.866667;
    point_triangle_distance(x, a, b, c, d, p);
    assert(p.sum() == 2); // p \in plane of triangle
    assert((p - q).norm() < EPSILON);

    // should project to a point in the edge of the triangle.
    x = 10 * Eigen::RowVector3d(0,1,1);
    q << 0, 1, 1;
    point_triangle_distance(x, a, b, c, d, p);
    assert((p - q).norm() < EPSILON);

    // should project points to the appropriate triangle vertex.
    x << 0, 0, 10;
    point_triangle_distance(x, a, b, c, d, p);
    assert((p - c).norm() < EPSILON);

    x << 0, 10, 0;
    point_triangle_distance(x, a, b, c, d, p);
    assert((p - b).norm() < EPSILON);

    x << 10, 0, 0;
    point_triangle_distance(x, a, b, c, d, p);
    assert((p - a).norm() < EPSILON);
}

void simple_eigen_test() {
    Eigen::MatrixXd X(3,3), Y(3,1), N(3,3);
    Eigen::Vector3d p;
    p << 0, 0, 1;
    N << 0, 1, 0, 0, 0, 1, 1, 0, 0;
    X << 1, 0, 0, 0, 1, 0, 0, 0, 1;

    Y << X.rowwise() - N.colwise().sum();
    std::cout << Y << "\n";
}

int main()
{
    project_to_triangle_test();
    point_triangle_test();

    simple_eigen_test();
}
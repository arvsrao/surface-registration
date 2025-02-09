#ifndef POINT_TRIANGLE_DISTANCE_H
#define POINT_TRIANGLE_DISTANCE_H
#include <Eigen/Core>

/** The model for barycentric coordinates is
 *   Given triangle defined by vertices a, b, c \in R^3, a point p \in R^3,
 *   there are scalars t1, t2, t3 s.t.
 *
 *   p = t1 * a + t2 * b + t3 * c
 *
 *   Barycentric coordinates divide the plane into 7 regions.
 *   See https://ceng2.ktu.edu.tr/~cakir/files/grafikler/Texture_Mapping.pdf
 */
struct BarycentricCoordinates {
    BarycentricCoordinates(
        const Eigen::RowVector3d &a,
        const Eigen::RowVector3d &b,
        const Eigen::RowVector3d &c,
        double t1,
        double t2,
        double t3);

    BarycentricCoordinates(
        const Eigen::RowVector3d& p,
        const Eigen::RowVector3d& a,
        const Eigen::RowVector3d& b,
        const Eigen::RowVector3d& c);

    void compute_barycentric_coords(const Eigen::RowVector3d& p);
    void compute_barycentric_coords_kramer_rule(const Eigen::RowVector3d& p);
    void printCoefficients();

    /** If the point p is not in the triangle project it to the nearst point on the triangle; either a vertex or
     * a point in an edge. Barycentric coordinates divide the plane into 7 regions.
     *
     * See https://ceng2.ktu.edu.tr/~cakir/files/grafikler/Texture_Mapping.pdf
     * @param p must be a point already in the plane of triangle.
     */
    Eigen::RowVector3d project_to_triangle();

    Eigen::RowVector3d project_to_edge(
        const Eigen::RowVector3d & p,
        const Eigen::RowVector3d & end_pt,
        const Eigen::RowVector3d & start_pt);

private:
    Eigen::RowVector3d _t;
    const Eigen::RowVector3d _p, _v1, _v2, _v3;
};

// Compute the distance `d` between a given point `x` and the closest point `p` on
// a given triangle with corners `a`, `b`, and `c`.
//
// Inputs:
//   x  3d query point
//   a  3d position of first triangle corner
//   b  3d position of second triangle corner
//   c  3d position of third triangle corner
// Outputs:
//   d  distance from x to closest point on triangle abc
//   p  3d position of closest point 
void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  const Eigen::RowVector3d & n,
  double & d,
  Eigen::RowVector3d & p);
#endif

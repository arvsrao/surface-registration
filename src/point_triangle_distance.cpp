#define DEBUG false
#include "point_triangle_distance.h"
#include "igl/per_face_normals.h"

Eigen::RowVector3d BarycentricCoordinates::project_to_edge(
        const Eigen::RowVector3d & p,
        const Eigen::RowVector3d & end_pt,
        const Eigen::RowVector3d & start_pt) {
    auto v = end_pt - start_pt;
    auto w = p - start_pt;
    double lambda = v.dot(w) / v.squaredNorm();

    if (lambda <= 0) return start_pt;
    else if (lambda >= 1) return end_pt;
    else return start_pt + lambda * v; // the perpendicular projection to 'v'
}

/** The model for barycentric coordinates is
 *   Given triangle defined by vertices a, b, c \in R^3, a point p \in R^3,
 *   there are scalars t1, t2, t3 s.t.
 *
 *   p = t1 * a + t2 * b + t3 * c
 *
 *   Barycentric coordinates divide the plane into 7 regions.
 *   See https://ceng2.ktu.edu.tr/~cakir/files/grafikler/Texture_Mapping.pdf
 */
void BarycentricCoordinates::compute_barycentric_coords(const Eigen::RowVector3d& p) {
    Eigen::RowVector3d ap = _v1 - p;
    Eigen::RowVector3d bp = _v2 - p;
    Eigen::RowVector3d cp = _v3 - p;

    Eigen::Vector3d v, w;
    v << ap.y(), bp.y(), cp.y();
    w << ap.z(), bp.z(), cp.z();
    _t = w.cross(v);
    _t /= _t.sum();

    printCoefficients();
}

void BarycentricCoordinates::printCoefficients() {
    if (!DEBUG) return;
    std::cout <<"Coefficients: ";
    std::cout << _t.x() << ", " << _t.y() << ", " << _t.z() << "\n";

    std::cout <<"generate point from coefficients and vertices: ";
    auto q = _t.x() * _v1 + _t.y() * _v2 + _t.z() * _v3;
    std::cout << q.x() << ", " << q.y() << ", " << q.z() << "\n";
}

void BarycentricCoordinates::compute_barycentric_coords_kramer_rule(const Eigen::RowVector3d& p) {
    Eigen::RowVector3d r   = p - _v3;
    Eigen::RowVector3d v13 = _v1 - _v3;
    Eigen::RowVector3d v23 = _v2 - _v3;

    double c1, c2, b1, b2, a1, a2;
    c1 = v13.dot(r);
    c2 = v23.dot(r);
    b1 = v23.dot(v13);
    b2 = v23.dot(v23);
    a1 = v13.dot(v13);
    a2 = v23.dot(v13);
    double detA = a1 * b2 - a2 * b1;

    _t.x() = c1 * b2 - b1 * c2;
    _t.x() /= detA;
    _t.y() = a1 * c2 - c1 * a2;
    _t.y() /= detA;
    _t.z() =  1 - _t.x() - _t.y();

    printCoefficients();
}

/** If the point p is not in the triangle project it to the nearst point on the triangle; either a vertex or
 *  a point in an edge.
 * @param p must be a point already in the plane of triangle.
 */
Eigen::RowVector3d BarycentricCoordinates::project_to_triangle()
{
    if (_t.x() >= 0 && _t.y() >= 0 && _t.z() >= 0) {
        return _p; // inside the triangle
    }
    else if (_t.x() > 0 && _t.y() <= 0 && _t.z() <= 0) {
        return _v1;
    }
    else if (_t.x() <= 0 && _t.y() > 0 && _t.z() <= 0) {
        return _v2;
    }
    else if (_t.x() <= 0 && _t.y() <= 0 && _t.z() > 0) {
        return _v3;
    }
    else if (_t.x() >= 0 && _t.y() >= 0 && _t.z() < 0) {
        return project_to_edge(_p, _v1, _v2);
    }
    else if (_t.x() < 0 && _t.y() >= 0 && _t.z() >= 0) {
        return project_to_edge(_p, _v2, _v3);
    }
    else if (_t.x() >= 0 && _t.y() < 0 && _t.z() >= 0) {
        return project_to_edge(_p, _v1, _v3);
    }
}

BarycentricCoordinates::BarycentricCoordinates(
            const Eigen::RowVector3d &a,
            const Eigen::RowVector3d &b,
            const Eigen::RowVector3d &c,
            const double t1,
            double t2,
            double t3) : _v1(a), _v2(b), _v3(c), _t(t1, t2, t3), _p(_t.x() * a + _t.y() * b + _t.z() * c) {}

BarycentricCoordinates::BarycentricCoordinates(
        const Eigen::RowVector3d& p,
        const Eigen::RowVector3d& a,
        const Eigen::RowVector3d& b,
        const Eigen::RowVector3d& c) : _p(p), _v1(a), _v2(b), _v3(c) { compute_barycentric_coords(p); }

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
  // project 'x' to the plane of the triangle.
  auto v = b - a;
  auto w = c - a;
  auto n = v.cross(w);
  double t = (a - x).dot(n) / n.dot(n);
  p = x + t * n;

  BarycentricCoordinates bcc = BarycentricCoordinates(p, a, b, c);
  p = bcc.project_to_triangle();
  d = (x-p).norm();
}

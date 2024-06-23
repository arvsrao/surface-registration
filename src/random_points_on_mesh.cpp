#include <random>

#include "random_points_on_mesh.h"
#include "igl/face_areas.h"
#include "igl/cumsum.h"

class GenerateNumberInUnitInterval {
public:
    GenerateNumberInUnitInterval(double upperLimit) : _random_engine(std::random_device{}()), _distribution(0, upperLimit) {}

    double operator()() { return _distribution(_random_engine); }

private:
    std::mt19937 _random_engine;
    std::uniform_real_distribution<double> _distribution;
};

Eigen::RowVector3d generate_point_on_face(
        const Eigen::RowVector3d& a1,
        const Eigen::RowVector3d& a2,
        const Eigen::RowVector3d& a3,
        GenerateNumberInUnitInterval& generator) {

    double a = generator();
    double b = generator();

    return a1 + (a * (a3 - a1)) + (b * (a2 -a1));
}

/** Binary search of a vector.
 *
 * @param t a randomly generated fraction of the total area of all faces.
 * @param C the vector (Nx1) to be searched.
 * @return an greatest index, idx, where t > C[idx].
 */
int binary_search_vector(const double t, const Eigen::MatrixXd& C) {
    int N = C.rows()-1;
    assert(C.cols() == 1);
    assert(C(0,0) == 0);
    assert(t <= C(N,0));

    auto func = [](int m) { return (int) std::ceil(double(m)/2.0); };

    for (int j = N/2, m = func(j); N > j && j > -1; m = func(m)) {

        if (C(j,0) < t && t <= C(j+1,0)) return j;
        j += (t > C(j+1,0)) ? m : -m;
    }

    // search failed
    return -1;
}

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  // compute the double areas of each face, and subsequently the cumulative sum
  // of the face area vector.
  Eigen::MatrixXd A, C;
  X.resize(n,3);
  A.resize(F.rows(),1);
  C.resize(F.rows()+1,1);
  igl::doublearea(V, F,A);
  igl::cumsum(A, 1, true,C);

  // create random number generator for range [0, areaSum]
  double areaSum            = C(C.rows() - 1, 0);
  auto faceGenerator        = GenerateNumberInUnitInterval(areaSum);
  auto pointInFaceGenerator = GenerateNumberInUnitInterval(1.0);

  // generate samples
  for (int i = 0; i < n; i++) {

      // find the face to sample
      int faceIdx = binary_search_vector(faceGenerator(), C);
      Eigen::RowVector3i face = F.row(faceIdx);

      X.row(i) << generate_point_on_face(
              V.row(face.x()),
              V.row(face.y()),
              V.row(face.z()),
              pointInFaceGenerator);

     // std::cout << X.row(i).x() << "," << X.row(i).y() << "," << X.row(i).z() << "\n";
  }

}


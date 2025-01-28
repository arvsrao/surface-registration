#include <random>

#include "random_points_on_mesh.h"
#include "igl/face_areas.h"
#include "igl/cumsum.h"

class GenerateNumberWithinInterval {
public:
    GenerateNumberWithinInterval(double upperLimit) : _random_engine(std::random_device{}()), _distribution(0, upperLimit) {}

    double operator()() { return _distribution(_random_engine); }

private:
    std::mt19937 _random_engine;
    std::uniform_real_distribution<double> _distribution;
};

Eigen::RowVector3d generate_point_on_face(
        const Eigen::RowVector3d& a,
        const Eigen::RowVector3d& b,
        const Eigen::RowVector3d& c,
        GenerateNumberWithinInterval& generator) {

    double t = generator();
    double s = generator();

    if (t + s > 1) {
        t = 1 - t;
        s = 1 - s;
    }

    return a + t * (c - a) + s * (b - a);
}

/** Binary search of a vector.
 *
 * @param t a randomly generated fraction of the total area of all faces.
 * @param C the vector (Nx1) to be searched.
 * @return the greatest index where t > C[idx].
 */
int iterative_binary_search_vector(const double t, const Eigen::MatrixXd& C) {
    int N = C.rows()-1;
    assert(C.cols() == 1);
    assert(C(0,0) == 0); // zero prefix
    assert(t <= C(N,0));

    auto func = [](int m) { return (int) std::ceil(double(m)/2.0); };

    for (int j = N/2, m = func(j); N > j && j > -1; m = func(m)) {

        if (C(j,0) < t && t <= C(j+1,0)) return j;
        j += (t > C(j+1,0)) ? m : -m;
    }

    // search failed
    return -1;
}

int binary_search(const double& t, int startIdx, int endIdx, const Eigen::MatrixXd& C) {
    if (startIdx == endIdx) return startIdx;

    int midIdx = std::ceil((endIdx - startIdx) / 2.0) + startIdx;
    // std::cout << "midIx = " + std::to_string(midIdx) << "\n";

    if (t < C(midIdx,0))
        return binary_search(t, startIdx, midIdx - 1, C);
    return binary_search(t, midIdx, endIdx, C);
}

int binary_search(const double& t, const Eigen::MatrixXd& C) {
    int N = C.rows() - 1;
    assert(C.cols() == 1);
    assert(C(0,0) == 0); // zero prefix
    assert(t <= C(N,0));
    return binary_search(t, 0, N, C);
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
  X.resize(n, 3);
  A.resize(F.rows(), 1);
  C.resize(F.rows()+1, 1);
  igl::doublearea(V, F, A);
  igl::cumsum(A, 1, true, C);

  // create random number generator for range [0, areaSum]
  double areaSum            = C(C.rows() - 1, 0);
  auto faceGenerator        = GenerateNumberWithinInterval(areaSum);
  auto pointInFaceGenerator = GenerateNumberWithinInterval(1.0);

  // generate samples
  for (int i = 0; i < n; i++) {

      // find the face to sample
      int faceIdx = binary_search(faceGenerator(), C);
      Eigen::RowVector3i face = F.row(faceIdx);

      X.row(i) << generate_point_on_face(
              V.row(face.x()),
              V.row(face.y()),
              V.row(face.z()),
              pointInFaceGenerator);

     // std::cout << X.row(i).x() << "," << X.row(i).y() << "," << X.row(i).z() << "\n";
  }

}


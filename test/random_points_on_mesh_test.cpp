#include <catch2/catch_test_macros.hpp>
#include <random_points_on_mesh.h>
#include <Eigen/Core>
#include <igl/cumsum.h>
#include <vector>
#include <iostream>
#include <string>

void binary_search_test() {
    std::vector<double> ts = { 0.5, 3.0, 5.0, 6.0, 13.0, 16.0, 19.0, 19.5 };
    std::vector<int> indices = { 0, 3, 4, 5, 7, 9, 10, 10 };

    // C is zero padded.
    Eigen::MatrixXd C;
    C.resize(12,1);
    C << 0, 1, 2.5, 2.8, 4.5, 5.9, 8.5, 10.8, 14.5, 15.7, 18.8, 20.0;

    for (int i = 0; i < ts.size(); i++) {
        int idx = binary_search_vector(ts[i], C);

        if (idx != indices[i]) {
            std::cout << "Failed!! idx = " + std::to_string(i) + " does NOT equal " + std::to_string(indices[i]) << "\n";
            assert(false);
        }
    }
}

void cumulative_area_test() {
    std::vector<double> ts = { 33, 11, 2, 3, 23, 10, 27 };
    std::vector<int> indices = { 8, 2, 0, 1, 6, 2, 7 };

    Eigen::MatrixXd A, C;
    A.resize(11,1);
    C.resize(A.rows() + 1,1);
    A << 2.94, 5.11, 5.67, 1.41, 1.28, 4.7, 5.67, 5.83, 2.24, 2.47, 4.6;

    // should generate vector
    // [ 0, 2.94, 8.05, 13.72, 15.13, 16.41, 21.11, 26.78, 32.61, 34.85, 37.32, 41.92 ]
    igl::cumsum(A, 1, true, C);

    for (int i = 0; i < ts.size(); i++) {
        int idx = binary_search_vector(ts[i], C);

        if (idx != indices[i]) {
            std::cout << "Failed!! idx = " + std::to_string(i) + " does NOT equal " + std::to_string(indices[i]) << "\n";
            assert(false);
        }
    }
}

int main() {
    binary_search_test();
    cumulative_area_test();
}
find_package(Catch2 3 REQUIRED)

include_directories("../include/")
file(GLOB SRCFILES src/*.cpp)

add_executable(point_mesh_distance ../src/point_mesh_distance.cpp ../src/point_triangle_distance.cpp "point_mesh_distance_test.cpp")
target_link_libraries(point_mesh_distance PRIVATE Catch2::Catch2WithMain igl::core)
add_test(point_mesh_distance_ctest point_mesh_distance)

add_executable(point_triangle_distance ../src/point_triangle_distance.cpp "point_triangle_distance_test.cpp")
target_link_libraries(point_triangle_distance PRIVATE Catch2::Catch2WithMain igl::core)
add_test(point_triangle_distance_ctest point_triangle_distance)

add_executable(random_points_on_mesh ../src/random_points_on_mesh.cpp "random_points_on_mesh_test.cpp")
target_link_libraries(random_points_on_mesh PRIVATE Catch2::Catch2WithMain igl::core)
add_test(random_points_on_mesh_ctest random_points_on_mesh)


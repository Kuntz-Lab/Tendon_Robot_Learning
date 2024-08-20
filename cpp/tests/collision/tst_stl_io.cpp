/**
 * author:        Michael Bentley
 * email:         mikebentley15@gmail.com
 * date-created:  30 April 2020
 */

#include "collision/stl_io.h"
#include "collision/Sphere.h"

#include <fcl/math/vec_3f.h>
#include <fcl/data_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtest/gtest.h>

#include <utility>    // for std::make_pair()

#include <cstdio>     // for std::remove()

using collision::operator<<;
using fcl::operator<<;
using collision::write_stl_file_ascii;
using collision::write_stl_file_binary;
using collision::read_stl_file;

namespace {

template <typename T, typename InputIt>
auto flatten_tripples(InputIt first, InputIt last) {
  size_t N = std::distance(first, last);
  std::vector<T> flattened;
  flattened.reserve(3 * N);
  for (; first != last; first++) {
    auto &tripple = *first;
    for (int i = 0; i < 3; i++) {
      flattened.emplace_back(tripple[i]);
    }
  }
  return flattened;
}

template <typename T, typename InputIt>
auto unflatten_tripples(InputIt first, InputIt last) {
  std::vector<T> unflattened;
  while(first != last) {
    T val{};
    for(int i = 0; first != last && i < 3; first++, i++) {
      val[i] = *first;
    }
    unflattened.emplace_back(std::move(val));
  }
  return unflattened;
}

auto eval_tris(const std::vector<fcl::Vec3f> &vertices,
               const std::vector<fcl::Triangle> &tri_idxs)
{
  using ValType = std::array<std::array<float, 3>, 3>;
  std::vector<ValType> vals_vec;
  for (auto &tri : tri_idxs) {
    ValType vals;
    for (size_t i = 0; i < 3; i++) {
      for (size_t j = 0; j < 3; j++) {
        vals[i][j] = vertices[tri[i]][j];
      }
    }
    vals_vec.emplace_back(std::move(vals));
  }
  return vals_vec;
}

auto create_mesh_model() {
  static decltype(collision::Sphere().to_fcl_mesh_model()) mesh;

  // cache result
  if (!mesh) {
    collision::Sphere sphere {{1.1, 2.2, 3.3}, 4.4};
    mesh = sphere.to_fcl_mesh_model();
  }

  return mesh;
}

// Usage:
//
//   auto [vertices, triangles] = create_mesh();
//
// - vertices: std::vector<fcl::Vec3f> with 3 values per vertex
// - triangles: std::vector<fcl::Triangle> with 3 values per triangle
auto create_mesh() {
  static std::vector<fcl::Vec3f> vertices;
  static std::vector<fcl::Triangle> triangles;

  // cache result
  if (vertices.size() == 0) {
    auto mesh = create_mesh_model();
    vertices = std::vector<fcl::Vec3f>(
        mesh->vertices, mesh->vertices + mesh->num_vertices);
    triangles = std::vector<fcl::Triangle>(
        mesh->tri_indices, mesh->tri_indices + mesh->num_tris);
  }

  return std::make_pair(vertices, triangles);
}

auto create_flattened_mesh() {
  static std::vector<float> flat_vertices;
  static std::vector<size_t> flat_tris;

  // cache result
  if (flat_vertices.size() == 0) {
    auto [vertices, triangles] = create_mesh();
    flat_vertices = flatten_tripples<float>(vertices.begin(), vertices.end());
    flat_tris = flatten_tripples<size_t>(triangles.begin(), triangles.end());
  }

  return std::make_pair(flat_vertices, flat_tris);
}

/** deletes the file in the destructor (RAII) */
struct FileDeleter {
  const std::string path;
  FileDeleter(const std::string &_path) : path(_path) {}
  ~FileDeleter() { std::remove(path.c_str()); } // ignore errors
};

std::string stl_filepath() {
  auto filepath = "/tmp/StlIoTests-tmp.stl";
  return filepath;
}

} // end of unnamed namespace

TEST(StlIoTests, write_stl_file_ascii_same_triangles) {
  auto [vertices, triangles] = create_mesh();
  auto expected_tris = eval_tris(vertices, triangles);

  // write to file
  auto [flat_vertices, flat_triangles] = create_flattened_mesh();
  auto filepath = stl_filepath();
  FileDeleter deleter(filepath);
  (void)deleter; // RAII

  write_stl_file_ascii(filepath, flat_vertices, flat_triangles);

  // read from file
  std::vector<float> read_flat_vertices, read_flat_normals;
  std::vector<size_t> read_flat_tris, read_flat_solid_ranges;
  read_stl_file(filepath.c_str(), read_flat_vertices, read_flat_normals,
                read_flat_tris, read_flat_solid_ranges);

  // unflatten read vertices and triangles
  auto read_vertices = unflatten_tripples<fcl::Vec3f>(
      read_flat_vertices.begin(), read_flat_vertices.end());
  auto read_tris = unflatten_tripples<fcl::Triangle>(
      read_flat_tris.begin(), read_flat_tris.end());

  auto actual_tris = eval_tris(read_vertices, read_tris);

  ASSERT_EQ(expected_tris, actual_tris);
}

TEST(StlIoTests, write_stl_file_ascii_unit_length_normals) {
  // write to file
  auto [flat_vertices, flat_triangles] = create_flattened_mesh();
  auto filepath = stl_filepath();
  FileDeleter deleter(filepath);
  (void)deleter; // RAII

  write_stl_file_ascii(filepath, flat_vertices, flat_triangles);

  // read from file
  std::vector<float> read_flat_vertices, read_flat_normals;
  std::vector<size_t> read_flat_tris, read_flat_solid_ranges;
  read_stl_file(filepath.c_str(), read_flat_vertices, read_flat_normals,
                read_flat_tris, read_flat_solid_ranges);

  // unflatten read normals
  auto read_normals = unflatten_tripples<Eigen::Vector3f>(
      read_flat_normals.begin(), read_flat_normals.end());

  // make sure normals are of unit length
  for (auto &normal : read_normals) {
    ASSERT_FLOAT_EQ(normal.norm(), 1.0f);
  }
}

TEST(StlIoTests, write_stl_file_ascii_perpendicular_normal) {
  // write to file
  auto [flat_vertices, flat_triangles] = create_flattened_mesh();
  auto filepath = stl_filepath();
  FileDeleter deleter(filepath);
  (void)deleter; // RAII

  write_stl_file_ascii(filepath, flat_vertices, flat_triangles);

  // read from file
  std::vector<float> read_flat_vertices, read_flat_normals;
  std::vector<size_t> read_flat_tris, read_flat_solid_ranges;
  read_stl_file(filepath.c_str(), read_flat_vertices, read_flat_normals,
                read_flat_tris, read_flat_solid_ranges);

  // unflatten read normals
  auto read_normals = unflatten_tripples<Eigen::Vector3f>(
      read_flat_normals.begin(), read_flat_normals.end());

  // evaluate triangles
  auto [vertices, triangles] = create_mesh();
  auto tris = eval_tris(vertices, triangles);

  ASSERT_EQ(read_normals.size(), tris.size());
  for (size_t i = 0; i < tris.size(); i++) {
    auto &tri = tris[i];
    auto &normal = read_normals[i];
    Eigen::Vector3f a(tri[0][0], tri[0][1], tri[0][2]);
    Eigen::Vector3f b(tri[1][0], tri[1][1], tri[1][2]);
    Eigen::Vector3f c(tri[2][0], tri[2][1], tri[2][2]);
    Eigen::Vector3f A = b - a;
    Eigen::Vector3f B = c - b;
    Eigen::Vector3f C = a - c;
    ASSERT_NEAR(0.0f, A.dot(normal), 1e-6);
    ASSERT_NEAR(0.0f, B.dot(normal), 1e-6);
    ASSERT_NEAR(0.0f, C.dot(normal), 1e-6);
  }
}

TEST(StlIoTests, write_stl_file_ascii_correct_normal_direction) {
  // write to file
  auto [flat_vertices, flat_triangles] = create_flattened_mesh();
  auto filepath = stl_filepath();
  FileDeleter deleter(filepath);
  (void)deleter; // RAII

  write_stl_file_ascii(filepath, flat_vertices, flat_triangles);

  // read from file
  std::vector<float> read_flat_vertices, read_flat_normals;
  std::vector<size_t> read_flat_tris, read_flat_solid_ranges;
  read_stl_file(filepath.c_str(), read_flat_vertices, read_flat_normals,
                read_flat_tris, read_flat_solid_ranges);

  // unflatten read normals
  auto read_normals = unflatten_tripples<Eigen::Vector3f>(
      read_flat_normals.begin(), read_flat_normals.end());

  // evaluate triangles
  auto [vertices, triangles] = create_mesh();
  auto tris = eval_tris(vertices, triangles);

  ASSERT_EQ(read_normals.size(), tris.size());
  for (size_t i = 0; i < tris.size(); i++) {
    auto &tri = tris[i];
    auto &normal = read_normals[i];
    Eigen::Vector3f a(tri[0][0], tri[0][1], tri[0][2]);
    Eigen::Vector3f b(tri[1][0], tri[1][1], tri[1][2]);
    Eigen::Vector3f c(tri[2][0], tri[2][1], tri[2][2]);
    Eigen::Vector3f A = b - a;
    Eigen::Vector3f B = c - b;
    Eigen::Vector3f C = a - c;
    ASSERT_GT(normal.dot(A.cross(B)), 0.0f);
    ASSERT_GT(normal.dot(B.cross(C)), 0.0f);
    ASSERT_GT(normal.dot(C.cross(A)), 0.0f);
  }
}

TEST(StlIoTests, write_stl_file_ascii_all_versions_agree) {
  auto [vertices, triangles] = create_mesh();
  auto [flat_vertices, flat_triangles] = create_flattened_mesh();
  auto filepath = stl_filepath();
  FileDeleter deleter(filepath);
  (void)deleter; // RAII

  // write with flattened meshes
  write_stl_file_ascii(filepath, flat_vertices, flat_triangles);

  std::vector<float> read_flat_vertices_1, read_flat_normals_1;
  std::vector<size_t> read_flat_tris_1, read_flat_solid_ranges_1;
  read_stl_file(filepath.c_str(), read_flat_vertices_1, read_flat_normals_1,
                read_flat_tris_1, read_flat_solid_ranges_1);

  // write with vectors of fcl::Vec3f and fcl::Triangle
  write_stl_file_ascii(filepath, vertices, triangles);

  std::vector<float> read_flat_vertices_2, read_flat_normals_2;
  std::vector<size_t> read_flat_tris_2, read_flat_solid_ranges_2;
  read_stl_file(filepath.c_str(), read_flat_vertices_2, read_flat_normals_2,
                read_flat_tris_2, read_flat_solid_ranges_2);

  // write from fcl::BVHModel<BV> mesh directly
  auto mesh = create_mesh_model();
  write_stl_file_ascii(filepath, *mesh);

  std::vector<float> read_flat_vertices_3, read_flat_normals_3;
  std::vector<size_t> read_flat_tris_3, read_flat_solid_ranges_3;
  read_stl_file(filepath.c_str(), read_flat_vertices_3, read_flat_normals_3,
                read_flat_tris_3, read_flat_solid_ranges_3);

  ASSERT_EQ(read_flat_tris_1, read_flat_tris_2);
  ASSERT_EQ(read_flat_tris_1, read_flat_tris_3);
  ASSERT_EQ(read_flat_solid_ranges_1, read_flat_solid_ranges_2);
  ASSERT_EQ(read_flat_solid_ranges_1, read_flat_solid_ranges_3);

  ASSERT_FLOAT_EQ(read_flat_vertices_1.size(), read_flat_vertices_2.size());
  ASSERT_FLOAT_EQ(read_flat_vertices_1.size(), read_flat_vertices_3.size());
  ASSERT_FLOAT_EQ(read_flat_normals_1.size(), read_flat_normals_2.size());
  ASSERT_FLOAT_EQ(read_flat_normals_1.size(), read_flat_normals_3.size());

  for (size_t i = 0; i < read_flat_vertices_1.size(); i++) {
    ASSERT_FLOAT_EQ(read_flat_vertices_1[i], read_flat_vertices_2[i]);
    ASSERT_FLOAT_EQ(read_flat_vertices_1[i], read_flat_vertices_3[i]);
    ASSERT_NEAR(read_flat_normals_1[i], read_flat_normals_2[i], 1e-5);
    ASSERT_NEAR(read_flat_normals_1[i], read_flat_normals_3[i], 1e-5);
  }
}

TEST(StlIoTests, write_stl_file_binary_same_triangles) {
  auto [vertices, triangles] = create_mesh();
  auto expected_tris = eval_tris(vertices, triangles);

  // write to file
  auto [flat_vertices, flat_triangles] = create_flattened_mesh();
  auto filepath = stl_filepath();
  FileDeleter deleter(filepath);
  (void)deleter; // RAII

  write_stl_file_binary(filepath, flat_vertices, flat_triangles);

  // read from file
  std::vector<float> read_flat_vertices, read_flat_normals;
  std::vector<size_t> read_flat_tris, read_flat_solid_ranges;
  read_stl_file(filepath.c_str(), read_flat_vertices, read_flat_normals,
                read_flat_tris, read_flat_solid_ranges);

  // unflatten read vertices and triangles
  auto read_vertices = unflatten_tripples<fcl::Vec3f>(
      read_flat_vertices.begin(), read_flat_vertices.end());
  auto read_tris = unflatten_tripples<fcl::Triangle>(
      read_flat_tris.begin(), read_flat_tris.end());

  auto actual_tris = eval_tris(read_vertices, read_tris);

  ASSERT_EQ(expected_tris, actual_tris);
}

TEST(StlIoTests, write_stl_file_binary_unit_length_normals) {
  // write to file
  auto [flat_vertices, flat_triangles] = create_flattened_mesh();
  auto filepath = stl_filepath();
  FileDeleter deleter(filepath);
  (void)deleter; // RAII

  write_stl_file_binary(filepath, flat_vertices, flat_triangles);

  // read from file
  std::vector<float> read_flat_vertices, read_flat_normals;
  std::vector<size_t> read_flat_tris, read_flat_solid_ranges;
  read_stl_file(filepath.c_str(), read_flat_vertices, read_flat_normals,
                read_flat_tris, read_flat_solid_ranges);

  // unflatten read normals
  auto read_normals = unflatten_tripples<Eigen::Vector3f>(
      read_flat_normals.begin(), read_flat_normals.end());

  // make sure normals are of unit length
  for (auto &normal : read_normals) {
    ASSERT_FLOAT_EQ(normal.norm(), 1.0f);
  }
}

TEST(StlIoTests, write_stl_file_binary_perpendicular_normal) {
  // write to file
  auto [flat_vertices, flat_triangles] = create_flattened_mesh();
  auto filepath = stl_filepath();
  FileDeleter deleter(filepath);
  (void)deleter; // RAII

  write_stl_file_binary(filepath, flat_vertices, flat_triangles);

  // read from file
  std::vector<float> read_flat_vertices, read_flat_normals;
  std::vector<size_t> read_flat_tris, read_flat_solid_ranges;
  read_stl_file(filepath.c_str(), read_flat_vertices, read_flat_normals,
                read_flat_tris, read_flat_solid_ranges);

  // unflatten read normals
  auto read_normals = unflatten_tripples<Eigen::Vector3f>(
      read_flat_normals.begin(), read_flat_normals.end());

  // evaluate triangles
  auto [vertices, triangles] = create_mesh();
  auto tris = eval_tris(vertices, triangles);

  ASSERT_EQ(read_normals.size(), tris.size());
  for (size_t i = 0; i < tris.size(); i++) {
    auto &tri = tris[i];
    auto &normal = read_normals[i];
    Eigen::Vector3f a(tri[0][0], tri[0][1], tri[0][2]);
    Eigen::Vector3f b(tri[1][0], tri[1][1], tri[1][2]);
    Eigen::Vector3f c(tri[2][0], tri[2][1], tri[2][2]);
    Eigen::Vector3f A = b - a;
    Eigen::Vector3f B = c - b;
    Eigen::Vector3f C = a - c;
    ASSERT_NEAR(0.0f, A.dot(normal), 1e-6);
    ASSERT_NEAR(0.0f, B.dot(normal), 1e-6);
    ASSERT_NEAR(0.0f, C.dot(normal), 1e-6);
  }
}

TEST(StlIoTests, write_stl_file_binary_correct_normal_direction) {
  // write to file
  auto [flat_vertices, flat_triangles] = create_flattened_mesh();
  auto filepath = stl_filepath();
  FileDeleter deleter(filepath);
  (void)deleter; // RAII

  write_stl_file_binary(filepath, flat_vertices, flat_triangles);

  // read from file
  std::vector<float> read_flat_vertices, read_flat_normals;
  std::vector<size_t> read_flat_tris, read_flat_solid_ranges;
  read_stl_file(filepath.c_str(), read_flat_vertices, read_flat_normals,
                read_flat_tris, read_flat_solid_ranges);

  // unflatten read normals
  auto read_normals = unflatten_tripples<Eigen::Vector3f>(
      read_flat_normals.begin(), read_flat_normals.end());

  // evaluate triangles
  auto [vertices, triangles] = create_mesh();
  auto tris = eval_tris(vertices, triangles);

  ASSERT_EQ(read_normals.size(), tris.size());
  for (size_t i = 0; i < tris.size(); i++) {
    auto &tri = tris[i];
    auto &normal = read_normals[i];
    Eigen::Vector3f a(tri[0][0], tri[0][1], tri[0][2]);
    Eigen::Vector3f b(tri[1][0], tri[1][1], tri[1][2]);
    Eigen::Vector3f c(tri[2][0], tri[2][1], tri[2][2]);
    Eigen::Vector3f A = b - a;
    Eigen::Vector3f B = c - b;
    Eigen::Vector3f C = a - c;
    ASSERT_GT(normal.dot(A.cross(B)), 0.0f);
    ASSERT_GT(normal.dot(B.cross(C)), 0.0f);
    ASSERT_GT(normal.dot(C.cross(A)), 0.0f);
  }
}

TEST(StlIoTests, write_stl_file_binary_all_versions_agree) {
  auto [vertices, triangles] = create_mesh();
  auto [flat_vertices, flat_triangles] = create_flattened_mesh();
  auto filepath = stl_filepath();
  FileDeleter deleter(filepath);
  (void)deleter; // RAII

  // write with flattened meshes
  write_stl_file_binary(filepath, flat_vertices, flat_triangles);

  std::vector<float> read_flat_vertices_1, read_flat_normals_1;
  std::vector<size_t> read_flat_tris_1, read_flat_solid_ranges_1;
  read_stl_file(filepath.c_str(), read_flat_vertices_1, read_flat_normals_1,
                read_flat_tris_1, read_flat_solid_ranges_1);

  // write with vectors of fcl::Vec3f and fcl::Triangle
  write_stl_file_binary(filepath, vertices, triangles);

  std::vector<float> read_flat_vertices_2, read_flat_normals_2;
  std::vector<size_t> read_flat_tris_2, read_flat_solid_ranges_2;
  read_stl_file(filepath.c_str(), read_flat_vertices_2, read_flat_normals_2,
                read_flat_tris_2, read_flat_solid_ranges_2);

  // write from fcl::BVHModel<BV> mesh directly
  auto mesh = create_mesh_model();
  write_stl_file_binary(filepath, *mesh);

  std::vector<float> read_flat_vertices_3, read_flat_normals_3;
  std::vector<size_t> read_flat_tris_3, read_flat_solid_ranges_3;
  read_stl_file(filepath.c_str(), read_flat_vertices_3, read_flat_normals_3,
                read_flat_tris_3, read_flat_solid_ranges_3);

  ASSERT_EQ(read_flat_tris_1, read_flat_tris_2);
  ASSERT_EQ(read_flat_tris_1, read_flat_tris_3);
  ASSERT_EQ(read_flat_solid_ranges_1, read_flat_solid_ranges_2);
  ASSERT_EQ(read_flat_solid_ranges_1, read_flat_solid_ranges_3);

  ASSERT_FLOAT_EQ(read_flat_vertices_1.size(), read_flat_vertices_2.size());
  ASSERT_FLOAT_EQ(read_flat_vertices_1.size(), read_flat_vertices_3.size());
  ASSERT_FLOAT_EQ(read_flat_normals_1.size(), read_flat_normals_2.size());
  ASSERT_FLOAT_EQ(read_flat_normals_1.size(), read_flat_normals_3.size());

  for (size_t i = 0; i < read_flat_vertices_1.size(); i++) {
    ASSERT_FLOAT_EQ(read_flat_vertices_1[i], read_flat_vertices_2[i]);
    ASSERT_FLOAT_EQ(read_flat_vertices_1[i], read_flat_vertices_3[i]);
    ASSERT_NEAR(read_flat_normals_1[i], read_flat_normals_2[i], 1e-5);
    ASSERT_NEAR(read_flat_normals_1[i], read_flat_normals_3[i], 1e-5);
  }
}

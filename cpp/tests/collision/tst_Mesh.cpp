#include "collision/Mesh.h"
#include "cpptoml/cpptoml.h"
#include "cpptoml/toml_conversions.h"

#include <gtest/gtest.h>

#include <cstdio> // for std::remove() of files

using collision::Mesh;

namespace fcl {

bool operator==(const Triangle &a, const Triangle &b) {
  return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}

}

namespace {

Mesh create_mesh() {
  Mesh m;

  // create a cube, hard-coded
  m.vertices.emplace_back(-1.0, -1.0, -1.0); // vertex 0
  m.vertices.emplace_back(-1.0, -1.0,  1.0); // vertex 1
  m.vertices.emplace_back(-1.0,  1.0, -1.0); // vertex 2
  m.vertices.emplace_back(-1.0,  1.0,  1.0); // vertex 3
  m.vertices.emplace_back( 1.0, -1.0, -1.0); // vertex 4
  m.vertices.emplace_back( 1.0, -1.0,  1.0); // vertex 5
  m.vertices.emplace_back( 1.0,  1.0, -1.0); // vertex 6
  m.vertices.emplace_back( 1.0,  1.0,  1.0); // vertex 7

  // cube side facing -z (0, 2, 4, 6)
  m.triangles.emplace_back(0, 2, 6);
  m.triangles.emplace_back(0, 6, 4);

  // cube side facing -x (0, 1, 2, 3)
  m.triangles.emplace_back(0, 3, 2);
  m.triangles.emplace_back(0, 1, 3);

  // cube side facing -y (0, 1, 4, 5)
  m.triangles.emplace_back(0, 4, 5);
  m.triangles.emplace_back(0, 5, 1);

  // cube side facing +x (4, 5, 6, 7)
  m.triangles.emplace_back(6, 5, 4);
  m.triangles.emplace_back(6, 7, 5);

  // cube side facing +y (2, 3, 6, 7)
  m.triangles.emplace_back(6, 3, 7);
  m.triangles.emplace_back(6, 2, 3);

  // cube side facing +z (1, 3, 5, 7)
  m.triangles.emplace_back(5, 3, 1);
  m.triangles.emplace_back(5, 7, 3);

  return m;
}

class FileRemover {
  const std::string _fname;
public:
  FileRemover(const std::string &fname) : _fname(fname) {}
  ~FileRemover() { std::remove(_fname.c_str()); }
  std::string fname() const { return _fname; }
};

void assert_vertices_equal(const Mesh &m1, const Mesh &m2) {
  ASSERT_EQ(m1.vertices.size(), m2.vertices.size());
  for (size_t i = 0; i < m1.vertices.size(); i++) {
    ASSERT_EQ(m1.vertices[i][0], m2.vertices[i][0]);
    ASSERT_EQ(m1.vertices[i][1], m2.vertices[i][1]);
    ASSERT_EQ(m1.vertices[i][2], m2.vertices[i][2]);
  }
}

}

TEST(MeshTests, to_and_from_stl_ascii) {
  auto mesh = create_mesh();
  bool binary_format = false;
  ASSERT_EQ(mesh.filename, "");
  FileRemover remover("/tmp/MeshTests-to-and-from-stl.stl");
  mesh.to_stl(remover.fname(), binary_format);
  ASSERT_EQ(mesh.filename, remover.fname());
  auto mesh2 = Mesh::from_stl(remover.fname());
  ASSERT_EQ(mesh.filename, mesh2.filename);
  ASSERT_EQ(mesh.triangles, mesh2.triangles);
  assert_vertices_equal(mesh, mesh2);
}

TEST(MeshTests, to_and_from_stl_binary) {
  auto mesh = create_mesh();
  bool binary_format = true;
  ASSERT_EQ(mesh.filename, "");
  FileRemover remover("/tmp/MeshTests-to-and-from-stl.stl");
  mesh.to_stl(remover.fname(), binary_format);
  ASSERT_EQ(mesh.filename, remover.fname());
  auto mesh2 = Mesh::from_stl(remover.fname());
  ASSERT_EQ(mesh.filename, mesh2.filename);
  ASSERT_EQ(mesh.triangles, mesh2.triangles);
  assert_vertices_equal(mesh, mesh2);
}

TEST(MeshTests, to_and_from_fcl) {
  auto mesh = create_mesh();
  FileRemover remover("/tmp/MeshTests-to-and-from-fcl.stl");
  mesh.to_stl(remover.fname()); // set the filename field of mesh
  auto fcl_model = mesh.to_fcl_model();
  auto from_fcl = Mesh::from_fcl(fcl_model);
  ASSERT_EQ(mesh.filename, remover.fname());
  ASSERT_EQ(from_fcl.filename, "");
  ASSERT_EQ(from_fcl.triangles, mesh.triangles);
  assert_vertices_equal(from_fcl, mesh);
}

TEST(MeshTests, empty_function) {
  Mesh mesh;
  ASSERT_TRUE(mesh.empty());
  mesh.vertices.emplace_back(0, 0, 0);
  ASSERT_TRUE(mesh.empty());
  mesh.vertices.emplace_back(1, 1, 1);
  ASSERT_TRUE(mesh.empty());
  mesh.vertices.emplace_back(2, 2, 2);
  ASSERT_TRUE(mesh.empty());
  mesh.triangles.emplace_back(0, 1, 2);
  ASSERT_FALSE(mesh.empty());

  mesh = Mesh{};
  mesh.triangles.emplace_back(0, 1, 2);
  ASSERT_TRUE(mesh.empty());

  mesh = create_mesh();
  ASSERT_FALSE(mesh.empty());
}

TEST(MeshTests, to_toml_empty) {
  collision::Mesh m;
  auto actual = collision::Mesh::from_toml(m.to_toml());
  ASSERT_EQ(m.filename,  actual.filename);
  ASSERT_EQ(m.triangles, actual.triangles);
  assert_vertices_equal(m, actual);
}

TEST(MeshTests, to_toml) {
  collision::Mesh m = create_mesh();
  ASSERT_EQ(m.filename, "");
  auto tbl = m.to_toml();
  ASSERT_FALSE(tbl->get("mesh")->as_table()->contains("filename"));
  ASSERT_TRUE (tbl->get("mesh")->as_table()->contains("vertices"));
  ASSERT_TRUE (tbl->get("mesh")->as_table()->contains("triangles"));

  auto actual = collision::Mesh::from_toml(tbl);
  ASSERT_EQ("",          actual.filename);
  ASSERT_EQ(m.triangles, actual.triangles);
  assert_vertices_equal(m, actual);
}

TEST(MeshTests, to_toml_empty_through_string) {
  collision::Mesh m;
  auto str = cpptoml::to_string(m.to_toml());
  std::istringstream in(str);
  cpptoml::parser toml_parser(in);
  auto actual = collision::Mesh::from_toml(toml_parser.parse());
  ASSERT_EQ(m.filename,  actual.filename);
  ASSERT_EQ(m.triangles, actual.triangles);
  assert_vertices_equal(m, actual);
}

TEST(MeshTests, to_toml_through_string) {
  collision::Mesh m = create_mesh();
  auto str = cpptoml::to_string(m.to_toml());
  std::istringstream in(str);
  cpptoml::parser toml_parser(in);
  auto actual = collision::Mesh::from_toml(toml_parser.parse());
  ASSERT_EQ("",          actual.filename);
  ASSERT_EQ(m.triangles, actual.triangles);
  assert_vertices_equal(m, actual);
}

TEST(MeshTests, to_toml_no_vertices_skips_table_array) {
  collision::Mesh m = create_mesh();
  m.vertices.clear();
  auto tbl = m.to_toml();
  ASSERT_FALSE(tbl->contains("vertices"));
}

TEST(MeshTests, to_toml_no_triangles_skips_table_array) {
  collision::Mesh m = create_mesh();
  m.triangles.clear();
  auto tbl = m.to_toml();
  ASSERT_FALSE(tbl->contains("triangles"));
}

TEST(MeshTests, to_toml_through_ascii_file) {
  collision::Mesh m = create_mesh();
  FileRemover remover("/tmp/MeshTests-to-and-from-fcl.stl");
  bool binary_format = false;
  m.to_stl(remover.fname(), binary_format); // set filename

  auto tbl = m.to_toml();
  ASSERT_TRUE (tbl->get("mesh")->as_table()->contains("filename"));
  ASSERT_FALSE(tbl->get("mesh")->as_table()->contains("vertices"));
  ASSERT_FALSE(tbl->get("mesh")->as_table()->contains("triangles"));

  auto actual = collision::Mesh::from_toml(tbl);
  ASSERT_EQ(remover.fname(), actual.filename);
  ASSERT_EQ(m.triangles, actual.triangles);
  assert_vertices_equal(m, actual);
}

TEST(MeshTests, to_toml_through_binary_file) {
  collision::Mesh m = create_mesh();
  FileRemover remover("/tmp/MeshTests-to-and-from-fcl.stl");
  bool binary_format = true;
  m.to_stl(remover.fname(), binary_format); // set filename

  auto tbl = m.to_toml();
  ASSERT_TRUE (tbl->get("mesh")->as_table()->contains("filename"));
  ASSERT_FALSE(tbl->get("mesh")->as_table()->contains("vertices"));
  ASSERT_FALSE(tbl->get("mesh")->as_table()->contains("triangles"));

  auto actual = collision::Mesh::from_toml(tbl);
  ASSERT_EQ(remover.fname(), actual.filename);
  ASSERT_EQ(m.triangles, actual.triangles);
  assert_vertices_equal(m, actual);
}

TEST(MeshTests, from_toml_nullptr) {
  ASSERT_THROW(collision::Mesh::from_toml(nullptr),
               std::invalid_argument);
}

TEST(MeshTests, from_toml_empty) {
  auto tbl = cpptoml::make_table();
  auto mesh = collision::Mesh::from_toml(tbl);
  ASSERT_TRUE(mesh.filename.empty());
  ASSERT_TRUE(mesh.vertices.empty());
  ASSERT_TRUE(mesh.triangles.empty());
}

TEST(MeshTests, from_toml_in_container) {
  auto m = create_mesh();
  auto tbl = m.to_toml();
  ASSERT_TRUE(tbl->contains("mesh"));
  auto actual = collision::Mesh::from_toml(tbl);
  ASSERT_EQ(m.filename,  actual.filename);
  ASSERT_EQ(m.triangles, actual.triangles);
  assert_vertices_equal(m, actual);
}

TEST(MeshTests, from_toml_not_in_container) {
  auto m = create_mesh();
  auto tbl = m.to_toml()->get("mesh")->as_table();
  ASSERT_FALSE(tbl->contains("mesh"));
  auto actual = collision::Mesh::from_toml(tbl);
  ASSERT_EQ(m.filename,  actual.filename);
  ASSERT_EQ(m.triangles, actual.triangles);
  assert_vertices_equal(m, actual);
}

TEST(MeshTests, from_toml_missing_vertices) {
  auto tbl = create_mesh().to_toml();
  tbl->get("mesh")->as_table()->erase("vertices");
  auto mesh = collision::Mesh::from_toml(tbl);
  ASSERT_TRUE(mesh.vertices.empty());
}

TEST(MeshTests, from_toml_missing_triangles) {
  auto tbl = create_mesh().to_toml();
  tbl->get("mesh")->as_table()->erase("triangles");
  auto mesh = collision::Mesh::from_toml(tbl);
  ASSERT_TRUE(mesh.triangles.empty());
}

TEST(MeshTests, from_toml_vertices_missing_vertex) {
  auto tbl = create_mesh().to_toml();
  auto vertices_array =
      tbl->get("mesh")->as_table()->get("vertices")->as_table_array();
  ASSERT_TRUE(bool(vertices_array));
  ASSERT_EQ(vertices_array->get().size(), size_t(8));
  // corrupt the 3rd one
  vertices_array->get().at(2)->erase("vertex");
  ASSERT_THROW(Mesh::from_toml(tbl), std::out_of_range);
}

TEST(MeshTests, from_toml_triangles_missing_indices) {
  auto tbl = create_mesh().to_toml();
  auto triangles_array =
      tbl->get("mesh")->as_table()->get("triangles")->as_table_array();
  ASSERT_TRUE(bool(triangles_array));
  ASSERT_EQ(triangles_array->get().size(), size_t(12));
  // corrupt the 5th one
  triangles_array->get().at(4)->erase("indices");
  ASSERT_THROW(Mesh::from_toml(tbl), std::out_of_range);
}

TEST(MeshTests, from_toml_wrong_type_filename) {
  auto tbl = create_mesh().to_toml();
  tbl->get("mesh")->as_table()->insert("filename", 5);
  ASSERT_THROW(collision::Mesh::from_toml(tbl), cpptoml::parse_exception);
}

TEST(MeshTests, from_toml_wrong_type_vertices) {
  auto tbl = create_mesh().to_toml();
  tbl->get("mesh")->as_table()->insert("vertices", "hello");
  ASSERT_THROW(collision::Mesh::from_toml(tbl), cpptoml::parse_exception);
}

TEST(MeshTests, from_toml_wrong_type_triangles) {
  auto tbl = create_mesh().to_toml();
  tbl->get("mesh")->as_table()->insert("triangles", "hello");
  ASSERT_THROW(collision::Mesh::from_toml(tbl), cpptoml::parse_exception);
}

TEST(MeshTests, from_toml_wrong_type_vertex) {
  auto tbl = create_mesh().to_toml();
  auto vertices_array =
      tbl->get("mesh")->as_table()->get("vertices")->as_table_array();
  ASSERT_TRUE(bool(vertices_array));
  ASSERT_EQ(vertices_array->get().size(), size_t(8));
  // corrupt the 2nd one
  vertices_array->get().at(1)->insert("vertex", "hello there");
  ASSERT_THROW(Mesh::from_toml(tbl), cpptoml::parse_exception);
}

TEST(MeshTests, from_toml_wrong_type_triangle_indices) {
  auto tbl = create_mesh().to_toml();
  auto triangles_array =
      tbl->get("mesh")->as_table()->get("triangles")->as_table_array();
  ASSERT_TRUE(bool(triangles_array));
  ASSERT_EQ(triangles_array->get().size(), size_t(12));
  // corrupt the 11th one
  triangles_array->get().at(10)->insert("indices", "Bob");
  ASSERT_THROW(Mesh::from_toml(tbl), cpptoml::parse_exception);
}

TEST(MeshTests, from_toml_wrong_type_vertex_values) {
  auto tbl = create_mesh().to_toml();
  auto vertices_array =
      tbl->get("mesh")->as_table()->get("vertices")->as_table_array();
  ASSERT_TRUE(bool(vertices_array));
  ASSERT_EQ(vertices_array->get().size(), size_t(8));
  // corrupt the 8th one
  auto array_values = cpptoml::make_array();
  array_values->push_back("my");
  array_values->push_back("name's");
  array_values->push_back("Mike");
  vertices_array->get().at(7)->insert("vertex", array_values);
  ASSERT_THROW(Mesh::from_toml(tbl), cpptoml::parse_exception);
}

TEST(MeshTests, from_toml_wrong_type_triangle_indices_values) {
  auto tbl = create_mesh().to_toml();
  auto triangles_array =
      tbl->get("mesh")->as_table()->get("triangles")->as_table_array();
  ASSERT_TRUE(bool(triangles_array));
  ASSERT_EQ(triangles_array->get().size(), size_t(12));
  // corrupt the 1st one
  auto array_values = cpptoml::make_array();
  array_values->push_back("hi");
  array_values->push_back("there");
  array_values->push_back("Tom");
  triangles_array->get().at(0)->insert("indices", array_values);
  ASSERT_THROW(Mesh::from_toml(tbl), cpptoml::parse_exception);
}

TEST(MeshTests, from_toml_vertex_values_too_short) {
  auto tbl = create_mesh().to_toml();
  auto vertices_array =
      tbl->get("mesh")->as_table()->get("vertices")->as_table_array();
  ASSERT_TRUE(bool(vertices_array));
  ASSERT_EQ(vertices_array->get().size(), size_t(8));
  // corrupt the 8th one
  auto array_values = vertices_array->get().at(7)->get("vertex")->as_array();
  ASSERT_EQ(array_values->get().size(), size_t(3));
  array_values->erase(array_values->begin()); // remove first one
  ASSERT_EQ(array_values->get().size(), size_t(2));
  ASSERT_THROW(Mesh::from_toml(tbl), std::out_of_range);
}

TEST(MeshTests, from_toml_vertex_values_too_long) {
  auto tbl = create_mesh().to_toml();
  auto vertices_array =
      tbl->get("mesh")->as_table()->get("vertices")->as_table_array();
  ASSERT_TRUE(bool(vertices_array));
  ASSERT_EQ(vertices_array->get().size(), size_t(8));
  // corrupt the 8th one
  auto array_values = vertices_array->get().at(7)->get("vertex")->as_array();
  ASSERT_EQ(array_values->get().size(), size_t(3));
  array_values->push_back(1.0);
  ASSERT_EQ(array_values->get().size(), size_t(4));
  ASSERT_THROW(Mesh::from_toml(tbl), cpptoml::parse_exception);
}

TEST(MeshTests, from_toml_triangle_indices_values_too_short) {
  auto tbl = create_mesh().to_toml();
  auto triangles_array =
      tbl->get("mesh")->as_table()->get("triangles")->as_table_array();
  ASSERT_TRUE(bool(triangles_array));
  ASSERT_EQ(triangles_array->get().size(), size_t(12));
  // corrupt the 2nd one
  auto array_values = triangles_array->get().at(1)->get("indices")->as_array();
  ASSERT_EQ(array_values->get().size(), size_t(3));
  array_values->erase(array_values->begin()); // remove first one
  ASSERT_EQ(array_values->get().size(), size_t(2));
  ASSERT_THROW(Mesh::from_toml(tbl), std::out_of_range);
}

TEST(MeshTests, from_toml_triangle_indices_values_too_long) {
  auto tbl = create_mesh().to_toml();
  auto triangles_array =
      tbl->get("mesh")->as_table()->get("triangles")->as_table_array();
  ASSERT_TRUE(bool(triangles_array));
  ASSERT_EQ(triangles_array->get().size(), size_t(12));
  // corrupt the 2nd one
  auto array_values = triangles_array->get().at(1)->get("indices")->as_array();
  ASSERT_EQ(array_values->get().size(), size_t(3));
  array_values->push_back(5);
  ASSERT_EQ(array_values->get().size(), size_t(4));
  ASSERT_THROW(Mesh::from_toml(tbl), cpptoml::parse_exception);
}

TEST(MeshTests, equal_not_same_triangles) {
  auto mesh = create_mesh();
  auto mesh_2 = create_mesh();
  mesh_2.triangles.emplace_back(1, 2, 3);
  ASSERT_FALSE(mesh.equal(mesh_2));
  ASSERT_FALSE(mesh_2.equal(mesh));
}

TEST(MeshTests, equal_not_same_vertices) {
  auto mesh = create_mesh();
  auto mesh_2 = create_mesh();
  mesh_2.vertices.emplace_back(1.0, 2.0, 3.0);
  ASSERT_FALSE(mesh.equal(mesh_2));
  ASSERT_FALSE(mesh_2.equal(mesh));
}

TEST(MeshTests, equal_not_same_filename) {
  auto mesh = create_mesh();
  auto mesh_2 = create_mesh();
  mesh.filename = "some/file.stl";
  mesh_2.filename = "other/file.stl";
  ASSERT_TRUE(mesh.equal(mesh_2));
  ASSERT_TRUE(mesh_2.equal(mesh_2));
}

TEST(MeshTests, equal_same_object) {
  auto mesh = create_mesh();
  auto mesh_2 = create_mesh();
  mesh.filename = "some/file.stl";
  mesh_2.filename = "other/file.stl";
  ASSERT_TRUE(mesh.equal(mesh));
  ASSERT_TRUE(mesh_2.equal(mesh_2));
}

TEST(MeshTests, equal_operator_not_same_triangles) {
  auto mesh = create_mesh();
  auto mesh_2 = create_mesh();
  mesh_2.triangles.emplace_back(1, 2, 3);
  ASSERT_FALSE(mesh.equal(mesh_2));
  ASSERT_FALSE(mesh_2.equal(mesh));
}

TEST(MeshTests, equal_operator_not_same_vertices) {
  auto mesh = create_mesh();
  auto mesh_2 = create_mesh();
  mesh_2.vertices.emplace_back(1.0, 2.0, 3.0);
  ASSERT_FALSE(mesh == mesh_2);
  ASSERT_FALSE(mesh_2 == mesh);
}

TEST(MeshTests, equal_operator_not_same_filename) {
  auto mesh = create_mesh();
  auto mesh_2 = create_mesh();
  mesh.filename = "some/file.stl";
  mesh_2.filename = "other/file.stl";
  ASSERT_FALSE(mesh == mesh_2);
  ASSERT_FALSE(mesh_2 == mesh);
}

TEST(MeshTests, equal_operator_same_object) {
  auto mesh = create_mesh();
  auto mesh_2 = create_mesh();
  mesh.filename = "some/file.stl";
  mesh_2.filename = "other/file.stl";
  ASSERT_TRUE(mesh == mesh);
  ASSERT_TRUE(mesh_2 == mesh_2);
}

/**
 * author:        Michael Bentley
 * email:         mikebentley15@gmail.com
 * date-created:  20 March 2020
 */

#include "motion-planning/Environment.h"
#include <collision/Point.h>
#include <collision/Sphere.h>
#include <collision/Capsule.h>
#include <collision/Mesh.h>
#include <cpptoml/cpptoml.h>
#include <cpptoml/toml_conversions.h>

#include <Eigen/Core>

#include <gtest/gtest.h>

using motion_planning::Environment;
using collision::Point;
using collision::Sphere;
using collision::Capsule;
using collision::Mesh;

using collision::operator<<;

namespace {

Environment create_environment() {
  Environment env;
  env.push_back(Point{1, 2, 3});
  env.push_back(Point{2, 3, 4});
  env.push_back(Sphere{{1, 3, 5}, 2});
  env.push_back(Sphere{{2, 4, 6}, 3});
  env.push_back(Sphere{{3, 5, 7}, 4});
  env.push_back(Capsule{{1, 2, 3}, {3, 2, 1}, 6.6});

  Mesh m1;
  m1.vertices = {{1, 2, 3}, {2, 3, 4}, {3, 4, 5}};
  m1.triangles = {{0, 1, 2}};
  env.push_back(std::move(m1));

  Mesh m2;
  m2.vertices = {{1, 1, 1}, {1, 1, 2}, {1, 2, 1}, {2, 1, 1}};
  m2.triangles = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
  m2.to_stl("/tmp/tst_Environment-mesh.stl"); // save and set filename
  env.push_back(std::move(m2));

  return env;
}

} // end of unnamed namespace

TEST(EnvironmentTests, empty_environment_does_not_collide_with_other) {
  Environment empty;
  auto nonempty = create_environment();
  ASSERT_FALSE(empty.collides(empty));
  ASSERT_FALSE(nonempty.collides(empty));
  ASSERT_FALSE(empty.collides(nonempty));
}

TEST(EnvironmentTests, all_collides_with_self) {
  auto env = create_environment();
  ASSERT_TRUE(env.collides(env));
}

TEST(EnvironmentTests, DISABLED_points_collide_with_self) {
  auto env = create_environment();
  Environment only_points;
  only_points.set_vec<Point>(env.points());
  ASSERT_TRUE(env.collides(only_points));
}

TEST(EnvironmentTests, to_toml_default) {
  Environment env;
  auto actual = Environment::from_toml(Environment().to_toml());
  ASSERT_EQ(env.points  (), actual.points  ());
  ASSERT_EQ(env.spheres (), actual.spheres ());
  ASSERT_EQ(env.capsules(), actual.capsules());
  ASSERT_EQ(env.meshes  (), actual.meshes  ());
  ASSERT_EQ(env, actual);
}

TEST(EnvironmentTests, to_toml) {
  Environment env = create_environment();
  auto actual = Environment::from_toml(env.to_toml());
  ASSERT_EQ(env.points  (), actual.points  ());
  ASSERT_EQ(env.spheres (), actual.spheres ());
  ASSERT_EQ(env.capsules(), actual.capsules());
  ASSERT_EQ(env.meshes  (), actual.meshes  ());
  ASSERT_EQ(env, actual);
}

TEST(EnvironmentTests, to_toml_default_through_string) {
  Environment env;
  auto str = cpptoml::to_string(env.to_toml());
  std::istringstream in(str);
  cpptoml::parser toml_parser(in);
  auto actual = Environment::from_toml(toml_parser.parse());
  ASSERT_EQ(env, actual);
}

TEST(EnvironmentTests, to_toml_through_string) {
  Environment env = create_environment();
  auto str = cpptoml::to_string(env.to_toml());
  std::istringstream in(str);
  cpptoml::parser toml_parser(in);
  auto actual = Environment::from_toml(toml_parser.parse());
  ASSERT_EQ(env, actual);
}

TEST(EnvironmentTests, to_toml_no_points_skips_points_table_array) {
  Environment env;
  auto tbl = Environment().to_toml();
  ASSERT_FALSE(tbl->contains("points"));
}

TEST(EnvironmentTests, to_toml_no_spheres_skips_spheres_table_array) {
  Environment env;
  auto tbl = Environment().to_toml();
  ASSERT_FALSE(tbl->contains("spheres"));
}

TEST(EnvironmentTests, to_toml_no_capsules_skips_capsules_table_array) {
  Environment env;
  auto tbl = Environment().to_toml();
  ASSERT_FALSE(tbl->contains("capsules"));
}

TEST(EnvironmentTests, to_toml_no_meshes_skips_capsules_table_array) {
  Environment env;
  auto tbl = Environment().to_toml();
  ASSERT_FALSE(tbl->contains("meshes"));
}

TEST(EnvironmentTests, from_toml_nullptr) {
  ASSERT_THROW(Environment::from_toml(nullptr), std::invalid_argument);
}

TEST(EnvironmentTests, from_toml_empty) {
  auto tbl = cpptoml::make_table();
  ASSERT_EQ(Environment::from_toml(tbl), Environment());
}

TEST(EnvironmentTests, from_toml_in_container) {
  auto env = create_environment();
  auto tbl = env.to_toml();
  ASSERT_TRUE(tbl->contains("environment"));
  auto actual = Environment::from_toml(tbl);
  ASSERT_EQ(env, actual);
}

TEST(EnvironmentTests, from_toml_not_in_container) {
  auto env = create_environment();
  auto tbl = env.to_toml()->get("environment")->as_table();
  ASSERT_FALSE(tbl->contains("environment"));
  auto actual = Environment::from_toml(tbl);
  ASSERT_EQ(env, actual);
}

TEST(EnvironmentTests, from_toml_missing_points) {
  auto tbl = create_environment().to_toml();
  tbl->get("environment")->as_table()->erase("points");
  auto actual = Environment::from_toml(tbl);
  ASSERT_TRUE(actual.points().empty());
}

TEST(EnvironmentTests, from_toml_missing_spheres) {
  auto tbl = create_environment().to_toml();
  tbl->get("environment")->as_table()->erase("spheres");
  auto actual = Environment::from_toml(tbl);
  ASSERT_TRUE(actual.spheres().empty());
}

TEST(EnvironmentTests, from_toml_missing_capsules) {
  auto tbl = create_environment().to_toml();
  tbl->get("environment")->as_table()->erase("capsules");
  auto actual = Environment::from_toml(tbl);
  ASSERT_TRUE(actual.capsules().empty());
}

TEST(EnvironmentTests, from_toml_missing_meshes) {
  auto tbl = create_environment().to_toml();
  tbl->get("environment")->as_table()->erase("meshes");
  auto actual = Environment::from_toml(tbl);
  ASSERT_TRUE(actual.meshes().empty());
}

TEST(EnvironmentTests, from_toml_wrong_type_points) {
  auto tbl = create_environment().to_toml();
  tbl->get("environment")->as_table()->insert("points", "name");
  ASSERT_THROW(Environment::from_toml(tbl), cpptoml::parse_exception);
}

TEST(EnvironmentTests, from_toml_wrong_type_spheres) {
  auto tbl = create_environment().to_toml();
  tbl->get("environment")->as_table()->insert("spheres", "name");
  ASSERT_THROW(Environment::from_toml(tbl), cpptoml::parse_exception);
}

TEST(EnvironmentTests, from_toml_wrong_type_capsules) {
  auto tbl = create_environment().to_toml();
  tbl->get("environment")->as_table()->insert("capsules", "name");
  ASSERT_THROW(Environment::from_toml(tbl), cpptoml::parse_exception);
}

TEST(EnvironmentTests, from_toml_wrong_type_meshes) {
  auto tbl = create_environment().to_toml();
  tbl->get("environment")->as_table()->insert("meshes", "name");
  ASSERT_THROW(Environment::from_toml(tbl), cpptoml::parse_exception);
}

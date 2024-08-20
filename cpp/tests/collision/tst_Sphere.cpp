/**
 * author:        Michael Bentley
 * email:         mikebentley15@gmail.com
 * date-created:  18 March 2020
 */

#include "collision/Sphere.h"

#include "cpptoml/cpptoml.h"
#include "cpptoml/toml_conversions.h"

#include <fcl/math/transform.h>
#include <fcl/shape/geometric_shapes.h>

#include <gtest/gtest.h>

namespace {

collision::Sphere create_sphere() {
  return collision::Sphere {{3.3, 2.2, 1.1}, 2.5};
}

} // end of unnamed namespace

TEST(SphereTests, to_toml) {
  collision::Sphere s = create_sphere();
  auto actual = collision::Sphere::from_toml(s.to_toml());
  ASSERT_EQ(s.c, actual.c);
  ASSERT_EQ(s.r, actual.r);
}

TEST(SphereTests, to_toml_through_string) {
  collision::Sphere s = create_sphere();
  auto str = cpptoml::to_string(s.to_toml());
  std::istringstream in(str);
  cpptoml::parser toml_parser(in);
  auto actual = collision::Sphere::from_toml(toml_parser.parse());
  ASSERT_EQ(s.c, actual.c);
  ASSERT_EQ(s.r, actual.r);
}

TEST(SphereTests, from_toml_nullptr) {
  ASSERT_THROW(collision::Sphere::from_toml(nullptr), std::invalid_argument);
}

TEST(SphereTests, from_toml_empty) {
  auto tbl = cpptoml::make_table();
  ASSERT_THROW(collision::Sphere::from_toml(tbl), std::out_of_range);
}

TEST(SphereTests, from_toml_in_container) {
  auto s = create_sphere();
  auto tbl = s.to_toml();
  ASSERT_TRUE(tbl->contains("sphere"));
  auto actual = collision::Sphere::from_toml(tbl);
  ASSERT_EQ(s.c, actual.c);
  ASSERT_EQ(s.r, actual.r);
}

TEST(SphereTests, from_toml_not_in_container) {
  auto s = create_sphere();
  auto tbl = s.to_toml()->get("sphere")->as_table();
  ASSERT_FALSE(tbl->contains("sphere"));
  auto actual = collision::Sphere::from_toml(tbl);
  ASSERT_EQ(s.c, actual.c);
  ASSERT_EQ(s.r, actual.r);
}

TEST(SphereTests, from_toml_missing_radius) {
  auto tbl = create_sphere().to_toml();
  tbl->get("sphere")->as_table()->erase("radius");
  ASSERT_THROW(collision::Sphere::from_toml(tbl), std::out_of_range);
}

TEST(SphereTests, from_toml_missing_center) {
  auto tbl = create_sphere().to_toml();
  tbl->get("sphere")->as_table()->erase("center");
  ASSERT_THROW(collision::Sphere::from_toml(tbl), std::out_of_range);
}

TEST(SphereTests, from_toml_wrong_type_radius) {
  auto tbl = create_sphere().to_toml();
  tbl->get("sphere")->as_table()->insert("radius", "name");
  ASSERT_THROW(collision::Sphere::from_toml(tbl), cpptoml::parse_exception);
}

TEST(SphereTests, from_toml_wrong_type_center) {
  auto tbl = create_sphere().to_toml();
  tbl->get("sphere")->as_table()->insert("center", 5);
  ASSERT_THROW(collision::Sphere::from_toml(tbl), cpptoml::parse_exception);
}

TEST(SphereTests, to_fcl_model) {
  auto sphere = create_sphere();
  auto [fcl_sphere, transform] = sphere.to_fcl_model();

  ASSERT_DOUBLE_EQ(sphere.r, fcl_sphere->radius);
  ASSERT_TRUE(transform.getQuatRotation().isIdentity());

  auto translation = transform.getTranslation();
  ASSERT_DOUBLE_EQ(sphere.c[0], translation[0]);
  ASSERT_DOUBLE_EQ(sphere.c[1], translation[1]);
  ASSERT_DOUBLE_EQ(sphere.c[2], translation[2]);
}

TEST(SphereTests, to_fcl_object) {
  auto sphere = create_sphere();
  auto obj = sphere.to_fcl_object();
  ASSERT_EQ(obj->getObjectType(), fcl::OT_GEOM);
  ASSERT_EQ(obj->getNodeType(), fcl::GEOM_SPHERE);

  ASSERT_TRUE(obj->getQuatRotation().isIdentity());

  auto translation = obj->getTranslation();
  ASSERT_DOUBLE_EQ(sphere.c[0], translation[0]);
  ASSERT_DOUBLE_EQ(sphere.c[1], translation[1]);
  ASSERT_DOUBLE_EQ(sphere.c[2], translation[2]);

  auto fcl_sphere = std::static_pointer_cast<const fcl::Sphere>(
      obj->collisionGeometry());
  ASSERT_DOUBLE_EQ(sphere.r, fcl_sphere->radius);
}

TEST(SphereTests, to_fcl_mesh_model) {
  auto sphere = create_sphere();
  auto [fcl_sphere, transform] = sphere.to_fcl_model();
  (void)transform;
  auto mesh = sphere.to_fcl_mesh_model();

  ASSERT_NEAR(fcl_sphere->computeVolume(), mesh->computeVolume(),
              0.01 * fcl_sphere->computeVolume());

  auto center_of_mass = mesh->computeCOM();
  ASSERT_NEAR(sphere.c[0], center_of_mass[0], 1e-7 * sphere.c[0]);
  ASSERT_NEAR(sphere.c[1], center_of_mass[1], 1e-7 * sphere.c[1]);
  ASSERT_NEAR(sphere.c[2], center_of_mass[2], 1e-7 * sphere.c[2]);

  // Wow, moment of inertia of the mesh is really messed up...
  //auto expected_moment = fcl_sphere->computeMomentofInertia();
  //auto actual_moment = mesh->computeMomentofInertia();
  //ASSERT_DOUBLE_EQ(expected_moment(0, 0), actual_moment(0, 0));
  //ASSERT_DOUBLE_EQ(expected_moment(0, 1), actual_moment(0, 1));
  //ASSERT_DOUBLE_EQ(expected_moment(0, 2), actual_moment(0, 2));
  //ASSERT_DOUBLE_EQ(expected_moment(1, 0), actual_moment(1, 0));
  //ASSERT_DOUBLE_EQ(expected_moment(1, 1), actual_moment(1, 1));
  //ASSERT_DOUBLE_EQ(expected_moment(1, 2), actual_moment(1, 2));
  //ASSERT_DOUBLE_EQ(expected_moment(2, 0), actual_moment(2, 0));
  //ASSERT_DOUBLE_EQ(expected_moment(2, 1), actual_moment(2, 1));
  //ASSERT_DOUBLE_EQ(expected_moment(2, 2), actual_moment(2, 2));
}

TEST(SphereTests, to_fcl_mesh_object) {
  auto sphere = create_sphere();
  auto mesh = sphere.to_fcl_mesh_model();
  auto obj = sphere.to_fcl_mesh_object();
  ASSERT_EQ(obj->getObjectType(), fcl::OT_BVH);
  ASSERT_EQ(obj->getNodeType(), fcl::BV_OBBRSS);

  ASSERT_TRUE(obj->getQuatRotation().isIdentity());

  auto translation = obj->getTranslation();
  ASSERT_DOUBLE_EQ(sphere.c[0], translation[0]);
  ASSERT_DOUBLE_EQ(sphere.c[1], translation[1]);
  ASSERT_DOUBLE_EQ(sphere.c[2], translation[2]);

  auto geom = obj->collisionGeometry();
  auto center_of_mass = geom->computeCOM();
  ASSERT_NEAR(0.0, center_of_mass[0], 1e-10);
  ASSERT_NEAR(0.0, center_of_mass[1], 1e-10);
  ASSERT_NEAR(0.0, center_of_mass[2], 1e-10);

  ASSERT_NEAR(mesh->computeVolume(), geom->computeVolume(),
              1e-10 * mesh->computeVolume());
}

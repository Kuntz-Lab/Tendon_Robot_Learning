/**
 * author:        Michael Bentley
 * email:         mikebentley15@gmail.com
 * date-created:  18 March 2020
 */

#include "collision/Capsule.h"

#include "cpptoml/cpptoml.h"
#include "cpptoml/toml_conversions.h"

#include <fcl/collision_object.h>
#include <fcl/math/transform.h>
#include <fcl/shape/geometric_shapes.h>

#include <gtest/gtest.h>

namespace {

collision::Capsule create_capsule() {
  return collision::Capsule{{3.3, 2.2, 1.1}, {4.4, 5.5, 6.6}, 2.5};
}

} // end of unnamed namespace

TEST(CapsuleTests, to_toml) {
  collision::Capsule c = create_capsule();
  auto actual = collision::Capsule::from_toml(c.to_toml());
  ASSERT_EQ(c.a, actual.a);
  ASSERT_EQ(c.b, actual.b);
  ASSERT_EQ(c.r, actual.r);
}

TEST(CapsuleTests, to_toml_through_string) {
  collision::Capsule c = create_capsule();
  auto str = cpptoml::to_string(c.to_toml());
  std::istringstream in(str);
  cpptoml::parser toml_parser(in);
  auto actual = collision::Capsule::from_toml(toml_parser.parse());
  ASSERT_EQ(c.a, actual.a);
  ASSERT_EQ(c.b, actual.b);
  ASSERT_EQ(c.r, actual.r);
}

TEST(CapsuleTests, from_toml_nullptr) {
  ASSERT_THROW(collision::Capsule::from_toml(nullptr), std::invalid_argument);
}

TEST(CapsuleTests, from_toml_empty) {
  auto tbl = cpptoml::make_table();
  ASSERT_THROW(collision::Capsule::from_toml(tbl), std::out_of_range);
}

TEST(CapsuleTests, from_toml_in_container) {
  auto c = create_capsule();
  auto tbl = c.to_toml();
  ASSERT_TRUE(tbl->contains("capsule"));
  auto actual = collision::Capsule::from_toml(tbl);
  ASSERT_EQ(c.a, actual.a);
  ASSERT_EQ(c.b, actual.b);
  ASSERT_EQ(c.r, actual.r);
}

TEST(CapsuleTests, from_toml_not_in_container) {
  auto c = create_capsule();
  auto tbl = c.to_toml()->get("capsule")->as_table();
  ASSERT_FALSE(tbl->contains("capsule"));
  auto actual = collision::Capsule::from_toml(tbl);
  ASSERT_EQ(c.a, actual.a);
  ASSERT_EQ(c.b, actual.b);
  ASSERT_EQ(c.r, actual.r);
}

TEST(CapsuleTests, from_toml_missing_a) {
  auto tbl = create_capsule().to_toml();
  tbl->get("capsule")->as_table()->erase("a");
  ASSERT_THROW(collision::Capsule::from_toml(tbl), std::out_of_range);
}

TEST(CapsuleTests, from_toml_missing_b) {
  auto tbl = create_capsule().to_toml();
  tbl->get("capsule")->as_table()->erase("b");
  ASSERT_THROW(collision::Capsule::from_toml(tbl), std::out_of_range);
}

TEST(CapsuleTests, from_toml_missing_radius) {
  auto tbl = create_capsule().to_toml();
  tbl->get("capsule")->as_table()->erase("radius");
  ASSERT_THROW(collision::Capsule::from_toml(tbl), std::out_of_range);
}

TEST(CapsuleTests, from_toml_wrong_type_a) {
  auto tbl = create_capsule().to_toml();
  tbl->get("capsule")->as_table()->insert("a", 5);
  ASSERT_THROW(collision::Capsule::from_toml(tbl), cpptoml::parse_exception);
}

TEST(CapsuleTests, from_toml_wrong_type_b) {
  auto tbl = create_capsule().to_toml();
  tbl->get("capsule")->as_table()->insert("b", 5);
  ASSERT_THROW(collision::Capsule::from_toml(tbl), cpptoml::parse_exception);
}

TEST(CapsuleTests, from_toml_wrong_type_radius) {
  auto tbl = create_capsule().to_toml();
  tbl->get("capsule")->as_table()->insert("radius", "name");
  ASSERT_THROW(collision::Capsule::from_toml(tbl), cpptoml::parse_exception);
}

TEST(CapsuleTests, to_fcl_model) {
  auto capsule = create_capsule();
  auto [fcl_capsule, transform] = capsule.to_fcl_model();

  ASSERT_DOUBLE_EQ(capsule.r, fcl_capsule->radius);
  ASSERT_DOUBLE_EQ((capsule.b - capsule.a).norm(), fcl_capsule->lz);

  auto actual_a = transform.transform(fcl::Vec3f(0, 0, -fcl_capsule->lz / 2.0));
  auto actual_b = transform.transform(fcl::Vec3f(0, 0,  fcl_capsule->lz / 2.0));
  ASSERT_DOUBLE_EQ(capsule.a[0], actual_a[0]);
  ASSERT_DOUBLE_EQ(capsule.a[1], actual_a[1]);
  ASSERT_DOUBLE_EQ(capsule.a[2], actual_a[2]);
  ASSERT_DOUBLE_EQ(capsule.b[0], actual_b[0]);
  ASSERT_DOUBLE_EQ(capsule.b[1], actual_b[1]);
  ASSERT_DOUBLE_EQ(capsule.b[2], actual_b[2]);
}

TEST(CapsuleTests, to_fcl_object) {
  auto capsule = create_capsule();
  auto obj = capsule.to_fcl_object();
  ASSERT_EQ(obj->getObjectType(), fcl::OT_GEOM);
  ASSERT_EQ(obj->getNodeType(), fcl::GEOM_CAPSULE);

  auto fcl_capsule = std::static_pointer_cast<const fcl::Capsule>(
      obj->collisionGeometry());
  auto transform = obj->getTransform();

  ASSERT_DOUBLE_EQ(capsule.r, fcl_capsule->radius);
  ASSERT_DOUBLE_EQ((capsule.b - capsule.a).norm(), fcl_capsule->lz);

  auto actual_a = transform.transform(fcl::Vec3f(0, 0, -fcl_capsule->lz / 2.0));
  auto actual_b = transform.transform(fcl::Vec3f(0, 0,  fcl_capsule->lz / 2.0));
  ASSERT_DOUBLE_EQ(capsule.a[0], actual_a[0]);
  ASSERT_DOUBLE_EQ(capsule.a[1], actual_a[1]);
  ASSERT_DOUBLE_EQ(capsule.a[2], actual_a[2]);
  ASSERT_DOUBLE_EQ(capsule.b[0], actual_b[0]);
  ASSERT_DOUBLE_EQ(capsule.b[1], actual_b[1]);
  ASSERT_DOUBLE_EQ(capsule.b[2], actual_b[2]);
}

// TODO: maybe implement these later (not built-in to FCL)
//TEST(CapsuleTests, to_fcl_mesh_model) {
//  
//}
//
//TEST(CapsuleTests, to_fcl_mesh_object) {
//  
//}

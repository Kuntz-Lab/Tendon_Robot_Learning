/**
 * author:        Michael Bentley
 * email:         mikebentley15@gmail.com
 * date-created:  20 March 2020
 */

#include "collision/CapsuleSequence.h"

#include "cpptoml/cpptoml.h"
#include "cpptoml/toml_conversions.h"

#include <gtest/gtest.h>

namespace {

collision::CapsuleSequence create_capsule_sequence() {
  return collision::CapsuleSequence {
    {{3.3, 2.2, 1.1}, {4.4, 5.5, 6.6}, {1, 2, 3}},
    2.5
  };
}

}

TEST(CapsuleSequenceTests, to_toml_empty) {
  collision::CapsuleSequence s{{}, 2.5};
  auto actual = collision::CapsuleSequence::from_toml(s.to_toml());
  ASSERT_EQ(s.points, actual.points);
  ASSERT_EQ(s.r,      actual.r);
}

TEST(CapsuleSequenceTests, to_toml) {
  collision::CapsuleSequence s = create_capsule_sequence();
  auto actual = collision::CapsuleSequence::from_toml(s.to_toml());
  ASSERT_EQ(s.points, actual.points);
  ASSERT_EQ(s.r,      actual.r);
}

TEST(CapsuleSequenceTests, to_toml_empty_through_string) {
  collision::CapsuleSequence s{{}, 2.5};
  auto str = cpptoml::to_string(s.to_toml());
  std::istringstream in(str);
  cpptoml::parser toml_parser(in);
  auto actual = collision::CapsuleSequence::from_toml(toml_parser.parse());
  ASSERT_EQ(s.points, actual.points);
  ASSERT_EQ(s.r,      actual.r);
}

TEST(CapsuleSequenceTests, to_toml_through_string) {
  auto s = create_capsule_sequence();
  auto str = cpptoml::to_string(s.to_toml());
  std::istringstream in(str);
  cpptoml::parser toml_parser(in);
  auto actual = collision::CapsuleSequence::from_toml(toml_parser.parse());
  ASSERT_EQ(s.points, actual.points);
  ASSERT_EQ(s.r,      actual.r);
}

TEST(CapsuleSequenceTests, to_toml_no_points_skips_points_table_array) {
  auto tbl = collision::CapsuleSequence().to_toml();
  ASSERT_FALSE(tbl->contains("points"));
}

TEST(CapsuleSequenceTests, from_toml_nullptr) {
  ASSERT_THROW(collision::CapsuleSequence::from_toml(nullptr),
               std::invalid_argument);
}

TEST(CapsuleSequenceTests, from_toml_empty) {
  auto tbl = cpptoml::make_table();
  ASSERT_THROW(collision::CapsuleSequence::from_toml(tbl), std::out_of_range);
}

TEST(CapsuleSequenceTests, from_toml_in_container) {
  auto cs = create_capsule_sequence();
  auto tbl = cs.to_toml();
  ASSERT_TRUE(tbl->contains("capsule_sequence"));
  auto actual = collision::CapsuleSequence::from_toml(tbl);
  ASSERT_EQ(cs.points, actual.points);
  ASSERT_EQ(cs.r     , actual.r     );
}

TEST(CapsuleSequenceTests, from_toml_not_in_container) {
  auto cs = create_capsule_sequence();
  auto tbl = cs.to_toml()->get("capsule_sequence")->as_table();
  ASSERT_FALSE(tbl->contains("capsule_sequence"));
  auto actual = collision::CapsuleSequence::from_toml(tbl);
  ASSERT_EQ(cs.points, actual.points);
  ASSERT_EQ(cs.r     , actual.r     );
}

TEST(CapsuleSequenceTests, from_toml_missing_radius) {
  auto tbl = create_capsule_sequence().to_toml();
  tbl->get("capsule_sequence")->as_table()->erase("radius");
  ASSERT_THROW(collision::CapsuleSequence::from_toml(tbl), std::out_of_range);
}

TEST(CapsuleSequenceTests, from_toml_missing_points) {
  auto tbl = create_capsule_sequence().to_toml();
  tbl->get("capsule_sequence")->as_table()->erase("points");
  auto actual = collision::CapsuleSequence::from_toml(tbl);
  ASSERT_TRUE(actual.points.empty());
}

TEST(CapsuleSequenceTests, from_toml_wrong_type_radius) {
  auto tbl = create_capsule_sequence().to_toml();
  tbl->get("capsule_sequence")->as_table()->insert("radius", "name");
  ASSERT_THROW(collision::CapsuleSequence::from_toml(tbl), cpptoml::parse_exception);
}

TEST(CapsuleSequenceTests, from_toml_wrong_type_points) {
  auto tbl = create_capsule_sequence().to_toml();
  tbl->get("capsule_sequence")->as_table()->insert("points", 5);
  ASSERT_THROW(collision::CapsuleSequence::from_toml(tbl), cpptoml::parse_exception);
}

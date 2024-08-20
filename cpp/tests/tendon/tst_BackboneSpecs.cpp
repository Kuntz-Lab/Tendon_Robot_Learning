/**
 * author:        Michael Bentley
 * email:         mikebentley15@gmail.com
 * date-created:  20 March 2020
 */

#include "tendon/BackboneSpecs.h"
#include <cpptoml/cpptoml.h>
#include <cpptoml/toml_conversions.h>

#include <gtest/gtest.h>

using tendon::BackboneSpecs;

namespace {

BackboneSpecs create_specs() {
  BackboneSpecs specs;
  specs.L = 3.345;
  specs.dL = 1.001;
  specs.ro = 2.02;
  specs.ri = 3.001;
  specs.E = 3.4e7;
  specs.nu = 0.53;
  return specs;
}

} // end of unnamed namespace

TEST(BackboneSpecsTests, to_toml_default) {
  BackboneSpecs specs;
  auto actual = BackboneSpecs::from_toml(BackboneSpecs().to_toml());
  ASSERT_EQ(specs.L , actual.L );
  ASSERT_EQ(specs.dL, actual.dL);
  ASSERT_EQ(specs.ro, actual.ro);
  ASSERT_EQ(specs.ri, actual.ri);
  ASSERT_EQ(specs.E , actual.E );
  ASSERT_EQ(specs.nu, actual.nu);
}

TEST(BackboneSpecsTests, to_toml) {
  auto specs = create_specs();
  auto actual = BackboneSpecs::from_toml(specs.to_toml());
  ASSERT_EQ(specs.L , actual.L );
  ASSERT_EQ(specs.dL, actual.dL);
  ASSERT_EQ(specs.ro, actual.ro);
  ASSERT_EQ(specs.ri, actual.ri);
  ASSERT_EQ(specs.E , actual.E );
  ASSERT_EQ(specs.nu, actual.nu);
}

TEST(BackboneSpecsTests, to_toml_default_through_string) {
  BackboneSpecs specs;
  auto str = cpptoml::to_string(specs.to_toml());
  std::istringstream in(str);
  cpptoml::parser toml_parser(in);
  auto actual = BackboneSpecs::from_toml(toml_parser.parse());
  ASSERT_EQ(specs.L , actual.L );
  ASSERT_EQ(specs.dL, actual.dL);
  ASSERT_EQ(specs.ro, actual.ro);
  ASSERT_EQ(specs.ri, actual.ri);
  ASSERT_EQ(specs.E , actual.E );
  ASSERT_EQ(specs.nu, actual.nu);
}

TEST(BackboneSpecsTests, to_toml_through_string) {
  BackboneSpecs specs = create_specs();
  auto str = cpptoml::to_string(specs.to_toml());
  std::istringstream in(str);
  cpptoml::parser toml_parser(in);
  auto actual = BackboneSpecs::from_toml(toml_parser.parse());
  ASSERT_EQ(specs.L , actual.L );
  ASSERT_EQ(specs.dL, actual.dL);
  ASSERT_EQ(specs.ro, actual.ro);
  ASSERT_EQ(specs.ri, actual.ri);
  ASSERT_EQ(specs.E , actual.E );
  ASSERT_EQ(specs.nu, actual.nu);
}

TEST(BackboneSpecsTests, from_toml_nullptr) {
  ASSERT_THROW(BackboneSpecs::from_toml(nullptr), std::invalid_argument);
}

TEST(BackboneSpecsTests, from_toml_empty) {
  auto tbl = cpptoml::make_table();
  ASSERT_THROW(BackboneSpecs::from_toml(tbl), std::out_of_range);
}

TEST(BackboneSpecsTests, from_toml_in_container) {
  auto specs = create_specs();
  auto tbl = specs.to_toml();
  ASSERT_TRUE(tbl->contains("backbone_specs"));
  auto actual = BackboneSpecs::from_toml(tbl);
  ASSERT_EQ(actual, specs);
}

TEST(BackboneSpecsTests, from_toml_missing_L) {
  auto tbl = create_specs().to_toml();
  tbl->get("backbone_specs")->as_table()->erase("length");
  ASSERT_THROW(BackboneSpecs::from_toml(tbl), std::out_of_range);
}

TEST(BackboneSpecsTests, from_toml_missing_dL) {
  auto tbl = create_specs().to_toml();
  tbl->get("backbone_specs")->as_table()->erase("length_discretization");
  ASSERT_THROW(BackboneSpecs::from_toml(tbl), std::out_of_range);
}

TEST(BackboneSpecsTests, from_toml_missing_ro) {
  auto tbl = create_specs().to_toml();
  tbl->get("backbone_specs")->as_table()->erase("ro");
  ASSERT_THROW(BackboneSpecs::from_toml(tbl), std::out_of_range);
}

TEST(BackboneSpecsTests, from_toml_missing_ri) {
  auto tbl = create_specs().to_toml();
  tbl->get("backbone_specs")->as_table()->erase("ri");
  ASSERT_THROW(BackboneSpecs::from_toml(tbl), std::out_of_range);
}

TEST(BackboneSpecsTests, from_toml_missing_E) {
  auto tbl = create_specs().to_toml();
  tbl->get("backbone_specs")->as_table()->erase("E");
  ASSERT_THROW(BackboneSpecs::from_toml(tbl), std::out_of_range);
}

TEST(BackboneSpecsTests, from_toml_missing_nu) {
  auto tbl = create_specs().to_toml();
  tbl->get("backbone_specs")->as_table()->erase("nu");
  ASSERT_THROW(BackboneSpecs::from_toml(tbl), std::out_of_range);
}

TEST(BackboneSpecsTests, from_toml_wrong_type_L) {
  auto tbl = create_specs().to_toml();
  tbl->get("backbone_specs")->as_table()->insert("length", "name");
  ASSERT_THROW(BackboneSpecs::from_toml(tbl), cpptoml::parse_exception);
}

TEST(BackboneSpecsTests, from_toml_wrong_type_dL) {
  auto tbl = create_specs().to_toml();
  tbl->get("backbone_specs")->as_table()
      ->insert("length_discretization", "name");
  ASSERT_THROW(BackboneSpecs::from_toml(tbl), cpptoml::parse_exception);
}

TEST(BackboneSpecsTests, from_toml_wrong_type_ro) {
  auto tbl = create_specs().to_toml();
  tbl->get("backbone_specs")->as_table()->insert("ro", "name");
  ASSERT_THROW(BackboneSpecs::from_toml(tbl), cpptoml::parse_exception);
}

TEST(BackboneSpecsTests, from_toml_wrong_type_ri) {
  auto tbl = create_specs().to_toml();
  tbl->get("backbone_specs")->as_table()->insert("ri", "name");
  ASSERT_THROW(BackboneSpecs::from_toml(tbl), cpptoml::parse_exception);
}

TEST(BackboneSpecsTests, from_toml_wrong_type_E) {
  auto tbl = create_specs().to_toml();
  tbl->get("backbone_specs")->as_table()->insert("E", "name");
  ASSERT_THROW(BackboneSpecs::from_toml(tbl), cpptoml::parse_exception);
}

TEST(BackboneSpecsTests, from_toml_wrong_type_nu) {
  auto tbl = create_specs().to_toml();
  tbl->get("backbone_specs")->as_table()->insert("nu", "name");
  ASSERT_THROW(BackboneSpecs::from_toml(tbl), cpptoml::parse_exception);
}

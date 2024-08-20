/**
 * author:        Michael Bentley
 * email:         mikebentley15@gmail.com
 * date-created:  21 March 2020
 */

#include "tendon/TendonSpecs.h"
#include <cpptoml/cpptoml.h>
#include <cpptoml/toml_conversions.h>

#include <gtest/gtest.h>

using tendon::TendonSpecs;

namespace {

TendonSpecs create_specs() {
  TendonSpecs specs;
  specs.C = Eigen::VectorXd(3);
  specs.C << 1, 3, 5;
  specs.D = Eigen::VectorXd(5);
  specs.D << 1, 2, 3, 4, 5;
  specs.max_tension = 1.234;
  specs.min_length = -0.123;
  specs.max_length =  2.345;
  return specs;
}

} // end of unnamed namespace

TEST(TendonSpecsTests, r_degree_empty) {
  TendonSpecs specs;
  ASSERT_EQ(specs.r_degree(), size_t(0));
}

TEST(TendonSpecsTests, r_degree_constant) {
  TendonSpecs specs;
  specs.D = Eigen::VectorXd(1);
  specs.D << 2.3;
  ASSERT_EQ(specs.r_degree(), size_t(0));
}

TEST(TendonSpecsTests, r_degree_linear) {
  TendonSpecs specs;
  specs.D = Eigen::VectorXd(2);
  specs.D << 2.3, 1.2;
  ASSERT_EQ(specs.r_degree(), size_t(1));
}

TEST(TendonSpecsTests, r_degree_many) {
  TendonSpecs specs;
  specs.D = Eigen::VectorXd(6);
  specs.D << 2.3, 1.2, 1.3, 3.5, 6, 7;
  ASSERT_EQ(specs.r_degree(), size_t(5));
}

TEST(TendonSpecsTests, r_degree_leading_zeros) {
  TendonSpecs specs;
  specs.D = Eigen::VectorXd(6);
  specs.D << 2.3, 1.2, 1.3, 0, 0, 0;
  ASSERT_EQ(specs.r_degree(), size_t(2));
}

TEST(TendonSpecsTests, theta_degree_empty) {
  TendonSpecs specs;
  ASSERT_EQ(specs.theta_degree(), size_t(0));
}

TEST(TendonSpecsTests, theta_degree_constant) {
  TendonSpecs specs;
  specs.C = Eigen::VectorXd(1);
  specs.C << 1.5;
  ASSERT_EQ(specs.theta_degree(), size_t(0));
}

TEST(TendonSpecsTests, theta_degree_linear) {
  TendonSpecs specs;
  specs.C = Eigen::VectorXd(2);
  specs.C << 1.5, 0.1;
  ASSERT_EQ(specs.theta_degree(), size_t(1));
}

TEST(TendonSpecsTests, theta_degree_many) {
  TendonSpecs specs;
  specs.C = Eigen::VectorXd(8);
  specs.C << 1.5, 0.1, 3, 4, 5, 6, 7, 8;
  ASSERT_EQ(specs.theta_degree(), size_t(7));
}

TEST(TendonSpecsTests, theta_degree_leading_zeros) {
  TendonSpecs specs;
  specs.C = Eigen::VectorXd(8);
  specs.C << 1.5, 0, 3, 0, 0, 0, 0, 0;
  ASSERT_EQ(specs.theta_degree(), size_t(2));
}

TEST(TendonSpecsTests, is_straight_empty) {
  TendonSpecs specs;
  ASSERT_TRUE(specs.is_straight());
}

TEST(TendonSpecsTests, is_straight_yes) {
  TendonSpecs specs;
  specs.C = Eigen::VectorXd(1);
  specs.C << 5.4;
  specs.D = Eigen::VectorXd(1);
  specs.D << 1.2;
  ASSERT_TRUE(specs.is_straight());
}

TEST(TendonSpecsTests, is_straight_helix) {
  TendonSpecs specs;
  specs.C = Eigen::VectorXd(2);
  specs.C << 5.4, 2.1;
  specs.D = Eigen::VectorXd(1);
  specs.D << 1.2;
  ASSERT_FALSE(specs.is_straight());
}

TEST(TendonSpecsTests, is_straight_multi) {
  auto specs = create_specs();
  ASSERT_FALSE(specs.is_straight());
}

TEST(TendonSpecsTests, is_straight_yes_leading_zeros) {
  TendonSpecs specs;
  specs.C = Eigen::VectorXd(5);
  specs.C << 5.4, 0, 0, 0, 0;
  specs.D = Eigen::VectorXd(3);
  specs.D << 1.2, 0, 0;
  ASSERT_TRUE(specs.is_straight());
}

TEST(TendonSpecsTests, is_helix_empty) {
  TendonSpecs specs;
  ASSERT_FALSE(specs.is_helix());
}

TEST(TendonSpecsTests, is_helix_linear) {
  TendonSpecs specs;
  specs.C = Eigen::VectorXd(1);
  specs.C << 5.4;
  specs.D = Eigen::VectorXd(1);
  specs.D << 1.2;
  ASSERT_FALSE(specs.is_helix());
}

TEST(TendonSpecsTests, is_helix_helix) {
  TendonSpecs specs;
  specs.C = Eigen::VectorXd(2);
  specs.C << 5.4, 2.1;
  specs.D = Eigen::VectorXd(1);
  specs.D << 1.2;
  ASSERT_TRUE(specs.is_helix());
}

TEST(TendonSpecsTests, is_helix_multi) {
  auto specs = create_specs();
  ASSERT_FALSE(specs.is_helix());
}

TEST(TendonSpecsTests, is_helix_yes_leading_zeros) {
  TendonSpecs specs;
  specs.C = Eigen::VectorXd(5);
  specs.C << 5.4, 2.1, 0, 0, 0;
  specs.D = Eigen::VectorXd(3);
  specs.D << 1.2, 0, 0;
  ASSERT_TRUE(specs.is_helix());
}

TEST(TendonSpecsTests, to_toml_default) {
  TendonSpecs specs;
  auto actual = TendonSpecs::from_toml(TendonSpecs().to_toml());
  ASSERT_EQ(specs.C          , actual.C          );
  ASSERT_EQ(specs.D          , actual.D          );
  ASSERT_EQ(specs.max_tension, actual.max_tension);
  ASSERT_EQ(specs.min_length , actual.min_length );
  ASSERT_EQ(specs.max_length , actual.max_length );
}

TEST(TendonSpecsTests, to_toml) {
  auto specs = create_specs();
  auto actual = TendonSpecs::from_toml(specs.to_toml());
  ASSERT_EQ(specs.C          , actual.C          );
  ASSERT_EQ(specs.D          , actual.D          );
  ASSERT_EQ(specs.max_tension, actual.max_tension);
  ASSERT_EQ(specs.min_length , actual.min_length );
  ASSERT_EQ(specs.max_length , actual.max_length );
}

TEST(TendonSpecsTests, to_toml_default_through_string) {
  TendonSpecs specs;
  auto str = cpptoml::to_string(specs.to_toml());
  std::istringstream in(str);
  cpptoml::parser toml_parser(in);
  auto actual = TendonSpecs::from_toml(toml_parser.parse());
  ASSERT_EQ(specs.C          , actual.C          );
  ASSERT_EQ(specs.D          , actual.D          );
  ASSERT_EQ(specs.max_tension, actual.max_tension);
  ASSERT_EQ(specs.min_length , actual.min_length );
  ASSERT_EQ(specs.max_length , actual.max_length );
}

TEST(TendonSpecsTests, to_toml_through_string) {
  auto specs = create_specs();
  auto str = cpptoml::to_string(specs.to_toml());
  std::istringstream in(str);
  cpptoml::parser toml_parser(in);
  auto actual = TendonSpecs::from_toml(toml_parser.parse());
  ASSERT_EQ(specs.C          , actual.C          );
  ASSERT_EQ(specs.D          , actual.D          );
  ASSERT_EQ(specs.max_tension, actual.max_tension);
  ASSERT_EQ(specs.min_length , actual.min_length );
  ASSERT_EQ(specs.max_length , actual.max_length );
}

TEST(TendonSpecsTests, from_toml_nullptr) {
  ASSERT_THROW(TendonSpecs::from_toml(nullptr), std::invalid_argument);
}

TEST(TendonSpecsTests, from_toml_empty) {
  auto tbl = cpptoml::make_table();
  ASSERT_THROW(TendonSpecs::from_toml(tbl), std::out_of_range);
}

TEST(TendonSpecsTests, from_toml_missing_C) {
  auto tbl = create_specs().to_toml();
  tbl->erase("C");
  ASSERT_THROW(TendonSpecs::from_toml(tbl), std::out_of_range);
}

TEST(TendonSpecsTests, from_toml_missing_D) {
  auto tbl = create_specs().to_toml();
  tbl->erase("D");
  ASSERT_THROW(TendonSpecs::from_toml(tbl), std::out_of_range);
}

TEST(TendonSpecsTests, from_toml_missing_max_tension) {
  auto tbl = create_specs().to_toml();
  tbl->erase("max_tension");
  ASSERT_THROW(TendonSpecs::from_toml(tbl), std::out_of_range);
}

TEST(TendonSpecsTests, from_toml_missing_min_length) {
  auto tbl = create_specs().to_toml();
  tbl->erase("min_length");
  ASSERT_THROW(TendonSpecs::from_toml(tbl), std::out_of_range);
}

TEST(TendonSpecsTests, from_toml_missing_max_length) {
  auto tbl = create_specs().to_toml();
  tbl->erase("max_length");
  ASSERT_THROW(TendonSpecs::from_toml(tbl), std::out_of_range);
}

TEST(TendonSpecsTests, from_toml_wrong_type_C) {
  auto tbl = create_specs().to_toml();
  tbl->insert("C", 5);
  ASSERT_THROW(TendonSpecs::from_toml(tbl), cpptoml::parse_exception);
}

TEST(TendonSpecsTests, from_toml_wrong_type_D) {
  auto tbl = create_specs().to_toml();
  tbl->insert("D", 5);
  ASSERT_THROW(TendonSpecs::from_toml(tbl), cpptoml::parse_exception);
}

TEST(TendonSpecsTests, from_toml_wrong_type_max_tension) {
  auto tbl = create_specs().to_toml();
  tbl->insert("max_tension", "name");
  ASSERT_THROW(TendonSpecs::from_toml(tbl), cpptoml::parse_exception);
}

TEST(TendonSpecsTests, from_toml_wrong_type_min_length) {
  auto tbl = create_specs().to_toml();
  tbl->insert("min_length", "name");
  ASSERT_THROW(TendonSpecs::from_toml(tbl), cpptoml::parse_exception);
}

TEST(TendonSpecsTests, from_toml_wrong_type_max_length) {
  auto tbl = create_specs().to_toml();
  tbl->insert("max_length", "name");
  ASSERT_THROW(TendonSpecs::from_toml(tbl), cpptoml::parse_exception);
}

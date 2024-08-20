/**
 * author:        Michael Bentley
 * email:         mikebentley15@gmail.com
 * date-created:  21 March 2020
 */

#include "tendon/TendonResult.h"
#include <cpptoml/cpptoml.h>
#include <cpptoml/toml_conversions.h>
#include <collision/Point.h>

#include <gtest/gtest.h>

using tendon::TendonResult;
using collision::operator<<;

namespace {

TendonResult create_results() {
  TendonResult result;
  result.t = {1.1, 2.2, 3.3, 4.4, 5.5};
  result.p = {{1,2,3}, {3,4,5}, {6,7,8}};
  Eigen::Matrix3d mat;
  mat << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;
  result.R.push_back(mat);
  mat << 2, 3, 4,
         5, 6, 7,
         8, 9, 0;
  result.R.push_back(mat);
  mat << 3, 4, 5,
         6, 7, 8,
         9, 0, 1;
  result.R.push_back(mat);
  result.L = 3.145;
  result.L_i = {2.1, 3.1, 4.1, 5.1};
  result.u_i = {1.0, 2.0, 3.0};
  result.v_i = {-1.0, -2.0, -3.0};
  result.u_f = {0.0, 0.2, 0.3};
  result.v_f = {-1.1,  0.0, 1.0};
  return result;
}

void assert_equal(const TendonResult &expected, const TendonResult &actual) {
  ASSERT_EQ(expected.t  , actual.t  );
  ASSERT_EQ(expected.p  , actual.p  );
  ASSERT_EQ(expected.R  , actual.R  );
  ASSERT_EQ(expected.L  , actual.L  );
  ASSERT_EQ(expected.L_i, actual.L_i);
  ASSERT_EQ(expected.u_i, actual.u_i);
  ASSERT_EQ(expected.v_i, actual.v_i);
  ASSERT_EQ(expected.u_f, actual.u_f);
  ASSERT_EQ(expected.v_f, actual.v_f);
}

} // end of unnamed namespace

TEST(TendonResultTests, rotate_z_zero) {
  auto result = create_results();
  auto actual = create_results();
  result.rotate_z(0);
  assert_equal(result, actual);
}

TEST(TendonResultTests, rotate_z_90_degrees) {
  auto result = create_results();
  Eigen::Matrix3d rot;
  rot << 0, -1, 0,
         1,  0, 0,
         0,  0, 1;
  for (auto &p : result.p) { p = rot * p; }
  for (auto &R : result.R) { R = rot * R; }

  auto actual = create_results();
  actual.rotate_z(M_PI / 2);

  ASSERT_EQ(result.t  , actual.t  );
  ASSERT_EQ(result.L  , actual.L  );
  ASSERT_EQ(result.L_i, actual.L_i);
  ASSERT_EQ(result.u_i, actual.u_i);
  ASSERT_EQ(result.v_i, actual.v_i);
  ASSERT_EQ(result.u_f, actual.u_f);
  ASSERT_EQ(result.v_f, actual.v_f);

  size_t N = result.p.size();
  ASSERT_EQ(N, result.p.size());
  ASSERT_EQ(N, result.R.size());
  ASSERT_EQ(N, actual.p.size());
  ASSERT_EQ(N, actual.R.size());
  for (size_t i = 0; i < N; ++i) {
    auto &rp = result.p[i];
    auto &ap = actual.p[i];
    auto &rR = result.R[i];
    auto &aR = actual.R[i];

    ASSERT_DOUBLE_EQ(rp[0], ap[0]);
    ASSERT_DOUBLE_EQ(rp[1], ap[1]);
    ASSERT_DOUBLE_EQ(rp[2], ap[2]);

    ASSERT_DOUBLE_EQ(rR(0, 0), aR(0, 0));
    ASSERT_DOUBLE_EQ(rR(0, 1), aR(0, 1));
    ASSERT_DOUBLE_EQ(rR(0, 2), aR(0, 2));
    ASSERT_DOUBLE_EQ(rR(1, 0), aR(1, 0));
    ASSERT_DOUBLE_EQ(rR(1, 1), aR(1, 1));
    ASSERT_DOUBLE_EQ(rR(1, 2), aR(1, 2));
    ASSERT_DOUBLE_EQ(rR(2, 0), aR(2, 0));
    ASSERT_DOUBLE_EQ(rR(2, 1), aR(2, 1));
    ASSERT_DOUBLE_EQ(rR(2, 2), aR(2, 2));
  }
}

TEST(TendonResultTests, to_toml_default) {
  TendonResult result;
  auto actual = TendonResult::from_toml(TendonResult().to_toml());
  assert_equal(result, actual);
}

TEST(TendonResultTests, to_toml) {
  auto result = create_results();
  auto actual = TendonResult::from_toml(result.to_toml());
  assert_equal(result, actual);
}

TEST(TendonResultTests, to_toml_default_through_string) {
  TendonResult result;
  auto str = cpptoml::to_string(result.to_toml());
  std::istringstream in(str);
  cpptoml::parser toml_parser(in);
  auto actual = TendonResult::from_toml(toml_parser.parse());
  assert_equal(result, actual);
}

TEST(TendonResultTests, to_toml_through_string) {
  auto result = create_results();
  auto str = cpptoml::to_string(result.to_toml());
  std::istringstream in(str);
  cpptoml::parser toml_parser(in);
  auto actual = TendonResult::from_toml(toml_parser.parse());
  assert_equal(result, actual);
}

TEST(TendonResultTests, to_toml_no_time_skips_table_array) {
  auto result = create_results();
  result.t.clear();
  auto tbl = result.to_toml();
  ASSERT_FALSE(tbl->contains("t"));
}

TEST(TendonResultTests, to_toml_no_points_skips_table_array) {
  auto result = create_results();
  result.p.clear();
  auto tbl = result.to_toml();
  ASSERT_FALSE(tbl->contains("p"));
}

TEST(TendonResultTests, to_toml_no_rotations_skips_table_array) {
  auto result = create_results();
  result.R.clear();
  auto tbl = result.to_toml();
  ASSERT_FALSE(tbl->contains("R"));
}

TEST(TendonResultTests, to_toml_no_lengths_skips_table_array) {
  auto result = create_results();
  result.L_i.clear();
  auto tbl = result.to_toml();
  ASSERT_FALSE(tbl->contains("L_i"));
}

TEST(TendonResultTests, from_toml_nullptr) {
  ASSERT_THROW(TendonResult::from_toml(nullptr), std::invalid_argument);
}

TEST(TendonResultTests, from_toml_empty) {
  auto tbl = cpptoml::make_table();
  ASSERT_THROW(TendonResult::from_toml(tbl), std::out_of_range);
}

TEST(TendonResultTests, from_toml_in_container) {
  auto result = create_results();
  auto tbl = result.to_toml();
  ASSERT_TRUE(tbl->contains("tendon_result"));
  auto actual = TendonResult::from_toml(tbl);
  assert_equal(result, actual);
}

TEST(TendonResultTests, from_toml_missing_t) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->erase("t");
  auto result = TendonResult::from_toml(tbl);
  ASSERT_TRUE(result.t.empty());
}

TEST(TendonResultTests, from_toml_missing_p) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->erase("p");
  auto result = TendonResult::from_toml(tbl);
  ASSERT_TRUE(result.p.empty());
}

TEST(TendonResultTests, from_toml_missing_R) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->erase("R");
  auto result = TendonResult::from_toml(tbl);
  ASSERT_TRUE(result.R.empty());
}

TEST(TendonResultTests, from_toml_missing_L) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->erase("L");
  ASSERT_THROW(TendonResult::from_toml(tbl), std::out_of_range);
}

TEST(TendonResultTests, from_toml_missing_L_i) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->erase("L_i");
  auto result = TendonResult::from_toml(tbl);
  ASSERT_TRUE(result.L_i.empty());
}

TEST(TendonResultTests, from_toml_missing_u_i) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->erase("u_i");
  auto result = TendonResult::from_toml(tbl);
  TendonResult defaults;
  ASSERT_EQ(result.u_i, defaults.u_i);
}

TEST(TendonResultTests, from_toml_missing_v_i) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->erase("v_i");
  auto result = TendonResult::from_toml(tbl);
  TendonResult defaults;
  ASSERT_EQ(result.v_i, defaults.v_i);
}

TEST(TendonResultTests, from_toml_missing_u_f) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->erase("u_f");
  auto result = TendonResult::from_toml(tbl);
  TendonResult defaults;
  ASSERT_EQ(result.u_f, defaults.u_f);
}

TEST(TendonResultTests, from_toml_missing_v_f) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->erase("v_f");
  auto result = TendonResult::from_toml(tbl);
  TendonResult defaults {};
  defaults.v_f = {};
  ASSERT_EQ(result.v_f, defaults.v_f)
    << result.v_f[0] << " != " << defaults.v_f[0] << ", "
    << result.v_f[1] << " != " << defaults.v_f[1] << ", "
    << result.v_f[2] << " != " << defaults.v_f[2] << ", ";
}

TEST(TendonResultTests, from_toml_wrong_type_t) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->insert("t", 5);
  ASSERT_THROW(TendonResult::from_toml(tbl), cpptoml::parse_exception);
}

TEST(TendonResultTests, from_toml_wrong_type_p) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->insert("p", "name");
  ASSERT_THROW(TendonResult::from_toml(tbl), cpptoml::parse_exception);
}

TEST(TendonResultTests, from_toml_wrong_type_R) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->insert("R", 25);
  ASSERT_THROW(TendonResult::from_toml(tbl), cpptoml::parse_exception);
}

TEST(TendonResultTests, from_toml_wrong_type_L) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->insert("L", "name");
  ASSERT_THROW(TendonResult::from_toml(tbl), cpptoml::parse_exception);
}

TEST(TendonResultTests, from_toml_wrong_type_L_i) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->insert("L_i", 5);
  ASSERT_THROW(TendonResult::from_toml(tbl), cpptoml::parse_exception);
}

TEST(TendonResultTests, from_toml_wrong_type_u_i) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->insert("u_i", 5);
  ASSERT_THROW(TendonResult::from_toml(tbl), cpptoml::parse_exception);
}

TEST(TendonResultTests, from_toml_wrong_type_v_i) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->insert("v_i", 5);
  ASSERT_THROW(TendonResult::from_toml(tbl), cpptoml::parse_exception);
}

TEST(TendonResultTests, from_toml_wrong_type_u_f) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->insert("u_f", 5);
  ASSERT_THROW(TendonResult::from_toml(tbl), cpptoml::parse_exception);
}

TEST(TendonResultTests, from_toml_wrong_type_v_f) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->insert("v_f", 5);
  ASSERT_THROW(TendonResult::from_toml(tbl), cpptoml::parse_exception);
}

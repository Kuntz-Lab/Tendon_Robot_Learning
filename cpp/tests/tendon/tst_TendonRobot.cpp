/**
 * author:        Michael Bentley
 * email:         mikebentley15@gmail.com
 * date-created:  19 March 2020
 */

#include "tendon/TendonRobot.h"
#include <tendon/TendonSpecs.h>
#include <cpptoml/cpptoml.h>
#include <cpptoml/toml_conversions.h>
#include <collision/Point.h>
#include <vistendon/marker_array_conversions.h>

#include <gtest/gtest.h>

#ifndef UNUSED
#define UNUSED(x) (void)x
#endif // UNUSED(x)

using tendon::TendonRobot;
using collision::operator<<;

namespace {

//// for better printing in assertion messages
//std::ostream& operator<<(std::ostream &out, const Eigen::VectorXd &vec) {
//  out << "[";
//  bool first = true;
//  for (decltype(vec.size()) i = 0; i < vec.size(); i++) {
//    if (!first) { out << ", "; }
//    out << vec[i];
//    first = false;
//  }
//  out << "]";
//  return out;
//}

TendonRobot create_robot() {
  TendonRobot robot;
  robot.r = 1.234;
  robot.specs.L = 2.345;
  robot.specs.dL = 0.001;
  robot.specs.ro = 0.02;
  robot.specs.ri = 0.001;
  robot.specs.E = 3.2e7;
  robot.specs.nu = 0.43;

  using Vx = Eigen::VectorXd;
  using TS = tendon::TendonSpecs;
  robot.tendons.push_back(TS{(Vx(4) << 2, 0, 0, 0).finished(), (Vx(2) << 3, 0).finished()}); // linear
  robot.tendons.push_back(TS{(Vx(4) << 2, 3, 0, 0).finished(), (Vx(2) << 1, 0).finished()}); // helical
  robot.tendons.push_back(TS{(Vx(4) << 1, 2, 3, 0).finished(), (Vx(2) << 1, 0).finished()});
  robot.tendons.push_back(TS{(Vx(4) << 2, 3, 0, 0).finished(), (Vx(2) << 1, 2).finished()});
  robot.tendons.push_back(TS{(Vx(4) << 4, 5, 6, 7).finished(), (Vx(2) << 3, 0).finished()});

  robot.enable_rotation   = true;
  robot.enable_retraction = true;

  return robot;
}

/// Get tendon shape and sum it up to get approximate length
std::vector<double> tendon_lengths_from_shapes(const TendonRobot &robot,
                                               const std::vector<double> &state)
{
  auto [robot_shape, tendon_shapes] =
      vistendon::robot_and_tendon_shape(robot, state);
  UNUSED(robot_shape);
  std::vector<double> tendon_shape_lengths;
  for (auto &shape : tendon_shapes) {
    double length = 0.0;
    for (size_t i = 0; i < shape.size(); i++) {
      const auto &piece = shape[i];
      length += (piece.a - piece.b).norm();
    }
    tendon_shape_lengths.emplace_back(length);
  }
  return tendon_shape_lengths;
}

} // end of unnamed namespace

TEST(TendonRobotTests, to_toml_default) {
  TendonRobot robot;
  auto actual = TendonRobot::from_toml(TendonRobot().to_toml());
  ASSERT_EQ(robot.r                , actual.r                );
  ASSERT_EQ(robot.specs            , actual.specs            );
  ASSERT_EQ(robot.tendons          , actual.tendons          );
  ASSERT_EQ(robot.enable_rotation  , actual.enable_rotation  );
  ASSERT_EQ(robot.enable_retraction, actual.enable_retraction);
}

TEST(TendonRobotTests, to_toml) {
  auto robot = create_robot();
  auto actual = TendonRobot::from_toml(robot.to_toml());
  ASSERT_EQ(robot.r                , actual.r                );
  ASSERT_EQ(robot.specs            , actual.specs            );
  ASSERT_EQ(robot.tendons          , actual.tendons          );
  ASSERT_EQ(robot.enable_rotation  , actual.enable_rotation  );
  ASSERT_EQ(robot.enable_retraction, actual.enable_retraction);
}

TEST(TendonRobotTests, to_toml_default_through_string) {
  TendonRobot robot;
  auto str = cpptoml::to_string(robot.to_toml());
  std::istringstream in(str);
  cpptoml::parser toml_parser(in);
  auto actual = TendonRobot::from_toml(toml_parser.parse());
  ASSERT_EQ(robot.r                , actual.r                );
  ASSERT_EQ(robot.specs            , actual.specs            );
  ASSERT_EQ(robot.tendons          , actual.tendons          );
  ASSERT_EQ(robot.enable_rotation  , actual.enable_rotation  );
  ASSERT_EQ(robot.enable_retraction, actual.enable_retraction);
}

TEST(TendonRobotTests, to_toml_through_string) {
  auto robot = create_robot();
  auto str = cpptoml::to_string(robot.to_toml());
  std::istringstream in(str);
  cpptoml::parser toml_parser(in);
  auto actual = TendonRobot::from_toml(toml_parser.parse());
  ASSERT_EQ(robot.r                , actual.r                );
  ASSERT_EQ(robot.specs            , actual.specs            );
  ASSERT_EQ(robot.tendons          , actual.tendons          );
  ASSERT_EQ(robot.enable_rotation  , actual.enable_rotation  );
  ASSERT_EQ(robot.enable_retraction, actual.enable_retraction);
}

TEST(TendonRobotTests, to_toml_no_tendons_skips_table_array) {
  auto robot = create_robot();
  robot.tendons.clear();
  auto tbl = robot.to_toml();
  ASSERT_FALSE(tbl->contains("tendons"));
}

TEST(TendonRobotTests, from_toml_nullptr) {
  ASSERT_THROW(TendonRobot::from_toml(nullptr), std::invalid_argument);
}

TEST(TendonRobotTests, from_toml_empty) {
  auto tbl = cpptoml::make_table();
  ASSERT_THROW(TendonRobot::from_toml(tbl), std::out_of_range);
}

TEST(TendonRobotTests, from_toml_in_container) {
  auto robot = create_robot();
  auto tbl = robot.to_toml();
  ASSERT_TRUE(tbl->contains("tendon_robot"));
  auto actual = TendonRobot::from_toml(tbl);
  ASSERT_EQ(robot.r                , actual.r                );
  ASSERT_EQ(robot.specs            , actual.specs            );
  ASSERT_EQ(robot.tendons          , actual.tendons          );
  ASSERT_EQ(robot.enable_rotation  , actual.enable_rotation  );
  ASSERT_EQ(robot.enable_retraction, actual.enable_retraction);
}

TEST(TendonRobotTests, from_toml_missing_r) {
  auto tbl = create_robot().to_toml();
  tbl->get("tendon_robot")->as_table()->erase("radius");
  ASSERT_THROW(TendonRobot::from_toml(tbl), std::out_of_range);
}

TEST(TendonRobotTests, from_toml_missing_specs) {
  auto tbl = create_robot().to_toml();
  tbl->erase("backbone_specs");
  ASSERT_THROW(TendonRobot::from_toml(tbl), std::out_of_range);
}

TEST(TendonRobotTests, from_toml_missing_tendons) {
  auto tbl = create_robot().to_toml();
  tbl->erase("tendons");
  auto robot = tendon::TendonRobot::from_toml(tbl);
  ASSERT_TRUE(robot.tendons.empty());
}

TEST(TendonRobotTests, from_toml_missing_enable_rotation) {
  auto robot = create_robot();
  auto tbl = robot.to_toml();
  tbl->get("tendon_robot")->as_table()->erase("enable_rotation");
  auto actual = TendonRobot::from_toml(tbl);
  ASSERT_EQ(robot.r                , actual.r                );
  ASSERT_EQ(robot.specs            , actual.specs            );
  ASSERT_EQ(robot.tendons          , actual.tendons          );
  ASSERT_EQ(false                  , actual.enable_rotation  );
  ASSERT_EQ(robot.enable_retraction, actual.enable_retraction);
}

TEST(TendonRobotTests, from_toml_missing_enable_retraction) {
  auto robot = create_robot();
  auto tbl = robot.to_toml();
  tbl->get("tendon_robot")->as_table()->erase("enable_retraction");
  auto actual = TendonRobot::from_toml(tbl);
  ASSERT_EQ(robot.r                , actual.r                );
  ASSERT_EQ(robot.specs            , actual.specs            );
  ASSERT_EQ(robot.tendons          , actual.tendons          );
  ASSERT_EQ(robot.enable_rotation  , actual.enable_rotation  );
  ASSERT_EQ(false                  , actual.enable_retraction);
}

TEST(TendonRobotTests, from_toml_wrong_type_r) {
  auto tbl = create_robot().to_toml();
  tbl->get("tendon_robot")->as_table()->insert("radius", "name");
  ASSERT_THROW(TendonRobot::from_toml(tbl), cpptoml::parse_exception);
}

TEST(TendonRobotTests, from_toml_wrong_type_specs) {
  auto tbl = create_robot().to_toml();
  tbl->insert("backbone_specs", 5);
  ASSERT_THROW(TendonRobot::from_toml(tbl), cpptoml::parse_exception);
}

TEST(TendonRobotTests, from_toml_wrong_type_tendons) {
  auto tbl = create_robot().to_toml();
  tbl->insert("tendons", 5);
  ASSERT_THROW(TendonRobot::from_toml(tbl), cpptoml::parse_exception);
}

TEST(TendonRobotTests, from_toml_wrong_type_enable_rotation) {
  auto tbl = create_robot().to_toml();
  tbl->get("tendon_robot")->as_table()->insert("enable_rotation", "name");
  ASSERT_THROW(TendonRobot::from_toml(tbl), cpptoml::parse_exception);
}

TEST(TendonRobotTests, from_toml_wrong_type_enable_retraction) {
  auto tbl = create_robot().to_toml();
  tbl->get("tendon_robot")->as_table()->insert("enable_retraction", "name");
  ASSERT_THROW(TendonRobot::from_toml(tbl), cpptoml::parse_exception);
}

TEST(TendonRobotTests, shape_near_piecewise_linear_length) {
  auto robot = create_robot();
  std::vector<double> zero_state(robot.state_size(), 0.0);
  auto shape = robot.shape(zero_state);
  auto tendon_shape_lengths = tendon_lengths_from_shapes(robot, zero_state);
  auto eps = 1.0;
  ASSERT_EQ(shape.L_i.size(), tendon_shape_lengths.size());
  for (size_t i = 0; i < shape.L_i.size(); i++) {
    ASSERT_NEAR(shape.L_i[i], tendon_shape_lengths[i], eps)
      << "  for i = " << i;
  }
}

TEST(TendonRobotTests, home_shape_near_piecewise_linear_length) {
  auto robot = create_robot();
  auto home_shape = robot.home_shape();
  auto eps = 1.0;
  std::vector<double> zero_state(robot.state_size(), 0.0);
  auto tendon_shape_lengths = tendon_lengths_from_shapes(robot, zero_state);
  ASSERT_EQ(home_shape.L_i.size(), tendon_shape_lengths.size());
  for (size_t i = 0; i < home_shape.L_i.size(); i++) {
    ASSERT_NEAR(home_shape.L_i[i], tendon_shape_lengths[i], eps)
      << "  for i = " << i;
  }
}

TEST(TendonRobotTests, home_shape_same_as_zero_shape) {
  auto robot = create_robot();
  std::vector<double> zero_state(robot.state_size(), 0.0);
  auto shape = robot.shape(zero_state);
  auto home_shape = robot.home_shape();

  ASSERT_EQ(shape.p.size(), home_shape.p.size());
  for (size_t i = 0; i < shape.p.size(); i++) {
    ASSERT_NEAR((shape.p[i] - home_shape.p[i]).norm(), 0.0, 1e-10)
      << "  p[" << i << "] = [" << shape.p[i].transpose() << "]"
      << ", home.p[" << i << "] = [" << home_shape.p[i].transpose() << "]";
  }
  ASSERT_EQ(shape.t       , home_shape.t       );
  ASSERT_EQ(shape.R       , home_shape.R       );
  ASSERT_NEAR(shape.L, home_shape.L, 0.001);
  ASSERT_EQ(shape.L_i.size(), home_shape.L_i.size());
  for (size_t i = 0; i < shape.L_i.size(); i++) {
    ASSERT_NEAR(shape.L_i[i], home_shape.L_i[i], 0.5) << "  for i = " << i;
  }
}

TEST(TendonRobotTests, shape_near_piecewise_linear_length_start_middle) {
  auto robot = create_robot();
  auto s_start = robot.specs.L / 2;
  std::vector<double> zero_state(robot.state_size(), 0.0);
  zero_state.back() = s_start;
  auto shape = robot.shape(zero_state);
  auto tendon_shape_lengths = tendon_lengths_from_shapes(robot, zero_state);
  auto eps = 1.0;
  ASSERT_EQ(shape.L_i.size(), tendon_shape_lengths.size());
  for (size_t i = 0; i < shape.L_i.size(); i++) {
    ASSERT_NEAR(shape.L_i[i], tendon_shape_lengths[i], eps)
      << "  for i = " << i;
  }
}

TEST(TendonRobotTests, home_shape_near_piecewise_linear_length_start_middle) {
  auto robot = create_robot();
  auto s_start = 0.4 * robot.specs.L;
  auto home_shape = robot.home_shape(s_start);
  auto eps = 1.0;
  std::vector<double> zero_state(robot.state_size(), 0.0);
  zero_state.back() = s_start;
  auto tendon_shape_lengths = tendon_lengths_from_shapes(robot, zero_state);
  ASSERT_EQ(home_shape.L_i.size(), tendon_shape_lengths.size());
  for (size_t i = 0; i < home_shape.L_i.size(); i++) {
    ASSERT_NEAR(home_shape.L_i[i], tendon_shape_lengths[i], eps)
      << "  for i = " << i;
  }
  ASSERT_DOUBLE_EQ(home_shape.L, robot.specs.L - s_start);
}

TEST(TendonRobotTests, DISABLED_shape_with_nan_s_start) {
  // Disabled: when compiling with -ffast-math, nan checking doesn't work
  auto robot = create_robot();
  auto s_start = std::numeric_limits<double>::quiet_NaN();
  std::vector<double> state(robot.state_size(), 0.0);
  state.back() = s_start;
  ASSERT_THROW(robot.shape(state), std::invalid_argument);
}

TEST(TendonRobotTests, DISABLED_home_shape_with_nan_s_start) {
  // Disabled: when compiling with -ffast-math, nan checking doesn't work
  auto robot = create_robot();
  auto s_start = std::numeric_limits<double>::quiet_NaN();
  std::vector<double> state(robot.state_size(), 0.0);
  state.back() = s_start;
  ASSERT_THROW(robot.home_shape(state), std::invalid_argument);
  ASSERT_THROW(robot.home_shape(s_start), std::invalid_argument);
}

TEST(TendonRobotTests, home_shape_same_as_zero_shape_start_middle) {
  auto robot = create_robot();
  auto s_start = 0.4 * robot.specs.L;
  std::vector<double> zero_state(robot.state_size(), 0.0);
  zero_state.back() = s_start;
  auto shape = robot.shape(zero_state);
  auto home_shape = robot.home_shape(zero_state);

  ASSERT_EQ(shape.p.size(), home_shape.p.size());
  for (size_t i = 0; i < shape.p.size(); i++) {
    ASSERT_NEAR((shape.p[i] - home_shape.p[i]).norm(), 0.0, 1e-10)
      << "  p[" << i << "] = [" << shape.p[i].transpose() << "]"
      << ", home.p[" << i << "] = [" << home_shape.p[i].transpose() << "]";
  }
  ASSERT_EQ(shape.t       , home_shape.t       );
  ASSERT_EQ(shape.R       , home_shape.R       );
  ASSERT_NEAR(shape.L, home_shape.L, 0.001);
  ASSERT_NEAR(robot.specs.L - s_start, home_shape.L, 0.001);
  ASSERT_EQ(shape.L_i.size(), home_shape.L_i.size());
  for (size_t i = 0; i < shape.L_i.size(); i++) {
    ASSERT_NEAR(shape.L_i[i], home_shape.L_i[i], 0.5) << "  for i = " << i;
  }
}

TEST(TendonRobotTests, DISABLED_shape_negative_tensions) {
  // Disabled: added ability to use negative tensions for the sake of Jacobian
  // computations with finite-differences.
  auto robot = create_robot();
  std::vector<double> state {1, 2, 3, 4, -5, M_PI/2, 0.0};
  ASSERT_THROW(robot.shape(state), std::invalid_argument);
}

TEST(TendonRobotTests, DISABLED_shape_hang) {
  // Disabled: added ability to use negative tensions for the sake of Jacobian
  // computations with finite-differences.

  // This was an example input that caused hangs in the shape() code
  // It caused a problem in the adaptive integration and essentially takes
  // forever.
  TendonRobot robot;
  robot.r = 0.015;

  robot.specs.E  = 2.1e6;
  robot.specs.L = 0.2;
  robot.specs.dL = 0.005;
  robot.specs.nu = 0.3;
  robot.specs.ri = 0.0;
  robot.specs.ro = 0.01;

  tendon::TendonSpecs tendon;
  tendon.max_tension = 20.0;
  tendon.C = Eigen::VectorXd::Zero(3);
  tendon.D = Eigen::VectorXd::Zero(3);

  tendon.C << 1.25323, -1.12617, 0.983026;
  tendon.D << 0.00830747, 0.00850971, 0.00195016;
  robot.tendons.emplace_back(tendon);

  tendon.C << -1.10566, 0.705502, -0.90896;
  tendon.D << 0.00491379, 0.00570704, 0.00589252;
  robot.tendons.emplace_back(tendon);

  tendon.C << -0.802465, 0.162978, 2.0944;
  tendon.D << 0.00809976, 0.00621551, 0.00745879;
  robot.tendons.emplace_back(tendon);

  // control with this tension results in bad commanded tensions
  //std::vector<double> state {1.0, 5.0, 6.0};
  std::vector<double> bad_tau {
      76.092797569098849,
      316.23924839197736,
      -159.83413043659209};

  ASSERT_THROW(robot.shape(bad_tau), std::invalid_argument);
}

TEST(TendonRobotTests, shape_dL_larger_than_L) {
  auto robot = create_robot();
  robot.specs.L = 1;
  robot.specs.dL = 1.2;
  robot.enable_rotation = false;
  robot.enable_retraction = false;
  std::vector<double> state {1, 2, 3, 4, 5};
  auto shape = robot.shape(state);
  ASSERT_EQ(shape.t.size(), size_t(2));
  ASSERT_DOUBLE_EQ(shape.t[0], 0.0);
  ASSERT_DOUBLE_EQ(shape.t[1], 1.0);
}

TEST(TendonRobotTests, shape_dL_larger_than_L_minus_s_start) {
  auto robot = create_robot();
  robot.specs.L = 0.95;
  robot.specs.dL = 0.5;
  robot.enable_rotation = false;
  double s_start = 0.66;
  std::vector<double> state {1, 2, 3, 4, 5, s_start};
  auto shape = robot.shape(state);
  ASSERT_EQ(shape.t.size(), size_t(2));
  ASSERT_DOUBLE_EQ(shape.t[0], s_start);
  ASSERT_DOUBLE_EQ(shape.t[1], robot.specs.L);
}

TEST(TendonRobotTests, shape_s_start_equals_L) {
  auto robot = create_robot();
  robot.specs.L = 0.95;
  robot.specs.dL = 0.01;
  robot.enable_rotation = false;
  double s_start = robot.specs.L;
  std::vector<double> state {1, 2, 3, 4, 5, s_start};
  auto shape = robot.shape(state);
  ASSERT_EQ(shape.t.size(), size_t(1));
  ASSERT_DOUBLE_EQ(shape.t[0], s_start);
}

TEST(TendonRobotTests, DISABLED_shape_s_start_negative) {
  // Disabled: added ability to use negative tensions for the sake of Jacobian
  // computations with finite-differences.

  auto robot = create_robot();
  robot.specs.L = 0.95;
  robot.specs.dL = 0.01;
  double s_start = -0.01;
  std::vector<double> state {1, 2, 3, 4, 5, M_PI/4, s_start};
  ASSERT_THROW(robot.shape(state), std::invalid_argument);
}

TEST(TendonRobotTests, DISABLED_shape_s_start_greater_than_L) {
  // Disabled: added ability to use past-the-end s_start for the sake of Jacobian
  // computations with finite-differences.

  auto robot = create_robot();
  robot.specs.L = 0.95;
  robot.specs.dL = 0.01;
  double s_start = 1.00;
  std::vector<double> state {1, 2, 3, 4, 5, 2*M_PI, s_start};
  ASSERT_THROW(robot.shape(state), std::invalid_argument);
}

TEST(TendonRobotTests, home_shape_dL_larger_than_L) {
  auto robot = create_robot();
  robot.specs.L = 1;
  robot.specs.dL = 1.2;
  auto home_shape = robot.home_shape();
  ASSERT_EQ(home_shape.t.size(), size_t(2));
  ASSERT_DOUBLE_EQ(home_shape.t[0], 0);
  ASSERT_DOUBLE_EQ(home_shape.t[1], 1);
}

TEST(TendonRobotTests, home_shape_dL_larger_than_L_minus_s_start) {
  auto robot = create_robot();
  robot.specs.L = 0.95;
  robot.specs.dL = 0.5;
  double s_start = 0.66;
  auto home_shape = robot.home_shape(s_start);
  ASSERT_EQ(home_shape.t.size(), size_t(2));
  ASSERT_DOUBLE_EQ(home_shape.t[0], s_start);
  ASSERT_DOUBLE_EQ(home_shape.t[1], robot.specs.L);
}

TEST(TendonRobotTests, home_shape_s_start_equals_L) {
  auto robot = create_robot();
  robot.specs.L = 0.95;
  robot.specs.dL = 0.01;
  double s_start = robot.specs.L;
  auto home_shape = robot.home_shape(s_start);
  ASSERT_EQ(home_shape.t.size(), size_t(1));
  ASSERT_DOUBLE_EQ(home_shape.t[0], s_start);
}

TEST(TendonRobotTests, DISABLED_home_shape_s_start_greater_than_L) {
  // Disabled: added ability to use past-the-end s_start for the sake of Jacobian
  // computations with finite-differences.

  auto robot = create_robot();
  robot.specs.L = 0.95;
  robot.specs.dL = 0.01;
  double s_start = 1.00;
  ASSERT_THROW(robot.home_shape(s_start), std::invalid_argument);
}

TEST(TendonRobotTests, DISABLED_home_shape_s_start_negative) {
  // Disabled: added ability to use negative s_start for the sake of Jacobian
  // computations with finite-differences.

  auto robot = create_robot();
  robot.specs.L = 0.95;
  robot.specs.dL = 0.01;
  double s_start = -0.01;
  ASSERT_THROW(robot.home_shape(s_start), std::invalid_argument);
}

TEST(TendonRobotTests, calc_dl_empty) {
  TendonRobot robot;
  std::vector<double> empty{};
  ASSERT_EQ(robot.calc_dl(empty, empty), empty);
}

TEST(TendonRobotTests, calc_dl_one_element) {
  TendonRobot robot;
  std::vector<double> home{1.023};
  std::vector<double> other{1.025};
  std::vector<double> expected_dl{-0.002};
  auto actual_dl = robot.calc_dl(home, other);
  ASSERT_EQ(actual_dl.size(), expected_dl.size());
  ASSERT_DOUBLE_EQ(actual_dl[0], expected_dl[0]);
}

TEST(TendonRobotTests, calc_dl_two_elements) {
  TendonRobot robot;
  std::vector<double> home{1.023, 3.14};
  std::vector<double> other{1.025, 3.00};
  std::vector<double> expected_dl{-0.002, 0.14};
  auto actual_dl = robot.calc_dl(home, other);
  ASSERT_EQ(actual_dl.size(), expected_dl.size());
  ASSERT_DOUBLE_EQ(actual_dl[0], expected_dl[0]);
  ASSERT_DOUBLE_EQ(actual_dl[1], expected_dl[1]);
}

TEST(TendonRobotTests, calc_dl_five_elements) {
  TendonRobot robot;
  std::vector<double> home{2.0, 1.0, 0.0, -1.0, -2.0};
  std::vector<double> other{1.0, 1.0, 1.0, 1.0, 1.0};
  std::vector<double> expected_dl{1.0, 0.0, -1.0, -2.0, -3.0};
  auto actual_dl = robot.calc_dl(home, other);
  ASSERT_EQ(actual_dl.size(), expected_dl.size());
  ASSERT_DOUBLE_EQ(actual_dl[0], expected_dl[0]);
  ASSERT_DOUBLE_EQ(actual_dl[1], expected_dl[1]);
  ASSERT_DOUBLE_EQ(actual_dl[2], expected_dl[2]);
  ASSERT_DOUBLE_EQ(actual_dl[3], expected_dl[3]);
  ASSERT_DOUBLE_EQ(actual_dl[4], expected_dl[4]);
}

TEST(TendonRobotTests, calc_dl_mismatched_sizes) {
  TendonRobot robot;
  std::vector<double> home{2.0};
  std::vector<double> other{1.0, 1.0};
  ASSERT_THROW(robot.calc_dl(home, other), std::out_of_range);
  home = {2.0, 2.5};
  other = {1.0};
  ASSERT_THROW(robot.calc_dl(home, other), std::out_of_range);
}

TEST(TendonRobotTests, is_within_length_limits_two_args_zero_tendons) {
  TendonRobot robot;
  std::vector<double> empty;
  ASSERT_TRUE(robot.is_within_length_limits(empty, empty));
}

TEST(TendonRobotTests, is_within_length_limits_two_args_yes) {
  auto robot = create_robot();
  ASSERT_EQ(robot.tendons.size(), size_t(5));
  robot.tendons[0].min_length = -1.0; robot.tendons[0].max_length =  1.0;
  robot.tendons[1].min_length = -0.5; robot.tendons[1].max_length =  0.5;
  robot.tendons[2].min_length =  0.0; robot.tendons[2].max_length =  0.5;
  robot.tendons[3].min_length =  0.5; robot.tendons[3].max_length =  1.5;
  robot.tendons[4].min_length = -0.5; robot.tendons[4].max_length =  2.5;
  std::vector<double> dl {0.7, -0.33, 0.49, 0.51, -0.1};
  auto home_shape = robot.home_shape();
  std::vector<double> other_lengths {
    home_shape.L_i[0] - dl[0],
    home_shape.L_i[1] - dl[1],
    home_shape.L_i[2] - dl[2],
    home_shape.L_i[3] - dl[3],
    home_shape.L_i[4] - dl[4],
  };
  ASSERT_TRUE(robot.is_within_length_limits(home_shape.L_i, other_lengths));
}

TEST(TendonRobotTests, is_within_length_limits_two_args_no) {
  auto robot = create_robot();
  ASSERT_EQ(robot.tendons.size(), size_t(5));
  robot.tendons[0].min_length = -1.0; robot.tendons[0].max_length =  1.0;
  robot.tendons[1].min_length = -0.5; robot.tendons[1].max_length =  0.5;
  robot.tendons[2].min_length =  0.0; robot.tendons[2].max_length =  0.5;
  robot.tendons[3].min_length =  0.5; robot.tendons[3].max_length =  1.5;
  robot.tendons[4].min_length = -0.5; robot.tendons[4].max_length =  2.5;
  std::vector<double> dl {0.7, -0.33, 0.49, 0.49, -0.1};
  auto home_shape = robot.home_shape();
  std::vector<double> other_lengths {
    home_shape.L_i[0] + dl[0],
    home_shape.L_i[1] + dl[1],
    home_shape.L_i[2] + dl[2],
    home_shape.L_i[3] + dl[3], // out of range
    home_shape.L_i[4] + dl[4],
  };
  ASSERT_FALSE(robot.is_within_length_limits(home_shape.L_i, other_lengths));
}

TEST(TendonRobotTests, is_within_length_limits_two_args_arg_size_mismatch) {
  auto robot = create_robot();
  std::vector<double> home_lengths {0, 1, 2, 3, 4};
  std::vector<double> other_lengths {1, 2, 3, 4, 5, 6}; // one extra value
  ASSERT_THROW(robot.is_within_length_limits(home_lengths, other_lengths),
               std::out_of_range);
}

TEST(TendonRobotTests, is_within_length_limits_two_args_tendon_size_mismatch) {
  auto robot = create_robot();
  std::vector<double> home_lengths {0, 1, 2, 3, 4, 5}; // one extra value
  std::vector<double> other_lengths {1, 2, 3, 4, 5, 6}; // one extra value
  ASSERT_THROW(robot.is_within_length_limits(home_lengths, other_lengths),
               std::out_of_range);
}

TEST(TendonRobotTests, is_within_length_limits_one_arg_zero_tendons) {
  TendonRobot robot;
  ASSERT_TRUE(robot.is_within_length_limits({}));
}

TEST(TendonRobotTests, is_within_length_limits_one_arg_yes) {
  auto robot = create_robot();
  robot.tendons[0].min_length = -1.0; robot.tendons[0].max_length =  1.0;
  robot.tendons[1].min_length = -0.5; robot.tendons[1].max_length =  0.5;
  robot.tendons[2].min_length =  0.0; robot.tendons[2].max_length =  0.5;
  robot.tendons[3].min_length =  0.5; robot.tendons[3].max_length =  1.5;
  robot.tendons[4].min_length = -0.5; robot.tendons[4].max_length =  2.5;
  std::vector<double> dl {0.7, -0.33, 0.49, 0.51, -0.1};
  ASSERT_TRUE(robot.is_within_length_limits(dl));
}

TEST(TendonRobotTests, is_within_length_limits_one_arg_no) {
  auto robot = create_robot();
  robot.tendons[0].min_length = -1.0; robot.tendons[0].max_length =  1.0;
  robot.tendons[1].min_length = -0.5; robot.tendons[1].max_length =  0.5;
  robot.tendons[2].min_length =  0.0; robot.tendons[2].max_length =  0.5;
  robot.tendons[3].min_length =  0.5; robot.tendons[3].max_length =  1.5;
  robot.tendons[4].min_length = -0.5; robot.tendons[4].max_length =  2.5;
  std::vector<double> dl {0.7, -0.33, 0.49, 0.49, -0.1};
  ASSERT_FALSE(robot.is_within_length_limits(dl));
}

TEST(TendonRobotTests, is_within_length_limits_one_arg_size_mismatch) {
  auto robot = create_robot();
  std::vector<double> dl {0.7, -0.33, 0.49, 0.49}; // one too few
  ASSERT_THROW(robot.is_within_length_limits(dl), std::out_of_range);
}

// TODO: add tests about rotation

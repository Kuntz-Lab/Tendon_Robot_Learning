/**
 * author:        Michael Bentley
 * email:         mikebentley15@gmail.com
 * date-created:  20 March 2020
 */

#include "motion-planning/Problem.h"
#include <motion-planning/Environment.h>
#include <motion-planning/RetractionSampler.h>
#include <motion-planning/ValidityChecker.h>
#include <cpptoml/cpptoml.h>
#include <cpptoml/toml_conversions.h>
#include <tendon/TendonRobot.h>

#include <ompl/base/StateSpace.h>
#include <ompl/base/StateSpaceTypes.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <gtest/gtest.h>

using motion_planning::Problem;
using motion_planning::ValidityChecker;
using collision::Point;
using collision::Sphere;
using collision::Capsule;

namespace ob = ompl::base;

namespace {

Problem create_problem() {
  Problem problem;

  problem.robot.r = 1.234;
  problem.robot.specs.L = 2.345;
  problem.robot.specs.dL = 0.001;
  problem.robot.specs.ro = 0.02;
  problem.robot.specs.ri = 0.001;
  problem.robot.specs.E = 3.2e7;
  problem.robot.specs.nu = 0.43;

  using Vx = Eigen::VectorXd;
  tendon::TendonSpecs specs;
  specs.C = Vx(4);
  specs.D = Vx(2);
  specs.max_tension = 5.0;

  specs.C << 1, 2, 3, 0;
  specs.D << 1, 0;
  problem.robot.tendons.push_back(specs);
  specs.C << 2, 3, 0, 0;
  specs.D << 1, 2;
  problem.robot.tendons.push_back(specs);
  specs.C << 4, 5, 6, 7;
  specs.D << 3, 0;
  problem.robot.tendons.push_back(specs);

  problem.robot.enable_rotation = true;
  problem.robot.enable_retraction = true;

  problem.env.push_back(Point{1, 2, 3});
  problem.env.push_back(Point{2, 3, 4});
  problem.env.push_back(Sphere{{1, 3, 5}, 2});
  problem.env.push_back(Sphere{{2, 4, 6}, 3});
  problem.env.push_back(Sphere{{3, 5, 7}, 4});
  problem.env.push_back(Capsule{{1, 2, 3}, {3, 2, 1}, 6.6});

  problem.start = {1, 1.1, 1.2};
  problem.goal  = {2, 2.2, 2.4};

  problem.min_tension_change = 0.004;

  problem.start_rotation    = M_PI/3;
  problem.start_retraction  = 1.2;
  problem.goal_rotation     = -M_PI/2;
  problem.goal_retraction   = 0.6;

  problem.sample_like_sphere = true;

  return problem;
}

void assert_problem_eq(const Problem &expected, const Problem &actual) {
  ASSERT_EQ(expected.robot             , actual.robot             );
  ASSERT_EQ(expected.env               , actual.env               );
  ASSERT_EQ(expected.start             , actual.start             );
  ASSERT_EQ(expected.goal              , actual.goal              );
  ASSERT_EQ(expected.min_tension_change, actual.min_tension_change);
  ASSERT_EQ(expected.start_rotation    , actual.start_rotation    );
  ASSERT_EQ(expected.start_retraction  , actual.start_retraction  );
  ASSERT_EQ(expected.goal_rotation     , actual.goal_rotation     );
  ASSERT_EQ(expected.goal_retraction   , actual.goal_retraction   );
  ASSERT_EQ(expected.sample_like_sphere, actual.sample_like_sphere);
}

void check_tendon_config_space(const Problem &problem,
                               std::shared_ptr<ob::StateSpace> space)
{
  ASSERT_FALSE(space->isCompound());
  ASSERT_EQ(space->getType(), ob::STATE_SPACE_REAL_VECTOR);
  ASSERT_EQ(space->getName(), ValidityChecker::tension_state_space_name());

  auto real_space = space->as<ob::RealVectorStateSpace>();
  auto bounds = real_space->getBounds();
  ASSERT_EQ(real_space->getDimension(), problem.robot.tendons.size());
  for (size_t i = 0; i < problem.robot.tendons.size(); ++i) {
    auto &tendon = problem.robot.tendons[i];
    ASSERT_EQ(bounds.low[i], 0.0);
    ASSERT_EQ(bounds.high[i], tendon.max_tension);
  }
}

void check_rotation_config_space(const Problem &problem,
                                 std::shared_ptr<ob::StateSpace> space)
{
  (void)problem;
  ASSERT_FALSE(space->isCompound());
  ASSERT_EQ(space->getType(), ob::STATE_SPACE_SO2);
  ASSERT_EQ(space->getName(), ValidityChecker::rotation_state_space_name());

  auto so2_space = std::dynamic_pointer_cast<ob::SO2StateSpace>(space);
  ASSERT_TRUE(so2_space);
}

void check_retraction_config_space(const Problem &problem,
                                   std::shared_ptr<ob::StateSpace> space)
{
  ASSERT_FALSE(space->isCompound());
  ASSERT_EQ(space->getType(), ob::STATE_SPACE_REAL_VECTOR);
  ASSERT_EQ(space->getName(), ValidityChecker::retraction_state_space_name());

  auto real_space = std::dynamic_pointer_cast<ob::RealVectorStateSpace>(space);
  ASSERT_TRUE(real_space);

  auto bounds = real_space->getBounds();
  ASSERT_EQ(real_space->getDimension(), 1u);
  ASSERT_EQ(bounds.low[0], 0.0);
  ASSERT_EQ(bounds.high[0], problem.robot.specs.L);

  auto ret_sampler = space->allocStateSampler();
  auto cast_sampler = std::dynamic_pointer_cast<
      motion_planning::RetractionSampler>(ret_sampler);
  ASSERT_EQ(bool(cast_sampler), problem.sample_like_sphere);
}

} // end of unnamed namespace

TEST(ProblemTests, to_toml_default) {
  auto actual = Problem::from_toml(Problem().to_toml());
  Problem expected;
  assert_problem_eq(expected, actual);
}

TEST(ProblemTests, to_toml) {
  Problem problem = create_problem();
  auto actual = Problem::from_toml(problem.to_toml());
  assert_problem_eq(problem, actual);
}

TEST(ProblemTests, to_toml_default_through_string) {
  Problem problem;
  auto str = cpptoml::to_string(problem.to_toml());
  std::istringstream in(str);
  cpptoml::parser toml_parser(in);
  auto actual = Problem::from_toml(toml_parser.parse());
  assert_problem_eq(problem, actual);
}

TEST(ProblemTests, to_toml_through_string) {
  Problem problem = create_problem();
  auto str = cpptoml::to_string(problem.to_toml());
  std::istringstream in(str);
  cpptoml::parser toml_parser(in);
  auto actual = Problem::from_toml(toml_parser.parse());
  assert_problem_eq(problem, actual);
}

TEST(ProblemTests, from_toml_nullptr) {
  ASSERT_THROW(Problem::from_toml(nullptr), std::invalid_argument);
}

TEST(ProblemTests, from_toml_empty) {
  auto tbl = cpptoml::make_table();
  ASSERT_THROW(Problem::from_toml(tbl), std::out_of_range);
}

TEST(ProblemTests, from_toml_missing_robot) {
  auto tbl = create_problem().to_toml();
  tbl->erase("tendon_robot");
  ASSERT_THROW(Problem::from_toml(tbl), std::out_of_range);
}

TEST(ProblemTests, from_toml_missing_env) {
  auto problem = create_problem();
  auto tbl = problem.to_toml();
  tbl->erase("environment");
  auto actual = Problem::from_toml(tbl);
  problem.env = motion_planning::Environment();
  assert_problem_eq(problem, actual);
}

TEST(ProblemTests, from_toml_missing_start) {
  auto tbl = create_problem().to_toml();
  tbl->get("problem")->as_table()->erase("start");
  ASSERT_THROW(Problem::from_toml(tbl), std::out_of_range);
}

TEST(ProblemTests, from_toml_missing_goal) {
  auto tbl = create_problem().to_toml();
  tbl->get("problem")->as_table()->erase("goal");
  ASSERT_THROW(Problem::from_toml(tbl), std::out_of_range);
}

TEST(ProblemTests, from_toml_missing_min_tension_change) {
  auto tbl = create_problem().to_toml();
  tbl->get("problem")->as_table()->erase("min_tension_change");
  ASSERT_THROW(Problem::from_toml(tbl), std::out_of_range);
}

TEST(ProblemTests, from_toml_missing_start_rotation_enabled) {
  auto problem = create_problem();
  problem.robot.enable_rotation = true;
  auto tbl = problem.to_toml();
  tbl->get("problem")->as_table()->erase("start_rotation");
  ASSERT_THROW(Problem::from_toml(tbl), std::out_of_range);
}

TEST(ProblemTests, from_toml_missing_start_rotation_disabled) {
  auto problem = create_problem();
  problem.robot.enable_rotation = false;
  auto tbl = problem.to_toml();
  tbl->get("problem")->as_table()->erase("start_rotation");
  auto actual = Problem::from_toml(tbl);
  problem.start_rotation = 0.0;
  assert_problem_eq(problem, actual);
}

TEST(ProblemTests, from_toml_missing_start_retraction_enabled) {
  auto problem = create_problem();
  problem.robot.enable_retraction = true;
  auto tbl = problem.to_toml();
  tbl->get("problem")->as_table()->erase("start_retraction");
  ASSERT_THROW(Problem::from_toml(tbl), std::out_of_range);
}

TEST(ProblemTests, from_toml_missing_start_retraction_disabled) {
  auto problem = create_problem();
  problem.robot.enable_retraction = false;
  auto tbl = problem.to_toml();
  tbl->get("problem")->as_table()->erase("start_retraction");
  auto actual = Problem::from_toml(tbl);
  problem.start_retraction = 0.0;
  assert_problem_eq(problem, actual);
}

TEST(ProblemTests, from_toml_missing_goal_rotation_enabled) {
  auto problem = create_problem();
  problem.robot.enable_rotation = true;
  auto tbl = problem.to_toml();
  tbl->get("problem")->as_table()->erase("goal_rotation");
  ASSERT_THROW(Problem::from_toml(tbl), std::out_of_range);
}

TEST(ProblemTests, from_toml_missing_goal_rotation_disabled) {
  auto problem = create_problem();
  problem.robot.enable_rotation = false;
  auto tbl = problem.to_toml();
  tbl->get("problem")->as_table()->erase("goal_rotation");
  auto actual = Problem::from_toml(tbl);
  problem.goal_rotation = 0.0;
  assert_problem_eq(problem, actual);
}

TEST(ProblemTests, from_toml_missing_goal_retraction_enabled) {
  auto problem = create_problem();
  problem.robot.enable_retraction = true;
  auto tbl = problem.to_toml();
  tbl->get("problem")->as_table()->erase("goal_retraction");
  ASSERT_THROW(Problem::from_toml(tbl), std::out_of_range);
}

TEST(ProblemTests, from_toml_missing_goal_retraction_disabled) {
  auto problem = create_problem();
  problem.robot.enable_retraction = false;
  auto tbl = problem.to_toml();
  tbl->get("problem")->as_table()->erase("goal_retraction");
  auto actual = Problem::from_toml(tbl);
  problem.goal_retraction = 0.0;
  assert_problem_eq(problem, actual);
}

TEST(ProblemTests, from_toml_missing_sample_like_sphere_retraction_enabled) {
  auto problem = create_problem();
  problem.robot.enable_retraction = true;
  problem.sample_like_sphere = false;
  auto tbl = problem.to_toml();
  tbl->get("problem")->as_table()->erase("sample_like_sphere");
  auto actual = Problem::from_toml(tbl);
  problem.sample_like_sphere = true;
  assert_problem_eq(problem, actual);

  // same outcome if sample_like_sphere was true
  problem.sample_like_sphere = true;
  tbl = problem.to_toml();
  tbl->get("problem")->as_table()->erase("sample_like_sphere");
  actual = Problem::from_toml(tbl);
  problem.sample_like_sphere = true;
  assert_problem_eq(problem, actual);
}

TEST(ProblemTests, from_toml_missing_sample_like_sphere_retraction_disabled) {
  auto problem = create_problem();
  problem.robot.enable_retraction = false;
  problem.sample_like_sphere = false;
  auto tbl = problem.to_toml();
  tbl->get("problem")->as_table()->erase("sample_like_sphere");
  auto actual = Problem::from_toml(tbl);
  problem.sample_like_sphere = true;
  assert_problem_eq(problem, actual);

  // same outcome if sample_like_sphere was true
  problem.sample_like_sphere = true;
  tbl = problem.to_toml();
  tbl->get("problem")->as_table()->erase("sample_like_sphere");
  actual = Problem::from_toml(tbl);
  problem.sample_like_sphere = true;
  assert_problem_eq(problem, actual);
}

TEST(ProblemTests, from_toml_wrong_type_robot) {
  auto tbl = create_problem().to_toml();
  tbl->insert("tendon_robot", "name");
  ASSERT_THROW(Problem::from_toml(tbl), cpptoml::parse_exception);
}

TEST(ProblemTests, from_toml_wrong_type_env) {
  auto tbl = create_problem().to_toml();
  tbl->insert("environment", "name");
  ASSERT_THROW(Problem::from_toml(tbl), cpptoml::parse_exception);
}

TEST(ProblemTests, from_toml_wrong_type_start) {
  auto tbl = create_problem().to_toml();
  tbl->get("problem")->as_table()->insert("start", "name");
  ASSERT_THROW(Problem::from_toml(tbl), cpptoml::parse_exception);
}

TEST(ProblemTests, from_toml_wrong_type_goal) {
  auto tbl = create_problem().to_toml();
  tbl->get("problem")->as_table()->insert("goal", "name");
  ASSERT_THROW(Problem::from_toml(tbl), cpptoml::parse_exception);
}

TEST(ProblemTests, from_toml_wrong_type_min_tension_change) {
  auto tbl = create_problem().to_toml();
  tbl->get("problem")->as_table()->insert("min_tension_change", "name");
  ASSERT_THROW(Problem::from_toml(tbl), cpptoml::parse_exception);
}

TEST(ProblemTests, from_toml_wrong_type_start_rotation_enabled) {
  auto problem = create_problem();
  problem.robot.enable_rotation = true;
  auto tbl = problem.to_toml();
  tbl->get("problem")->as_table()->insert("start_rotation", "name");
  ASSERT_THROW(Problem::from_toml(tbl), cpptoml::parse_exception);
}

TEST(ProblemTests, from_toml_wrong_type_start_rotation_disabled) {
  auto problem = create_problem();
  problem.robot.enable_rotation = false;
  auto tbl = problem.to_toml();
  tbl->get("problem")->as_table()->insert("start_rotation", "name");
  ASSERT_THROW(Problem::from_toml(tbl), cpptoml::parse_exception);
}

TEST(ProblemTests, from_toml_wrong_type_start_retraction_enabled) {
  auto problem = create_problem();
  problem.robot.enable_retraction = true;
  auto tbl = problem.to_toml();
  tbl->get("problem")->as_table()->insert("start_retraction", "name");
  ASSERT_THROW(Problem::from_toml(tbl), cpptoml::parse_exception);
}

TEST(ProblemTests, from_toml_wrong_type_start_retraction_disabled) {
  auto problem = create_problem();
  problem.robot.enable_retraction = false;
  auto tbl = problem.to_toml();
  tbl->get("problem")->as_table()->insert("start_retraction", "name");
  ASSERT_THROW(Problem::from_toml(tbl), cpptoml::parse_exception);
}

TEST(ProblemTests, from_toml_wrong_type_goal_rotation_enabled) {
  auto problem = create_problem();
  problem.robot.enable_rotation = true;
  auto tbl = problem.to_toml();
  tbl->get("problem")->as_table()->insert("goal_rotation", "name");
  ASSERT_THROW(Problem::from_toml(tbl), cpptoml::parse_exception);
}

TEST(ProblemTests, from_toml_wrong_type_goal_rotation_disabled) {
  auto problem = create_problem();
  problem.robot.enable_rotation = false;
  auto tbl = problem.to_toml();
  tbl->get("problem")->as_table()->insert("goal_rotation", "name");
  ASSERT_THROW(Problem::from_toml(tbl), cpptoml::parse_exception);
}

TEST(ProblemTests, from_toml_wrong_type_goal_retraction_enabled) {
  auto problem = create_problem();
  problem.robot.enable_retraction = true;
  auto tbl = problem.to_toml();
  tbl->get("problem")->as_table()->insert("goal_retraction", "name");
  ASSERT_THROW(Problem::from_toml(tbl), cpptoml::parse_exception);
}

TEST(ProblemTests, from_toml_wrong_type_goal_retraction_disabled) {
  auto problem = create_problem();
  problem.robot.enable_retraction = false;
  auto tbl = problem.to_toml();
  tbl->get("problem")->as_table()->insert("goal_retraction", "name");
  ASSERT_THROW(Problem::from_toml(tbl), cpptoml::parse_exception);
}

TEST(ProblemTests, from_toml_wrong_type_sample_like_sphere_retraction_enabled) {
  auto problem = create_problem();
  problem.robot.enable_retraction = true;
  problem.sample_like_sphere = false;
  auto tbl = problem.to_toml();
  tbl->get("problem")->as_table()->insert("goal_retraction", "name");
  ASSERT_THROW(Problem::from_toml(tbl), cpptoml::parse_exception);

  // same outcome if sample_like_sphere was true
  problem.sample_like_sphere = true;
  tbl = problem.to_toml();
  tbl->get("problem")->as_table()->insert("goal_retraction", "name");
  ASSERT_THROW(Problem::from_toml(tbl), cpptoml::parse_exception);
}

TEST(ProblemTests, from_toml_wrong_type_sample_like_sphere_retraction_disabled) {
  auto problem = create_problem();
  problem.robot.enable_retraction = false;
  problem.sample_like_sphere = false;
  auto tbl = problem.to_toml();
  tbl->get("problem")->as_table()->insert("goal_retraction", "name");
  ASSERT_THROW(Problem::from_toml(tbl), cpptoml::parse_exception);

  // same outcome if sample_like_sphere was true
  problem.sample_like_sphere = true;
  tbl = problem.to_toml();
  tbl->get("problem")->as_table()->insert("goal_retraction", "name");
  ASSERT_THROW(Problem::from_toml(tbl), cpptoml::parse_exception);
}

TEST(ProblemTests, make_plan_continuous) {
  const double pi = std::acos(-1);
  auto problem = create_problem();
  problem.robot.enable_rotation = true;
  problem.robot.enable_retraction = false;
  motion_planning::Problem::PlanType plan {
    // tau1,  tau2,  tau3, rotation
    { 1.234, 2.234, 3.234,  0.000   },        // 0
    { 4.421, 0.000, 2.321,  1.000   },        // 1
    { 4.421, 0.000, 2.321,  2.000   },        // 2
    { 4.421, 0.000, 2.321,  3.000   },        // 3
    { 4.421, 0.000, 2.321, -3.000   },        // 4
    { 4.421, 0.000, 2.321, -2.000   },        // 5
    { 4.421, 0.000, 2.321, -1.000   },        // 6
    { 4.421, 0.000, 2.321,  0.000   },        // 7
  };
  motion_planning::Problem::PlanType expected {
    // tau1,  tau2,  tau3, rotation
    { 1.234, 2.234, 3.234,  0.000   },        // 0
    { 4.421, 0.000, 2.321,  1.000   },        // 1
    { 4.421, 0.000, 2.321,  2.000   },        // 2
    { 4.421, 0.000, 2.321,  3.000   },        // 3
    { 4.421, 0.000, 2.321, -3.000 + 2*pi  },  // 4
    { 4.421, 0.000, 2.321, -2.000 + 2*pi  },  // 5
    { 4.421, 0.000, 2.321, -1.000 + 2*pi  },  // 6
    { 4.421, 0.000, 2.321,  0.000 + 2*pi  },  // 7
  };
  problem.make_plan_continuous(plan);
  ASSERT_EQ(plan.size(), expected.size());
  for (size_t i = 0; i < plan.size(); ++i) {
    ASSERT_EQ(plan[i].size(), expected[i].size()) << "At index " << i;
    ASSERT_EQ(plan[i].size(), size_t(4)) << "At index " << i;
    ASSERT_EQ(plan[i][0], expected[i][0]) << "At index " << i;
    ASSERT_EQ(plan[i][1], expected[i][1]) << "At index " << i;
    ASSERT_EQ(plan[i][2], expected[i][2]) << "At index " << i;
    ASSERT_DOUBLE_EQ(plan[i][3], expected[i][3]) << "At index " << i; // the one that matters
  }
}

TEST(ProblemTests, create_space_information_with_just_tensions) {
  auto problem = create_problem();
  problem.robot.enable_rotation = false;
  problem.robot.enable_retraction = false;
  auto si = problem.create_space_information();
  auto space = si->getStateSpace();
  ASSERT_TRUE(space->isCompound());
  ASSERT_EQ(space->getType(), ob::STATE_SPACE_UNKNOWN);
  ASSERT_EQ(space->getName(), "CompoundStateSpace");
  auto cspace = space->as<ob::CompoundStateSpace>();

  ASSERT_EQ(cspace->getSubspaceCount(), 1u);
  auto s0 = cspace->getSubspace(0);

  check_tendon_config_space(problem, s0);
}

TEST(ProblemTests, create_space_information_with_rot) {
  auto problem = create_problem();
  problem.robot.enable_rotation = true;
  problem.robot.enable_retraction = false;
  auto si = problem.create_space_information();
  auto space = si->getStateSpace();
  ASSERT_TRUE(space->isCompound());
  ASSERT_EQ(space->getType(), ob::STATE_SPACE_UNKNOWN);
  ASSERT_EQ(space->getName(), "CompoundStateSpace");
  auto cspace = space->as<ob::CompoundStateSpace>();

  ASSERT_EQ(cspace->getSubspaceCount(), 2u);
  auto s0 = cspace->getSubspace(0);
  auto s1 = cspace->getSubspace(1);

  check_tendon_config_space(problem, s0);
  check_rotation_config_space(problem, s1);
}

TEST(ProblemTests, create_space_information_with_ret) {
  auto problem = create_problem();
  problem.robot.enable_rotation = false;
  problem.robot.enable_retraction = true;
  auto si = problem.create_space_information();
  auto space = si->getStateSpace();
  ASSERT_TRUE(space->isCompound());
  ASSERT_EQ(space->getType(), ob::STATE_SPACE_UNKNOWN);
  ASSERT_EQ(space->getName(), "CompoundStateSpace");
  auto cspace = space->as<ob::CompoundStateSpace>();

  ASSERT_EQ(cspace->getSubspaceCount(), 2u);
  auto s0 = cspace->getSubspace(0);
  auto s1 = cspace->getSubspace(1);

  check_tendon_config_space(problem, s0);
  check_retraction_config_space(problem, s1);
}

TEST(ProblemTests, create_space_information_with_rot_and_ret) {
  auto problem = create_problem();
  problem.robot.enable_rotation = true;
  problem.robot.enable_retraction = true;
  problem.sample_like_sphere = true;
  auto si = problem.create_space_information();
  auto space = si->getStateSpace();
  ASSERT_TRUE(space->isCompound());
  ASSERT_EQ(space->getType(), ob::STATE_SPACE_UNKNOWN);
  ASSERT_EQ(space->getName(), "CompoundStateSpace");
  auto cspace = space->as<ob::CompoundStateSpace>();

  ASSERT_EQ(cspace->getSubspaceCount(), 3u);
  auto s0 = cspace->getSubspace(0);
  auto s1 = cspace->getSubspace(1);
  auto s2 = cspace->getSubspace(2);

  check_tendon_config_space(problem, s0);
  check_rotation_config_space(problem, s1);
  check_retraction_config_space(problem, s2);
}

TEST(ProblemTests, create_space_information_with_retraction_enabled_sample_like_sphere) {
  auto problem = create_problem();
  problem.robot.enable_rotation = true;
  problem.robot.enable_retraction = true;
  problem.sample_like_sphere = true;
  auto si = problem.create_space_information();
  auto space = si->getStateSpace();
  ASSERT_TRUE(space->isCompound());
  ASSERT_EQ(space->getType(), ob::STATE_SPACE_UNKNOWN);
  ASSERT_EQ(space->getName(), "CompoundStateSpace");
  auto cspace = space->as<ob::CompoundStateSpace>();

  ASSERT_EQ(cspace->getSubspaceCount(), 3u);
  auto s0 = cspace->getSubspace(0);
  auto s1 = cspace->getSubspace(1);
  auto s2 = cspace->getSubspace(2);

  check_tendon_config_space(problem, s0);
  check_rotation_config_space(problem, s1);
  check_retraction_config_space(problem, s2);
}

TEST(ProblemTests, create_space_information_with_retraction_enabled_no_sample_like_sphere) {
  auto problem = create_problem();
  problem.robot.enable_rotation = true;
  problem.robot.enable_retraction = true;
  problem.sample_like_sphere = false;
  auto si = problem.create_space_information();
  auto space = si->getStateSpace();
  ASSERT_TRUE(space->isCompound());
  ASSERT_EQ(space->getType(), ob::STATE_SPACE_UNKNOWN);
  ASSERT_EQ(space->getName(), "CompoundStateSpace");
  auto cspace = space->as<ob::CompoundStateSpace>();

  ASSERT_EQ(cspace->getSubspaceCount(), 3u);
  auto s0 = cspace->getSubspace(0);
  auto s1 = cspace->getSubspace(1);
  auto s2 = cspace->getSubspace(2);

  check_tendon_config_space(problem, s0);
  check_rotation_config_space(problem, s1);
  check_retraction_config_space(problem, s2);
}

TEST(ProblemTests, create_space_information_with_retraction_disabled_sample_like_sphere) {
  auto problem = create_problem();
  problem.robot.enable_rotation = true;
  problem.robot.enable_retraction = false;
  problem.sample_like_sphere = true;
  auto si = problem.create_space_information();
  auto space = si->getStateSpace();
  ASSERT_TRUE(space->isCompound());
  ASSERT_EQ(space->getType(), ob::STATE_SPACE_UNKNOWN);
  ASSERT_EQ(space->getName(), "CompoundStateSpace");
  auto cspace = space->as<ob::CompoundStateSpace>();

  ASSERT_EQ(cspace->getSubspaceCount(), 2u);
  auto s0 = cspace->getSubspace(0);
  auto s1 = cspace->getSubspace(1);

  check_tendon_config_space(problem, s0);
  check_rotation_config_space(problem, s1);
}

TEST(ProblemTests, create_space_information_with_retraction_disabled_no_sample_like_sphere) {
  auto problem = create_problem();
  problem.robot.enable_rotation = true;
  problem.robot.enable_retraction = false;
  problem.sample_like_sphere = false;
  auto si = problem.create_space_information();
  auto space = si->getStateSpace();
  ASSERT_TRUE(space->isCompound());
  ASSERT_EQ(space->getType(), ob::STATE_SPACE_UNKNOWN);
  ASSERT_EQ(space->getName(), "CompoundStateSpace");
  auto cspace = space->as<ob::CompoundStateSpace>();

  ASSERT_EQ(cspace->getSubspaceCount(), 2u);
  auto s0 = cspace->getSubspace(0);
  auto s1 = cspace->getSubspace(1);

  check_tendon_config_space(problem, s0);
  check_rotation_config_space(problem, s1);
}

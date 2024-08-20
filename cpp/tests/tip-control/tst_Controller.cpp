/**
 * author:        Michael Bentley
 * email:         mikebentley15@gmail.com
 * date-created:  05 February 2021
 */

#include "tip-control/Controller.h"
#include "tendon/TendonRobot.h"

#include <Eigen/Core>

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace E = Eigen;

TEST(ControllerTests, inverse_kinematics_outside_range_bug) {
  // This test replicates a bug with commanding controls out of bounds for a
  // requested IK.

  tendon::TendonRobot robot;
  robot.specs.E           = 2.1e6;
  robot.specs.L           = 0.12;
  robot.specs.dL          = 0.00059;
  robot.specs.nu          = 0.3;
  robot.specs.ri          = 0.0;
  robot.specs.ro          = 0.003;
  robot.r                 = 3e-3;
  robot.enable_rotation   = true;
  robot.enable_retraction = true;

  using Vx = Eigen::VectorXd;
  tendon::TendonSpecs tendon {Vx(2), Vx(1)};

  tendon.C << 0.0, 0.0;
  tendon.D << 0.0025;
  tendon.max_tension = 3.5;
  tendon.min_length = -0.029;
  tendon.max_length = 0.048;
  robot.tendons.emplace_back(tendon);

  tendon.C << 1.571, 50.0;
  tendon.D << 0.0025;
  tendon.max_tension = 3.5;
  tendon.min_length = -0.027;
  tendon.max_length = 0.046;
  robot.tendons.emplace_back(tendon);

  tendon.C << -1.571, -50.0;
  tendon.D << 0.0025;
  tendon.max_tension = 3.5;
  tendon.min_length = -0.027;
  tendon.max_length = 0.046;
  robot.tendons.emplace_back(tendon);

  Controller controller(std::move(robot));
  std::vector<double> start_state {
    1.3431772975663234,
    1.4623882507676742,
    0.96446718783967644,
    -2.7209384807464065,
    0.11999880376877663,
  };
  E::Vector3d desired_tip {
    -0.0028674899999999999,
    0.0052556,
    -0.0014133800000000001,
  };
  const size_t max_iters                 = 30;
  const double threshold                 = robot.specs.dL / 5.0;
  const double mu_init                   = 1e-3;
  const double stop_threshold_JT_err_inf = threshold / 200;
  const double stop_threshold_Dp         = threshold / 2.0;
  const double stop_threshold_err        = threshold;
  const double finite_difference_delta   = threshold / 20.0;

  // asserting here that we do not throw
  auto result = controller.inverse_kinematics(
      start_state,
      desired_tip,
      max_iters,
      mu_init,
      stop_threshold_JT_err_inf,
      stop_threshold_Dp,
      stop_threshold_err,
      finite_difference_delta);

  // another example that caused the same bug
  start_state = {
    1.9124234975747305,
    2.9817077719027312,
    0.58622861698620632,
    1.7465114717296215,
    0.11999655382167507,
  };
  result = controller.inverse_kinematics(
      start_state,
      desired_tip,
      max_iters,
      mu_init,
      stop_threshold_JT_err_inf,
      stop_threshold_Dp,
      stop_threshold_err,
      finite_difference_delta);

  // another example that might cause the same bug
  start_state = {
    1.9124234975747305,
    2.9817077719027312,
    0.58622861698620632,
    1.7465114717296215,
    -0.00000000000000007,
  };
  result = controller.inverse_kinematics(
      start_state,
      desired_tip,
      max_iters,
      mu_init,
      stop_threshold_JT_err_inf,
      stop_threshold_Dp,
      stop_threshold_err,
      finite_difference_delta);
  
  // TODO: assert that we are within our control limits
}

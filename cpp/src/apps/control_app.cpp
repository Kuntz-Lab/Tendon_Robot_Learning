#include "cliparser/CliParser.h"
#include "collision/CapsuleSequence.h"
#include "cpptoml/toml_conversions.h"
#include "tendon/TendonRobot.h"
#include "tendon/TendonSpecs.h"
#include "tip-control/Controller.h"
#include "util/openfile_check.h"  // for openfile_check()
#include "util/vector_ops.h"      // for operator<<()
#include "vistendon/view_path.h"

#include <Eigen/Core>
#include <Eigen/LU>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <fstream>
#include <iomanip>
#include <ios>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>

#include <cmath>
#include <cstdlib>

#include <sys/stat.h>   // for mkdir()

namespace E = Eigen;

namespace {

namespace defaults {
  const int max_iter = 10000;
  const double mu_init = 1;
  const double eps1 = 1e-9;
  const double eps2 = 1e-9;
  const double eps3 = 1e-9;
  const double fd_delta = 0.001;
} // end of namespace defaults

std::string to_string_sci(double val) {
  std::ostringstream out;
  out << std::scientific << std::setprecision(2) << val;
  return out.str();
}

void create_directory(const std::string &directory, int mode = 0755) {
  int err = 0;
  err = ::mkdir(directory.c_str(), mode); // drwx------
  if (err != 0) {
    std::string msg = "Could not create directory: ";
    msg += strerror(err);
    throw std::ios_base::failure(msg);
  }
}

void populate_parser(CliParser &parser) {
  parser.add_argflag("-r", "--robot");
  parser.add_flag("-c", "--clamped");
  parser.add_flag("-v", "--view-path");
  parser.add_flag("-w", "--write");
  parser.add_argflag("--max-iter");
  parser.add_argflag("--mu-init");
  parser.add_argflag("--eps1");
  parser.add_argflag("--eps2");
  parser.add_argflag("--eps3");
  parser.add_argflag("--fd-delta");
  parser.add_flag("--no-controller");
  parser.add_flag("--no-ik");
  parser.set_description("--robot", "Read robot configuration from a toml file");
  parser.set_description("--clamped", "Use the clamped version of the controller.\n"
      "                By default, we use the non-clamped version which does not\n"
      "                limit the step size in each iteration.\n");
  parser.set_description("--view-path", "Use this option to view the robot motion\n"
      "                and end-effector trajectory");
  parser.set_description("--write","Use this option to write all necessary\n"
      "                data (errors, tip positions, configuration states,\n"
      "                and backbone states) to text files in the data folder");
  parser.set_description("--max-iter", "The maximum number of iterations to use\n"
      "                for inverse kinematics.\n"
      "                Default is " + std::to_string(defaults::max_iter) + ".\n");
  parser.set_description("--mu-init", "For inverse kinematics, initial value\n"
      "                for mu in the Levenberg Marquardt algorithm.\n"
      "                Default is " + to_string_sci(defaults::mu_init) + ".\n");
  parser.set_description("--eps1", "For inverse kinematics, stopping criteria\n"
      "                value for |J^T err|_inf.\n"
      "                Default is " + to_string_sci(defaults::eps1) + ".\n");
  parser.set_description("--eps2", "For inverse kinematics, stopping criteria\n"
      "                value for |Dp|_2 which is like a relative step size\n"
      "                threshold.\n"
      "                Default is " + to_string_sci(defaults::eps2) + ".\n");
  parser.set_description("--eps3", "For inverse kinematics, stopping criteria\n"
      "                value for |err|_2.  This is how close we are to the goal\n"
      "                Default is " + to_string_sci(defaults::eps3) + ".\n");
  parser.set_description("--fd-delta", "For inverse kinematics, delta value\n"
      "                used for finite differences.\n"
      "                Default is " + to_string_sci(defaults::fd_delta) + "\n.");
  parser.set_description("--no-controller", "Do not run Rahul's controller");
  parser.set_description("--no-ik", "Do not run inverse kinematics using levmar");
}

tendon::TendonRobot default_robot() {
  tendon::TendonRobot robot;
  double max_tau = 100;

  tendon::TendonSpecs current_tendon {E::VectorXd(3), E::VectorXd(3)};
  current_tendon.max_tension = max_tau;

  current_tendon.C << 0    , 0 , 0;
  current_tendon.D << 0.01 , 0 , 0;
  robot.tendons.push_back(current_tendon);

  current_tendon.C << 2*M_PI/3 , 0 , 0;
  current_tendon.D << 0.01     , 0 , 0;
  robot.tendons.push_back(current_tendon);

  current_tendon.C << -2*M_PI/3 , 0 , 0;
  current_tendon.D <<  0.01     , 0 , 0;
  robot.tendons.push_back(current_tendon);

  return robot;
}

} // end of unnamed namespace

int main(int argCount, char* argList[]) {
  using util::operator<<;
  using vecd = std::vector<double>;

  CliParser parser;
  populate_parser(parser);
  parser.parse(argCount, argList);

  std::cout << "Tendon routing for ith tendon is parameterized as\n"
               "  phi_i = C1 + C2*s + C3*s^2 ... + Cn*s^(n-1)\n"
               "  r_i = D1 + D2*s + D3*s^2 ... + Dn*s^(n-1)\n"
               "\n";

  char clamped_char;
  std::cout << "Using clamped controller? ";
  if (parser.has("--clamped")) {
    clamped_char = 'y';
    std::cout << "yes";
  } else {
    clamped_char = 'n';
    std::cout << "no";
  }
  std::cout << "\n\n";

  tendon::TendonRobot robot;

  if (parser.has("--robot")) {
    robot = cpptoml::from_file<tendon::TendonRobot>(parser["--robot"]);
  } else {
    robot = default_robot();
  }

  // initial state
  vecd partial_state {6, 1, 4};
  vecd state(robot.state_size(), 0.0);
  if (robot.state_size() <= 1) { state[0] = partial_state[0]; }
  if (robot.state_size() <= 2) { state[1] = partial_state[1]; }
  if (robot.state_size() <= 3) { state[2] = partial_state[2]; }

  cpptoml::to_stream(std::cout, robot.to_toml());
  std::cout << "\n";

  E::Vector3d des { 0.1, 0.05, 0.07 };
  auto max_iter = parser.get("--max-iter", defaults::max_iter);
  auto mu_init = parser.get("--mu-init", defaults::mu_init);
  auto eps1 = parser.get("--eps1", defaults::eps1);
  auto eps2 = parser.get("--eps2", defaults::eps2);
  auto eps3 = parser.get("--eps3", defaults::eps3);
  auto delta = parser.get("--fd-delta", defaults::fd_delta);
  bool verbose = true;
  std::cout << std::boolalpha <<
    "Desired position  = [" << des.transpose() << "]\n"
    "IK settings\n"
    "  max iter       = " << max_iter << "\n"
    "  mu init        = " << mu_init << "\n"
    "  eps1           = " << eps1 << "\n"
    "  eps2           = " << eps2 << "\n"
    "  eps3           = " << eps3 << "\n"
    "  fd delta       = " << delta << "\n"
    "  verbose        = " << verbose << "\n"
    "\n" << std::flush;

  Controller cont(robot);

  auto calc_error = [&robot](const vecd &state, const E::Vector3d &des) {
    auto shape = robot.forward_kinematics(state);
    double err = (shape.back() - des).norm();
    return err;
  };

  Controller::ControlResult results;
  if (!parser.has("--no-controller")) {
    results = cont.control(state, des, clamped_char);
    auto &[configs, tipPositions, errors, backbones, timing, stat] = results;
    std::cout << std::boolalpha <<
      "Rahul's Controller:\n"
      "  Control iters   = " << tipPositions.size() << "\n"
      "  Final tip pos   = [" << tipPositions.back().transpose() << "]\n"
      "  Final config    = " << configs.back() << "\n"
      "  Reported error  = " << errors.back() * 100 << " cm\n"
      "  Time            = " << timing << " sec\n"
      "  Success?        = " << stat << "\n"
      "  Calc tip pos    = ["
        << robot.forward_kinematics(configs.back()).back().transpose() << "]\n"
      "  Calc tip error  = " << calc_error(configs.back(), des) << "\n";
  }

  if (!parser.has("--no-ik")) {
    auto before = std::chrono::high_resolution_clock::now();
    auto soln = cont.inverse_kinematics(state, des, max_iter, mu_init, eps1,
                                        eps2, eps3, delta, verbose);
    auto after = std::chrono::high_resolution_clock::now();

    std::cout << std::boolalpha <<
      "Inverse Kinematics using levmar:\n"
      "  Time            = "
        << std::chrono::duration<double>(after - before).count()
        << " secs\n"
      "  Reported soln   = " << soln.state << "\n"
      "  Reported iters  = " << soln.iters << "\n"
      "  Reported error  = " << soln.error * 100 << " cm\n"
      "  Reported fk's   = " << soln.num_fk_calls << "\n"
      "  Calc tip pos    = ["
        << robot.forward_kinematics(soln.state).back().transpose() << "]\n"
      "  Calc tip error  = " << calc_error(soln.state, des) << "\n"
      "\n";
  }

  if (parser.has("--write") && !parser.has("--no-controller")){
    std::cout << "Writing to text files in ../data/ folder\n";

    try {
      create_directory("../data");
    } catch (std::ios_base::failure &) {
      // do nothing if already exists
    }

    std::ofstream configs_file;
    util::openfile_check(configs_file, "../data/configs.txt");
    for (auto &config : results.states) {
      configs_file << config[0] << " "
                   << config[1] << " "
                   << config[2] << " "
                   << config[3] << std::endl;
    }
    configs_file.close();
    std::cout << "  - wrote ../data/configs.txt" << std::endl;

    std::ofstream tips_file;
    util::openfile_check(tips_file, "../data/tips.txt");
    for (auto &pos : results.tip_positions) {
      tips_file << pos[0] << " "
                << pos[1] << " "
                << pos[2] << std::endl;
    }
    tips_file.close();
    std::cout << "  - wrote ../data/tips.txt" << std::endl;

    std::ofstream errors_file;
    util::openfile_check(errors_file, "../data/errors.txt");
    for (auto &error : results.errors) {
      errors_file << error << std::endl;
    }
    errors_file.close();
    std::cout << "  - wrote ../data/errors.txt" << std::endl;

    std::ofstream backbone_file;
    util::openfile_check(backbone_file, "../data/backbones.txt");
    for (auto &backbone : results.backbones) {
      for (auto &pos : backbone) {
        backbone_file << pos[0] << " "
                      << pos[1] << " "
                      << pos[2] << " ";
      }
      backbone_file << std::endl;
    }
    backbone_file.close();
    std::cout << "  - wrote ../data/backbones.txt\n"
                 "Done writing to files\n\n";
  }

  if (parser.has("--view-path") && !parser.has("--no-controller")){
    rclcpp::init(argCount, argList);
    auto subsampled_configs = vistendon::subsample(results.states, 100);
    vistendon::view_path(subsampled_configs, robot, results.tip_positions[0], des);
  }

  return 0;
}


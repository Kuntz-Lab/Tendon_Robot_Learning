#include "cliparser/CliParser.h"
#include "tendon/TendonRobot.h"
#include "tendon/TendonSpecs.h"
#include "util/vector_ops.h"

#include <Eigen/Core>

#include <chrono>
#include <vector>

namespace E = Eigen;

namespace {

void populate_parser(CliParser &parser) {
  parser.set_program_description(
      "A simple example demonstrating and timing the running of foward\n"
      "  kinematics on a single example configuration.");
}

} // end of unnamed namespace

int main(int arg_count, char *arg_list[]) {
  CliParser parser;
  populate_parser(parser);
  parser.parse(arg_count, arg_list);

  using secs = std::chrono::duration<double>;
  using util::operator<<;

  // Default values
  tendon::TendonRobot robot;
  tendon::TendonSpecs current_tendon {E::VectorXd(2), E::VectorXd(1)};
  current_tendon.C <<  0.0       ,   0.0;
  current_tendon.D <<  0.01;
  robot.tendons.push_back(current_tendon);
  current_tendon.C <<  2*M_PI/3  ,   0.0;
  current_tendon.D <<  0.01;
  robot.tendons.push_back(current_tendon);
  current_tendon.C <<  -2*M_PI/3 ,   0.0;
  current_tendon.D <<  0.01;
  robot.tendons.push_back(current_tendon);

  std::vector<double> tau {30.0, 0.0, 0.0};

  std::cout << "\n"
            << "tendons:\n";
  for (auto &tendon : robot.tendons) {
    std::cout << "  " << tendon << '\n';
  }
  std::cout << "tau:   " << tau << "\n"
            << "specs: " << robot.specs << "\n\n";

  std::cout << "Running robot.shape(tau)" << std::endl;
  auto shape = robot.shape(tau);
  std::cout << "End-effector position: [" << shape.p.back().transpose() << "]\n"
            << "L:                     " << shape.L << "\n"
            << "L_i:                   " << shape.L_i << "\n"
            << "\n"
            << "Performing timing measurements:\n";

  auto before = std::chrono::high_resolution_clock::now();
  int N = 8000;
# pragma omp parallel for
  for (int i = 0; i < N; i++) {
    robot.shape(tau);
  }
  auto after = std::chrono::high_resolution_clock::now();
  auto diff = secs(after - before).count();
  std::cout
      << "Total time for " << N << " runs: " << diff << " seconds\n"
      << "Average time:              " << diff / double(N) << " seconds\n"
      << "Frequency:                 " << N / diff << " Hz\n"
      << "\n";
  return 0;
}

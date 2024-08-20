#include "tendon/BackboneSpecs.h"
#include "tendon/TendonResult.h"
#include "tendon/TendonRobot.h"
#include "util/vector_ops.h"

#include <flit/flit.h>
#include <Eigen/Core>

#include <chrono>
#include <string>
#include <vector>

#include <cmath>

namespace E = Eigen;

using util::operator<<;

template <typename T>
class TendonShapeTest : public flit::TestBase<T> {
public:
  TendonShapeTest(std::string id) : flit::TestBase<T>(std::move(id)) {}

  virtual size_t getInputsPerRun() override { return 0; }
  virtual std::vector<T> getDefaultInput() override { return {}; }

  virtual long double compare(const std::vector<T> &ground_truth,
                              const std::vector<T> &test_results)
  const override {
    long double sum = 0.0;
    for (size_t i = 0; i < ground_truth.size(); i++) {
      sum += std::abs(test_results[i] - ground_truth[i]); // L1 norm
    }
    return sum;
  }

protected:
  virtual flit::Variant run_impl(const std::vector<T> &ti) override {
    FLIT_UNUSED(ti);
    return {};
  }

protected:
  using flit::TestBase<T>::id;
};

template<>
flit::Variant TendonShapeTest<double>::run_impl(const std::vector<double> &ti) {
  std::ostream &out = flit::info_stream;

  // Default values
  tendon::TendonRobot robot;
  tendon::TendonSpecs current_tendon{Eigen::VectorXd(2), Eigen::VectorXd(1)};
  current_tendon.C <<  0.0       ,   0.0;
  current_tendon.D <<  0.01;
  robot.tendons.push_back(current_tendon);
  current_tendon.C <<  M_PI      ,   0.0;
  current_tendon.D <<  0.01;
  robot.tendons.push_back(current_tendon);
  current_tendon.C <<  M_PI / 2.0,  20.0;
  current_tendon.D <<  0.01;
  robot.tendons.push_back(current_tendon);
  current_tendon.C << -M_PI / 2.0, -20.0;
  current_tendon.D <<  0.01;
  robot.tendons.push_back(current_tendon);

  std::vector<double> tau {2.0, 0.0, 20.0, 2.0};

  out << robot << "\n";

  out << "Running robot.shape(tau)" << std::endl;
  auto [t, p, R, L, L_i] = robot.shape(tau);

  out << "L:   " << L << "\n"
      << "L_i: " << L_i << "\n";
  out << std::endl;

  return L_i;
}

REGISTER_TYPE(TendonShapeTest)

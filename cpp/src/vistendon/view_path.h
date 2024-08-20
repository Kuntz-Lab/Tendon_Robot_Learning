#ifndef VIEW_PATH_H
#define VIEW_PATH_H

#include "motion-planning/plan.h"
#include "motion-planning/Environment.h"
#include "tendon/TendonRobot.h"

#include <string>

namespace vistendon {

void view_path(
    const std::vector<std::vector<double>> &tensions,
    const tendon::TendonRobot& robot,
    const Eigen::Vector3d &start,
    const Eigen::Vector3d &goal);

template <typename T>
std::vector<T> subsample(const std::vector<T> &vec, size_t interval) {
  std::vector<T> subsampled;
  subsampled.reserve(1 + vec.size() / interval);
  for (size_t i = 0; i < vec.size(); i += interval) {
    subsampled.emplace_back(vec[i]);
  }
  return subsampled;
}

} // end of namespace vistendon

#endif

#include "solve_initial_bending.h"

#include "get_r_info.h"
#include "TendonSpecs.h"
#include <util/vector_ops.h>

namespace E = Eigen;

namespace tendon {

std::tuple<E::Vector3d, E::Vector3d, int>
solve_initial_bending(
    const E::Vector3d &v_guess,
    const E::Vector3d &u_guess,
    const std::vector<TendonSpecs> &tendons,
    const std::vector<double> &tau,
    const E::Matrix3d &K_bt_inv,
    const E::Matrix3d &K_se_inv,
    int iter_max,
    double dv_threshold,
    double du_threshold,
    double s_start,
    rInfo &info,
    rInfoCache &cache
) {
  int iters = 0;
  auto v = v_guess;
  auto u = u_guess;
  auto N_t = tendons.size();

  get_r_info2(tendons, s_start, info, cache);
  auto &[r, r_dot, r_ddot] = info;
  for (iters = 0; iters < iter_max; iters++) {
    E::Matrix3d uhat = util::hat(u);
    E::Vector3d n = E::Vector3d::Zero();
    E::Vector3d m = E::Vector3d::Zero();

    for (decltype(N_t) k = 0; k < N_t; k++) {
      E::Vector3d pi_dot = uhat * r[k] + r_dot[k] + v;
      n -= tau[k] * pi_dot / pi_dot.norm();
      m -= tau[k] * util::hat(r[k]) * pi_dot / pi_dot.norm();
    }
    E::Vector3d v_new = K_se_inv * n + E::Vector3d(0, 0, 1);
    E::Vector3d u_new = K_bt_inv * m;

    if ((v_new - v).norm() / v.norm() < dv_threshold &&
        (u_new - u).norm() / u.norm() < du_threshold)
    {
      break;
    }

    v = v_new;
    u = u_new;
  }

  return std::tuple{v, u, iters};
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d, int>
solve_initial_bending_unopt(
    const Eigen::Vector3d &v_guess,
    const Eigen::Vector3d &u_guess,
    const std::vector<TendonSpecs> &tendons,
    const std::vector<double> &tau,
    const Eigen::Matrix3d &K_bt_inv,
    const Eigen::Matrix3d &K_se_inv,
    int iter_max,
    double dv_threshold,
    double du_threshold,
    double s_start
) {
  rInfo info;
  rInfoCache cache;
  const auto N_t = tendons.size();
  info.resize(N_t);
  cache.resize(tendons);
  return solve_initial_bending(v_guess, u_guess, tendons, tau, K_bt_inv,
                               K_se_inv, iter_max, dv_threshold, du_threshold,
                               s_start, info, cache);
}

} // end of namespace tendon

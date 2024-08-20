#ifndef MULTI_GOAL_PLANNING_COST_IK_H
#define MULTI_GOAL_PLANNING_COST_IK_H

#include "collision/VoxelOctree.h"
#include "motion-planning/VoxelEnvironment.h"
#include "tendon/TendonRobot.h"
#include "util/macros.h"

#include <Eigen/Core>

#include <memory>
#include <vector>

namespace cpptoml {
class table;
}

namespace design_optimization {

/** Cost to evaluate a robot design by planning to a set of tip positions
 *
 * This class is a Functor object, where the cost funtion is implemented as the
 * () operator, which means this object acts like a function.
 * 
 * Example:
 *
 *   design_optimization::MultiGoalPlanningCostIK cost;
 *   cost.set_start_config({0.0, 0.0, 0.0, 0.0, 0.0, 0.2});
 *   cost.set_goal_tips({
 *     {0.1, 0.1, 0.1},
 *     {0.1, -0.1, -0.1},
 *     {-0.1, 0.1, -0.1}
 *     });
 *
 *   tendon::TendonRobot robot;
 *   auto costval = cost(robot); // blocks for up to time limit
 *
 * This cost function returns the number of the goal tips that were reached
 * (within the given tip tolerance) within the time limit using the motion
 * planner.
 *
 * This class specifically uses voxels as the collision representation for
 * simplicity and speed.  It also expects that the collision space has been
 * dilated by the robot radius so that collision only check agaist the
 * voxelized centerline of the robot backbone.
 *
 * The underlying planner is RRT.  For now, this is not configurable.
 */
struct MultiGoalPlanningCostIK {
public:
  std::vector<double>          start_config; // used both for IK and planning
  std::vector<Eigen::Vector3d> goal_tips;    // optimization criteria
  motion_planning::VoxelEnvironment venv;    // for collision checking

  double tip_threshold = 1e-4;    // used now for IK only
  double time_limit_seconds = 30; // for each call to the cost function

  // IK-specific params
  int    ik_max_iters                 = 100;
  double ik_mu_init                   = 1e-3;
  double ik_stop_threshold_JT_err_inf = 1e-9;
  double ik_stop_threshold_Dp         = 1e-4;
  double ik_finite_difference_delta   = 1e-6;

  // RRT-specific params
  double rrt_grow_distance            = 1.0;  // config distance for one RRT step
  double rrt_goal_bias                = 0.05; // in [0, 1], how often to use a goal as sample

public:
  MultiGoalPlanningCostIK();

  /** Evaluates the cost function for this robot specification
   *
   * The cost is the number of the goal tips that are reached using motion
   * planning within the time limit.
   *
   * Simply returns reachable_goal_tips(robot).size();
   */
  int operator() (const tendon::TendonRobot &robot) const {
    return this->reachable_goal_tips(robot).size();
  }

  /** Returns the actual goal tips that are reachable using motion planning
   * within the time limit.
   */
  std::vector<Eigen::Vector3d> reachable_goal_tips (
      const tendon::TendonRobot &robot) const;

  std::shared_ptr<cpptoml::table> to_toml() const;
  static MultiGoalPlanningCostIK from_toml(
      std::shared_ptr<cpptoml::table> table);
}; // end of struct MultiGoalPlanningCostIK

} // end of namespace design_optimization

#endif // MULTI_GOAL_PLANNING_COST_IK_H

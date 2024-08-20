#include "cpptoml/toml_conversions.h"
#include "design-optimization/MultiGoalPlanningCostIK.h"
#include "motion-planning/Problem.h"
#include "tip-control/Controller.h"
#include "util/vector_ops.h"
#include "util/macros.h"

#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/util/Time.h>

#include <Eigen/Core>

#include <memory>
#include <stdexcept>
#include <vector>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace otime = ompl::time;

namespace E = Eigen;

namespace design_optimization {

namespace {

std::vector<double> state2vec(const ob::ScopedState<> &state) {
  std::vector<double> vec;
  auto space = state.getSpace();
  space->copyToReals(vec, state.get());
  return vec;
}

} // end of unnamed namespace

MultiGoalPlanningCostIK::MultiGoalPlanningCostIK() {}

std::vector<E::Vector3d> MultiGoalPlanningCostIK::reachable_goal_tips (
    const tendon::TendonRobot &robot) const
{
  bool verbose = true;
  using util::operator<<;
  std::vector<E::Vector3d> reached_goals;

  // start the timer
  auto before = otime::now();

  auto obstacles = this->venv.get_obstacles();

  // create planner
  motion_planning::Problem problem;
  problem.robot = robot;
  problem.venv = this->venv;
  problem.start.resize(robot.tendons.size());
  auto si = problem.create_space_information();
  problem.set_voxel_backbone_state_checker(si, *obstacles);
  problem.set_voxel_swept_motion_checker(si, *obstacles);
  auto space = si->getStateSpace();
  ob::ScopedState<> start_state(si);
  space->copyFromReals(start_state.get(), this->start_config);
  auto pdef(std::make_shared<ob::ProblemDefinition>(si));
  pdef->addStartState(start_state);
  auto planner(std::make_shared<og::RRT>(si));
  planner->setGoalBias(this->rrt_goal_bias);
  planner->setRange(this->rrt_grow_distance);
  planner->setProblemDefinition(pdef);

  // perform IK
  Controller ik_solver(robot);
  auto state_comparator =
    [&space](const ob::ScopedState<> &a, const ob::ScopedState<> &b) {
      std::vector<double> va, vb;
      space->copyToReals(va, a.get());
      space->copyToReals(vb, b.get());
      if (va.size() > vb.size()) { return true; }
      for (size_t i = 0; i < va.size(); ++i) {
        if (va[i] > vb[i]) { return true; }
        if (vb[i] > va[i]) { return false;}
      }
      return false;
    };
  std::map<ob::ScopedState<>, E::Vector3d, decltype(state_comparator)>
    ik_solns(state_comparator);
  #pragma omp parallel for
  for (size_t i = 0; i < goal_tips.size(); ++i) {
    auto &tip = goal_tips[i];

    auto result = ik_solver.inverse_kinematics(
        this->start_config,
        tip,
        this->ik_max_iters,
        this->ik_mu_init,
        this->ik_stop_threshold_JT_err_inf,
        this->ik_stop_threshold_Dp,
        this->tip_threshold,
        this->ik_finite_difference_delta,
        false);

    // only keep those within tip_threshold
    if (result.error < this->tip_threshold) {
      ob::ScopedState<> resultant_state(si);
      space->copyFromReals(resultant_state.get(), result.state);
      // only keep those that are valid (e.g., out of collision)
      if (si->isValid(resultant_state.get())) {
        #pragma omp critical
        ik_solns[resultant_state] = tip;
      }
    }
  }
  if (verbose) {
    std::cout << "IK solutions:\n";
    for (auto &kv : ik_solns) {
      std::cout << "  " << state2vec(kv.first) << "\n";
    }
  }

  // plan using the remaining time
  auto calc_remaining_time = [this, &before]() {
    return this->time_limit_seconds - otime::seconds(otime::now() - before);
  };
  for (auto remaining_time = calc_remaining_time();
       remaining_time > 0.0 && ik_solns.size() > 0;
       remaining_time = calc_remaining_time())
  {
    // add all successful IK as goals to planner
    auto goals(std::make_shared<ob::GoalStates>(si));
    for (auto &ik_soln : ik_solns) {
      goals->addState(ik_soln.first);
    }
    pdef->clearSolutionPaths();
    pdef->setGoal(goals);

    // plan until either timeout or a solution
    auto solve_status = planner->ob::Planner::solve(remaining_time);

    // check the solve status
    switch (ob::PlannerStatus::StatusType(solve_status)) {
      // successfully found a solution
      case ob::PlannerStatus::EXACT_SOLUTION: {
        // if solution, remove that goal, add to reached set
        auto *path = static_cast<og::PathGeometric*>(
            pdef->getSolutionPath().get());
        auto reached_goal = path->getStates().back();
        ob::ScopedState<> scoped_goal(space, reached_goal);
        if (verbose) {
          std::cout << "Found plan to " << state2vec(scoped_goal) << std::endl;
        }
        auto it = ik_solns.find(scoped_goal);
        if (it == ik_solns.end()) {
          std::ostringstream msg_builder;
          msg_builder << "Found path does not end in a known IK solution\n"
                      << "  " << state2vec(scoped_goal) << "\n"
                      << "known values:\n";
          for (auto kv : ik_solns) {
            msg_builder << "  " << state2vec(kv.first) << "\n";
          }
          std::cerr << msg_builder.str() << std::endl;
          throw std::runtime_error(msg_builder.str());
        }
        reached_goals.emplace_back(ik_solns.at(scoped_goal));
        ik_solns.erase(scoped_goal);
        break;
      }

      // ran out of time
      case ob::PlannerStatus::TIMEOUT:
      case ob::PlannerStatus::APPROXIMATE_SOLUTION: {
        break;
      }

      // something unexpected
      default: {
        std::string msg = "Unexpected solve status: " + solve_status.asString();
        std::cerr << msg << std::endl;
        throw std::runtime_error(msg);
      }
    }
  }

  return reached_goals;
}

std::shared_ptr<cpptoml::table> MultiGoalPlanningCostIK::to_toml() const {
  auto container = cpptoml::make_table();
  auto tbl = cpptoml::make_table();
  container->insert("multi_goal_problem", tbl);

  tbl->insert("start_config", cpptoml::to_toml(start_config));

  auto arr = cpptoml::make_array();
  for (auto &tip : this->goal_tips) {
    arr->push_back(cpptoml::to_toml(tip));
  }
  tbl->insert("goal_tips", arr);

  tbl->insert("tip_threshold", tip_threshold);
  tbl->insert("time_limit_seconds", time_limit_seconds);

  tbl->insert("ik_max_iters"                , ik_max_iters                );
  tbl->insert("ik_mu_init"                  , ik_mu_init                  );
  tbl->insert("ik_stop_threshold_JT_err_inf", ik_stop_threshold_JT_err_inf);
  tbl->insert("ik_stop_threshold_Dp"        , ik_stop_threshold_Dp        );
  tbl->insert("ik_finite_difference_delta"  , ik_finite_difference_delta  );
  tbl->insert("rrt_grow_distance"           , rrt_grow_distance           );
  tbl->insert("rrt_goal_bias"               , rrt_goal_bias               );

  auto subtbl = venv.to_toml();
  for (auto &[key, val] : *subtbl) {
    container->insert(key, val);
  }

  return container;
}

MultiGoalPlanningCostIK MultiGoalPlanningCostIK::from_toml(
    std::shared_ptr<cpptoml::table> tbl)
{
  if (!tbl) { throw std::invalid_argument("null table given"); }

  auto subtbl = tbl->get("multi_goal_problem")->as_table();
  if (!subtbl) {
    throw cpptoml::parse_exception(
        "Wrong type detected for 'multi_goal_problem': not a table");
  }

  auto start_config = subtbl->get("start_config")->as_array();
  auto goal_tips    = subtbl->get("goal_tips")->as_array();

  if (!(start_config && goal_tips)) {
    throw cpptoml::parse_exception("Wrong type detected");
  }

  // required fields
  MultiGoalPlanningCostIK cost;
  cost.start_config = cpptoml::to_stdvec<double>(start_config);
  for (auto val : goal_tips->get()) {
    auto arr = val->as_array();
    cost.goal_tips.emplace_back(cpptoml::to_point(arr));
  }

  // voxel environment
  if (tbl->contains("voxel_environment")) {
    cost.venv = motion_planning::VoxelEnvironment::from_toml(tbl);
  }

  // optional fields (all double precision types)
  std::map<std::string, double*> optional_fields;
  optional_fields["tip_threshold"               ] = &cost.tip_threshold               ;
  optional_fields["time_limit_seconds"          ] = &cost.time_limit_seconds          ;
  optional_fields["ik_mu_init"                  ] = &cost.ik_mu_init                  ;
  optional_fields["ik_stop_threshold_JT_err_inf"] = &cost.ik_stop_threshold_JT_err_inf;
  optional_fields["ik_stop_threshold_Dp"        ] = &cost.ik_stop_threshold_Dp        ;
  optional_fields["ik_finite_difference_delta"  ] = &cost.ik_finite_difference_delta  ;
  optional_fields["rrt_grow_distance"           ] = &cost.rrt_grow_distance           ;
  optional_fields["rrt_goal_bias"               ] = &cost.rrt_goal_bias               ;
  for (auto &[key, addr] : optional_fields) {
    if (subtbl->contains(key)) {
      auto entry = subtbl->get(key)->as<double>();
      if (!entry) {
        throw cpptoml::parse_exception("Wrong type detected for " + key);
      }
      *addr = entry->get();
    }
  }
  if (subtbl->contains("ik_max_iters")) {
    auto entry = subtbl->get("ik_max_iters")->as<int64_t>();
    if (!entry) {
      throw cpptoml::parse_exception("Wrong type detected for ik_max_iters");
    }
    cost.ik_max_iters = entry->get();
  }

  return cost;
}

} // end of namespace design_optimization

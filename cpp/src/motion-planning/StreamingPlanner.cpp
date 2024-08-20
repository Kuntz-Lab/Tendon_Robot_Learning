#include "motion-planning/StreamingPlanner.h"
#include "motion-planning/VoxelCachedLazyPRM.h"
#include "motion-planning/plan.h"
#include <util/openfile_check.h>

#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/goals/GoalStates.h>

namespace ob = ompl::base;

namespace motion_planning {

namespace {

void print_solve_status(ob::PlannerStatus::StatusType status) {
  using PS = ob::PlannerStatus;
  switch (status) {
    case PS::UNKNOWN:
      std::cerr << "Warning: unknown problem occurred in planning" << std::endl;
      break;
    case PS::INVALID_START:
      std::cerr << "Warning: invalid start" << std::endl;
      break;
    case PS::INVALID_GOAL:
      std::cerr << "Warning: invalid goal" << std::endl;
      break;
    case PS::UNRECOGNIZED_GOAL_TYPE:
      std::cerr << "Warning: unrecognized goal type" << std::endl;
      break;
    case PS::TIMEOUT:
      std::cerr << "Warning: timeout" << std::endl;
      break;
    case PS::APPROXIMATE_SOLUTION:
      std::cerr << "Warning: could only find approximate solution" << std::endl;
      break;
    case PS::EXACT_SOLUTION:
      std::cout << "Exact solution obtained" << std::endl;
      break;
    case PS::CRASH:
      std::cerr <<
        "Warning: motion-planner seems to have crashed your system" << std::endl;
      break;
    case PS::ABORT:
      std::cerr << "Warning: motion-planner was aborted." << std::endl;
      break;
    case PS::TYPE_COUNT:
      std::cerr << "Warning: motion-planner had a miscellaneous error." << std::endl;
      break;
  }
}

} // end of unnamed namespace

StreamingPlanner::StreamingPlanner(
    Problem *problem,
    const std::string &roadmap,
    QObject *parent,
    const std::string &logfile,
    size_t ik_neighbors,
    size_t ik_max_iters,
    double ik_tolerance,
    VoxelCachedLazyPRM::RoadmapIkOpts ikopt,
    bool check_vertex_validity,
    bool check_edge_validity,
    bool remove_disconnected)
  : QObject(parent)
  , _problem(problem)
  , _goal(0.0, 0.0, 0.0)
  , _last_goal(0.0, 0.0, 0.0)
  , _current_state(nullptr)
  , _planner(nullptr)
  , _ik_neighbors(ik_neighbors)
  , _ik_tolerance(ik_tolerance)
  , _ikopt(ikopt)
  , _logout()
  , _logger(nullptr)
  , _milestone_count(0)
  , _milestone_timer()
  , _ik_timer()
  , _solve_timer()
{
  if (!_problem) {
    throw std::invalid_argument("Must provide a valid problem pointer");
  }
  qRegisterMetaType<Problem::PlanType>("PlanType");

  util::openfile_check(_logout, logfile);
  _logger = std::make_unique<csv::CsvWriter>(_logout);
  *_logger << "name" << "goal_number" << "value";
  _logger->new_row();

  log_event("settings:roadmap", roadmap);
  log_event("settings:ik-neightbors", ik_neighbors);
  log_event("settings:ik-tolerance", ik_tolerance);
  log_event("settings:ik-max-iters", ik_max_iters);
  log_event("settings:log", logfile);
  log_event("settings:skip-roadmap-vertex-check", !check_vertex_validity);
  log_event("settings:skip-roadmap-edge-check", !check_edge_validity);
  log_event("settings:keep-disconnected-vertices", !remove_disconnected);
  log_event("settings:ik-auto-add", bool(ikopt & VoxelCachedLazyPRM::RMAP_IK_AUTO_ADD));
  log_event("settings:ik-accurate", bool(ikopt & VoxelCachedLazyPRM::RMAP_IK_ACCURATE));
  log_event("settings:ik-lazy-add", bool(ikopt & VoxelCachedLazyPRM::RMAP_IK_LAZY_ADD));

  _planner = std::dynamic_pointer_cast<motion_planning::VoxelCachedLazyPRM>(
      _problem->create_planner("VoxelCachedLazyPRM"));
  if (!_planner) {
    throw std::runtime_error(
        "Could not make an instance of motion_planning::VoxelCachedLazyPRM");
  }
  bool is_backbone = true;
  bool swept_volume = true;
  _problem->update_to_voxel_validators(_planner, is_backbone, swept_volume);
  std::cout << "\nLoading roadmap..." << std::endl;
  float timing;
  util::time_function_call(
      [this, &roadmap, &check_vertex_validity, &check_edge_validity]() {
        _planner->loadRoadmapFromFile(roadmap, check_vertex_validity,
                                      check_edge_validity);
      }, timing);
  log_event("time:loadRoadmapFromFile", timing);

  if (remove_disconnected) {
    util::time_function_call([this]() {
        _planner->clearDisconnectedVertices();
      }, timing);
    log_event("time:clearDisconnectedVertices", timing);
  }
  std::cout << "  done loading roadmap" << std::endl;

  _planner->setDefaultIkController(ik_max_iters, ik_tolerance);
}

StreamingPlanner::~StreamingPlanner() {}

//
// = public slots
//

void StreamingPlanner::update_goal(const QVector3D &goal) {
  _goal = goal;
  // queue calling streaming_replan() so that all update_goal() calls can be
  // finished before calling the first streaming_replan(), even though it is
  // from the same thread.
  QMetaObject::invokeMethod(this, "streaming_replan", Qt::QueuedConnection);
}

void StreamingPlanner::streaming_replan() {
  if (_goal != _last_goal) {
    emit planning_begin();
    auto plan = _milestone_timer.time(
        [this]() {
          return this->replan_impl(
              {this->_goal.x(), this->_goal.y(), this->_goal.z()});
        });
    log_milestone_event("time:milestone", _milestone_timer.get_times().back());
    emit new_plan(plan);
    emit planning_end();
  }
}

ob::State* StreamingPlanner::vec2newstate(
    const std::vector<double> &vec) const
{
  auto si    = _planner->getSpaceInformation();
  auto space = si->getStateSpace();
  auto state = si->allocState();
  space->copyFromReals(state, vec);
  space->enforceBounds(state);
  return state;
}

std::vector<double> StreamingPlanner::state2vec(ob::State* state) const {
  auto si    = _planner->getSpaceInformation();
  auto space = si->getStateSpace();
  std::vector<double> vec;
  space->copyToReals(vec, state);
  return vec;
}

// TODO: make this work for any type of planner, not just mine
motion_planning::Problem::PlanType StreamingPlanner::replan_impl(
    const Eigen::Vector3d &goal_tip)
{
  auto si    = _planner->getSpaceInformation();
  auto space = si->getStateSpace();
  auto pdef  = _planner->getProblemDefinition();
  auto checker = std::dynamic_pointer_cast<
      motion_planning::AbstractValidityChecker>(si->getStateValidityChecker());
  auto motion_validator = std::dynamic_pointer_cast<
      motion_planning::AbstractVoxelMotionValidator>(si->getMotionValidator());
  if (!checker) {
    throw std::runtime_error("State checker is not an AbstractValidityChecker");
  }
  if (!motion_validator) {
    throw std::runtime_error(
        "Motion validator is not an AbstractVoxelMotionValidator");
  }

  // should only happen the first time
  if (_current_state == nullptr) {
    _current_state = si->cloneState(pdef->getStartState(0));
  }

  _milestone_count += 1;

  // set the start state to the current configuration
  _planner->clearQuery();
  pdef->clearStartStates();
  pdef->addStartState(_current_state);

  // perform inverse kinematics
  std::cout << "\nRunning IK for [" << goal_tip.transpose() << "]" << std::endl;
  auto ik_results = _ik_timer.time([this, &goal_tip]
        () -> std::vector<VoxelCachedLazyPRM::IKResult>
      {
        auto ik = _planner->roadmapIk(
            goal_tip, _ik_tolerance, _ik_neighbors, _ikopt);
        // convert to a vector
        if (ik) { return {*ik}; }
        return {}; // empty vector
      });
  log_milestone_event("time:roadmapIk", _ik_timer.get_times().back());
  if (ik_results.size() == 0) {
    throw std::runtime_error("no IK results returned");
  }
  std::cout << "\nFor tip [" << goal_tip.transpose() << "], returned "
            << ik_results.size() << " ik results" << std::endl;

  // log the IK results
  log_milestone_event("ik:goal",
      std::to_string(goal_tip[0]) + " " + std::to_string(goal_tip[1]) + " "
      + std::to_string(goal_tip[2]));
  log_milestone_event("ik:count", ik_results.size());
  for (auto &result : ik_results) {
    std::ostringstream builder;
    bool first = true;
    for (auto &val : result.neighbor) {
      if (!first) { builder << " "; }
      first = false;
      builder << val;
    }
    log_milestone_event("ik:neighbor", builder.str());

    builder.str("");
    first = true;
    for (auto &val : result.controls) {
      if (!first) { builder << " "; }
      first = false;
      builder << val;
    }
    log_milestone_event("ik:solution", builder.str());
    log_milestone_event("ik:tip-error", result.error);
  }

  // set the goal to the found IK locations
  auto goals(std::make_shared<ob::GoalStates>(si));
  for (auto &result : ik_results) {
    auto goal = vec2newstate(result.controls);
    goals->addState(goal);
  }
  pdef->setGoal(goals);

  // run the planner
  auto solve_status = _solve_timer.time(
      [this]() { return this->_planner->solveWithRoadmap(); });
  log_milestone_event("time:solveWithRoadmap", _solve_timer.get_times().back());
  print_solve_status(solve_status);
  log_milestone_event("status:solveWithRoadmap", solve_status.asString());

  log_and_clear_timers(_planner);
  log_and_clear_timers(checker);
  log_and_clear_timers(motion_validator);

  if (solve_status != ob::PlannerStatus::EXACT_SOLUTION) {
    log_milestone_event("fail:milestone", 0);
    //throw std::runtime_error("could not reach goal from IK using the roadmap");
    OMPL_ERROR("could not reach goal from IK using the roadmap");
    auto current = state2vec(_current_state);
    return {current, current};
  }
  auto local_plan = motion_planning::get_solution(_planner);

  // log the tip error
  auto fk_shape = _problem->robot.forward_kinematics(local_plan.back());
  log_milestone_event("solution:tip-error",
                      (goal_tip - fk_shape.back()).norm());
  log_milestone_event("solution:waypoints", local_plan.size());
  log_milestone_event("solution:cost",
                      motion_planning::solution_cost(_planner).value());

  if (local_plan.size() == 0) {
    throw std::runtime_error("local plan is empty");
  }

  _current_state = vec2newstate(local_plan.back());
  _last_goal = QVector3D{float(goal_tip[0]), float(goal_tip[1]), float(goal_tip[2])};

  return local_plan;
}

} // end of namespace motion_planning

#include "cliparser/CliParser.h"
#include "cpptoml/cpptoml.h"
#include "cpptoml/toml_conversions.h"
#include "csv/Csv.h"
#include "motion-planning/Problem.h"
#include "util/openfile_check.h"
#include "util/time_function_call.h"
#include "util/vector_ops.h"

#include <ompl/base/PlannerStatus.h>
#include <ompl/base/goals/GoalState.h>

#include <boost/filesystem.hpp>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <cmath>
#include <cstdio>

using util::operator<<;

namespace bfs = boost::filesystem;
namespace ob = ompl::base;

namespace {

namespace defaults {
  const std::string directory = "tasks";
  const int number = 1;
  const double timeout = 120;
} // end of namespace defaults

void populate_parser(CliParser &parser) {
  parser.set_program_description(
      "Generate non-trivial tasks for the given problem.  A non-trivial task\n"
      "  is a different start and goal configuration that can be solved with a\n"
      "  motion planner but does not have a straight line solution.");

  parser.add_positional("problem");
  parser.set_required("problem");
  parser.set_description("problem", "The problem specification toml file to use");

  parser.add_argflag("-d", "--directory");
  parser.set_description("--directory",
                      "Directory to store the generated tasks.  They will be\n"
      "                generated as 'task-0001.toml' with the numbers\n"
      "                increasing.  Note, this will not overwrite existing\n"
      "                files in that directory.\n"
      "                (default is '" + defaults::directory + "')");

  parser.add_argflag("-N", "--number");
  parser.set_description("--number",
                      "Number of tasks to create, each becoming a separate\n"
      "                problem file.\n"
      "                (default is " + std::to_string(defaults::number) + ")");

  parser.add_argflag("-t", "--timeout");
  parser.set_description("--timeout",
                      "Seconds to give each potential task to be solved with\n"
      "                RRT Connect before rejecting it to sample another task.\n"
      "                Longer timeouts refer to the possability of harder\n"
      "                problems being accepted.\n"
      "                (default is " + std::to_string(defaults::timeout) + ")");

  parser.add_flag("--voxel");
  parser.set_description("--voxel",
                      "Enable the use of voxels instead of the\n"
      "                Environment class.  This will ignore the [environment]\n"
      "                section of the toml file and will instead use the\n"
      "                [voxel_environment] section which expects a voxel\n"
      "                image file as the only obstacle.\n"
      "                It is assumed the voxel environment has already been\n"
      "                scaled and translated to be in the robot frame.\n"
      "                But, the rotation will be applied to the robot before\n"
      "                collision checking.");

  parser.add_flag("--backbone");
  parser.set_description("--backbone",
                      "Only applicable if --voxel is specified.\n"
      "                Only collision check against the backbone path of the\n"
      "                tendon robot instead of the full robot with radius\n"
      "                thickness.");

  parser.add_flag("--swept-volume");
  parser.set_description("--swept-volume",
                      "Only applicable if --voxel is specified.\n"
      "                Use the generated swept volume instead of the default\n"
      "                motion validator for the motion-planning edge\n"
      "                checker.  Also, can only be used with --backbone\n"
      "                since swept volume of full robot is not implemented.");
}

// tries to create a new empty file.  Returns true on success.
bool touch_new(const std::string fname) {
  // "x" in std::fopen() means only make new file (C++17)
  auto fd = std::fopen(fname.c_str(), "wx");
  if (fd != nullptr) { std::fclose(fd); }
  return fd != nullptr;
}

// opens the next file name in the directory, returning the name
bfs::path next_file(std::ofstream &out, const bfs::path &directory,
                    const std::string filebase = "task",
                    const std::string extension = "toml")
{
  bfs::path fname;
  char suffix[10];
  bool success = false;
  for (int i = 1; i < 10000 ; i++) {
    std::snprintf(suffix, 8, "-%04d.", i);
    fname = directory / (filebase + suffix + extension);
    if (touch_new(fname.native())) { success = true; break; }
  }
  if (!success) {
    throw std::ios_base::failure("Unable to create task file");
  }
  util::openfile_check(out, fname.native());
  return fname;
}

auto try_planner(ob::PlannerPtr planner, double timeout_secs) {
  float timing;
  auto solve_status = util::time_function_call(
      [&planner, &timeout_secs]() {
        return planner->solve(timeout_secs);
      }, timing);
  bool is_solvable = (solve_status == ob::PlannerStatus::EXACT_SOLUTION);

  return std::make_pair(is_solvable, timing);
}

ob::PlannerPtr create_planner(const motion_planning::Problem &problem,
                              const std::string &planner_name,
                              bool voxel, bool backbone,
                              bool swept_volume)
{
  auto planner = problem.create_planner(planner_name);
  if (voxel) {
    problem.update_to_voxel_validators(planner, backbone, swept_volume);
  }
  return planner;
}

ob::PlannerPtr create_trivial_planner(const motion_planning::Problem &problem,
                                      bool voxel, bool backbone,
                                      bool swept_volume)
{
  return create_planner(problem, "StraightLinePlanner", voxel, backbone,
                        swept_volume);
}

ob::PlannerPtr create_rrt_connect_planner(
    const motion_planning::Problem &problem,
    bool voxel, bool backbone, bool swept_volume)
{
  return create_planner(problem, "RRTConnect", voxel, backbone, swept_volume);
}

double config_distance(const motion_planning::Problem &problem) {
  auto planner = problem.create_planner();
  auto si      = planner->getSpaceInformation();
  auto pdef    = planner->getProblemDefinition();
  auto start   = pdef->getStartState(0);
  auto goal    = pdef->getGoal()->as<ob::GoalState>()->getState();
  return si->distance(start, goal);
}

void randomize_start_and_goal(motion_planning::Problem &problem,
                              bool voxel, bool backbone) {

  // sample valid start and goal configs
  std::vector<double> start_config, goal_config;
  {
    auto dummy_planner  = create_trivial_planner(problem, voxel, backbone, false);
    auto si             = dummy_planner->getSpaceInformation();
    auto space          = si->getStateSpace();
    auto sampler        = si->allocValidStateSampler();
    auto state          = si->allocState();
    sampler->sample(state);
    space->copyToReals(start_config, state);
    sampler->sample(state);
    space->copyToReals(goal_config, state);
    si->freeState(state);
  }

  auto N                     = problem.robot.tendons.size();
  problem.start              = std::vector<double>(start_config.begin(),
                                                   start_config.begin() + N);
  problem.goal               = std::vector<double>(goal_config.begin(),
                                                   goal_config.begin() + N);
  if (problem.robot.enable_rotation) {
    problem.start_rotation   = start_config[N];
    problem.goal_rotation    = goal_config[N];
  }
  if (problem.robot.enable_retraction) {
    problem.start_retraction = start_config.back();
    problem.goal_retraction  = goal_config.back();
  }
}

} // end of unnamed namespace

int main(int argCount, char* argList[]) {
  CliParser parser;
  populate_parser(parser);
  parser.parse(argCount, argList);

  std::string problem_file = parser["problem"];
  std::string directory    = parser.get("--directory", defaults::directory);
  int N                    = parser.get("--number", defaults::number);
  double timeout_secs      = parser.get("--timeout", defaults::timeout);
  bool voxel               = parser.has("--voxel");
  bool backbone            = parser.has("--backbone");
  bool swept_volume        = parser.has("--swept-volume");

  std::cout << "Loading '" << problem_file << "'" << std::endl;
  auto problem = cpptoml::from_file<motion_planning::Problem>(problem_file);


  bfs::create_directories(directory);
  std::ofstream logfile;
  auto log_fname = next_file(logfile, directory, "log", "csv");
  csv::CsvWriter log_writer(logfile);
  logfile << std::setprecision(std::numeric_limits<double>::digits10 + 1);

  log_writer << "fname"
             << "type"
             << "time"
             << "config-distance"
             << "tip-distance"
             << "problem"
             << "voxel"
             << "backbone"
             << "swept-volume"
             << "start_x" << "start_y" << "start_z"
             << "goal_x" << "goal_y" << "goal_z";
  for (size_t i = 0; i < problem.robot.tendons.size(); ++i) {
    log_writer << "start_tau_" + std::to_string(i);
  }
  if (problem.robot.enable_rotation)   { log_writer << "start_rotation"; }
  if (problem.robot.enable_retraction) { log_writer << "start_retraction"; }
  for (size_t i = 0; i < problem.robot.tendons.size(); ++i) {
    log_writer << "goal_tau_" + std::to_string(i);
  }
  if (problem.robot.enable_rotation)   { log_writer << "goal_rotation"; }
  if (problem.robot.enable_retraction) { log_writer << "goal_retraction"; }
  log_writer.new_row();

  // convenience lambda for writing to the log file
  auto write_row = [&log_writer, &problem, problem_file, voxel, backbone, swept_volume]
                   (const auto &fname, const std::string &type, double time)
  {
    auto start_tip = problem.start_shape().p.back();
    auto goal_tip  = problem.goal_shape ().p.back();
    auto dist      = (goal_tip - start_tip).norm();

    log_writer << fname.filename().native()
               << type
               << time
               << config_distance(problem)
               << dist
               << problem_file
               << voxel
               << backbone
               << swept_volume
               << start_tip[0] << start_tip[1] << start_tip[2]
               <<  goal_tip[0] <<  goal_tip[1] <<  goal_tip[2];
    for (auto &val : problem.start)       { log_writer << val; }
    if  (problem.robot.enable_rotation)   { log_writer << problem.start_rotation; }
    if  (problem.robot.enable_retraction) { log_writer << problem.start_retraction; }
    for (auto &val : problem.goal)        { log_writer << val; }
    if  (problem.robot.enable_rotation)   { log_writer << problem.goal_rotation; }
    if  (problem.robot.enable_retraction) { log_writer << problem.goal_retraction; }
    log_writer.new_row();
  };

  enum class SolnType {
    ST_TRIVIAL,
    ST_UNSOLVED,
    ST_SOLVED,
  };
  auto output_result = [&directory, &write_row, &timeout_secs]
                       (SolnType type, const auto &prob, double time)
  {
    const char* fbase = nullptr;
    const char* typestr = nullptr;
    switch (type) {
      case SolnType::ST_TRIVIAL:
        fbase = "trivial";
        typestr = "trivial";
        std::cerr << "  trivially solvable task, skipping\n";
        break;
      case SolnType::ST_UNSOLVED:
        fbase = "not-solved";
        typestr = "unsolved";
        std::cerr << "  is not solvable by RRT Connect in " << timeout_secs
          << " seconds\n";
        break;
      case SolnType::ST_SOLVED:
        fbase = "task";
        typestr = "solved";
        break;
    }
    std::ofstream out;
    auto fname = next_file(out, directory, fbase);
    std::cout << "  Writing " << fname << "\n\n" << std::flush;
    cpptoml::to_stream(out, prob.to_toml());
    write_row(fname, typestr, time);
  };

  auto N_remaining = N;
  while (N_remaining > 0) {
    // only decrement N_remaining when we actually solve for the solution
    auto N_run = N_remaining;
    #pragma omp parallel for schedule(dynamic, 1) firstprivate(problem)
    for (int i = 0; i < N_run; i++) {
      // randomly sample two sets of configurations
      // if not in collision
      // and if not solvable by a straight line in configuration space
      // and if solvable by RRT within the time limit
      // then save it, else resample
      SolnType type;
      double solve_secs = 0.0;
      randomize_start_and_goal(problem, voxel, backbone);
      auto planner = create_trivial_planner(problem, voxel, backbone,
                                            swept_volume);
      auto [is_trivial, trivial_secs] = try_planner(planner, timeout_secs);
      if (is_trivial) {
        type = SolnType::ST_TRIVIAL;
        solve_secs = trivial_secs;
      } else {
        auto planner = create_rrt_connect_planner(problem, voxel, backbone,
                                                  swept_volume);
        auto [is_solvable_by_rrt, rrt_time] =
            try_planner(planner, timeout_secs);
        if (is_solvable_by_rrt) {
          type = SolnType::ST_SOLVED;
          solve_secs = rrt_time;
        } else {
          type = SolnType::ST_UNSOLVED;
          solve_secs = timeout_secs;
        }
      }

      #pragma omp critical
      {
        if (type == SolnType::ST_SOLVED) { N_remaining--; }
        output_result(type, problem, solve_secs);
      }
    }
  }

  return 0;
}

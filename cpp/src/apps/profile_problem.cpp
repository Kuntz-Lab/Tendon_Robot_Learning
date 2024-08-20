#include <cliparser/CliParser.h>
#include <cpptoml/toml_conversions.h>
#include <csv/Csv.h>
#include <motion-planning/Problem.h>
#include <motion-planning/ValidityChecker.h>
#include <motion-planning/VoxelCachedLazyPRM.h>
#include <motion-planning/VoxelEnvironment.h>
#include <motion-planning/plan.h>
#include <util/FunctionTimer.h>
#include <util/ompl_logging.h>
#include <util/openfile_check.h>
#include <util/vector_ops.h>

#include <3rdparty/nlohmann/json.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/util/Time.h>

#include <Eigen/Core>

#include <boost/filesystem.hpp>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace ob    = ompl::base;
namespace otime = ompl::time;
namespace bfs = boost::filesystem;


namespace {

namespace defaults {
  const std::string ompl_log_level = "DEBUG";
  const std::string planner_name = "RRTConnect";
  const std::string output = "profile.csv";
  const int number = 3;
  const double timeout = 10; // seconds
} // end of namespace defaults

void populate_parser(CliParser &parser) {
  parser.set_program_description(
      "Runs a planner multiple times to get runtime statistics.\n"
      "  A csv file is generated with the following columns:\n"
      "\n"
      "    - problem: the problem file (from the command-line)\n"
      "    - planner: name of the planner (from --planner-name)\n"
      "    - runs: number of runs of the problem (from --number)\n"
      "    - timeout-time: seconds for timeout (from --timeout)\n"
      "    - timeouts: number of the runs that timed out\n"
      "\n"
      "  Then the rest of the columns each have a min, mean, median, and max\n"
      "\n"
      "    - cost: cost of a found path\n"
      "    - plan: time taken for a found path\n"
      "    - fk-calls: number of forward-kinematics calls (including timeouts)\n"
      "    - fk-time: time of a single forward kinematics call (including timeouts)\n"
      "    - collision-calls: number of collision checks (including timeouts)\n"
      "    - collision-time: time for a collision check (including timeouts)\n"
      "\n"
      "  There may be other rows based on additional timing done by the chosen\n"
      "  state validator.  Thes will be reported with statistics on the number\n"
      "  of calls and the times.\n"
      "\n"
      "  If you give multiple problem files, there will be one row per problem\n"
      "  file.\n"
      );

  parser.add_positional("problems");
  parser.set_required("problems");
  parser.set_description("problems", "one or more problem toml file");

  parser.add_argflag("-P", "--planner-name");
  parser.set_description("--planner-name", "Name of the panner to use."
                    "  Run 'query_planner --list-planners'\n"
      "                for available planners.\n"
      "                (default is " + defaults::planner_name + ")");

  parser.add_argflag("-o", "--output");
  parser.set_description("--output", "Where to output the stats as a CSV file\n"
      "                (default is " + defaults::output + ")");

  parser.add_argflag("-N", "--number");
  parser.set_description("--number", "Number of times to run each problem\n"
      "                (default is " + std::to_string(defaults::number) + ")");

  parser.add_argflag("-t", "--timeout");
  parser.set_description("--timeout", "Timeout for planning in seconds\n"
      "                (default is " + std::to_string(defaults::timeout) + ")");

  parser.add_flag("-v", "--voxel");
  parser.set_description("--voxel", "Enable the use of voxels instead of the\n"
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

  parser.add_argflag("--ompl-log-level");
  parser.set_description("--ompl-log-level",
                      "Set the log level used in OMPL.  Choices (in order of\n"
      "                most to least verbose) are 'DEV2', 'DEV1', 'DEBUG',\n"
      "                'INFO', 'WARN', 'ERROR', 'NONE'.\n"
      "                (default is '" + defaults::ompl_log_level + "')");

  parser.add_argflag("--roadmap-file");
  parser.set_description("--roadmap-file",
                      "Load the PRM roadmap from the given file.  Only\n"
      "                applicable if using the planner named\n"
      "                VoxelCachedLazyPRM.\n"
      "                Supports toml, toml.gz, json, bson, cbor, msgpack,\n"
      "                and ubjson.");

  // separate out into skipping vertex check and skipping edge check
  parser.add_flag("--skip-roadmap-vertex-check");
  parser.set_description("--skip-roadmap-vertex-check",
                      "Skip the step of checking for vertex collisions at\n"
      "                the time of loading the roadmap.  This causes\n"
      "                vertices to be checked during planning lazily.");

  parser.add_flag("--skip-roadmap-edge-check");
  parser.set_description("--skip-roadmap-edge-check",
                      "Skip the step of checking for edge collisions at\n"
      "                the time of loading the roadmap.  This causes\n"
      "                edges to be checked during planning lazily.");

  parser.add_argflag("--plan-dir");
  parser.set_description("--plan-dir",
                      "Directory to output the generated plans (with\n"
      "                multiple executions, newer ones overwrite older\n"
      "                ones).  If this is not specified, then plans are not\n"
      "                output to file.");

  // TODO: add --optimize to allow optimizing planners to use the whole time
  // TODO: add --planner-options
}

struct ParsedArgs {
  std::vector<std::string> problems;
  std::string planner_name;
  std::string output;
  int number;
  double timeout;
  bool voxel;
  bool backbone;
  bool swept_volume;
  std::string ompl_log_level;
  bool has_roadmap_file;
  std::string roadmap_file;
  bool check_vert_validity;
  bool check_edge_validity;
  bool has_plan_dir;
  std::string plan_dir;
};

// parse the arguments and return parsed args
ParsedArgs parse_args(CliParser &parser, int argCount, char* argList[]) {
  parser.parse(argCount, argList);
  ParsedArgs args;

  args.problems.emplace_back(parser["problems"]);
  auto &remaining = parser.remaining();
  args.problems.insert(args.problems.end(), remaining.begin(), remaining.end());

  args.planner_name     = parser.get("--planner-name", defaults::planner_name);
  args.output           = parser.get("--output", defaults::output);
  args.number           = parser.get("--number", defaults::number);
  args.timeout          = parser.get("--timeout", defaults::timeout);
  args.voxel            = parser.has("--voxel");
  args.backbone         = parser.has("--backbone");
  args.swept_volume     = parser.has("--swept-volume");
  args.ompl_log_level   = parser.get("--ompl-log-level", defaults::ompl_log_level);
  args.has_roadmap_file = parser.has("--roadmap-file");
  args.roadmap_file     = parser.get("--roadmap-file", std::string(""));
  args.check_vert_validity = !parser.has("--skip-roadmap-vertex-check");
  args.check_edge_validity = !parser.has("--skip-roadmap-edge-check");
  args.has_plan_dir     = parser.has("--plan-dir");
  args.plan_dir         = parser.get("--plan-dir", std::string(""));

  if (args.backbone && !args.voxel) {
    auto msg = "Error: --backbone can only be used with --voxel";
    std::cerr << msg << std::endl;
    throw CliParser::ParseError(msg);
  }

  if (args.swept_volume && !args.voxel) {
    auto msg = "Error: --swept-volume can only be used with --voxel";
    std::cerr << msg << std::endl;
    throw CliParser::ParseError(msg);
  }

  if (args.swept_volume && !args.backbone) {
    auto msg = "Error: --swept-volume only supports --backbone mode";
    std::cerr << msg << std::endl;
    throw CliParser::ParseError(msg);
  }

  if (args.has_roadmap_file && args.planner_name != "VoxelCachedLazyPRM") {
    auto msg = "Error: --roadmap-file can only be used with --planner-name "
               "VoxelCachedLazyPRM";
    std::cerr << msg << std::endl;
    throw CliParser::ParseError(msg);
  }

  return args;
}

double plan_cost(ob::PlannerPtr planner) {
  auto pdef = planner->getProblemDefinition();
  auto opt = pdef->getOptimizationObjective();
  auto path = pdef->getSolutionPath();
  return path->cost(opt).value();
}

} // end of unnamed namspace

int main(int argCount, char* argList[]) {
  CliParser parser;
  populate_parser(parser);
  auto args = parse_args(parser, argCount, argList);

  if (args.has_plan_dir) {
    bfs::create_directories(args.plan_dir);
  }

  ompl::RNG::setSeed(42);

  util::set_ompl_log_level(args.ompl_log_level);

  std::ofstream out;
  util::openfile_check(out, args.output);
  csv::CsvWriter writer(out);

  std::vector<std::string> header {
    "problem",
    "planner",
    "runs",
    "timeout-time",
    "timeouts",
    "cost-min",
    "cost-mean",
    "cost-median",
    "cost-max",
    "setup-time-min",
    "setup-time-mean",
    "setup-time-median",
    "setup-time-max",
    "plan-time-min",
    "plan-time-mean",
    "plan-time-median",
    "plan-time-max",
  };
  //writer.write_row(header);  // done later after finishing the header

  auto before_time = otime::now();

  using Elem = std::variant<int, double, std::string>;
  using Row = std::unordered_map<std::string, Elem>;
  auto write_row = [&writer, &header](const Row &new_row) {
    for (auto hval : header) {
      if (new_row.count(hval) == 0) {
        std::cerr << "Row does not have key '" << hval << "'" << std::endl;
      }
      std::visit([&writer](auto &&val) { writer << val; }, new_row.at(hval));
    }
    writer.new_row();
  };

  bool first = true;
  std::vector<std::string> timer_names;
  for (size_t p = 0; p < args.problems.size(); p++) {
    auto &probfile = args.problems[p];
    Row row;
    row["problem"]      = probfile;
    row["planner"]      = args.planner_name;
    row["runs"]         = args.number;
    row["timeout-time"] = args.timeout;

    auto problem = cpptoml::from_file<motion_planning::Problem>(probfile);

    // run planner "--number" times while capturing raw data
    util::FunctionTimer setup_timer;
    util::FunctionTimer plan_timer;
    plan_timer.enable();
    std::vector<double> costs;
    std::unordered_map<std::string, std::vector<int>> calls;
    std::unordered_map<std::string, std::vector<float>> timing;
    int n_timeouts = 0;
    for (decltype(args.number) i = 0; i < args.number; i++) {
      auto current_timing = otime::seconds(otime::now() - before_time);
      std::cout << "\r\e[K\r" // clear output line
        << probfile << " "
        << "(file " << p+1 << " of " << args.problems.size() << ")"
        << ":  run " << i+1 << " of " << args.number
        << "  (" << current_timing << " sec)"
        << std::flush;

      auto planner = setup_timer.time([&args, &problem]() {
            auto planner = problem.create_planner(args.planner_name, {}, true);
            if (args.voxel) {
              problem.update_to_voxel_validators(planner, args.backbone,
                                                 args.swept_volume);
            }
            if (args.has_roadmap_file) {
              auto vplanner = std::dynamic_pointer_cast<
                  motion_planning::VoxelCachedLazyPRM>(planner);
              if (vplanner) {
                vplanner->loadRoadmapFromFile(args.roadmap_file,
                                              args.check_vert_validity,
                                              args.check_edge_validity);
              } else {
                std::string msg = "Error: planner is not a VoxelCachedLazyPRM,"
                                  " cannot load roadmap";
                std::cerr << msg << std::endl;
                throw std::runtime_error(msg);
              }
            }
            return planner;
          });
      // vplanner will be set only if we are using my planner
      auto vplanner = std::dynamic_pointer_cast<
          motion_planning::VoxelCachedLazyPRM>(planner);

      auto si = planner->getSpaceInformation();
      auto checker = std::dynamic_pointer_cast<
          motion_planning::AbstractValidityChecker>(
            si->getStateValidityChecker());
      auto motion_validator = std::dynamic_pointer_cast<
          motion_planning::AbstractVoxelMotionValidator>(
            si->getMotionValidator());

      // Dynamically determine the header based on the timers present in the
      // checker, but only on the first run of the first problem.
      // Also write out the header to the output
      if (first) {
        auto add_header_stat = [&header] (const std::string &name) {
          header.emplace_back(name + "-min");
          header.emplace_back(name + "-mean");
          header.emplace_back(name + "-median");
          header.emplace_back(name + "-max");
        };
        auto add_header_timers =
          [&add_header_stat, &timer_names, &calls, &timing](auto &timers) {
            for (const auto &[name, timer] : timers) {
              UNUSED_VAR(timer);
              timer_names.emplace_back(name);
              calls[name] = {};
              timing[name] = {};
              add_header_stat(name + "-calls");
              add_header_stat(name + "-time");
            }
          };
        add_header_timers(checker->timers());
        if (motion_validator) {
          add_header_timers(motion_validator->timers());
          timer_names.emplace_back("voxelize-errors");
          calls["voxelize-errors"] = {};
          add_header_stat("voxelize-errors-calls");
        }
        if (vplanner) {
          add_header_timers(vplanner->timers());
        }
        writer.write_row(header);
      }
      first = false;

      // run planner
      auto solve_status = plan_timer.time(
          [&planner, &args]() { return planner->solve(args.timeout); });
      if (solve_status != ob::PlannerStatus::EXACT_SOLUTION) {
        n_timeouts++;
      } else {
        costs.emplace_back(plan_cost(planner));
      }

      if (args.has_plan_dir) {
        auto plan = motion_planning::get_solution(planner);
        std::ofstream fout;
        std::ostringstream fname_builder;
        char prefix_buffer[8];
        std::snprintf(prefix_buffer, 6, "%04lu", p + 1); // zero-padded
        fname_builder << args.plan_dir << "/" << prefix_buffer << "-plan.csv";
        util::openfile_check(fout, fname_builder.str());
        std::cout << "writing plan to " << fname_builder.str() << std::endl;
        problem.write_plan(fout, plan);
      }

      auto capture_timers = [&calls, &timing](auto &timers) {
        for (const auto &[name, timer] : timers) {
          calls[name].emplace_back(timer.get_times().size());
          const auto &subtiming = timer.get_times();
          timing[name].insert(timing[name].end(),
                              subtiming.begin(), subtiming.end());
        }
      };
      capture_timers(checker->timers());
      if (motion_validator) {
        capture_timers(motion_validator->timers());
        calls["voxelize-errors"].emplace_back(
            motion_validator->num_voxelize_errors());
      }
      if (vplanner) {
        capture_timers(vplanner->timers());
      }
    }
    row["timeouts"] = n_timeouts;

    auto add_stats = [&row](auto vec, const std::string &base) {
      auto stats = util::calc_stats(vec);
      row[base + "-min"]    = stats.min;
      row[base + "-mean"]   = stats.mean;
      row[base + "-median"] = stats.median;
      row[base + "-max"]    = stats.max;
    };

    add_stats(setup_timer.get_times(),  "setup-time"     );
    add_stats(plan_timer.get_times(),   "plan-time"      );
    add_stats(costs,                    "cost"           );
    for (const auto &name : timer_names) {
      if (calls.find(name) != calls.end()) {
        add_stats(calls[name],          name + "-calls"  );
      }
      if (timing.find(name) != timing.end()) {
        add_stats(timing[name],         name + "-time"   );
      }
    }

    write_row(row);
  }

  auto current_timing = otime::seconds(otime::now() - before_time);
  std::cout << "\ndone (" << current_timing << " sec)" << std::endl;

  return 0;
}

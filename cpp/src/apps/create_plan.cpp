#include "cliparser/CliParser.h"
#include "collision/CapsuleSequence.h"
#include "collision/Sphere.h"
#include "cpptoml/toml_conversions.h"
#include "motion-planning/plan.h"
#include "motion-planning/ompl_planners.h"
#include "motion-planning/VoxelCachedLazyPRM.h"
#include "tendon/TendonSpecs.h"
#include "util/ompl_logging.h"

#include <Eigen/Core>

namespace {

namespace defaults {
  const std::string ompl_log_level = "DEBUG";
  const double timeout = 10; // seconds
  const std::string planner_name = "RRTConnect";
  const std::string output_basename = "solution";
} // end of namespace defaults

void populate_parser(CliParser &parser) {
  parser.set_program_description(
      "Run a planner and view the found tendon plan in RViz");

  // TODO: add --planner-options

  parser.add_argflag("--default-problem");
  parser.set_description("--default-problem",
      "Create a file containing the default problem specification");

  parser.add_argflag("-p", "--problem");
  parser.set_description("--problem",
                      "Specify the problem specification to use\n"
      "                (structured like the --default-problem)");

  parser.add_argflag("-P", "--planner-name");
  parser.set_description("--planner-name", "Name of the planner to use."
                    "  See -L for choices.\n"
      "                (default is '" + defaults::planner_name + "').");

  parser.add_argflag("-t", "--timeout");
  parser.set_description("--timeout", "Planning timeout in seconds\n"
      "                (default is " + std::to_string(defaults::timeout)
                        + " seconds).");

  parser.add_argflag("-o", "--output-basename");
  parser.set_description("--output-basename",
                      "Base of the plan's output filename.\n"
      "                For --optimize, adds '-1.csv' to the first solution and\n"
      "                '-2.csv' for the second solution.  Without specifying\n"
      "                --optimize, will simply add '.csv' to the end of this\n"
      "                filepath.\n"
      "                (default is '" + defaults::output_basename + "')");

  parser.add_flag("-L", "--list-planners");
  parser.set_description("--list-planners", "List available planners and exit");

  parser.add_flag("--optimize");
  parser.set_description("--optimize",
                      "If the given planner is an anytime algorithm,\n"
      "                then use the entire timeout to try to improve the plan.");

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
}

void parse_args(CliParser &parser, int argCount, char* argList[]) {
  parser.parse(argCount, argList);

  if (parser.has("--problem") && parser.has("--default-problem")) {
    throw CliParser::ParseError(
        "Cannot specify both --problem and --default-problem");
  }
  if (!parser.remaining().empty()) {
    throw CliParser::ParseError(
        "Unrecognized argument: " + parser.remaining()[0]);
  }
}

motion_planning::Problem load_problem(const std::string &toml_file) {
  return cpptoml::from_file<motion_planning::Problem>(toml_file);
}

motion_planning::Problem default_problem() {
  motion_planning::Problem problem;

  problem.robot.r = 0.015;

  tendon::TendonSpecs current_tendon {Eigen::VectorXd(2),
                                      Eigen::VectorXd(1)};
  current_tendon.C <<        0.0       ,   0.0;
  current_tendon.D <<        0.01;
  problem.robot.tendons.push_back(current_tendon);
  current_tendon.C <<  2.0 * M_PI / 3.0,   0.0;
  current_tendon.D <<        0.01;
  problem.robot.tendons.push_back(current_tendon);
  current_tendon.C << -2.0 * M_PI / 3.0,   0.0;
  current_tendon.D <<        0.01;
  problem.robot.tendons.push_back(current_tendon);
  current_tendon.C <<        M_PI / 2.0,  20.0;
  current_tendon.D <<        0.01;
  problem.robot.tendons.push_back(current_tendon);
  current_tendon.C <<       -M_PI / 2.0, -20.0;
  current_tendon.D <<        0.01;
  problem.robot.tendons.push_back(current_tendon);

  problem.env.push_back(collision::Sphere{{ 0,    0,   0.18}, 0.08});
  problem.env.push_back(collision::Sphere{{ 0.1,  0,   0   }, 0.05});
  problem.env.push_back(collision::Sphere{{-0.1,  0,   0   }, 0.05});
  problem.env.push_back(collision::Sphere{{ 0,    0.1, 0   }, 0.05});
  problem.env.push_back(collision::Sphere{{ 0,   -0.1, 0   }, 0.05});
  problem.env.push_back(collision::Sphere{{-0.1,  0.1, 0.1 }, 0.05});

  problem.start = {20.0,   0.0,  0.0,  0.0,  0.0};
  problem.goal  = { 0.0,  10.0, 10.0,  0.0, 10.0};

  return problem;
}

} // end of unnamed namespace

int main(int argCount, char* argList[]) {
  CliParser parser;
  populate_parser(parser);

  try {
    parse_args(parser, argCount, argList);
  } catch (CliParser::ParseError &ex) {
    std::cerr << ex.what() << std::endl;
    return -1;
  }

  if (parser.has("--list-planners")) {
    for (auto planner : motion_planning::available_planners()) {
      std::cout << planner << "\n";
    }
    return 0;
  }

  auto timeout         = parser.get("--timeout", defaults::timeout);
  auto planner_name    = parser.get("--planner-name", defaults::planner_name);
  auto optimize        = parser.has("--optimize");
  auto output_basename = parser.get("--output-basename",
                                    defaults::output_basename);
  auto ompl_log_level  = parser.get("--ompl-log-level",
                                    defaults::ompl_log_level);
  bool has_roadmap     = parser.has("--roadmap-file");
  auto roadmap_file    = parser.get("--roadmap-file", std::string{});
  auto check_vert_validity = !parser.has("--skip-roadmap-vertex-check");
  auto check_edge_validity = !parser.has("--skip-roadmap-edge-check");

  util::set_ompl_log_level(ompl_log_level);

  // define problem
  motion_planning::Problem problem;
  if (parser.has("--problem")) {
    problem = load_problem(parser["--problem"]);
  } else {
    problem = default_problem();
  }

  if (parser.has("--default-problem")) {
    cpptoml::to_file(problem, parser["--default-problem"]);
    std::cout << "Wrote default problem to " << parser["--default-problem"]
              << std::endl;
    return 0;
  }

  auto planner = problem.create_planner(planner_name);
  if (parser.has("--voxel")) {
    problem.update_to_voxel_validators(planner, parser.has("--backbone"),
                                       parser.has("--swept-volume"));
  }
  if (has_roadmap) {
    auto vplanner = std::dynamic_pointer_cast<
        motion_planning::VoxelCachedLazyPRM>(planner);
    if (vplanner) {
      vplanner->loadRoadmapFromFile(roadmap_file,
                                    check_vert_validity,
                                    check_edge_validity);
    } else {
      std::string msg = "Error: planner is not a VoxelCachedLazyPRM,"
                        " cannot load roadmap";
      std::cerr << msg << std::endl;
      throw std::runtime_error(msg);
    }
  }
  motion_planning::WritePlanFunc write_plan = [&problem]
    (const std::string fname, const motion_planning::Problem::PlanType &path) {
      problem.save_plan(fname, path);
    };
  motion_planning::plan(planner, output_basename, write_plan, timeout,
                        optimize);
  return 0;
}

#include "cliparser/CliParser.h"
#include "cpptoml/cpptoml.h"
#include "cpptoml/toml_conversions.h"
#include "csv/Csv.h"
#include "motion-planning/Problem.h"
#include "motion-planning/VoxelCachedLazyPRM.h"
#include "tip-control/Controller.h"
#include "util/macros.h"
#include "util/ompl_logging.h"
#include "util/openfile_check.h"

#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>

#include <Eigen/Core>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

struct Config;

namespace E = Eigen;

namespace {

namespace defaults {
  const std::string output         = "roadmap-ik.csv";
  const std::string ompl_log_level = "DEBUG";
  const int         num_neighbors  = 5;
  const double      tolerance      = 0.001;  // one millimeter
  const int         max_iter       = 10;
  const double      mu_init        = 10.0;
} // end of namespace defaults

void populate_parser(CliParser &parser) {
  parser.set_program_description(
      "Perform inverse kinematics using a precomputed roadmap.  The nearest\n"
      "  K states are starting points for the IK solver.  One iteration of\n"
      "  damped least squares is performed, collision checked, and as long\n"
      "  as not in collision, continues until convergence.  If one iteration\n"
      "  comes in collision, we will backtrack to find the closest\n"
      "  configuration not in collision.  If at least one of the IKs\n"
      "  succeed, then return only the successful IK solutions.  If none of\n"
      "  the IKs succeed, then return only the one closest configuration not\n"
      "  in collision.\n"
      "\n"
      "  The interface is similar to cheap_ik.  A csv file of requested IK\n"
      "  workspace tip positions is given and this program outputs a CSV\n"
      "  file of generated inverse kinematics.  The main difference is that\n"
      "  each IK can generate more than one IK result, from one to K.");

  parser.add_positional("problem");
  parser.set_required("problem");
  parser.set_description("problem", "problem file description");

  parser.add_positional("roadmap");
  parser.set_required("roadmap");
  parser.set_description("roadmap",
                      "Load the PRM roadmap from the given file.\n"
      "                Supports toml, toml.gz, json, bson, cbor, msgpack,\n"
      "                and ubjson.");

  parser.add_positional("requested-ik");
  parser.set_required("requested-ik");
  parser.set_description("requested-ik",
                      "CSV file of inverse kinematics requests.  Expected columns are\n"
      "                  - tip_x\n"
      "                  - tip_y\n"
      "                  - tip_z\n");

  parser.add_argflag("-o", "--output");
  parser.set_description("--output",
                      "Output CSV file, one line per requested IK.  Columns are\n"
      "                  - i: row number from the requested-ik CSV file\n"
      "                  - tip_error\n"
      "                  - tip_x\n"
      "                  - tip_y\n"
      "                  - tip_z\n"
      "                  - tau_i   (for i in [0 to # tendons])\n"
      "                  - theta   (if enable_rotation)\n"
      "                  - s_start (if enable_retraction)\n"
      "                (default is " + defaults::output + ")");

  parser.add_argflag("-L", "--ompl-log-level");
  parser.set_description("--ompl-log-level",
                      "Set the log level used in OMPL.  Choices (in order of\n"
      "                most to least verbose) are 'DEV2', 'DEV1', 'DEBUG',\n"
      "                'INFO', 'WARN', 'ERROR', 'NONE'.\n"
      "                (default is '" + defaults::ompl_log_level + "')");

  parser.add_argflag("-k", "--num-neighbors");
  parser.set_description("--num-neighbors",
                      "Number of neighbors to attempt in IK.\n"
      "                (default is " + std::to_string(defaults::num_neighbors)
                        + ")");

  parser.add_argflag("-t", "--tolerance");
  parser.set_description("--tolerance",
                      "Tolerance of L2 norm tip position error for IK.\n"
      "                (default is " + std::to_string(defaults::tolerance)
                        + " meters)");

  parser.add_argflag("-i", "--max-iter");
  parser.set_description("--max-iter",
                      "Maximum number of iterations for IK controller.\n"
      "                (default is " + std::to_string(defaults::max_iter)
                        + ")");

  parser.add_argflag("-mu", "--mu-init");
  parser.set_description("--mu-init",
                      "Initial damping factor for Levenberg-Marquardt (LM)\n"
      "                (also known as damped least squares (DLS).\n"
      "                Convergence may depend heavily on this.  A larger\n"
      "                value means smaller iteration step sizes, a smaller\n"
      "                value means larger iteration step sizes (roughly).\n"
      "                (default is "
                        + std::to_string(defaults::mu_init) + ")");

  parser.add_flag("-c", "--connect");
  parser.set_description("--connect",
                      "This flag does two things:\n"
      "                1. Instead of finding the closest valid config, find\n"
      "                   the closest valid config that can be directly\n"
      "                   connected to the roadmap.\n"
      "                2. Add the found result to the roadmap, causing\n"
      "                   subsequent IK queries to utilize this newly added\n"
      "                   point.\n"
      "                By default, this program simply uses the roadmap as a\n"
      "                group of starting guesses for IK, utilizing the\n"
      "                k-nearest.");

  parser.add_flag("-a", "--accurate");
  parser.set_description("--accurate",
                      "Only applicable if --connect.  From the IK solution,\n"
      "                when connecting, try to connect from many neighbors\n"
      "                rather than just the IK initial guess.  If none get\n"
      "                to the destination, return the resultant connection\n"
      "                endpoint that got closest.");

  parser.add_flag("-l", "--lazy-add");
  parser.set_description("--lazy-add",
                      "Only applicable if --connect.  After IK and\n"
      "                connecting it to the roadmap, the new vertex is fully\n"
      "                connected to the roadmap.  This flag disables\n"
      "                voxelization and checking of those additional edges\n"
      "                to be \"lazily evaluated\" if used in planning.  This\n"
      "                should have no influence on the IK results, just on\n"
      "                the completeness of the roadmap cache.");

  parser.add_flag("--skip-roadmap-vertex-check");
  parser.set_description("--skip-roadmap-vertex-check",
                      "Skip the step of checking for vertex collisions at\n"
      "                the time of loading the roadmap.  This causes\n"
      "                vertices to be checked during IK lazily.");

  parser.add_flag("--skip-roadmap-edge-check");
  parser.set_description("--skip-roadmap-edge-check",
                      "Skip the step of checking for edge collisions at\n"
      "                the time of loading the roadmap.  If we check for\n"
      "                edges at load-time (which is the default behavior),\n"
      "                then we will also prune off disconnected vertices\n"
      "                before running IK.  Disabling edge checking\n"
      "                effectively allows all vertices to be used in\n"
      "                roadmapIk.");

  parser.add_argflag("--output-roadmap");
  parser.set_description("--output-roadmap",
                      "Output the final resulting roadmap to a file");
}


std::vector<E::Vector3d> load_tip_samples(const std::string &csv_file) {
  std::ifstream in;
  util::openfile_check(in, csv_file);
  csv::CsvReader reader(in);
  csv::CsvRow row;
  std::vector<E::Vector3d> samples;
  while (reader >> row) {
    samples.emplace_back(E::Vector3d{std::stod(row["tip_x"]),
                                     std::stod(row["tip_y"]),
                                     std::stod(row["tip_z"])});
  }
  return samples;
}

void write_header(csv::CsvWriter &writer, const motion_planning::Problem &problem) {
  writer << "i" << "tip_error" << "tip_x" << "tip_y" << "tip_z";
  for (size_t i = 0; i < problem.robot.tendons.size(); ++i) {
    writer << "tau_" + std::to_string(i+1);
  }
  if (problem.robot.enable_rotation) {
    writer << "theta";
  }
  if (problem.robot.enable_retraction) {
    writer << "s_start";
  }
  writer.new_row();
}

void write_row(csv::CsvWriter &writer, const int i,
               const std::vector<double> &controls,
               const E::Vector3d &tip_position,
               const double tip_error)
{
  writer << i << tip_error
         << tip_position[0] << tip_position[1] << tip_position[2];
  for (auto &val : controls) { writer << val; }
  writer.new_row();
}

void my_assert(bool value, const std::string &msg = "failed assert") {
  if (!value) {
    std::cerr << "\n\nmy_assert: " << msg
      << std::endl
      << std::endl
      << std::endl; // really really flush before exiting
    //std::exit(1);
    throw std::runtime_error(msg);
  }
}

} // end of unnamed namespace

int main(int arg_count, char *arg_list[]) {
  CliParser parser;
  populate_parser(parser);
  parser.parse(arg_count, arg_list);

  auto problem = cpptoml::from_file<motion_planning::Problem>(parser["problem"]);

  const auto roadmap_file = parser["roadmap"];
  const auto ik_requests  = load_tip_samples(parser["requested-ik"]);
  const auto output       = parser.get("--output", defaults::output);
  const auto log_level    = parser.get("--ompl-log-level", defaults::ompl_log_level);
  const bool is_voxel     = true; //parser.has("--voxel");
  const bool is_backbone  = true; //parser.has("--backbone");
  const bool swept_volume = true; //parser.has("--swept-volume");
  const auto tolerance    = parser.get("--tolerance", defaults::tolerance);
  const auto n_neighbors  = parser.get("--num-neighbors", defaults::num_neighbors);
  const auto max_iter     = parser.get("--max-iter", defaults::max_iter);
  const auto mu_init      = parser.get("--mu-init", defaults::mu_init);
  const bool connect      = parser.has("--connect");
  const bool accurate     = parser.has("--accurate");
  const bool lazy_add     = parser.has("--lazy-add");
  const bool check_vert_validity = !parser.has("--skip-roadmap-vertex-check");
  const bool check_edge_validity = !parser.has("--skip-roadmap-edge-check");
  const bool remove_disconnected = check_vert_validity || check_edge_validity;
  const bool should_output_roadmap = parser.has("--output-roadmap");
  const auto output_roadmap_fname = parser.get("--output-roadmap", std::string());

  util::set_ompl_log_level(log_level);

  using Planner = motion_planning::VoxelCachedLazyPRM;
  auto planner  = problem.create_planner("VoxelCachedLazyPRM");
  auto vplanner = planner->as<Planner>();
  if (is_voxel) {
    problem.update_to_voxel_validators(planner, is_backbone, swept_volume);
  }

  std::cout << "\n"
    "setting default IK controller:\n"
    "  max_iter:                " << max_iter << "\n"
    "  tolerance:               " << tolerance << "\n"
    "  mu_init:                 " << mu_init << "\n";
  vplanner->setDefaultIkController(max_iter, tolerance, mu_init);

  // TODO: perform timing and report statistics to the console
  // TODO-  - loading roadmap
  // TODO-  - ik
  // TODO-  - timers stored in planner, checker, & motion validator
  // TODO- This can be used to profile setup times for planning
  // TODO- This can also be used to determine the timings of the internals of
  // TODO- ik, such as the number of fk calls, voxelizations, etc. and their
  // TODO- timings.

  std::cout << std::boolalpha <<
    "loading roadmap from file:\n"
    "  check vertex validity:   " << check_vert_validity << "\n"
    "  check edge validity:     " << check_edge_validity << "\n"
    "  remove disconnected:     " << remove_disconnected << "\n";
  vplanner->loadRoadmapFromFile(
      roadmap_file, check_vert_validity, check_edge_validity);
  if (remove_disconnected) { vplanner->clearDisconnectedVertices(); }

  std::ofstream out;
  util::openfile_check(out, output);
  csv::CsvWriter writer(out);

  std::cout << std::boolalpha <<
    "running IK:\n"
    "  connect:                 " << connect << "\n"
    "  accurate:                " << accurate << "\n"
    "  lazy-add:                " << lazy_add << "\n"
    "  write to:                " << output << "\n";

  auto si = vplanner->getSpaceInformation();
  auto vchecker = si->getStateValidityChecker();

  ob::ScopedState<> buf(si);
  auto is_config_valid = [&vchecker, &buf](const auto &controls) {
    buf = controls;
    return vchecker->isValid(buf.get());
  };

  auto ikopt = Planner::RMAP_IK_SIMPLE;
  if (connect)  { ikopt |= Planner::RMAP_IK_AUTO_ADD; }
  if (accurate) { ikopt |= Planner::RMAP_IK_ACCURATE; }
  if (lazy_add) { ikopt |= Planner::RMAP_IK_LAZY_ADD; }

  write_header(writer, problem);
  for (size_t i = 0; i < ik_requests.size(); ++i) {
    const auto &request = ik_requests[i];
    auto ik = vplanner->roadmapIk(request, tolerance, n_neighbors, ikopt);
    if (ik) {
      write_row(writer, i, ik->controls, ik->tip_position, ik->error);
      my_assert(is_config_valid(ik->controls));
    }
  }

  if (should_output_roadmap) {
    std::cout << "writing roadmap to " << output_roadmap_fname << "\n"
              << std::endl;
    vplanner->saveRoadmapToFile(output_roadmap_fname);
  }

  return 0;
}

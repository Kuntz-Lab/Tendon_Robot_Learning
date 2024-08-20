#include "cliparser/CliParser.h"
#include "cpptoml/cpptoml.h"
#include "cpptoml/toml_conversions.h"
#include "csv/Csv.h"
#include "motion-planning/Problem.h"
#include "util/openfile_check.h"

#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>

#include <Eigen/Core>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include <cassert>

struct Config;

namespace E = Eigen;
using NNType = ompl::NearestNeighborsGNATNoThreadSafety<Config>;

struct Config {
  std::vector<double> state;  // configuration values
  E::Vector3d         tip;    // resulting tip position

  bool operator ==(const Config &other) const {
    return state == other.state
        && tip   == other.tip;
  }

  bool operator !=(const Config &other) const { return !(*this == other); }
};

namespace {

namespace defaults {
  const std::string output = "cheap-ik.csv";
} // end of namespace defaults

void populate_parser(CliParser &parser) {
  parser.set_program_description(
      "Do a poor-man's version of inverse kinematics.  From a set of samples,\n"
      "  choose the sample that is closest to the requested inverse kinematics\n"
      "  workspace position.");

  parser.add_positional("problem");
  parser.set_required("problem");
  parser.set_description("problem", "problem file description");

  parser.add_positional("samples");
  parser.set_required("samples");
  parser.set_description("samples",
                      "CSV file of samples to draw from.  Expected columns are\n"
      "                  - tip_x\n"
      "                  - tip_y\n"
      "                  - tip_z\n"
      "                  - tau_i   (for i in [0 to # tendons])\n"
      "                  - theta   (if enable_rotation)\n"
      "                  - s_start (if enable_retraction)");

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
      "                  - tip_x\n"
      "                  - tip_y\n"
      "                  - tip_z\n"
      "                  - tau_i   (for i in [0 to # tendons])\n"
      "                  - theta   (if enable_rotation)\n"
      "                  - s_start (if enable_retraction)\n"
      "                  - requested_tip_x\n"
      "                  - requested_tip_y\n"
      "                  - requested_tip_z\n"
      "                  - error\n"
      "                (default is " + defaults::output + ")");
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

std::vector<Config> load_config_samples(
    const motion_planning::Problem &problem,
    const std::string &csv_file)
{
  // read in the controls and tip positions separately (for convenience)
  std::ifstream in;
  util::openfile_check(in, csv_file);
  auto controls      = problem.read_plan(in);

  auto tip_positions = load_tip_samples(csv_file);

  assert(controls.size() == tip_positions.size());

  // populate the samples
  std::vector<Config> samples(controls.size());
  for (size_t i = 0; i < samples.size(); ++i) {
    Config &config = samples[i];
    config.state   = std::move(controls[i]);
    config.tip     = std::move(tip_positions[i]);
  }
  return samples;
}

void write_header(csv::CsvWriter &writer, const motion_planning::Problem &problem) {
  writer
    << "tip_x"
    << "tip_y"
    << "tip_z";
  for (size_t i = 0; i < problem.robot.tendons.size(); ++i) {
    writer << "tau_" + std::to_string(i+1);
  }
  if (problem.robot.enable_rotation) {
    writer << "theta";
  }
  if (problem.robot.enable_retraction) {
    writer << "s_start";
  }
  writer
    << "requested_tip_x"
    << "requested_tip_y"
    << "requested_tip_z"
    << "error";
  writer.new_row();
}

void write_row(csv::CsvWriter &writer,
               const E::Vector3d &requested_tip,
               const Config conf)
{
  writer
    << conf.tip[0]
    << conf.tip[1]
    << conf.tip[2];
  for (auto &control : conf.state) {
    writer << control;
  }
  writer
    << requested_tip[0]
    << requested_tip[1]
    << requested_tip[2]
    << (conf.tip - requested_tip).norm();
  writer.new_row();
}

} // end of unnamed namespace

int main(int arg_count, char *arg_list[]) {
  CliParser parser;
  populate_parser(parser);
  parser.parse(arg_count, arg_list);

  auto problem     = cpptoml::from_file<motion_planning::Problem>(parser["problem"]);
  auto samples     = load_config_samples(problem, parser["samples"]);
  auto ik_requests = load_tip_samples(parser["requested-ik"]);
  auto output      = parser.get("--output", defaults::output);

  NNType neighborhood;
  neighborhood.setDistanceFunction(
      [](auto a, auto b) { return (b.tip - a.tip).norm(); });
  neighborhood.add(samples);

  auto nearest_config = [&neighborhood](E::Vector3d tip) {
    Config temporary_config;
    temporary_config.tip = std::move(tip);
    return neighborhood.nearest(temporary_config);
  };

  std::ofstream out;
  util::openfile_check(out, output);
  csv::CsvWriter writer(out);
  std::cout << "writing to " << output << std::endl;

  write_header(writer, problem);

  for (auto &request : ik_requests) {
    auto config = nearest_config(request);
    write_row(writer, request, config);
  }

  return 0;
}

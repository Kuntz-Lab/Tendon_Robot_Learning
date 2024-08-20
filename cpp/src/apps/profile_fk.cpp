#include <cliparser/CliParser.h>
#include <tendon/TendonRobot.h>
#include <tendon/TendonSpecs.h>
#include <csv/Csv.h>
#include <cpptoml/toml_conversions.h>
#include <util/LifetimeTimer.h>
#include <util/openfile_check.h>

#include <Eigen/Core>

#include <iostream>

namespace E = Eigen;

namespace {
  
namespace defaults {
  std::string output = "fk.csv";
}

void populate_parser(CliParser &parser) {
  parser.set_program_description(
      "Profile the speed of forward-kinematics (FK) on some input configurations.\n"
      "\n"
      "  Note: This app utilizes OpenMP to parallelize by running multiple\n"
      "    instances of FK on separate configurations.  If you want to control\n"
      "    the number of threads, set the OMP_NUM_THREADS environment variable");

  parser.add_positional("robot_toml");
  parser.set_required("robot_toml");
  parser.set_description("robot_toml", "Toml file describing robot configuration");

  parser.add_positional("configs_csv");
  parser.set_required("configs_csv");
  parser.set_description("configs_csv",
                      "CSV file containing robot configurations.\n"
      "                Required columns:\n"
      "                - tau_i   (for i in [1 : # tendons])\n"
      "                - theta   (if robot.enable_rotation)\n"
      "                - s_start (if robot.enable_retraction)");

  parser.add_argflag("-o", "--output");
  parser.set_description("--output",
                       "Output FK results to the given CSV file.\n"
      "                 This will repeat the input file, but will add the\n"
      "                 following columns to the end:\n"
      "                 - tip_x\n"
      "                 - tip_y\n"
      "                 - tip_z\n"
      "                 (defaults to " + defaults::output + ")");

  // TODO: add timing csv output
  // TODO: add --single-shoot using the single-shoot method
  // TODO: verify and report on boundary condition accuracy
}



} // end of unnamed namespace

int main(int arg_count, char *arg_list[]) {
  CliParser parser;
  populate_parser(parser);
  parser.parse(arg_count, arg_list);

  auto robot       = cpptoml::from_file<tendon::TendonRobot>(parser["robot_toml"]);
  auto configs_csv = parser["configs_csv"];
  auto configs     = robot.load_config_csv(configs_csv);
  auto output      = parser.get("--output", defaults::output);
  auto N           = configs.size();

  std::cout <<
    "robot:         " << parser["robot_toml"] << "\n"
    "configs:       " << configs_csv << "\n"
    "output:        " << output << "\n"
    "# configs:     " << N << "\n"
    "\n"
    "state size:    " << robot.state_size() << "\n"
    "config size:   " << configs[0].size() << "\n"
    "\n" << std::flush;

  std::vector<E::Vector3d> tips(N);

  double time = 0.0;
  {
    util::LifetimeTimer local_timer(time);
    #pragma omp parallel for
    for (size_t i = 0; i < N; ++i) {
      tips[i] = robot.forward_kinematics(configs[i]).back();
    }
  }

  std::vector<E::Vector3d> tmp(N);
  double loop_overhead = 0.0;
  {
    util::LifetimeTimer local_timer(loop_overhead);
    #pragma omp parallel for
    for (size_t i = 0; i < N; ++i) {
      auto &c = configs[0];
      tmp[i] = {c[0], c[0], c[0]};
    }
  }

  // print timing results
  double fk_time = time - loop_overhead;
  std::cout <<
    "Timing for " << N << " FK calls\n"
    " - tot:             " << time << " secs\n"
    " - loop overhead:   " << loop_overhead << " secs\n"
    " - fk time:         " << fk_time << " secs\n"
    " - avg:             " << fk_time / N << " secs/call\n"
    " - avg freq:        " << N / fk_time << " Hz\n"
    "\n" << std::flush;

  // read in the input csv file
  csv::CsvRow header;
  std::vector<csv::CsvRow> rows;
  {
    std::ifstream in;
    util::openfile_check(in, configs_csv);
    csv::CsvReader reader(in);
    header = *reader.header();
    csv::CsvRow row;
    while (reader >> row) {
      rows.emplace_back(row);
    }
  }

  header.emplace_back("tip_x");
  header.emplace_back("tip_y");
  header.emplace_back("tip_z");

  // output csv file
  {
    std::cout << "writing to " << output << std::endl;
    std::ofstream out;
    out << std::setprecision(std::numeric_limits<double>::digits10 + 1);
    util::openfile_check(out, output);
    csv::CsvWriter writer(out);
    writer.write_row(header);
    for (size_t i = 0; i < rows.size(); ++i) {
      for (auto &elem : rows[i]) {
        writer << elem;
      }
      auto &t = tips[i];
      writer << t[0] << t[1] << t[2];
      writer.new_row();
    }
  }

  return 0;
}

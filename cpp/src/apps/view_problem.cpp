#include "cliparser/CliParser.h"
#include "collision/VoxelOctree.h"
#include "cpptoml/cpptoml.h"
#include "cpptoml/toml_conversions.h"
#include "csv/Csv.h"
#include "motion-planning/Problem.h"
#include "util/openfile_check.h"
#include "util/time_function_call.h"
#include "util/vector_ops.h"
#include "vistendon/EnvironmentRvizPublisher.h"
#include "vistendon/ManualRvizMarkerArrayPublisher.h"
#include "vistendon/TendonBackboneRvizPublisher.h"
#include "vistendon/TendonRvizPublisher.h"
#include "vistendon/marker_array_conversions.h"

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Core>

#include <iostream>
#include <string>
#include <vector>

using util::operator<<;

namespace E = Eigen;

namespace {

void populate_parser(CliParser &parser) {
  parser.set_program_description(
      "View a planning problem in RViz.  Obstacles are blue, start is yellow,\n"
      "goal is green, and tendons are in black.");
  parser.add_positional("problem");
  parser.set_required("problem");
  parser.set_description("problem", "The problem specification toml file to use");

  parser.add_argflag("-s", "--samples");
  parser.set_description("--samples",
                      "A samples CSV file to also visualize as tiny spheres.\n"
      "                This CSV file is like the one generated from the\n"
      "                sample_configs application.");
  parser.add_flag("--voxel");
  parser.set_description("--voxel",
                      "Show voxel environment instead of the environment.\n"
      "                Give a toml file with a [voxel_environment] section.\n"
      "                Cannot be specified with --sphere.");
  parser.add_flag("--backbone");
  parser.set_description("--backbone",
                      "Visualize the tendon robot only as a voxelized line\n"
      "                representing the tendon robot centerline.");
}

std::vector<E::Vector3d> load_tip_samples(const std::string &samples_file) {
  std::ifstream in;
  util::openfile_check(in, samples_file);
  csv::CsvReader reader(in);

  std::vector<E::Vector3d> tip_positions;
  csv::CsvRow row;
  while (reader >> row) {
    tip_positions.emplace_back(E::Vector3d{std::stod(row["tip_x"]),
                                           std::stod(row["tip_y"]),
                                           std::stod(row["tip_z"])});
  }
  return tip_positions;
}

void view_problem(const motion_planning::Problem &problem,
                  const std::vector<E::Vector3d> &samples,
                  bool backbone)
{
  auto start_shape      = problem.start_shape();
  auto goal_shape       = problem.goal_shape();
  auto start_home_shape = problem.start_home_shape();
  auto goal_home_shape  = problem.goal_home_shape();

  auto [start_backbone, start_tendons] = vistendon::robot_and_tendon_shape(
      problem.robot, start_shape, start_home_shape);
  auto [goal_backbone, goal_tendons] = vistendon::robot_and_tendon_shape(
      problem.robot, goal_shape, goal_home_shape);

  // print some settings
  std::cout << std::boolalpha
            << "rotation enabled:   " << problem.robot.enable_rotation << "\n"
            << "retraction enabled: " << problem.robot.enable_retraction << "\n"
            << std::endl;

  // check to see if start is in collision
  std::cout << "backbone shape length: " << start_backbone.size() << std::endl;
  auto start_validity_time = util::time_function_call(
    [&start_home_shape, &start_shape, &problem]() {
      if (!problem.is_valid(problem.start, start_home_shape, start_shape)) {
        std::cerr << "Warning: start state is not valid\n";
      }
    });
  std::cout << "Start state validity check: " << start_validity_time << " sec\n";

  // check to see if goal is in collision
  auto goal_validity_time = util::time_function_call(
    [&goal_home_shape, &goal_shape, &problem]() {
      if (!problem.is_valid(problem.goal, goal_home_shape, goal_shape)) {
        std::cerr << "Warning: goal state is in collision\n";
      }
    });
  std::cout << "Goal state validity check:  " << goal_validity_time << " sec\n";

  std::cout << "\n"
            << "Start Home lengths:  " << start_home_shape.L_i << "\n"
            << "Start lengths:       " << start_shape.L_i << "\n"
            << "Goal Home Lengths:   " << goal_home_shape.L_i << "\n"
            << "Goal lengths:        " << goal_shape.L_i << "\n"
            << "Start dl:            "
              << problem.robot.calc_dl(start_home_shape.L_i, start_shape.L_i)
              << "\n"
            << "Goal dl:             "
              << problem.robot.calc_dl(goal_home_shape.L_i, goal_shape.L_i)
              << "\n"
            << std::endl;

  // TODO: calculate voxel occupancy of samples

  // initialize visualization
  std::string frame = "/map";
  std::string start_namespace = "robot-start";
  std::string env_namespace = "obstacles";
  std::string goal_namespace = "robot-goal";
  std::string start_tendon_namespace = start_namespace + "-tendons";
  std::string goal_tendon_namespace = goal_namespace + "-tendons";
  std::string samples_namespace = "tip-samples";
  auto tendon_node = std::make_shared<rclcpp::Node>("view_tendon");

  std::unique_ptr<vistendon::RvizMarkerArrayPublisher> start_publisher;
  std::unique_ptr<vistendon::RvizMarkerArrayPublisher> goal_publisher;
  if (backbone) {
    start_publisher.reset(
        new vistendon::ManualRvizMarkerArrayPublisher(tendon_node, frame,
                                                      start_namespace));
    goal_publisher.reset(
        new vistendon::ManualRvizMarkerArrayPublisher(tendon_node, frame,
                                                      goal_namespace));
    auto manual_start_pub =
        static_cast<vistendon::ManualRvizMarkerArrayPublisher*>(
          start_publisher.get());
    auto manual_goal_pub =
        static_cast<vistendon::ManualRvizMarkerArrayPublisher*>(
          goal_publisher.get());

    auto voxels = collision::VoxelOctree::from_file(problem.venv.filename);
    auto start_voxels = voxels.empty_copy();
    auto  goal_voxels = voxels.empty_copy();
    start_voxels.add_piecewise_line(start_backbone.points);
    goal_voxels .add_piecewise_line( goal_backbone.points);
    manual_start_pub->add_marker(vistendon::to_marker(start_voxels.to_mesh()));
    manual_goal_pub ->add_marker(vistendon::to_marker( goal_voxels.to_mesh()));
  } else {
    start_publisher.reset(
        new vistendon::TendonBackboneRvizPublisher(tendon_node, frame,
                                                   start_namespace));
    goal_publisher.reset(
        new vistendon::TendonBackboneRvizPublisher(tendon_node, frame,
                                                   goal_namespace));
    auto tendon_start_pub =
        static_cast<vistendon::TendonBackboneRvizPublisher*>(
          start_publisher.get());
    auto tendon_goal_pub =
        static_cast<vistendon::TendonBackboneRvizPublisher*>(
          goal_publisher.get());

    tendon_start_pub->set_robot(start_backbone);
    tendon_goal_pub->set_robot(goal_backbone);
  }
  start_publisher->set_color(1.0f, 1.0f, 0.0f, 0.4f); // transparent yellow
  goal_publisher->set_color(0.0f, 1.0f, 0.0f, 0.4f); // transparent green

  vistendon::TendonRvizPublisher start_tendon_publisher(
      tendon_node, frame, start_tendon_namespace);
  vistendon::TendonRvizPublisher goal_tendon_publisher(
      tendon_node, frame, goal_tendon_namespace);
  vistendon::EnvironmentRvizPublisher env_publisher(
      tendon_node, frame, env_namespace);
  vistendon::ManualRvizMarkerArrayPublisher samples_publisher(
      tendon_node, frame, samples_namespace);

  start_tendon_publisher.set_color(0.0f, 0.0f, 0.0f, 1.0f); // solid black
  start_tendon_publisher.set_tendons(start_tendons);

  goal_tendon_publisher.set_color(0.0f, 0.0f, 0.0f, 1.0f); // solid black
  goal_tendon_publisher.set_tendons(goal_tendons);

  env_publisher.set_color(0.0f, 0.0f, 1.0f, 0.2f); // transparent blue
  env_publisher.set_env(problem.env);

  using Marker = visualization_msgs::msg::Marker;
  Marker samples_marker;
  samples_marker.type = Marker::SPHERE_LIST;
  samples_marker.action = Marker::ADD;
  samples_marker.scale.x = problem.robot.specs.dL*2;
  samples_marker.scale.y = problem.robot.specs.dL*2;
  samples_marker.scale.z = problem.robot.specs.dL*2;
  samples_marker.pose.position.x = 0;
  samples_marker.pose.position.y = 0;
  samples_marker.pose.position.z = 0;
  samples_marker.pose.orientation.w = 1;
  samples_marker.pose.orientation.x = 0;
  samples_marker.pose.orientation.y = 0;
  samples_marker.pose.orientation.z = 0;
  for (auto &sample : samples) {
    geometry_msgs::msg::Point p;
    p.x = sample[0];
    p.y = sample[1];
    p.z = sample[2];
    samples_marker.points.emplace_back(std::move(p));
  }
  samples_publisher.set_color(1.0f, 0.0f, 0.0f, 0.5f); // transparent red
  samples_publisher.add_marker(samples_marker);

  std::cout << "\n"
               "Publishing rviz MarkerArray to /visualization_marker_array "
               "ROS 2 topic\n"
               "  frame:                   " << frame << "\n"
               "  start namespace:         " << start_namespace << "\n"
               "  goal namespace:          " << goal_namespace << "\n"
               "  start tendon namespace:  " << start_tendon_namespace << "\n"
               "  goal tendon namespace:   " << start_tendon_namespace << "\n"
               "  obstacles namespace:     " << env_namespace << "\n"
               "\n"
               "Press Ctrl-C to quit\n"
               "\n";

  rclcpp::spin(tendon_node);
  rclcpp::shutdown();
}

} // end of unnamed namespace

int main(int argCount, char* argList[]) {
  rclcpp::init(argCount, argList);
  CliParser parser;
  populate_parser(parser);
  parser.parse(argCount, argList);
  if (parser.has("--backbone") && !parser.has("--voxel")) {
    std::cerr << "Cannot specify --backbone without --voxel" << std::endl;
    return 1;
  }

  std::cout << "Loading " << parser["problem"] << "\n"
            << "voxel env?          " << parser.has("--voxel") << "\n"
            << "robot backbone?     " << parser.has("--backbone") << "\n"
            << std::endl;
  auto problem = cpptoml::from_file<motion_planning::Problem>(parser["problem"]);

  decltype(load_tip_samples("")) samples;
  if (parser.has("--samples")) {
    samples = load_tip_samples(parser["--samples"]);
  }
  if (parser.has("--voxel")) {
    // replace the environment with the voxel shape
    auto voxels = collision::VoxelOctree::from_file(problem.venv.filename);
    problem.env = motion_planning::Environment();
    problem.env.push_back(voxels.to_mesh());
  }

  // blocks.  Press Ctrl-C to cancel
  view_problem(problem, samples, parser.has("--backbone"));

  return 0;
}

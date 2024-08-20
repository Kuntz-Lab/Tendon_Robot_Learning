#include "collision/VoxelOctree.h"
#include "cliparser/CliParser.h"
#include "cpptoml/cpptoml.h"
#include "cpptoml/toml_conversions.h"
#include "motion-planning/Problem.h"
#include "util/macros.h"
#include "util/time_function_call.h"
#include "util/vector_ops.h"
#include "vistendon/EnvironmentRvizPublisher.h"
#include "vistendon/TendonRvizPublisher.h"
#include "vistendon/VoxelRvizPublisher.h"
#include "vistendon/marker_array_conversions.h"

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>

#include <iostream>
#include <vector>

// TODO: use only the motion_planning::VoxelEnvironment
// TODO: make other app calld "voxelize_env" to create a voxel image

using util::operator<<;

namespace {

void populate_parser(CliParser &parser) {
  parser.set_program_description(
      "View a planning problem in RViz in voxel form.  Obstacles are blue,\n"
      "  start is yellow, and goal is green.  The tendons will be displayed\n"
      "  as lines instead of boxes for added understanding.\n"
      "\n"
      //"  Currently only points, spheres, and voxel objects are supported in\n"
      //"  the input problem file.\n"
      "  Currently only points and spheres are supported in the input problem\n"
      "  file.  All others will be ignored.\n"
      );
  parser.add_positional("problem");
  parser.set_required("problem");
  parser.set_description("problem", "The problem specification toml file to use");

  parser.add_argflag("-s", "--size");
  parser.set_description("--size", "Size of the voxelization.  Choices are 4, 8,\n"
      "                16, 32, 64, 128, 256, and 512.\n"
      "                Default is 128.\n");
}

collision::VoxelOctree to_voxels(
    const std::vector<collision::Point> &backbone,
    double r,
    const collision::VoxelOctree &env)
{
  auto voxel_robot = env.empty_copy();
  for (auto &point : backbone) {
    voxel_robot.add_sphere(collision::Sphere{point, r});
  }
  return voxel_robot;
}

void view_problem(const motion_planning::Problem &problem,
                  size_t voxel_dim = 256)
{
  auto start_shape      = problem.start_shape();
  auto goal_shape       = problem.goal_shape();
  auto start_home_shape = problem.start_home_shape();
  auto goal_home_shape  = problem.goal_home_shape();

  auto [start_backbone, start_tendons] =
      vistendon::robot_and_tendon_shape(problem.robot, start_shape, start_home_shape);
  auto [goal_backbone, goal_tendons] =
      vistendon::robot_and_tendon_shape(problem.robot, goal_shape, goal_home_shape);
  UNUSED_VAR(start_backbone);
  UNUSED_VAR(goal_backbone);

  collision::VoxelOctree voxel_env(voxel_dim);
  auto L = problem.robot.specs.L;
  auto r = problem.robot.r;
  voxel_env.set_xlim(-L-r, L+r);
  voxel_env.set_ylim(-L-r, L+r);
  voxel_env.set_zlim(-r, L+r);
  auto start_voxels = voxel_env.empty_copy();
  auto goal_voxels = voxel_env.empty_copy();

  std::cout << "Voxel limits:\n"
               "  xlim: [" << voxel_env.xlim().first << ", "
                           << voxel_env.xlim().second << "]"
                           << ", dz: " << voxel_env.dx() << "\n"
               "  ylim: [" << voxel_env.ylim().first << ", "
                           << voxel_env.ylim().second << "]"
                           << ", dz: " << voxel_env.dy() << "\n"
               "  zlim: [" << voxel_env.zlim().first << ", "
                           << voxel_env.zlim().second << "]"
                           << ", dz: " << voxel_env.dz() << "\n"
            << std::endl;

  if (problem.env.capsules().size() > 0) {
    std::cerr
      << "Error: problem has capsules in the environment\n"
      << "  This is not supported for VoxelOctree for now\n"
      << "  Skipping this file.\n"
      << std::endl;
    return;
  }
  if (problem.env.meshes().size() > 0) {
    std::cerr
      << "Error: problem has meshes in the environment\n"
      << "  This is not supported for VoxelOctree for now\n"
      << "  Skipping this file.\n"
      << std::endl;
    return;
  }

  // add environment obstacles
  for (auto &p : problem.env.points())  { voxel_env.add(p); }
  for (auto &s : problem.env.spheres()) {
    std::cout << "Converting sphere " << s << " to voxels";
    voxel_env.add_sphere(s);
    std::cout << ", nblocks: " << voxel_env.nblocks() << std::endl;
  }

  std::cout << "Removing interior";
  voxel_env.remove_interior();
  std::cout << ", now at " << voxel_env.nblocks() << "blocks\n"
            << std::endl;

  // print some settings
  std::cout << std::boolalpha
            << "rotation enabled:   " << problem.robot.enable_rotation << "\n"
            << "retraction enabled: " << problem.robot.enable_retraction << "\n"
            << std::endl;

  // check to see if start is in collision
  std::cout << "backbone shape length: " << start_backbone.size() << std::endl;
  auto start_validity_time = util::time_function_call(
    [&start_home_shape, &start_shape, &problem, &voxel_env, &start_voxels]() {
      if (!problem.robot.is_valid(problem.start, start_home_shape, start_shape)) {
        std::cerr << "Warning: start state is not valid\n";
      } else {
        start_voxels = to_voxels(start_shape.p, problem.robot.r, voxel_env);
        start_voxels.remove_interior();
        if (voxel_env.collides(start_voxels)) {
          std::cerr << "Warning: start state is in collision\n";
        }
      }
    });
  std::cout << "Start state validity check: " << start_validity_time << " sec\n";

  // check to see if goal is in collision
  auto goal_validity_time = util::time_function_call(
    [&goal_home_shape, &goal_shape, &problem, &voxel_env, &goal_voxels]() {
      if (!problem.robot.is_valid(problem.goal, goal_home_shape, goal_shape)) {
        std::cerr << "Warning: goal state is not valid\n";
      } else {
        goal_voxels = to_voxels(goal_shape.p, problem.robot.r, voxel_env);
        goal_voxels.remove_interior();
        if (voxel_env.collides(goal_voxels)) {
          std::cerr << "Warning: goal state is in collision\n";
        }
      }
    });
  std::cout << "Goal state validity check:  " << goal_validity_time << " sec\n";

  std::cout << "\n"
            << "Start Home lengths: " << start_home_shape.L_i << "\n"
            << "Goal Home lengths:  " << goal_home_shape.L_i << "\n"
            << "Start lengths:      " << start_shape.L_i << "\n"
            << "Goal lengths:       " << goal_shape.L_i << "\n"
            << "Start dl:           "
              << problem.robot.calc_dl(start_home_shape.L_i, start_shape.L_i) << "\n"
            << "Goal dl:            "
              << problem.robot.calc_dl(goal_home_shape.L_i, goal_shape.L_i) << "\n"
            << "Env   voxel blocks:  " << voxel_env.nblocks() << "\n"
            << "Start voxel blocks:  " << start_voxels.nblocks() << "\n"
            << "Goal  voxel blocks:  " << goal_voxels.nblocks() << std::endl;

  // initialize visualization
  std::string frame = "/map";
  std::string start_namespace = "robot-start";
  std::string env_namespace = "obstacles";
  std::string goal_namespace = "robot-goal";
  std::string start_tendon_namespace = start_namespace + "-tendons";
  std::string goal_tendon_namespace = goal_namespace + "-tendons";
  auto tendon_node = std::make_shared<rclcpp::Node>("view_tendon");

  vistendon::VoxelRvizPublisher start_publisher(
      tendon_node, frame, start_namespace);
  vistendon::VoxelRvizPublisher goal_publisher(
      tendon_node, frame, goal_namespace);
  vistendon::TendonRvizPublisher start_tendon_publisher(
      tendon_node, frame, start_tendon_namespace);
  vistendon::TendonRvizPublisher goal_tendon_publisher(
      tendon_node, frame, goal_tendon_namespace);
  vistendon::VoxelRvizPublisher env_publisher(
      tendon_node, frame, env_namespace);

  start_publisher.set_color(1.0f, 1.0f, 0.0f, 1.0f); // solid yellow
  start_publisher.set_voxels(start_voxels);

  goal_publisher.set_color(0.0f, 1.0f, 0.0f, 1.0f); // solid green
  goal_publisher.set_voxels(goal_voxels);

  start_tendon_publisher.set_color(0.0f, 0.0f, 0.0f, 1.0f); // solid black
  start_tendon_publisher.set_tendons(start_tendons);

  goal_tendon_publisher.set_color(0.0f, 0.0f, 0.0f, 1.0f); // solid black
  goal_tendon_publisher.set_tendons(goal_tendons);

  env_publisher.set_color(0.0f, 0.0f, 1.0f, 1.0f); // solid blue
  env_publisher.set_voxels(voxel_env);

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

  std::cout << "Loading " << parser["problem"] << std::endl;
  auto problem = cpptoml::from_file<motion_planning::Problem>(parser["problem"]);

  int size = parser.get("--size", 128);
  view_problem(problem, size);

  return 0;
}

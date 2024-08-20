#include "cliparser/CliParser.h"
#include "collision/VoxelOctree.h"
#include "cpptoml/cpptoml.h"
#include "cpptoml/toml_conversions.h"
#include "csv/Csv.h"
#include "motion-planning/Problem.h"
#include "tip-control/Controller.h"
#include "tip-control/tip_control.h"
#include "util/macros.h"
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

using namespace std::literals::chrono_literals;

using util::operator<<;

namespace E = Eigen;

namespace {

namespace defaults {
  double max_speed = 0.005; // m/s
} // end of namespace defaults

void populate_parser(CliParser &parser) {
  parser.set_program_description(
      "View a planning problem in RViz and control from start to goal using\n"
      "resolved-rate control.  Obstacles are blue, start is yellow,\n"
      "goal is green, and tendons are in black.");
  parser.add_positional("problem");
  parser.set_required("problem");
  parser.set_description("problem", "The problem specification toml file to use");

  parser.add_argflag("-s", "--max-speed", "Max tip speed for controller (default = 0.005 m/s)");
}

class Viewer {
public:
  Viewer(const motion_planning::Problem &problem);

  rclcpp::Node* node() { return _tendon_node.get(); }
  const rclcpp::Node* node() const { return _tendon_node.get(); }

  vistendon::TendonBackboneRvizPublisher* current_pub()
  { return _current_pub.get(); }
  const vistendon::TendonBackboneRvizPublisher* current_pub() const
  { return _current_pub.get(); }

  void start() {
    rclcpp::spin(_tendon_node);
    rclcpp::shutdown();
  }

private:
  const motion_planning::Problem &_problem;
  std::vector<double> _state;
  collision::CapsuleSequence _current_backbone;
  std::shared_ptr<rclcpp::Node> _tendon_node;
  std::unique_ptr<vistendon::TendonBackboneRvizPublisher> _start_pub;
  std::unique_ptr<vistendon::TendonBackboneRvizPublisher> _goal_pub;
  std::unique_ptr<vistendon::TendonBackboneRvizPublisher> _current_pub;
  std::unique_ptr<vistendon::EnvironmentRvizPublisher>    _env_pub;
};

Viewer::Viewer(const motion_planning::Problem &problem)
  : _problem(problem)
{
  auto start_shape      = problem.start_shape();
  auto goal_shape       = problem.goal_shape();
  auto start_home_shape = problem.start_home_shape();
  auto goal_home_shape  = problem.goal_home_shape();

  auto [start_backbone, start_tendons] = vistendon::robot_and_tendon_shape(
      problem.robot, start_shape, start_home_shape);
  auto [goal_backbone, goal_tendons] = vistendon::robot_and_tendon_shape(
      problem.robot, goal_shape, goal_home_shape);
  UNUSED_VAR(start_tendons);
  UNUSED_VAR(goal_tendons);

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
  std::string current_namespace = "robot-current";
  std::string samples_namespace = "tip-samples";
  _tendon_node = std::make_shared<rclcpp::Node>("view_tendon");

  _start_pub =
      std::make_unique<vistendon::TendonBackboneRvizPublisher>(
        _tendon_node, frame, start_namespace);
  _goal_pub =
      std::make_unique<vistendon::TendonBackboneRvizPublisher>(
        _tendon_node, frame, goal_namespace);
  _current_pub =
      std::make_unique<vistendon::TendonBackboneRvizPublisher>(
        _tendon_node, frame, current_namespace);

  _start_pub->set_robot(start_backbone);
  _goal_pub ->set_robot(goal_backbone);
  _current_pub->set_robot(start_backbone);

  _start_pub->set_color(1.0f, 1.0f, 0.0f, 0.4f); // transparent yellow
  _goal_pub ->set_color(0.0f, 1.0f, 0.0f, 0.4f); // transparent green
  _current_pub->set_color(0.0f, 0.0f, 1.0f, 0.4f); // transparent blue

  _env_pub = std::make_unique<vistendon::EnvironmentRvizPublisher>(
      _tendon_node, frame, env_namespace);

  _env_pub->set_color(0.0f, 0.0f, 1.0f, 0.2f); // transparent blue
  _env_pub->set_env(problem.env);

  std::cout << "\n"
               "Publishing rviz MarkerArray to /visualization_marker_array "
               "ROS 2 topic\n"
               "  frame:                   " << frame << "\n"
               "  start namespace:         " << start_namespace << "\n"
               "  goal namespace:          " << goal_namespace << "\n"
               "  current namespace:       " << current_namespace << "\n"
               "  obstacles namespace:     " << env_namespace << "\n"
               "\n"
               "Press Ctrl-C to quit\n"
               "\n";
}

} // end of unnamed namespace

int main(int argCount, char* argList[]) {
  rclcpp::init(argCount, argList);
  CliParser parser;
  populate_parser(parser);
  parser.parse(argCount, argList);
  

  std::cout << "Loading " << parser["problem"] << "\n"
            << std::endl;
  auto problem = cpptoml::from_file<motion_planning::Problem>(parser["problem"]);

  std::vector<double> state = problem.start_state();
  auto tip = problem.robot.shape(state).p.back();
  auto goal_tip = problem.robot.shape(problem.goal_state()).p.back();
  Controller controller(problem.robot);

  auto dt = 100ms;
  std::chrono::duration<double> dt_secs = dt;
  auto max_speed = parser.get("--max-speed", defaults::max_speed);
  auto max_speed_times_dt = dt_secs.count() * max_speed;

  Viewer viewer(problem);
  auto timer = viewer.node()->create_wall_timer(dt,
      [&]() -> void {
        auto v_times_dt = tip_control::clamped_v_times_dt(tip, goal_tip, max_speed_times_dt);
        // update state with resolved-rate update
        state = controller.damped_resolved_rate_update(state, v_times_dt);

        std::cout
            << "v_times_dt:       " << v_times_dt.transpose() << "\n"
            << "tip error:        " << (goal_tip - tip).transpose() << "\n"
            << "|tip error|:      " << (goal_tip - tip).norm() << "\n"
            << "state:            " << state << "\n"
            << "\n";

        auto shape = problem.robot.shape(state);
        tip = shape.p.back();  // simulated measurement
        auto home_shape = problem.robot.home_shape(state);
        auto [backbone, tendons] = vistendon::robot_and_tendon_shape(
            problem.robot, shape, home_shape);
        UNUSED_VAR(tendons);

        viewer.current_pub()->set_robot(backbone);
      });

  // blocks.  Press Ctrl-C to cancel
  viewer.start();

  return 0;
}

#include "GoalTracker.h"
#include "InteriorTracker.h"

#include <cliparser/CliParser.h>
#include <collision/Point.h>
#include <collision/VoxelOctree.h>
#include <cpptoml/toml_conversions.h>
#include <haptic/HapticQSubscriber.h>
#include <haptic/HapticTransform.h>
#include <haptic/StreamingTransformer.h>
#include <motion-planning/Problem.h>
#include <motion-planning/StreamingPlanner.h>
#include <util/macros.h>
#include <util/ompl_logging.h>
#include <vistendon/EnvironmentRvizPublisher.h>
#include <vistendon/StreamingInterleaver.h>
#include <vistendon/StreamingRvizTipPublisher.h>
#include <vistendon/TendonBackboneRvizPublisher.h>
#include <vistendon/TrajectoryVisualizer.h>

#include <tendon/config.h> // for TENDON_RCLCPP_VERSION_MAJOR definition
#include <3rdparty/libros2qt/qt_executor.h>

#include <rclcpp/rclcpp.hpp>

#include <QCoreApplication>
#include <QObject>
#include <QThread>
#include <QVector3D>

namespace {

namespace defaults {
  const std::string  log             = "streaming-planner.log";
  const std::string  ompl_log_level  = "DEBUG";
  const size_t       ik_neighbors    = 3;
  const size_t       ik_max_iters    = 10;
  const double       ik_tolerance    = 0.0005;
} // end of namespace defaults

void populate_parser(CliParser &parser) {
  parser.set_program_description(
      "Listen to the haptic device on ROS and plan accordingly.  The tip\n"
    "  position of the haptic device is used to control the tip of the\n"
    "  robot.  A sphere representing the current tip position from the\n"
    "  haptic device will be sent to RViz.  When the button on the haptic\n"
    "  device is pressed, the current position will be set as the next goal\n"
    "  for planning, the planner will make a plan, and the trajectory will\n"
    "  be played and sent to RViz.\n"
    "\n"
    "  Transform information may be specified in the problem.toml file.  For\n"
    "  example:\n"
    "\n"
    "    [haptic-transform]\n"
    "    translation = [-0.0230, -0.0220, 0.0157]\n"
    "    scale = 0.5\n"
    "\n"
    "  If that is not found in the toml file, it will default to an identity\n"
    "  transform."
    "\n"
    "  Note: this app only supports the VoxelCachedLazyPRM planner and\n"
    "  only supports it with the backbone specification of only doing\n"
    "  collision checking on the shape of the backbone portion of the\n"
    "  robot.  It also only supports the swept volume option where a swept\n"
    "  volume of the backbone is calculated and voxelized."
    );

  parser.add_positional("problem");
  parser.set_required("problem");
  parser.set_description("problem",
                      "problem toml file that may also have the\n"
      "                [haptic_transform] section.  The start configuration\n"
      "                is used as the starting point, but the goal\n"
      "                configuration is ignored.  Instead, waypoints from\n"
      "                clicked points from the haptic device are used as\n"
      "                goals.\n"
      "\n"
      "                If the interior voxel environment is given, then this\n"
      "                program will indicate if the current tip is within the\n"
      "                interior.");

  parser.add_positional("roadmap");
  parser.set_required("roadmap");
  parser.set_description("roadmap", "PRM roadmap file\n"
      "                Supports json, bson, cbor, msgpack, and ubjson.");

  parser.add_argflag("-l", "--log"); // CSV for timing events
  parser.set_description("--log",
                      "CSV file containing raw profiling data with computed\n"
      "                statistics at the end.  There are only two columns,\n"
      "                - name\n"
      "                - milestone: which milestone this applies to\n"
      "                - value\n"
      "                (default is '" + defaults::log + "')");

  parser.add_argflag("--ompl-log-level");
  parser.set_description("--ompl-log-level",
                      "Set the log level used in OMPL.  Choices (in order of\n"
      "                most to least verbose) are 'DEV2', 'DEV1', 'DEBUG',\n"
      "                'INFO', 'WARN', 'ERROR', 'NONE'.\n"
      "                (default is '" + defaults::ompl_log_level + "')");

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

  parser.add_argflag("-k", "--ik-neighbors");
  parser.set_description("--ik-neighbors",
                      "Number of neighbors from the roadmap to start IK\n"
      "                (default is "
                       + std::to_string(defaults::ik_neighbors) + ")");

  parser.add_argflag("--ik-tolerance");
  parser.set_description("--ik-tolerance",
                      "Tip error stopping tolerance for IK.\n"
      "                (default is "
                        + std::to_string(defaults::ik_tolerance) + " meters)");

  parser.add_argflag("--ik-max-iters");
  parser.set_description("--ik-max-iters",
                      "Maximum number of iterations for IK.\n"
      "                (default is "
                        + std::to_string(defaults::ik_max_iters) + ")");

  parser.add_flag("--keep-disconnected-vertices");
  parser.set_description("--keep-disconnected-vertices",
                      "Usually, the disconnected vertices are removed after\n"
      "                loading the roadmap.  We usually keep only the\n"
      "                largest connected component after collision check.\n"
      "                Using this flag keeps the vertices that are\n"
      "                disconnected from the largest connected component.");

  // TODO: add --ik-mu-init
  // TODO: remove --lazy-add
  // TODO: add --ik-lazy-add
  // TODO: add --ik-auto-add
  // TODO: add --ik-accurate
  // TODO: add --timeout
  //
  //parser.add_argflag("-t", "--timeout");
  //parser.set_description("--timeout", "Timeout for planning in seconds\n"
  //    "                (default is " + std::to_string(defaults::timeout) + ")");
  //parser.add_flag("--ik-lazy-add");
  //parser.set_description("--ik-lazy-add",
  //                    "After finding one or more IK solutions that are\n"
  //    "                good, we connect them to the roadmap.  This\n"
  //    "                option makes those connections lazy, at least\n"
  //    "                the ones that weren't already evaluated.\n"
  //    "                Improves average runtime, but degrades the\n"
  //    "                worst-case time.");
  //parser.add_flag("--ik-auto-add");
  //parser.set_description("--ik-auto-add",
  //                    "When doing roadmapIk, do not add to the roadmap,\n"
  //    "                but instead add as a goal state if/when used in\n"
  //    "                planning.");
  //parser.add_flag("--ik-accurate");
  //parser.set_description("--ik-accurate",
  //                    "After IK config, try to connect from many nearest\n"
  //    "                neighbors in the map rather than just the neighbor\n"
  //    "                used for IK.  If using --ik-lazy-add, then this is\n"
  //    "                only used if all IK solutions were either invalid or\n"
  //    "                not within threshold.");

  parser.add_flag("--lazy-add");
  parser.set_description("--lazy-add",
                      "By default, the inverse kinematics step will\n"
      "                automatically add the found points to the roadmap\n"
      "                along with calculating the added edge voxelization\n"
      "                and doing collision checking.  This flag turns off\n"
      "                that behavior and makes the IK solutions added to the\n"
      "                roadmap and evaluated lazily (meaning only voxelizing\n"
      "                the collision checking the edges as-needed).");
}

} // end of unnamed namespace

int main(int arg_count, char* arg_list[]) {
  QCoreApplication app(arg_count, arg_list);
  rclcpp::init(arg_count, arg_list);

  CliParser parser;
  populate_parser(parser);
  parser.parse(arg_count, arg_list);

  auto problemfile     = parser["problem"];
  auto roadmapfile     = parser["roadmap"];
  auto planner_logfile = parser.get("--log", defaults::log);
  auto ik_neighbors    = parser.get("--ik-neighbors", defaults::ik_neighbors);
  auto ik_max_iters    = parser.get("--ik-max-iters", defaults::ik_max_iters);
  auto ik_tolerance    = parser.get("--ik-tolerance", defaults::ik_tolerance);
  auto ompl_log_level  = parser.get("--ompl-log-level", defaults::ompl_log_level);
  auto check_vert_validity = !parser.has("--skip-roadmap-vertex-check");
  auto check_edge_validity = !parser.has("--skip-roadmap-edge-check");
  bool remove_disconnected = !parser.has("--keep-disconnected-vertices");
  bool auto_add        = !parser.has("--lazy-add");

  util::set_ompl_log_level(ompl_log_level);

  auto problem = cpptoml::from_file<motion_planning::Problem>(problemfile);
  auto haptic_transform =
      cpptoml::from_file<haptic::HapticTransform>(problemfile);

  // get the interior voxels
  std::shared_ptr<collision::VoxelOctree> interior_voxels;
  InteriorTracker::IsInFunc is_in_func;
  if (problem.venv.interior_fname != "") {
    std::cout << "Interior collision checking enabled" << std::endl;
    interior_voxels = problem.venv.get_interior();
    is_in_func = [&interior_voxels, &problem](const QVector3D &p) {
      collision::Point point{p.x(), p.y(), p.z()};
      point = problem.venv.rotate_point(point);
      return interior_voxels->collides(point);
    };
  } else {
    is_in_func = [](const QVector3D &p) {
      UNUSED_VAR(p);
      return true;
    };
  }

  std::string frame = "/map";
  std::string env_namespace = "obstacles";
  std::string haptic_tip_namespace = "haptic-tip";
  std::string goal_tip_namespace = "goal-tip";
  std::string backbone_namespace = "tendon-backbone";

  std::cout << haptic_transform << std::endl;

  // main thread objects
  auto node = std::make_shared<rclcpp::Node>("haptic_planner");
  haptic::HapticQSubscriber listener(node);
  haptic::StreamingTransformer transformer;
  transformer.set_transform(haptic_transform.to_transform());
  GoalTracker goal_tracker;
  InteriorTracker interior_tracker(is_in_func);
  vistendon::StreamingRvizTipPublisher haptic_tip_publisher(
      node, frame, haptic_tip_namespace);
  vistendon::StreamingRvizTipPublisher goal_tip_publisher(
      node, frame, goal_tip_namespace);
  auto backbone_publisher =
      std::make_shared<vistendon::TendonBackboneRvizPublisher>(
        node, frame, backbone_namespace);
  vistendon::TrajectoryVisualizer robot_publisher(backbone_publisher);
  vistendon::EnvironmentRvizPublisher env_publisher(
      node, frame, env_namespace);

  haptic_tip_publisher.set_radius(problem.robot.r / 2.0);
  haptic_tip_publisher.set_yellow();
  goal_tip_publisher.set_radius(problem.robot.r);
  goal_tip_publisher.set_green();
  env_publisher.set_color(0.0f, 0.0f, 1.0f, 0.2f); // transparent blue
  env_publisher.set_env(problem.env);


  //
  // Setup Threads
  //

  using Planner = motion_planning::VoxelCachedLazyPRM;
  auto ikopt = Planner::RMAP_IK_SIMPLE;
  if (auto_add) { ikopt |= Planner::RMAP_IK_AUTO_ADD; }

  // planner thread
  QThread planner_thread;
  planner_thread.moveToThread(&planner_thread);
  motion_planning::StreamingPlanner planner(
      &problem,
      roadmapfile,
      nullptr,
      planner_logfile,
      ik_neighbors,
      ik_max_iters,
      ik_tolerance,
      ikopt,
      check_vert_validity,
      check_edge_validity,
      remove_disconnected);
  planner.moveToThread(&planner_thread);
  planner_thread.start();

  // plan interleaver thread
  QThread interleaver_thread;
  interleaver_thread.moveToThread(&interleaver_thread);
  vistendon::StreamingInterleaver interleaver(
      &problem.robot, problem.min_tension_change * 10.0);
  interleaver.moveToThread(&interleaver_thread);
  interleaver.set_smooth_tip(true);
  interleaver.set_tip_threshold(problem.robot.specs.dL * 2);
  interleaver_thread.start();


  //
  // Connections
  //
  
  // connect the haptic listener
  QObject::connect(
      &listener,    &haptic::HapticQSubscriber::new_pose,
      &transformer, &haptic::StreamingTransformer::streaming_transform);
  QObject::connect(
      &listener,    &haptic::HapticQSubscriber::white_button_pressed,
      &goal_tracker, &GoalTracker::emit_current_goal);
  QObject::connect(
      &listener,    &haptic::HapticQSubscriber::grey_button_pressed,
      &goal_tracker, &GoalTracker::emit_current_goal);

  // connect the streaming transformer
  QObject::connect(
      &transformer, &haptic::StreamingTransformer::transformed,
      &haptic_tip_publisher, &vistendon::StreamingRvizTipPublisher::update_tip);
  QObject::connect(
      &transformer, &haptic::StreamingTransformer::transformed,
      &goal_tracker, &GoalTracker::update_goal);
  QObject::connect(
      &transformer, &haptic::StreamingTransformer::transformed,
      &interior_tracker, &InteriorTracker::update_point);

  // connect the goal tracker (listening for button pushes)
  QObject::connect(
      &goal_tracker, &GoalTracker::new_goal,
      &goal_tip_publisher, &vistendon::StreamingRvizTipPublisher::update_tip);
  QObject::connect(
      &goal_tracker, &GoalTracker::new_goal,
      &planner, &motion_planning::StreamingPlanner::update_goal);
  QObject::connect(
      &goal_tracker, &GoalTracker::new_goal,
      [&interior_tracker, &goal_tip_publisher]() {
        if (interior_tracker.is_interior()) {
          goal_tip_publisher.mark_as_not_collision();
        } else {
          goal_tip_publisher.mark_as_collision();
        }
      });

  // connect the interior tracker
  QObject::connect(
      &interior_tracker, &InteriorTracker::exited,
      &haptic_tip_publisher, &vistendon::StreamingRvizTipPublisher::mark_as_collision);
  QObject::connect(
      &interior_tracker, &InteriorTracker::entered,
      &haptic_tip_publisher, &vistendon::StreamingRvizTipPublisher::mark_as_not_collision);

  // connect the streaming planner
  QObject::connect(
      &planner, &motion_planning::StreamingPlanner::new_plan,
      &interleaver, &vistendon::StreamingInterleaver::stream_interleave);
  QObject::connect(
      &planner, &motion_planning::StreamingPlanner::planning_begin,
      &goal_tracker, &GoalTracker::disable);
  QObject::connect(
      &planner, &motion_planning::StreamingPlanner::planning_begin,
      &haptic_tip_publisher, &vistendon::StreamingRvizTipPublisher::mark_disabled);

  // connect the streaming interleaver
  QObject::connect(
      &interleaver, &vistendon::StreamingInterleaver::next_shape,
      &robot_publisher, &vistendon::TrajectoryVisualizer::enqueue_shape,
      Qt::DirectConnection); // enqueueing is thread-safe.  Direct prevents copies
  QObject::connect(
      &interleaver, &vistendon::StreamingInterleaver::interleave_end,
      &goal_tracker, &GoalTracker::enable);
  QObject::connect(
      &interleaver, &vistendon::StreamingInterleaver::interleave_end,
      &haptic_tip_publisher, &vistendon::StreamingRvizTipPublisher::mark_enabled);


  //
  // start the event loops on the main thread
  //

  QtExecutor executor;
  executor.add_node(node);
  executor.start();

  auto exit_code = app.exec();
  rclcpp::shutdown();
  planner_thread.quit();
  planner_thread.wait();
  interleaver_thread.quit();
  interleaver_thread.wait();

  return exit_code;
}

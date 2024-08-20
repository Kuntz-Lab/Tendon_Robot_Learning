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
#include <util/macros.h>
#include <util/ompl_logging.h>
#include <vistendon/EnvironmentRvizPublisher.h>
#include <vistendon/StreamingInterleaver.h>
#include <vistendon/StreamingRvizTipPublisher.h>
#include <vistendon/TendonBackboneRvizPublisher.h>
#include <vistendon/TrajectoryVisualizer.h>
#include <tip-control/StreamingIK.h>

#include <3rdparty/libros2qt/qt_executor.h>

#include <rclcpp/rclcpp.hpp>

#include <QCoreApplication>
#include <QObject>
#include <QThread>
#include <QVector3D>

namespace {

namespace defaults {
  const std::string  log             = "streaming-planner.log";
  const size_t       ik_max_iters    = 10;
  const double       ik_tolerance    = 0.0005;
} // end of namespace defaults

void populate_parser(CliParser &parser) {
  parser.set_program_description(
      // TODO: update description
      "Listen to the haptic device on ROS and plan accordingly.  The tip\n"
    "  position of the haptic device is used to control the tip of the\n"
    "  robot.  A sphere representing the current tip position from the\n"
    "  haptic device will be sent to RViz.  When the button on the haptic\n"
    "  device is pressed, the current position will be set as the next goal\n"
    "  for planning, the planner will make a plan, and the trajectory will\n"
    "  be played and sent to RViz.\n"
    "\n"
    "  Transform information may be specified in the robot.toml file.  For\n"
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

  parser.add_positional("robot");
  parser.set_required("robot");
  parser.set_description("robot",
                      "robot toml file that may also have the\n"
      "                [haptic_transform] section.");

  parser.add_argflag("-l", "--log"); // CSV for timing events
  parser.set_description("--log",
                      "CSV file containing raw profiling data with computed\n"
      "                statistics at the end.  There are only two columns,\n"
      "                - name\n"
      "                - milestone: which milestone this applies to\n"
      "                - value\n"
      "                (default is '" + defaults::log + "')");

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
}

} // end of unnamed namespace

int main(int arg_count, char* arg_list[]) {
  QCoreApplication app(arg_count, arg_list);
  rclcpp::init(arg_count, arg_list);

  CliParser parser;
  populate_parser(parser);
  parser.parse(arg_count, arg_list);

  auto robotfile       = parser["robot"];
  auto logfile         = parser.get("--log", defaults::log);
  auto ik_max_iters    = parser.get("--ik-max-iters", defaults::ik_max_iters);
  auto ik_tolerance    = parser.get("--ik-tolerance", defaults::ik_tolerance);

  auto robot = cpptoml::from_file<tendon::TendonRobot>(robotfile);
  auto haptic_transform =
      cpptoml::from_file<haptic::HapticTransform>(robotfile);

  // get the interior voxels
  InteriorTracker::IsInFunc is_in_func =
    [&robot](const QVector3D &p) {
      return p.length() > robot.specs.L;
    };;

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
  // TODO: create a different class that filters to 60 Hz, but not as a movie
  vistendon::TrajectoryVisualizer robot_publisher(backbone_publisher);

  haptic_tip_publisher.set_radius(problem.robot.r / 2.0);
  haptic_tip_publisher.set_yellow();
  goal_tip_publisher.set_radius(problem.robot.r);
  goal_tip_publisher.set_green();


  //
  // Setup Threads
  //

  // IK thread
  QThread ik_thread;
  ik_thread.moveToThread(&ik_thread);
  Controller controller(robot);
  StreamingIK ik(controller);
  ik.set_max_iters(ik_max_iters);
  ik.set_tip_threshold(ik_tolerance);
  ik.moveToThread(ik_thread);
  ik_thread.start();


  //
  // Connections
  //
  // Top-level view
  //
  //   listener.new_pose
  //     -> transformer.streaming_transform
  //       -> goal_tracker.update_goal
  //       -> interior_tracker.update_goal
  //         -> haptic_tip_publisher.set_color
  //       -> haptic_tip_publisher
  //
  //   listener.button_pressed
  //     -> goal_tracker.emit_current_goal
  //       -> goal_tip_publisher
  //       -> ik
  //         -> robot_publisher
  //         -> (lambda)
  //           - convert to lengths
  //           - send to arduino
  
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
      &ik, &StreamingIK::set_goal);
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

  // connect the ik solver
  QObject::connect(
      &ik, &StreamingIK::solved_ik,
      [&robot](const QVector<double> state, double err) {
        qDebug() << "IK solution:  err =" << err << ", state =" << state;
        // TODO: convert state to lengths
        // TODO: send to arduino
      });

  //
  // start the event loops on the main thread
  //

  QtExecutor executor;
  executor.add_node(node);
  executor.start();

  auto exit_code = app.exec();
  rclcpp::shutdown();
  ik_thread.quit();
  ik_thread.wait();

  return exit_code;
}

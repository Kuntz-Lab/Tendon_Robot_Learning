#include "GoalTracker.h"
#include "InteriorTracker.h"

#include <tendon/TendonRobot.h>
#include <cliparser/CliParser.h>
#include <collision/Point.h>
#include <collision/CapsuleSequence.h>
#include <cpptoml/toml_conversions.h>
#include <haptic/HapticQSubscriber.h>
#include <haptic/HapticTransform.h>
#include <haptic/StreamingTransformer.h>
#include <util/macros.h>
#include <vistendon/EnvironmentRvizPublisher.h>
#include <vistendon/StreamingInterleaver.h>
#include <vistendon/StreamingRvizTipPublisher.h>
#include <vistendon/TendonBackboneRvizPublisher.h>
#include <vistendon/TrajectoryVisualizer.h>
#include <vistendon/TendonRvizPublisher.h>
#include <tip-control/StreamingIK.h>
#include <phys-robot/trajectory_conversion.h>
#include <phys-robot/MotorControlPublisher.h>
#include "serialport/SerialPort.h"

#include <3rdparty/libros2qt/qt_executor.h>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

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
                "  for IK, the configuration is found by the IK solver, and the motor controls will\n"
                "  be sent to the arduino on the robot.\n"
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
    parser.add_flag("-r", "--rviz-robot");
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
    trajectory_conversion conv;
    std::string port_name="/dev/ttyACM0";
    SerialPort serial_port(port_name);
    auto home_shape = robot.home_shape();
    std::vector<double> c = conv.get_motor_control_constant(robot);
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
    std::string home_namespace = "backbone-home";

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
//    auto backbone_publisher =
//            std::make_shared<vistendon::TendonBackboneRvizPublisher>(
//                node, frame, backbone_namespace);
//    // TODO: create a different class that filters to 60 Hz, but not as a movie

//    vistendon::TrajectoryVisualizer robot_publisher(backbone_publisher);
    vistendon::TendonBackboneRvizPublisher bpublisher(node, frame, backbone_namespace);
    vistendon::TendonBackboneRvizPublisher home_publisher(node, frame, home_namespace);
    vistendon::TendonRvizPublisher tendon_publisher(node, frame, "tendons");



    //auto [home_robot_shape, home_tendon_shapes] =
    //        vistendon::robot_and_tendon_shape(robot,std::vector{0.0,0.0,0.0,0.0}, home_shape);
    home_publisher.set_color(0.0f, 0.0f, 1.0f, 0.5f); // transparent blue
    home_publisher.set_robot(collision::CapsuleSequence{home_shape.p, robot.r});


    haptic_tip_publisher.set_radius(0.0075);
    haptic_tip_publisher.set_green();
    goal_tip_publisher.set_radius(robot.r);
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
    ik.moveToThread(&ik_thread);
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
    QObject::connect(
                &listener,    &haptic::HapticQSubscriber::white_button_pressed,
                &haptic_tip_publisher, &vistendon::StreamingRvizTipPublisher::set_red);
    QObject::connect(
                &listener,    &haptic::HapticQSubscriber::grey_button_pressed,
                &haptic_tip_publisher, &vistendon::StreamingRvizTipPublisher::set_red);
    QObject::connect(
                &listener,    &haptic::HapticQSubscriber::grey_button_released,
                &haptic_tip_publisher, &vistendon::StreamingRvizTipPublisher::set_green);
    QObject::connect(
                &listener,    &haptic::HapticQSubscriber::white_button_released,
                &haptic_tip_publisher, &vistendon::StreamingRvizTipPublisher::set_green);


    //     connect the streaming transformer

    QObject::connect(
                &transformer, &haptic::StreamingTransformer::transformed,
                &goal_tracker, &GoalTracker::update_goal);
    QObject::connect(
                &transformer, &haptic::StreamingTransformer::transformed,
                &interior_tracker, &InteriorTracker::update_point);
    QObject::connect(
                &transformer, &haptic::StreamingTransformer::transformed,
                &haptic_tip_publisher, &vistendon::StreamingRvizTipPublisher::update_tip);

    //     connect the goal tracker (listening for button pushes)
    //    QObject::connect(
    //                &goal_tracker, &GoalTracker::new_goal,
    //                &goal_tip_publisher, &vistendon::StreamingRvizTipPublisher::update_tip);
    QObject::connect(
                &goal_tracker, &GoalTracker::new_goal,
                &ik, &StreamingIK::set_goal);
    //    QObject::connect(
    //                &goal_tracker, &GoalTracker::new_goal,
    //                [&interior_tracker, &goal_tip_publisher]() {
    //        if (interior_tracker.is_interior()) {
    //            goal_tip_publisher.mark_as_not_collision();
    //        } else {
    //            goal_tip_publisher.mark_as_collision();
    //        }
    //    });

    //    //     connect the interior tracker
    //    QObject::connect(
    //                &interior_tracker, &InteriorTracker::exited,
    //                &haptic_tip_publisher, &vistendon::StreamingRvizTipPublisher::mark_as_collision);
    //    QObject::connect(
    //                &interior_tracker, &InteriorTracker::entered,
    //                &haptic_tip_publisher, &vistendon::StreamingRvizTipPublisher::mark_as_not_collision);

    // connect the ik solver
    QObject::connect(
                &ik, &StreamingIK::solved_ik,
                [&robot,&c,&serial_port,&conv,&home_shape](const QVector<double> state, double err) {
        qDebug() << "IK solution:  err =" << err << ", state =" << state;
        std::vector<double> motor_control= conv.length_to_motor_control_conversion(conv.tension_to_length_conversion(state.toStdVector(),robot),c);
        qDebug() << "Tendons are pulled by " << robot.calc_dl(home_shape.L_i,conv.tension_to_length_conversion(state.toStdVector(),robot));
        serial_port.writeData(conv.message_builder(motor_control).c_str());
    });

    QObject::connect(
                &ik, &StreamingIK::solved_ik,
                [&robot, &bpublisher, &tendon_publisher, &home_shape, &parser]
                (const QVector<double> state, [[maybe_unused]] double err) {
        if (parser.has("-r")){
        auto [robot_shape, tendon_shapes] =
            vistendon::robot_and_tendon_shape(robot, state.toStdVector(), home_shape);

        bpublisher.set_color(1.0f, 1.0f, 0.0f, 0.5f); // transparent yellow
        bpublisher.set_robot(std::move(robot_shape));

        tendon_publisher.set_color(0.0f, 0.0f, 0.0f, 1.0f); // solid black
        tendon_publisher.set_tendons(std::move(tendon_shapes));

        }
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

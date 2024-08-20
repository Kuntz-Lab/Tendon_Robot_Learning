#include <cliparser/CliParser.h>
#include <haptic/HapticQSubscriber.h>
#include <vistendon/StreamingRvizTipPublisher.h>

#include <3rdparty/libros2qt/qt_executor.h>

#include <rclcpp/rclcpp.hpp>

#include <QCoreApplication>
#include <QObject>
#include <QVector3D>
#include <QDebug>

namespace {

void populate_parser(CliParser &parser) {
  parser.set_program_description(
      "Listens to the haptic device on ROS2 topics and publishes the\n"
    "  tip position to RViz.  When the grey button is pressed, it\n"
    "  changes the tip to red.  When the white button is pressed, it\n"
    "  changes the tip to yellow.\n");
}

} // end of unnamed namespace

int main(int arg_count, char* arg_list[]) {
    QCoreApplication app(arg_count, arg_list);
    rclcpp::init(arg_count, arg_list);

    CliParser parser;
    populate_parser(parser);
    parser.parse(arg_count, arg_list);

    std::string frame = "/map";
    std::string haptic_tip_namespace = "haptic-tip";

    auto node = std::make_shared<rclcpp::Node>("haptic_tip_rviz_viewer");
    haptic::HapticQSubscriber listener(node);
    vistendon::StreamingRvizTipPublisher haptic_tip_publisher(
                node, frame, haptic_tip_namespace);
    haptic_tip_publisher.set_radius(0.0075);
    haptic_tip_publisher.set_green();

    // connect the haptic listener
    QObject::connect(
        &listener, &haptic::HapticQSubscriber::new_pose,
        &haptic_tip_publisher, &vistendon::StreamingRvizTipPublisher::update_tip);
    QObject::connect(
        &listener,    &haptic::HapticQSubscriber::white_button_pressed,
        &haptic_tip_publisher, &vistendon::StreamingRvizTipPublisher::set_yellow);
    QObject::connect(
        &listener,    &haptic::HapticQSubscriber::grey_button_pressed,
        &haptic_tip_publisher, &vistendon::StreamingRvizTipPublisher::set_red);
    // do the dumb thing and set green when released, ignoring if the other
    // button is still pressed
    QObject::connect(
        &listener,    &haptic::HapticQSubscriber::grey_button_released,
        &haptic_tip_publisher, &vistendon::StreamingRvizTipPublisher::set_green);
    QObject::connect(
        &listener,    &haptic::HapticQSubscriber::white_button_released,
        &haptic_tip_publisher, &vistendon::StreamingRvizTipPublisher::set_green);

    //
    // start the event loops on the main thread
    //

    QtExecutor executor;
    executor.add_node(node);
    executor.start();

    auto exit_code = app.exec();
    rclcpp::shutdown();

    return exit_code;
}

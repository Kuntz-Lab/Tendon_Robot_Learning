//#include <rclcpp/node.hpp>
//#include <rclcpp/logger.hpp>
//#include <rcl_interfaces/msg/parameter_descriptor.hpp>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "phys-robot/MotorControlPublisher.h"
#include "phys-robot/trajectory_conversion.h"
#include "csv/Csv.h"
#include "cliparser/CliParser.h"
#include "util/openfile_check.h"
#include "cpptoml/toml_conversions.h"
#include "tendon/TendonRobot.h"
#include "tendon/TendonSpecs.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include<fstream>
#include <algorithm>
#include <utility>
#include <vector>
#include <sstream>

#include <cassert>


void populate_parser(CliParser &parser) {
  parser.set_program_description(
        "Read in a trajectory and command the robot to follow it,\n"
        );


  parser.add_positional("plan-csv");
  parser.set_required("plan-csv");
  parser.set_description("plan-csv",
                      "CSV file with a plan to show.  Expected columns are\n"
      "                  - tau_i   (for i in [1 : # tendons])\n"
      "                  - theta   (if problem.enable_rotation)\n"
      "                  - s_start (if problem.enable_retraction)");

}


int main(int argc, char * argv[])
{
  CliParser parser;
  populate_parser(parser);
  parser.parse(argc, argv);
  std::vector<std::vector<double>> traj;
  trajectory_conversion conv;

  traj = conv.load_tension_trajectory(parser["plan-csv"]);

  tendon::TendonRobot robot;

  robot = cpptoml::from_file<tendon::TendonRobot>("phys_robot_limits.toml");

  auto home_shape = robot.home_shape();
  std::vector<double> L_h=home_shape.L_i;
  double x_h=0.04;
  std::vector<double> c;
  rclcpp::init(argc, argv);
  std::vector<std_msgs::msg::String> messages;
  for(auto l : L_h){
    c.push_back(x_h-l);
  }
  auto motor_controls = conv.length_to_motor_control_conversion(conv.tension_to_length_conversion(traj,robot),c);
  for (auto &step : motor_controls) {


    std_msgs::msg::String msg;
    msg.data = conv.message_builder(step);
    messages.push_back(msg);
  }

  auto control_node = std::make_shared<rclcpp::Node>("publish_control");
  MotorControlPublisher tip_publisher;
  auto timer = control_node->create_wall_timer(2s,
                                               [&tip_publisher,&messages]() -> void {
    static size_t i = 0;
    tip_publisher.publish(messages[i]);
    i = (i + 1) % messages.size();
  });

  rclcpp::spin(control_node);
  rclcpp::shutdown();
  return 0;
}

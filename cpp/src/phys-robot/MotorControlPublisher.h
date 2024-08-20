#ifndef TIP_POSITION_PUBLISHER_H
#define TIP_POSITION_PUBLISHER_H
//#include <rclcpp/node.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <rclcpp/logger.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

using namespace std::chrono_literals;

class MotorControlPublisher : public rclcpp::Node
{
  public:
    MotorControlPublisher()
    : Node("motor_control_publisher"), i(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    }

    void publish(std_msgs::msg::String msg){
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
      this->publisher_->publish(msg);
    }


  private:

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t i;
    const std::vector<std_msgs::msg::String> msg_;
  };

#endif


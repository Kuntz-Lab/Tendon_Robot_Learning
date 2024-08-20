#ifndef TIP_POSITION_SUBSCRIBER_H
#define TIP_POSITION_SUBSCRIBER_H
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "serialport/SerialPort.h"
using std::placeholders::_1;

class Subscriber : public rclcpp::Node, public SerialPort
{
  public:
    Subscriber()
    : Node("subscriber"), SerialPort("/dev/ttyACM0")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&Subscriber::topic_callback, this, _1));
    }

  private:

    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "From publisher, I heard '%s'", msg->data.c_str());
      writeData(msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

};

#endif

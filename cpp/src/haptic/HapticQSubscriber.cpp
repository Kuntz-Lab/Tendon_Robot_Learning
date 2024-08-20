#include "HapticQSubscriber.h"

#include <QDebug>

#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <functional> // for std::bind

using std::placeholders::_1;

namespace haptic {

HapticQSubscriber::HapticQSubscriber(rclcpp::Node::SharedPtr node, QObject* parent)
  : QObject(parent)
{
  _pose_subscriber = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/phantom/pose", 10,
      std::bind(&HapticQSubscriber::pose_callback, this, _1));
  _grey_button_subscriber = node->create_subscription<std_msgs::msg::Bool>(
      "/phantom/button_grey", 10,
      std::bind(&HapticQSubscriber::grey_button_callback, this, _1));
  _white_button_subscriber = node->create_subscription<std_msgs::msg::Bool>(
      "/phantom/button_white", 10,
      std::bind(&HapticQSubscriber::white_button_callback, this, _1));
}


//
// private methods
//

void HapticQSubscriber::pose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  QVector3D position(
      pose->pose.position.x,
      pose->pose.position.y,
      pose->pose.position.z
      );
  QQuaternion orientation (
      pose->pose.orientation.w,
      pose->pose.orientation.x,
      pose->pose.orientation.y,
      pose->pose.orientation.z
      );
  emit new_pose(position, orientation);
}

void HapticQSubscriber::grey_button_callback(
    const std_msgs::msg::Bool::SharedPtr is_down)
{
  emit grey_button_toggled(is_down->data);
  if (is_down->data) {
    emit grey_button_pressed();
  } else {
    emit grey_button_released();
  }
}

void HapticQSubscriber::white_button_callback(
    const std_msgs::msg::Bool::SharedPtr is_down)
{
  emit white_button_toggled(is_down->data);
  if (is_down->data) {
    emit white_button_pressed();
  } else {
    emit white_button_released();
  }
}

} // end of namespace haptic

#ifndef HAPTIC_Q_SUBSCRIBER_H
#define HAPTIC_Q_SUBSCRIBER_H

#include <QObject>
#include <QString>
#include <QQuaternion>
#include <QVector3D>

#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

namespace haptic {

class HapticQSubscriber : public QObject {
  Q_OBJECT

public:
  HapticQSubscriber(rclcpp::Node::SharedPtr node, QObject* parent = nullptr);

signals:
  void new_pose(const QVector3D &position, const QQuaternion &orientation);

  void grey_button_pressed();
  void grey_button_released();
  void grey_button_toggled(bool is_down);

  void white_button_pressed();
  void white_button_released();
  void white_button_toggled(bool is_down);

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
  void grey_button_callback(const std_msgs::msg::Bool::SharedPtr is_down);
  void white_button_callback(const std_msgs::msg::Bool::SharedPtr is_down);

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _pose_subscriber;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _grey_button_subscriber;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _white_button_subscriber;

}; // end of class HapticQSubscriber

} // end of namespace haptic

#endif // HAPTIC_Q_SUBSCRIBER_H

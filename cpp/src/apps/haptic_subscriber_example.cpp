#include <haptic/HapticQSubscriber.h>
#include <haptic/StreamingTransformer.h>

#include <tendon/config.h> // for TENDON_RCLCPP_VERSION_MAJOR definition
#include <3rdparty/libros2qt/qt_executor.h>

#include <rclcpp/rclcpp.hpp>

#include <QCoreApplication>
#include <QDebug>

int main(int arg_count, char* arg_list[]) {
  QCoreApplication app(arg_count, arg_list);
  rclcpp::init(arg_count, arg_list);

  auto node = std::make_shared<rclcpp::Node>("haptic_listener");
  haptic::HapticQSubscriber listener(node);

  // make connections for test_message that tests each signal
  QObject::connect(&listener , &haptic::HapticQSubscriber::new_pose,
      [](const QVector3D &position, const QQuaternion &orientation) {
        qDebug() << "emitted new_pose("
                      << "Position{" << position.x() << ", "
                                     << position.y() << ", "
                                     << position.z() << "}, "
                      << "Quaternion{" << orientation.x() << ", "
                                       << orientation.y() << ", "
                                       << orientation.z() << ", "
                                       << orientation.scalar() << "}"
                    << ")";
      });
  QObject::connect(&listener, &haptic::HapticQSubscriber::grey_button_toggled,
      [](bool is_down) {
        qDebug() << "emitted grey_button_toggled(" << is_down << ")";
      });
  QObject::connect(&listener, &haptic::HapticQSubscriber::white_button_toggled,
      [](bool is_down) {
        qDebug() << "emitted white_button_toggled(" << is_down << ")";
      });
  QObject::connect(&listener, &haptic::HapticQSubscriber::grey_button_pressed,
      []() {
        qDebug() << "emitted grey_button_pressed()";
      });
  QObject::connect(&listener, &haptic::HapticQSubscriber::grey_button_released,
      []() {
        qDebug() << "emitted grey_button_released()";
      });
  QObject::connect(&listener, &haptic::HapticQSubscriber::white_button_pressed,
      []() {
        qDebug() << "emitted white_button_pressed()";
      });
  QObject::connect(&listener, &haptic::HapticQSubscriber::white_button_released,
      []() {
        qDebug() << "emitted white_button_released()";
      });

  haptic::StreamingTransformer transformer;
  transformer.transform().scale(0.3);
  QObject::connect(&listener, &haptic::HapticQSubscriber::new_pose,
                   &transformer, &haptic::StreamingTransformer::streaming_transform);
  QObject::connect(&transformer, &haptic::StreamingTransformer::transformed,
      [](const QVector3D &vec) {
        qDebug() << "  transformed vector: ["
                 << vec.x() << ", "
                 << vec.y() << ", "
                 << vec.z() << "]";
      });

  QtExecutor executor;
  executor.add_node(node);
  executor.start();

  auto res = app.exec();
  std::cout << "Exited Qt thread\n";
  rclcpp::shutdown();
  return res;
}

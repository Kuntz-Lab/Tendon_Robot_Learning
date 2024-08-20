#ifndef RVIZ_MARKER_ARRAY_PUBLISHER
#define RVIZ_MARKER_ARRAY_PUBLISHER

#include "vistendon/marker_array_conversions.h"

#include <rclcpp/node.hpp>
#include <rclcpp/logger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <functional>
#include <limits>
#include <string>
#include <chrono>

#include <cmath>

namespace vistendon {

/// Subclass this and add methods to populate _markers and call publish()
class RvizMarkerArrayPublisher {
public:
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  RvizMarkerArrayPublisher(
      rclcpp::Node::SharedPtr node,
      const std::string &frame = "/map",
      const std::string &rviz_namespace = "tendon-backbone")
    : _frame(frame)
    , _rviz_namespace(rviz_namespace)
    , _color{0.0f, 1.0f, 0.0f, 1.0f} // solid green
    , _logger(node->get_logger())
  {
    using namespace std::chrono_literals;
    _publisher = node->create_publisher<MarkerArray>(
        "visualization_marker_array", 10);

    _timer = node->create_wall_timer(
        2s, std::bind(&RvizMarkerArrayPublisher::publish, this));
  }

  Color color() const { return _color; }

  // Note: not thread-safe
  void set_color(const Color &color) {
    _color = color;
    for (auto &marker : _markers.markers) {
      marker.color.r = _color.r;
      marker.color.g = _color.g;
      marker.color.b = _color.b;
      marker.color.a = _color.a;
    }
    RCLCPP_DEBUG(_logger, "Updated color to (%f, %f, %f, %f)",
                 color.r, color.g, color.b, color.a);
    publish();
  }

  void set_color(float r, float g, float b, float a = 1.0f) {
    set_color({r, g, b, a});
  }

  void publish() {
    this->_publisher->publish(_markers);
  }

protected:
  std::string _frame;
  std::string _rviz_namespace;
  Color _color;
  MarkerArray _markers;
  rclcpp::Logger _logger;
  rclcpp::Publisher<MarkerArray>::SharedPtr _publisher;
  rclcpp::TimerBase::SharedPtr _timer;
};

} // end of namespace vistendon

#endif // RVIZ_MARKER_ARRAY_PUBLISHER


#ifndef MANUAL_RVIZ_MARKER_ARRAY_PUBLISHER_H
#define MANUAL_RVIZ_MARKER_ARRAY_PUBLISHER_H

#include "RvizMarkerArrayPublisher.h"

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace vistendon {

class ManualRvizMarkerArrayPublisher : public RvizMarkerArrayPublisher {
public:
  using Marker      = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  ManualRvizMarkerArrayPublisher(
      rclcpp::Node::SharedPtr node,
      const std::string &frame = "/map",
      const std::string &rviz_namespace = "tendon-backbone")
    : RvizMarkerArrayPublisher(node, frame, rviz_namespace)
  {
    RCLCPP_DEBUG(_logger, "Created ManualRvizMarkerArrayPublisher");
  }

  void add_marker(const Marker &marker) {
    Marker copy = marker;
    add_marker(std::move(copy));
  }

  void add_marker(Marker &&marker) {
    populate_marker_attributes(marker, _rviz_namespace, _frame, _color);
    _markers.markers.emplace_back(std::move(marker));
    _markers.markers.back().id = _markers.markers.size();
  }

  void set_marker_array(const MarkerArray &markers) {
    _markers = markers;
    for (auto &marker : _markers.markers) {
      populate_marker_attributes(marker, _rviz_namespace, _frame, _color);
    }
    RCLCPP_DEBUG(_logger, "Updated manual markers");
    publish();
  }
};

} // end of namespace vistendon

#endif // MANUAL_RVIZ_MARKER_ARRAY_PUBLISHER_H

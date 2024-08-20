#ifndef TENDON_RVIZ_PUBLISHER_H
#define TENDON_RVIZ_PUBLISHER_H

#include "marker_array_conversions.h"

#include <rclcpp/node.hpp>
#include <rclcpp/logger.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <functional>
#include <limits>
#include <string>

#include <cmath>

namespace vistendon {

/// For sending rviz messages for a set of robot tendons
class TendonRvizPublisher : public RvizMarkerArrayPublisher {
public:
  TendonRvizPublisher(
      rclcpp::Node::SharedPtr node,
      const std::string &frame = "/map",
      const std::string &rviz_namespace = "tendons")
    : RvizMarkerArrayPublisher(node, frame, rviz_namespace)
  {
    RCLCPP_DEBUG(_logger, "Created TendonRvizPublisher");
  }

  // Note: not thread-safe
  void set_tendons(const std::vector<CapsuleSequence> &tendons) {
    _markers.markers.clear();
    for (auto &tendon : tendons) {
      auto tendon_marker = capsule_sequence_to_line_strip(
          tendon, _rviz_namespace, _frame, _color);
      tendon_marker.id = _markers.markers.size();
      _markers.markers.emplace_back(std::move(tendon_marker));
      //auto tendon_markers = capsule_sequence_to_markers(
      //    tendon, _rviz_namespace, _frame, _color);
      //for (auto &marker : tendon_markers.markers) {
      //  marker.id = _markers.markers.size();
      //  _markers.markers.emplace_back(marker);
      //}
    }
    RCLCPP_DEBUG(_logger, "Updated tendons");
    publish();
  }
};

} // end of namespace vistendon

#endif // TENDON_RVIZ_PUBLISHER_H

#ifndef ENVIRONMENT_RVIZ_PUBLISHER_H
#define ENVIRONMENT_RVIZ_PUBLISHER_H

#include "RvizMarkerArrayPublisher.h"
#include "marker_array_conversions.h"

namespace vistendon {

class EnvironmentRvizPublisher : public RvizMarkerArrayPublisher {
public:
  EnvironmentRvizPublisher(
      rclcpp::Node::SharedPtr node,
      const std::string &frame = "/map",
      const std::string &rviz_namespace = "obstacles")
    : RvizMarkerArrayPublisher(node, frame, rviz_namespace)
  {
    RCLCPP_DEBUG(_logger, "Created EnvironmentRvizPublisher");
  }

  // Note: not thread-safe
  void set_env(const motion_planning::Environment &env) {
    _markers = env_to_markers(
        env, _rviz_namespace, _frame, _color);
    RCLCPP_DEBUG(_logger, "Updated environment");
    publish();
  }
};

} // end of namespace vistendon

#endif // ENVIRONMENT_RVIZ_PUBLISHER_H

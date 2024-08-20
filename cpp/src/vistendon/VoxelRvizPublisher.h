#ifndef VOXEL_RVIZ_PUBLISHER_H
#define VOXEL_RVIZ_PUBLISHER_H

#include <collision/VoxelOctree.h>
#include <vistendon/marker_array_conversions.h>
#include <vistendon/RvizMarkerArrayPublisher.h>

#include <rclcpp/node.hpp>

namespace vistendon {

class VoxelRvizPublisher : public RvizMarkerArrayPublisher {
public:
  VoxelRvizPublisher(
      rclcpp::Node::SharedPtr node,
      const std::string &frame = "/map",
      const std::string &rviz_namespace = "environment")
    : RvizMarkerArrayPublisher(node, frame, rviz_namespace)
  {
    RCLCPP_DEBUG(_logger, "Created VoxelRvizPublisher");
  }

  void set_voxels(const collision::VoxelOctree &v) {
    _markers.markers.clear();
    _markers.markers.emplace_back(to_marker(
        v, _rviz_namespace, _frame, _color));
    RCLCPP_DEBUG(_logger, "Updated voxels");
    publish();
  }
};

} // end of namespace vistendon

#endif // VOXEL_RVIZ_PUBLISHER_H

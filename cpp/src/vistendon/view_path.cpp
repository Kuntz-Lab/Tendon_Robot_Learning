#include "vistendon/view_path.h"

#include "collision/CapsuleSequence.h"
#include "collision/Sphere.h"
#include "motion-planning/Environment.h"
#include "motion-planning/plan.h"
#include "vistendon/EnvironmentRvizPublisher.h"
#include "vistendon/TendonBackboneRvizPublisher.h"
#include "vistendon/TendonRvizPublisher.h"

#include <rclcpp/rclcpp.hpp>

#include <chrono>

using namespace std::chrono_literals;

using collision::Sphere;
using collision::CapsuleSequence;

namespace vistendon {

void view_path(
    const std::vector<std::vector<double>> &states,
    const tendon::TendonRobot& robot,
    const Eigen::Vector3d &start,
    const Eigen::Vector3d &goal)
{
  if (states.size() == 0) {
    std::cerr << "You must call view_path() with at least one configuration\n";
    return;
  }

  // initialize visualization
  auto tendon_node = std::make_shared<rclcpp::Node>("view_tendon");

  TendonBackboneRvizPublisher robot_publisher(tendon_node);
  TendonRvizPublisher tendon_publisher(tendon_node);
  EnvironmentRvizPublisher env_publisher(tendon_node);

  auto home_shape = robot.home_shape();

  {
    auto [start_robot_shape, start_tendon_shapes] =
        vistendon::robot_and_tendon_shape(robot, states[0], home_shape);

    robot_publisher.set_color(1.0f, 1.0f, 0.0f, 0.5f); // transparent yellow
    robot_publisher.set_robot(std::move(start_robot_shape));

    tendon_publisher.set_color(0.0f, 0.0f, 0.0f, 1.0f); // solid black
    tendon_publisher.set_tendons(std::move(start_tendon_shapes));
  }

  {
    motion_planning::Environment env;
    env.push_back(Sphere{start, 0.003});
    env.push_back(Sphere{goal, 0.003});
    env_publisher.set_color(1.0f, 0.0f, 0.0f, 1.0f); // red
    env_publisher.set_env(std::move(env));
  }

  std::vector<CapsuleSequence> plan_poses;
  std::vector<std::vector<CapsuleSequence>> plan_tendons;
  for (auto &state: states) {
    auto [robot_shape, tendon_shapes] =
        vistendon::robot_and_tendon_shape(robot, state, home_shape);
    plan_poses.emplace_back(std::move(robot_shape));
    plan_tendons.emplace_back(std::move(tendon_shapes));
  }

  // create timer to cycle through the plan
  RCLCPP_INFO(tendon_node->get_logger(), "plan size: %d", states.size());
  RCLCPP_INFO(tendon_node->get_logger(), "plan_poses size: %d", plan_poses.size());
  auto timer = tendon_node->create_wall_timer(33ms,
    [&plan_poses, &plan_tendons, &robot_publisher, &tendon_publisher]() -> void {
      static size_t i = 0;
      robot_publisher.set_robot(plan_poses[i]);
      tendon_publisher.set_tendons(plan_tendons[i]);
      i = (i + 1) % plan_poses.size();
    });

  rclcpp::spin(tendon_node);
  rclcpp::shutdown();
}

} // end of namespace vistendon

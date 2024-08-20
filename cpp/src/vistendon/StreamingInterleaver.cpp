#include "vistendon/StreamingInterleaver.h"
#include <util/vector_ops.h>

#include <iterator> // for std::make_move_iterator
#include <utility> // for std::move()

namespace vistendon {

//
// public methods
//

StreamingInterleaver::StreamingInterleaver(
    tendon::TendonRobot *robot,
    double dstate,
    QObject *parent)
  : QObject(parent)
  , _robot(robot)
  , _dstate(dstate)
  , _smooth_tip(true)
  , _tip_threshold(0.0)
{
  if (robot == nullptr) {
    throw std::invalid_argument("must provide a valid robot pointer");
  }
  _tip_threshold = robot->specs.dL;
}

std::vector<collision::CapsuleSequence> StreamingInterleaver::interleave(
    const motion_planning::Problem::PlanType &plan) const
{
  auto interpolated = this->interpolate(plan);
  auto shapes = this->to_shapes(interpolated);
  if (_smooth_tip) {
    tip_smoothing(interpolated, shapes);
  }
  return shapes;
}

//
// public slots
//

void StreamingInterleaver::stream_interleave(
    const motion_planning::Problem::PlanType &plan) const
{
  // TODO: do it in sections so that we can start displaying while we
  // TODO- interleave the rest of the plan (if it is large)
  emit interleave_begin();
  auto interleaved = this->interleave(plan);
  for (auto &shape : interleaved) {
    emit next_shape(shape);
  }
  emit interleave_end();
}

//
// private methods
//

StreamingInterleaver::PlanType StreamingInterleaver::interpolate(
    const StreamingInterleaver::PlanType &plan) const
{
  if (plan.size() < 2) { throw std::invalid_argument("plan is too short"); }
  PlanType interpolated;
  for (size_t i = 1; i < plan.size(); ++i) {
    auto range = util::range(plan[i-1], plan[i], _dstate);
    interpolated.insert(interpolated.end(),
                        std::make_move_iterator(range.begin()),
                        std::make_move_iterator(range.end()));
  }
  return interpolated;
}

StreamingInterleaver::ShapePlanType StreamingInterleaver::to_shapes(
    const StreamingInterleaver::PlanType &plan) const
{
  ShapePlanType shapes;
  shapes.reserve(plan.size());
  for (auto &state : plan) {
    auto fk = _robot->forward_kinematics(state);
    shapes.emplace_back(ShapeType{std::move(fk), _robot->r});
  }
  return shapes;
}

void StreamingInterleaver::tip_smoothing(
    StreamingInterleaver::PlanType &plan,
    StreamingInterleaver::ShapePlanType &poses) const
{
  if (plan.size() != poses.size()) {
    throw std::invalid_argument("argument sizes do not match");
  }
  PlanType new_plan;
  ShapePlanType new_poses;
  new_plan.reserve(plan.size());
  new_poses.reserve(poses.size());

  // pop off one element from given vectors and move to new vectors
  auto move_pop = [&plan, &poses, &new_plan, &new_poses]()
  {
    new_plan.emplace_back(std::move(plan.back()));
    new_poses.emplace_back(std::move(poses.back()));
    plan.pop_back();
    poses.pop_back();
  };

  // move the final position
  move_pop();

  // work my way from the back of the plan
  while (plan.size() > 0) {
    auto &prev_tip = new_poses.back().points.back();
    auto &curr_tip = poses.back().points.back();
    double dtip = (prev_tip - curr_tip).norm();

    // if the tip distance is larger than our threshold, then subdivide
    // otherwise, check if the next pose is within tip distance of our threshold
    if (dtip > _tip_threshold) {
      // I can add to the plan and poses so that the next while loop iteration
      // will check this subdivision.
      size_t N(std::ceil(dtip / _tip_threshold));
      auto plan_subdiv = util::linspace(plan.back(), new_plan.back(), N + 1);
      plan_subdiv.erase(plan_subdiv.begin());
      plan_subdiv.pop_back();
      auto poses_subdiv = this->to_shapes(plan_subdiv);
      plan.insert(plan.end(), std::make_move_iterator(plan_subdiv.begin()),
                              std::make_move_iterator(plan_subdiv.end()));
      poses.insert(poses.end(), std::make_move_iterator(poses_subdiv.begin()),
                                std::make_move_iterator(poses_subdiv.end()));
      continue;
    }

    // see if a future pose is still within the tip tolerance.
    // we can drop the poses between the previous and this future pose
    auto it = poses.rbegin();
    for (; it != poses.rend() && dtip < _tip_threshold; ++it) {
      auto &future_tip = it->points.back();
      dtip = (prev_tip - future_tip).norm();
    }
    auto to_remove = std::distance(poses.rbegin(), std::prev(it)) - 1;
    // skip these ones
    if (to_remove > 0) {
      plan.erase(std::prev(plan.end(), to_remove), plan.end());
      poses.erase(std::prev(poses.end(), to_remove), poses.end());
    }

    // move over the last one
    move_pop();
  }

  plan.clear();
  poses.clear();

  plan.reserve(new_plan.size());
  poses.reserve(new_poses.size());

  // move back and reverse ordering
  plan.insert(plan.end(), std::make_move_iterator(new_plan.rbegin()),
                          std::make_move_iterator(new_plan.rend()));
  poses.insert(poses.end(), std::make_move_iterator(new_poses.rbegin()),
                            std::make_move_iterator(new_poses.rend()));
}


} // end of namespace vistendon

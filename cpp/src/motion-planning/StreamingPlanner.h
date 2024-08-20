#ifndef STREAMING_PLANNER_H
#define STREAMING_PLANNER_H

#include "motion-planning/VoxelCachedLazyPRM.h"
#include "motion-planning/Problem.h"
#include <csv/Csv.h>
#include <util/FunctionTimer.h>
#include <util/vector_ops.h>

#include <QMetaType>
#include <QObject>
#include <QVector3D>

#include <Eigen/Core>

#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace ompl {
  namespace base {
    class State;
  }
}

namespace motion_planning {

//class VoxelCachedLazyPRM;

// TODO: Make this streamer work with any planner
// TODO: Instead of having this streamer initialize and load the roadmap, give
// TODO- it a set of functions, like FK(), IK(), plan(), etc.

// TODO: Contain into a different non-Qt object, then surround that object with
// TODO- this one.

class StreamingPlanner : public QObject {
  Q_OBJECT

public:
  StreamingPlanner(
      Problem *problem,
      const std::string &roadmap,
      QObject *parent = nullptr,
      const std::string &logfile = "streaming-planner.log",
      size_t ik_neighbors = 3,
      size_t ik_max_iters = 10,
      double ik_tolerance = 0.0005,
      VoxelCachedLazyPRM::RoadmapIkOpts ikopt = VoxelCachedLazyPRM::RMAP_IK_AUTO_ADD,
      bool check_vertex_validity = true,
      bool check_edge_validity = true,
      bool remove_disconnected = true);

  ~StreamingPlanner();

public slots:
  void update_goal(const QVector3D &goal);

signals:
  void planning_begin();
  void planning_end();
  void new_plan(const motion_planning::Problem::PlanType &plan);

private slots:
  void streaming_replan();

private:
  template <typename M, typename V>
  void log_event_generic(
      const std::string &name, const M &milestone, const V &value);

  template <typename V>
  void log_event(const std::string &name, const V &value);

  template <typename V>
  void log_milestone_event(const std::string &name, const V &value);

  template <typename T>
  void log_and_clear_timers(const T &timed_object);

  ompl::base::State* vec2newstate(const std::vector<double> &vec) const;
  std::vector<double> state2vec(ompl::base::State* state) const;
  bool state_vecs_equal(const std::vector<double> &a,
                        const std::vector<double> &b) const;

  motion_planning::Problem::PlanType replan_impl(const Eigen::Vector3d &goal_tip);

private:
  motion_planning::Problem *_problem;
  QVector3D _goal;
  QVector3D _last_goal;
  ompl::base::State *_current_state;
  std::shared_ptr<motion_planning::VoxelCachedLazyPRM> _planner;
  size_t _ik_neighbors;
  double _ik_tolerance;
  VoxelCachedLazyPRM::RoadmapIkOpts _ikopt;
  std::ofstream _logout;
  std::unique_ptr<csv::CsvWriter> _logger;
  size_t _milestone_count;
  // TODO: follow same interface as time-able objects
  util::FunctionTimer _milestone_timer;
  util::FunctionTimer _ik_timer;
  util::FunctionTimer _solve_timer;

}; // end of class StreamingPlanner

template <typename M, typename V>
void StreamingPlanner::log_event_generic(
    const std::string &name, const M &milestone, const V &value)
{
  *_logger << name << milestone << value;
  _logger->new_row();
}

template <typename V>
void StreamingPlanner::log_event(const std::string &name, const V &value) {
  log_event_generic(name, "N/A", value);
}

template <typename V>
void StreamingPlanner::log_milestone_event(const std::string &name,
                                           const V &value)
{
  log_event_generic(name, _milestone_count, value);
}

template <typename T>
void StreamingPlanner::log_and_clear_timers(const T &timed_object) {
  for (const auto &[name, timer] : timed_object->timers()) {
    log_milestone_event("calls:" + name, timer.get_times().size());
    if (timer.get_times().size() > 0) {
      auto timename = "time:" + name;
      auto stats = util::calc_stats(timer.get_times());
      log_milestone_event(timename + "-total",  stats.total);
      log_milestone_event(timename + "-min",    stats.min);
      log_milestone_event(timename + "-mean",   stats.mean);
      log_milestone_event(timename + "-median", stats.median);
      log_milestone_event(timename + "-max",    stats.max);
    }
  }
  timed_object->clear_timing();
}


} // end of namespace motion_planning

Q_DECLARE_METATYPE(motion_planning::Problem::PlanType);

#endif // STREAMING_PLANNER_H

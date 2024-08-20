#ifndef GOAL_TRACKER_H
#define GOAL_TRACKER_H

#include <QObject>
#include <QVector3D>

/** Tracks the current position and emits only when asked */
class GoalTracker : public QObject {
  Q_OBJECT

public:
  GoalTracker() = default;

public slots:
  void update_goal(const QVector3D &position) { _current_position = position; }

  void disable() { _enabled = false; }
  void enable()  { _enabled = true; }
  void set_enabled(bool enabled) { _enabled = enabled; }

  void emit_current_goal() {
    if (_enabled) {
      emit new_goal(_current_position);
    }
  }

signals:
  void new_goal(const QVector3D &position);

private:
  QVector3D _current_position {};
  bool _enabled {true};
};

#endif // GOAL_TRACKER_H

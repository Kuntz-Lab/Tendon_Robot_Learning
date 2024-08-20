#ifndef STREAMING_IK_H
#define STREAMING_IK_H

#include "Controller.h"

#include <QObject>
#include <QVector>
#include <QDebug>
#include <Eigen/Core>


namespace defaults_1 {
  const int max_iter = 100;
  const double mu_init = 1e-3;
  const double eps1 = 1e-9;
  const double eps2 = 1e-4;
  const double eps3 = 1e-4;
  const double fd_delta = 1e-6;
}


class StreamingIK : public QObject {
  Q_OBJECT

public:
  StreamingIK(Controller controller, QObject *parent = nullptr)
    : QObject(parent), _controller(std::move(controller))
  {

      _state.resize(_controller.robot().state_size(), 0.0);

  }

  // getters and setters
  int max_iters() const { return _max_iters; }
  void set_max_iters(int iters) { _max_iters = iters; }

  double tip_threshold() const { return _tip_threshold; }
  void set_tip_threshold(double threshold) { _tip_threshold = threshold; }

  // getters
  Controller& controller() { return _controller; }
  const Controller& controller() const { return _controller; }

  std::vector<double>& last_solved() { return _state; }
  const std::vector<double>& last_solved() const { return _state; }


public slots:
  void set_goal(const QVector3D &position) {


      _goal = Eigen::Vector3d{position.x(),position.y(),position.z()};
      qDebug()<<"Goal is"<<position;

      auto soln = _controller.inverse_kinematics(std::vector(_controller.robot().state_size(),0.0), _goal, _max_iters, defaults_1::mu_init, defaults_1::eps1,
                                          defaults_1::eps2,_tip_threshold, defaults_1::fd_delta);

      _state = soln.state;

      emit solved_ik(QVector<double>::fromStdVector(_state), soln.error);

  }

signals:
  void solved_ik(const QVector<double> state, double err);

private:
  Controller _controller;
  int _max_iters;
  double _tip_threshold;
  std::vector<double> _state;
  Eigen::Vector3d _goal;

}; // end of class StreamingIK

#endif // STREAMING_IK_H


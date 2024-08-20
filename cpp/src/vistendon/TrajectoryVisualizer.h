#ifndef TRAJECTORY_VISUALIZER_H
#define TRAJECTORY_VISUALIZER_H

#include "vistendon/TendonBackboneRvizPublisher.h"
#include <collision/CapsuleSequence.h>

#include <QObject>
#include <QThread>
#include <QTimer>

#include <boost/lockfree/spsc_queue.hpp> // single producer, single consumer

#include <memory>

namespace vistendon {

/** Handles a stream of frames and will process them no faster than 60 Hz. */
class TrajectoryVisualizer : public QObject {
  Q_OBJECT

public:
  TrajectoryVisualizer(std::shared_ptr<TendonBackboneRvizPublisher> publisher,
                       QObject *parent = nullptr)
    : QObject(parent)
    , _publisher(publisher)
    , _queue()
    , _frame_timer()
  {
    if (!_publisher) {
      throw std::invalid_argument("must give a valid publisher");
    }
    this->connect(&_frame_timer, &QTimer::timeout,
                  this, &TrajectoryVisualizer::visualize_next_frame);
    _frame_timer.start(16); // 60 Hz = 16 ms / frame (approximately)
  }

public slots:
  // can be called from only one other thread.
  // if full, will spin until it can be enqueued
  void enqueue_shape(const collision::CapsuleSequence &shape) {
    while (!_queue.push(shape)) {
      QThread::msleep(33); // sleep for two frame's time
    }
  }

private slots:
  // show the next frame only if there is one
  void visualize_next_frame() {
    collision::CapsuleSequence shape;
    if (_queue.pop(shape)) {
      _publisher->set_robot(shape);
    }
  }

private:
  std::shared_ptr<TendonBackboneRvizPublisher> _publisher;
  // the queue can hold only a few frames
  boost::lockfree::spsc_queue<
    collision::CapsuleSequence, boost::lockfree::capacity<5>> _queue;
  QTimer _frame_timer; // enforces the 60 Hz
}; // end of class TrajectoryVisualizer

} // end of namespace vistendon

#endif // TRAJECTORY_VISUALIZER_H

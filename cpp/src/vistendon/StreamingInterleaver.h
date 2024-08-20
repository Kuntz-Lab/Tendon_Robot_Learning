#ifndef STREAMING_INTERLEAVER_H
#define STREAMING_INTERLEAVER_H

#include <collision/CapsuleSequence.h>
#include <motion-planning/Problem.h>
#include <tendon/TendonRobot.h>

#include <QObject>

namespace vistendon {

/** Used for interleaving plans, only of the backbone - not the tendons (for now)
 *
 * emits a signal for the next shape as fast as it can.  The best thing to do
 * is to either capture these and store them in a vector or list, or even a
 * queue.
 *
 * Note: since this emits a type not registered with Qt, you either need to use
 * a Qt::DirectConnection with connect() or register the emitted type yourself.
 * If you are sending this from one thread to another, I recommend using a
 * lock-free thread-safe queue like boost::lockfree::spsc_queue.
 */
class StreamingInterleaver : public QObject {
  Q_OBJECT

public:
  using PlanType      = motion_planning::Problem::PlanType;
  using ShapeType     = collision::CapsuleSequence;
  using ShapePlanType = std::vector<ShapeType>;

  StreamingInterleaver(tendon::TendonRobot *robot,
                       double dstate,
                       QObject *parent = nullptr);

  tendon::TendonRobot& robot() { return *_robot; }
  const tendon::TendonRobot& robot() const { return *_robot; }

  double dstate() const { return _dstate; }
  void set_dstate(double dstate) { _dstate = dstate; }

  bool smooth_tip() { return _smooth_tip; }
  void set_smooth_tip(bool on = true) { _smooth_tip = on; }

  // only applicable if smooth_tip is enabled
  double tip_threshold() const { return _tip_threshold; }
  void set_tip_threshold(double threshold) { _tip_threshold = threshold; }

  // interleave in one lump sum
  ShapePlanType interleave(const PlanType &plan) const;

public slots:
  // may take a while and may emit many next_shape() signals
  void stream_interleave(const PlanType &plan) const;

signals:
  void interleave_begin() const;
  void interleave_end() const;
  void next_shape(const ShapeType &caps) const;

private:
  PlanType interpolate(const PlanType &plan) const;
  ShapePlanType to_shapes(const PlanType &plan) const;
  void tip_smoothing(PlanType &plan, ShapePlanType &shape_plan) const;

private:
  tendon::TendonRobot *_robot;         // robot description
  double               _dstate;        // largest state distance allowed
  bool                 _smooth_tip;    // below maximum tip movement threshold
  double               _tip_threshold; // threshold for smooth tip
}; // end of class StreamingInterleaver

} // end of namespace vistendon

#endif // STREAMING_INTERLEAVER_H

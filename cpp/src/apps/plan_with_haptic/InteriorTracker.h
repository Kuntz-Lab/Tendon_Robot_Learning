#ifndef INTERIOR_TRACKER_H
#define INTERIOR_TRACKER_H

#include <collision/Point.h>

#include <QObject>

#include <memory>
#include <functional>

/** Tracks the status of the given positions being within the interior region
 *
 * Occupied cells in the given VoxelOctree are "free" and mark the interior
 * region.  There are two signals, exited() and entered() that are
 * emitted when the state changes.
 */
class InteriorTracker : public QObject {
  Q_OBJECT
public:
  using IsInFunc = std::function<bool(const QVector3D&)>;

  InteriorTracker(IsInFunc &is_in, QObject *parent = nullptr)
    : QObject(parent)
    , _is_in(is_in)
    , _is_interior(false)
  {}

  bool is_interior() const { return _is_interior; }
  bool is_exterior() const { return !is_interior(); }

public slots:
  /** updates the state based on the point
   *
   * Returns true if the point is in the interior.  Emits entered() or exited()
   * if the state changes.
   */
  bool update_point(const QVector3D &point) {
    bool point_is_inside = _is_in(point);
    if (point_is_inside != _is_interior) {
      _is_interior = point_is_inside;
      if (_is_interior) {
        emit entered();
      } else {
        emit exited();
      }
    }
    return point_is_inside;
  }

signals:
  void entered() const;
  void exited() const;

private:
  IsInFunc &_is_in;
  bool _is_interior;
}; // end of class InteriorTracker

#endif // INTERIOR_TRACKER_H

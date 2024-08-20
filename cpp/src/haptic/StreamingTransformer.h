#ifndef STREAMING_TRANSFORMER_H
#define STREAMING_TRANSFORMER_H

#include <QObject>
#include <QMatrix4x4>

namespace haptic {

/** A streaming class that takes a position and transforms it (rotation,
 * translation, scaling, etc.).
 *
 * There are two modes to using this class.  The first is to call transform(vec)
 * on a QVector3D instance and get the transformed QVector3D.  The second is to
 * use the public slot streaming_transform() that takes a QVector3D.  After
 * transforming the vector, it is emitted in a signal transformed().
 */
class StreamingTransformer : public QObject {
  Q_OBJECT

public:
  StreamingTransformer(QObject *parent = nullptr)
    : QObject(parent), _transform() {}

  QMatrix4x4& transform() { return _transform; }
  const QMatrix4x4& transform() const { return _transform; }
  void set_transform(const QMatrix4x4 &transform) { _transform = transform; }

  QVector3D transform(const QVector3D &vec) { return _transform * vec; }

public slots:
  void streaming_transform(const QVector3D &vec) {
    auto new_vec = this->transform(vec);
    emit transformed(new_vec);
  }

signals:
  void transformed(const QVector3D &transformed);

private:
  QMatrix4x4 _transform;
}; // end of class StreamingTransformer

} // end of namespace haptic

#endif // STREAMING_TRANSFORMER_H

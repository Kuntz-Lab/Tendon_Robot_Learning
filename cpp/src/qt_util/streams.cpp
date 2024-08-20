#include "streams.h"

#include <QByteArray>
#include <QQuaternion>
#include <QString>
#include <QVector3D>

#include <ostream>

namespace qt_util {

std::ostream& operator<< (std::ostream &out, const QByteArray &b) {
  return out << b.constData();
}

std::ostream& operator<< (std::ostream &out, const QString &s) {
  return out << qPrintable(s);
}

std::ostream& operator<< (std::ostream &out, const QVector3D &v) {
  return out << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
}

std::ostream& operator<< (std::ostream &out, const QQuaternion &q) {
  return out << "Quat("
                  "w:" << q.scalar()
             << ", x:" << q.x()
             << ", y:" << q.y()
             << ", z:" << q.z() << ")";
}

} // end of namespace qt_util

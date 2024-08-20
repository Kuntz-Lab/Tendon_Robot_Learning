#ifndef qt_util_streams_h
#define qt_util_streams_h

#include <ostream>

class QByteArray;
class QQuaternion;
class QString;
class QVector3D;

namespace qt_util {

std::ostream& operator<< (std::ostream &out, const QByteArray &b);
std::ostream& operator<< (std::ostream &out, const QString &s);
std::ostream& operator<< (std::ostream &out, const QVector3D &v);
std::ostream& operator<< (std::ostream &out, const QQuaternion &q);

} // end of namespace qt_util

#endif // qt_util_streams_h

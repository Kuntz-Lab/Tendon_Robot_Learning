/**
 * author:       Michael Bentley
 * created:      07 July 2021
 * description:  ToolData class which contains information from the TX and BX
 *               commands
 */

#ifndef ndi_ToolData_H
#define ndi_ToolData_H

#include <QByteArray>
#include <QVector3D>
#include <QQuaternion>
#include <QtGlobal> // for quint32


namespace ndi {

struct ToolData {
  QByteArray   id;           // 2-character hex tool ID
  bool         missing;      // true means the tool is reported missing
  quint32      status;       // each bit means something different
  quint32      frame;        // like a time step
  QVector3D    position;     // position of the tool
  QQuaternion  orientation;  // orientation of the tool
  float        error;        // calculated error (only for 6DOF tools)

  // convert only the relevant regions of a TX and BX response into a ToolData
  static ToolData from_tx(const QByteArray &data);
  static ToolData from_bx(const QByteArray &data);
};

} // end of namespace ndi

#endif // ndi_ToolData_H

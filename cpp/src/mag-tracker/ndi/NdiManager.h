#ifndef ndi_NdiManager_h
#define ndi_NdiManager_h

#include "ToolData.h"
#include "ParseError.h"

#include <QByteArray>
#include <QObject>
#include <QQuaternion>
#include <QVector3D>
#include <QVector>

// forward declarations
class QSerialPort;

namespace ndi {

/** Manages state and communication
 */
class NdiManager : public QObject {
  Q_OBJECT
public:
  struct ConnectionError : public std::runtime_error
  { using std::runtime_error::runtime_error; };

  using ParseError = ::ndi::ParseError;

public:
  explicit NdiManager(QObject *parent = nullptr);
  ~NdiManager() noexcept;

  QSerialPort* serial_port() { return _serial; }
  const QSerialPort* serial_port() const { return _serial; }

  static QVector<qint32> available_bauds() {
    return {
      9600,
      38400,
      57600,
      115200,
      230400,
      921600,
    };
  }

  // create the serial port and initiate a connection
  void init(const QString &port, qint32 baud = 921600);

  /** Close the connection and delete the serial port (will be nullptr after)
   *
   * Can call init() again after calling close().
   */
  void close() noexcept;

  //
  // parsing static functions which can be used with data from send_raw()
  //

  // Returns true if the response indicates an error
  static bool is_error(const QByteArray &response) {
    return response.startsWith("ERROR");
  }

  /// Convert response starting with "ERROR" into a human-readable message
  static QString error_string(const QByteArray &response);

  /** returns the tool data from the TX and BX response
   *
   * Since these are static methods, they do not update the internal cache and
   * do not emit signals.  They are used within the class and are useful when
   * using the send_raw() interface.
   *
   * Throws ParseError if the responses are not in the correct format.
   */
  static QVector<ToolData> parse_tx(const QByteArray &response);
  static QVector<ToolData> parse_bx(const QByteArray &response);
  static QVector<QPair<QByteArray, quint32>>
    parse_phsr(const QByteArray &response);

public slots:
  /** sends directly to the serial port nicely
   *
   * If the command should end with a space and doesn't, one is added.
   * If the command does not end in a '\r', one is added.
   *
   * This will block until it returns or times out
   */
  QByteArray send_raw(const QByteArray &data, int timeout_ms = 1000);

  /// causes the appropriate signal to be emitted
  void parse_raw(const QByteArray &sent, const QByteArray &received);

  /// convenience to both send and parse
  QByteArray send_and_parse_raw(const QByteArray &data, int timeout_ms = 1000) {
    auto received = send_raw(data, timeout_ms);
    parse_raw(data, received);
    return received;
  }

  void initialize_tools();
  void start_tracking();

signals:
  /// emitted when parse_raw is given "OKAY" message
  void ok();

  /// an error message was parsed in parse_raw, emits a human-readable
  /// description
  void error(const QString &description);

  void apirev(const QByteArray &val);

  // status of the BEEP command
  void beeped(bool success);
  void beep_success();
  void beep_failure();

  void tool_initialized(const QByteArray &id);
  void tool_enabled(const QByteArray &id);

  // missing tool detected in a TX or BX reply
  void tool_missing(const QByteArray &id);

  void new_pose(const QByteArray &id,
                const QVector3D pos,
                const QQuaternion &quat,
                quint32 frame);

  // for unsupported and unrecognized replies
  void unrecognized(const QByteArray &reply);

private:
  QSerialPort *_serial;
};

} // end of namespace ndi

#endif // ndi_NdiManager_h

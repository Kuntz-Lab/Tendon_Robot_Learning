#include "NdiManager.h"
#include "char_parse.h"
#include "qt_util/asKeyValueRange.h"

#include <QDeadlineTimer>
#include <QDebug>
#include <QSerialPort>
#include <QThread>

#include <iostream>
#include <memory>
#include <cstring>

namespace ndi {

// helper functions
namespace {

/** Receives bytes until the '\r' character or until timeout
 *
 * The response is returned regardless.  You will know if it finished early if
 * the response ends with '\r'.
 */
QByteArray wait_for_text_response(QSerialPort *s, int timeout_ms = 1000) {
  // wait until the '\r' character or until timeout
  QByteArray response;
  QDeadlineTimer deadline(timeout_ms);
  do {
    if (s->bytesAvailable() > 0) {
      char ch;
      if (!s->getChar(&ch)) {
        throw NdiManager::ConnectionError(
            "Unable to read from the serial port when it"
            " reports that it is ready to read");
      }
      response.append(ch);
      if (ch == '\r') { break; }
    } else {
      s->waitForReadyRead(deadline.remainingTime());
    }
  } while (!deadline.hasExpired());

  if (deadline.hasExpired()) {
    std::cerr << "Warning: timed out while waiting for a text response"
              << std::endl;
  }

  return response;
}

/** Reads and returns a binary response (from the BX command)
 *
 * Blocks until the response has been fully read.  Bytes 3-4 encode a 16-bit
 * number with the number of bytes in the message (plus two more bytes for the
 * CRC16 code).  It will return when the number of bytes specified there have
 * been successfully read.
 *
 * Will return the full response.
 * - bytes 1-2: always '\xa5\xc4' for a response to the BX command (the only
 *   binary message so far)
 * - bytes 3-4: number of bytes in the payload
 * - remaining bytes: payload
 * - two more bytes: CRC16 code
 *
 * Does not end in a '\r' character
 */
QByteArray wait_for_bin_response(QSerialPort *s, int timeout_ms = 100) {
  // - first two bytes are \xa5\xc4 to indicate a proper response to a BX
  //   command
  QByteArray response;
  QDeadlineTimer deadline(timeout_ms);
  int payload_size = 0;

  // - next two bytes are the length of the message (plus two more bytes for
  //   the CRC16 code)
  // - read until we have read the full message

  do {
    if (s->bytesAvailable() > 0) {
      char ch;
      if (!s->getChar(&ch)) {
        throw NdiManager::ConnectionError(
            "Unable to read from the serial port when it"
            " reports that it is ready to read");
      }
      response.append(ch);
      if (response.size() == 4) {
        payload_size = bin_to_uint(response.mid(2, 2));
      } else if (response.size() > 4) {
        if (response.size() == payload_size + 8) {
          break;
        }
      }
    } else {
      s->waitForReadyRead(deadline.remainingTime());
    }
  } while (!deadline.hasExpired());

  if (deadline.hasExpired()) {
    std::cerr << "Warning: timed out while waiting for binary response"
              << std::endl;
    qDebug() << "Expected " << payload_size + 8 << " bytes";
  }

  return response;
}

QByteArray serial_send_raw(QSerialPort *s, QByteArray data,
                           int timeout_ms = 1000)
{
//  qDebug() << "sending " << data << " (timeout " << timeout_ms << " ms)";
  if (data.contains('\n') || data.contains('\r')) {
    std::cerr << "send_raw: contains a newline, not allowed.  Ignoring..."
              << std::endl;
    return "";
  }

  // If the command should end with a space and doesn't, one is added.
  if (!data.contains(' ')) {
    data.append(' ');
  }

  // If the command does not end in a '\r', one is added.
  data.append('\r');

  // send over the serial port
  s->write(data);

  // wait for the respnose
  QByteArray response;
  if (data.startsWith("bx") || data.startsWith("BX")) {
    response = wait_for_bin_response(s, timeout_ms);
//    qDebug() << "  response: (" << response.size() << " binary bytes )";
  } else {
    response = wait_for_text_response(s, timeout_ms);
    qDebug() << "  response: " << response;
  }
  return response;
}

} // end of unnamed namespace

//
// =Public Methods=
//

NdiManager::NdiManager(QObject *parent)
  : QObject(parent)
  , _serial(nullptr)
{ }

NdiManager::~NdiManager() noexcept {
  this->close();
}

void NdiManager::init(const QString &port, qint32 baud) {
  if (_serial) {
    throw std::runtime_error(
        "Already initialized, call close() before calling init() again.");
  }

  static const qint32 default_baud = 9600;

  // create the serial port in a std::unique_ptr
  auto s = std::make_unique<QSerialPort>(port);

  // connect at 9600 baud (the default baud rate)
  s->setBaudRate(default_baud);
  if (!s->open(QSerialPort::ReadWrite)) {
    throw ConnectionError("failed to connect to serial port: "
                          + s->errorString().toStdString());
  }

  QThread::msleep(2000); // allow connection to stablize

  // there is usually some garbage sent the first time, so send a dummy
  // string and ignore the returned message
  QByteArray response = serial_send_raw(s.get(), "echo hello");
  if (response.isEmpty()) {
    qDebug() << "No response received, perhaps it's on a different baud rate";
    qDebug() << "  Sending serial break to restart it";
    s->setBreakEnabled(true);
    QThread::msleep(100);
    s->setBreakEnabled(false);
    qDebug() << "  Done, you should have heard two beeps";
    qDebug() << "  Awaiting a response (can take up to 12 seconds)";
    wait_for_text_response(s.get(), 12000);
    qDebug() << "    Done.";
  }
  // doesn't matter what the response is, just that we got one

  // make sure it is a valid NDI machine
  response = serial_send_raw(s.get(), "init", 5000);
  if (response.startsWith("ERROR")) {
    throw ConnectionError(
        "Failed to initialize (" + response.toStdString() + "): "
        + error_string(response).toStdString());
  }
  if (!response.startsWith("OKAY")) {
    throw ConnectionError("Failed to confirm the device as an NDI machine");
  }

  // if the baud is different from 9600, set the baud on the device
  if (baud != default_baud) {
    QByteArray comm_command ("comm 10000");
    // change the '1' based on the desired baud rate
    switch (baud) {
      case   9600: comm_command[5] = '0'; break;
      case  19200: comm_command[5] = '2'; break;
      case  38400: comm_command[5] = '3'; break;
      case  57600: comm_command[5] = '4'; break;
      case 115200: comm_command[5] = '5'; break;
      case 921600: comm_command[5] = '6'; break;
      case 230400: comm_command[5] = 'A'; break;
      default: throw ConnectionError("Invalid baud rate provided");
    }
    response = serial_send_raw(s.get(), comm_command);
    if (response.startsWith("ERROR")) {
      throw ConnectionError(
          "Failed to change baud rate on the device: "
          + error_string(response).toStdString());
    }

    // change the baud of the serial port
    if (!s->setBaudRate(baud)) {
      throw ConnectionError(
          "Failed to change baud rate on QSerialPort on this device: "
          + s->errorString().toStdString());
    }
    QThread::msleep(100); // give it some time to take effect

    // confirm we can send new commands
    response = serial_send_raw(s.get(), "echo hello"); // ignore first response
    response = serial_send_raw(s.get(), "echo hello");
    if (!response.startsWith("hello")) {
      throw ConnectionError(
          "Could not confirm the baud rate change worked.");
    }
  }


  // pull the pointer out of the std::unique_ptr to keep it
  _serial = s.release();
}

void NdiManager::close() noexcept {
  if (_serial) { _serial->close(); }
  delete _serial;
  _serial = nullptr;
}


//
// =Public Static Methods=
//

QString NdiManager::error_string(const QByteArray &response) {
  if (!response.startsWith("ERROR")) {
    return "is not an error code..."; // just return the same thing...
  }

  auto code = hex_to_uint(response.mid(5, 2));
  switch (code) {
    case 0x01: return "Invalid command";
    case 0x02: return "Command too long";
    case 0x03: return "Command too short";
    case 0x04: return "Invalid CRC calculated for command; calculated CRC does"
                      " not match the one sent";
    case 0x05: return "Time-out on command execution";
    case 0x06: return "Unable to set up new communication parameters; happens"
                      " if one of the communication parameters is out of"
                      " range";
    case 0x07: return "Incorrect number of parameters";
    case 0x08: return "Invalid port handle selected";
    case 0x09: return "Invalid mode selected; either "
                      "(a) the tool tracking priority is out of range, or "
                      "(b) the tool has sensors deviced and 'button box' was"
                      " selected";
    case 0x0A: return "Invalid LED selected; out of range";
    case 0x0B: return "Invalid LED state selected; out of range";
    case 0x0C: return "Command is invalid while in the current operating mode";
    case 0x0D: return "No tool is assigned to the selected port handle";
    case 0x0E: return "Selected port handle not initialized; it must be"
                      " initialized before you can use this command";
    case 0x0F: return "Selected port handle not enabled; it must be enabled"
                      " before you can use this command";
    case 0x10: return "System not initialized; it must be initialized before"
                      " you can use this command";
    case 0x11: return "Unable to stop tracking; happens with hardware"
                      " problems";
    case 0x12: return "Unable to start tracking; happens with hardware"
                      " problems";
    case 0x13: return "Unable to initialize the port handle";
    case 0x14: return "Invalid Field Generator characterization parameters or"
                      " incompatible hardware";
    case 0x15: return "Unable to initialize the system; "
                      "(a) the system could not return to Setup mode, or "
                      "(b) there are internal hardware problems";
    case 0x19: return "Unable to read device's firmware revision information; "
                      "(a) the processor selected is out of range, or "
                      "(b) the system is unable to inquire firmware revision"
                      " information from a processor";
    case 0x1A: return "Internal system error; the system is unable to recover"
                      " after a system processing exception";
    case 0x1D: return "Unable to search for SROM device IDs";
    case 0x1E: return "Unable to read SROM device data; "
                      "(a) unable to auto-select the first SROM device on the"
                      " given port handle as a target to read from, or "
                      "(b) unable to read a page of SROM device data"
                      " successfully";
    case 0x1F: return "Unable to write SROM device data; "
                      "(a) starting address out of range, "
                      "(b) unable to autoselect the first SROM device, "
                      "(c) an SROM device was not previously selected with the"
                      " PSEL command, or "
                      "(d) the system is unable to write a page of SROM device"
                      " data successfully.";
    case 0x20: return "Unable to select SROM device for given port handle and"
                      " SROM device ID";
    case 0x23: return "Command parameter is out of range";
    case 0x24: return "Unable to select parameters by volume; either "
                      "(a) the selected valume is not available or "
                      "(b) there are internal hardware errors";
    case 0x29: return "Main processor firmware is corrupt";
    case 0x2A: return "No memory is available for dynamic allocation (heap is full)";
    case 0x2B: return "The requested port handle has not been allocated";
    case 0x2C: return "The requested port handle has become unoccupied";
    case 0x2D: return "All handles have been allocated";
    case 0x31: return "Invalid input or output state";
    case 0x33: return "Feature not available";
    case 0x34: return "User parameter does not exist";
    case 0x35: return "Invalid value type (e.g., string instead of integer)";
    case 0x36: return "User parameter value set is out of valid range";
    case 0x37: return "User parameter array index is out of valid range";
    case 0x38: return "User parameter size is incorrect";
    case 0x39: return "Permission denied; file or user parameter is read-only";
    case 0x3A: return "Reply buffer too small";
    case 0x42: return "Device not present; occurs when the command is specific"
                      " to a device that is not connected to the system";
    case 0xC5: return "The data bits parameter (set using COMM) must be set to"
                      " 8 bits in order to use the BX command";
    case 0xF4: return "Unable to erase Flash SROM device";
    case 0xF5: return "Unable to write Flash SROM device";
    case 0xF6: return "Unable to read Flash SROM device";

    // all reserved codes
    case 0x16:
    case 0x17:
    case 0x18:
    case 0x1B:
    case 0x1C:
    case 0x00:
    case 0x21:
    case 0x22:
    case 0x25:
    case 0x26:
    case 0x27:
    case 0x28:
    case 0x2E:
    case 0x2F:
    case 0x30:
    case 0x32:
    case 0x3B:
    case 0x3C:
    case 0x3D:
    case 0x3E:
    case 0x3F:
    case 0x40:
    case 0x41:
    case 0x43:
    case 0x44:
    case 0x45:
    case 0x46:
    case 0x47:
    case 0x48:
    case 0x49:
    case 0x4A:
    case 0x4B:
    case 0x4C:
    case 0x4D:
    case 0x4E:
    case 0x4F:
    case 0x50:
    case 0x51:
    case 0x52:
    case 0x53:
    case 0x54:
    case 0x55:
    case 0x56:
    case 0x57:
    case 0x58:
    case 0x59:
    case 0x5A:
    case 0x5B:
    case 0x5C:
    case 0x5D:
    case 0x5E:
    case 0x5F:
    case 0x60:
    case 0x61:
    case 0x62:
    case 0x63:
    case 0x64:
    case 0x65:
    case 0x66:
    case 0x67:
    case 0x68:
    case 0x69:
    case 0x6A:
    case 0x6B:
    case 0x6C:
    case 0x6D:
    case 0x6E:
    case 0x6F:
    case 0x70:
    case 0x71:
    case 0x72:
    case 0x73:
    case 0x74:
    case 0x75:
    case 0x76:
    case 0x77:
    case 0x78:
    case 0x79:
    case 0x7A:
    case 0x7B:
    case 0x7C:
    case 0x7D:
    case 0x7E:
    case 0x7F:
    case 0x80:
    case 0x81:
    case 0x82:
    case 0x83:
    case 0x84:
    case 0x85:
    case 0x86:
    case 0x87:
    case 0x88:
    case 0x89:
    case 0x8A:
    case 0x8B:
    case 0x8C:
    case 0x8D:
    case 0x8E:
    case 0x8F:
    case 0x90:
    case 0x91:
    case 0x92:
    case 0x93:
    case 0x94:
    case 0x95:
    case 0x96:
    case 0x97:
    case 0x98:
    case 0x99:
    case 0x9A:
    case 0x9B:
    case 0x9C:
    case 0x9D:
    case 0x9E:
    case 0x9F:
    case 0xA0:
    case 0xA1:
    case 0xA2:
    case 0xA3:
    case 0xA4:
    case 0xA5:
    case 0xA6:
    case 0xA7:
    case 0xA8:
    case 0xA9:
    case 0xAA:
    case 0xAB:
    case 0xAC:
    case 0xAD:
    case 0xAE:
    case 0xAF:
    case 0xB0:
    case 0xB1:
    case 0xB2:
    case 0xB3:
    case 0xB4:
    case 0xB5:
    case 0xB6:
    case 0xB7:
    case 0xB8:
    case 0xB9:
    case 0xBA:
    case 0xBB:
    case 0xBC:
    case 0xBD:
    case 0xBE:
    case 0xBF:
    case 0xC0:
    case 0xC1:
    case 0xC2:
    case 0xC3:
    case 0xC4:
    case 0xC6:
    case 0xC7:
    case 0xC8:
    case 0xC9:
    case 0xCA:
    case 0xCB:
    case 0xCC:
    case 0xCD:
    case 0xCE:
    case 0xCF:
    case 0xD0:
    case 0xD1:
    case 0xD2:
    case 0xD3:
    case 0xD4:
    case 0xD5:
    case 0xD6:
    case 0xD7:
    case 0xD8:
    case 0xD9:
    case 0xDA:
    case 0xDB:
    case 0xDC:
    case 0xDD:
    case 0xDE:
    case 0xDF:
    case 0xE0:
    case 0xE1:
    case 0xE2:
    case 0xE3:
    case 0xE4:
    case 0xE5:
    case 0xE6:
    case 0xE7:
    case 0xE8:
    case 0xE9:
    case 0xEA:
    case 0xEB:
    case 0xEC:
    case 0xED:
    case 0xEE:
    case 0xEF:
    case 0xF0:
    case 0xF1:
    case 0xF2:
    case 0xF3:
    case 0xF7:
    case 0xF8:
    case 0xF9:
    case 0xFA:
    case 0xFB:
    case 0xFC:
    case 0xFD:
    case 0xFE:
    case 0xFF:
    default:
      return "Reserved";
  }
}

QVector<ToolData> NdiManager::parse_tx(const QByteArray &response) {
  auto num_handles = hex_to_uint(response.left(2));

  auto lines = response.mid(2).split('\n'); // from character 2+: split into lines
  if (quint32(lines.count()) != num_handles + 1) {
    throw ParseError("TX: unexpected number of handles present");
  }

  QVector<ToolData> tools;
  for (quint32 i = 0; i < num_handles; ++i) {
    tools.append(ToolData::from_tx(lines[i]));
  }

  // TODO: analyze system status
  //auto system_status_bytes = lines.back().left(4);

  return tools;
}

QVector<ToolData> NdiManager::parse_bx(const QByteArray &response) {
  // verify the start of the message
  if (!response.startsWith("\xc4\xa5")) {
    throw ParseError("BX start sequence not correct");
  }

  // verify the length of the message
  auto len = bin_to_uint(response.mid(2, 2));
  if (quint32(response.size()) != len + 8) { // payload + start + length + 2*CRC
    throw ParseError("BX reply does not match expected length");
  }

  // bytes 4-5 are CRC of the header: ignored for now

  auto num_tools = bin_to_uint(response.mid(6, 1));  // one byte for # tools

  int idx = 7;
  int tool_msg_size = 42; // number of bytes for one tool update
  QVector<ToolData> tools;
  tools.reserve(num_tools);
  for (quint32 n = 0; n < num_tools; ++n) {
    tools.append(ToolData::from_bx(
          response.mid(idx + tool_msg_size * n, tool_msg_size)));
  }

  return tools;
}

QVector<QPair<QByteArray, quint32>> NdiManager::parse_phsr(
    const QByteArray &response)
{
  if (response.startsWith("ERROR")) { return {}; }

  auto num_handles = hex_to_uint(response.left(2));
  QVector<QPair<QByteArray, quint32>> tool_statuses;
  tool_statuses.reserve(num_handles);
  for (decltype(num_handles) i = 0; i < num_handles; ++i) {
    tool_statuses.append({response.mid(2 + 5 * i, 2),
                          hex_to_uint(response.mid(4 + 5 * i, 3))});
  }
  return tool_statuses;
}

//
// =Public Slots=
//

QByteArray NdiManager::send_raw(const QByteArray &data, int timeout_ms) {
  return serial_send_raw(_serial, data, timeout_ms);
}

void NdiManager::parse_raw(const QByteArray &sent, const QByteArray &received) {

  // handle errors first
  if (received.startsWith("ERROR")) {
    emit error(error_string(received));
    return;
  }

  auto lsent = sent.toLower();
  auto rec_msg = received.left(received.size() - 5); // remove CRC and '\r'

  if (lsent.startsWith("pinit")) {
    emit tool_initialized(sent.right(2));
  } else if (lsent.startsWith("pena")) {
    emit tool_enabled(sent.mid(5, 2));
  }

  // first handle generic responses
  if (received.startsWith("OKAY")) {
    emit ok();
    return;
  }

  // next handle responses specific to sent commands

  // pose information
  if (lsent.startsWith("bx") || lsent.startsWith("tx")) {
    decltype(parse_bx(received)) tools;
    if      (lsent.startsWith("bx")) { tools = parse_bx(received); }
    else if (lsent.startsWith("tx")) { tools = parse_tx(received); }
    for (auto &tool : tools) {
      if (tool.missing) {
        emit tool_missing(tool.id);
//        qDebug()<<"tool missing";
      } else {
        // TODO: emit status
        // TODO: do we want to just emit the tool?
//          qDebug()<<"emitting new pose";
        emit new_pose(tool.id, tool.position, tool.orientation, tool.frame);
      }
    }
  }

  // api revision
  else if (lsent.startsWith("apirev")) { emit apirev(rec_msg); }

  // beep status
  else if (lsent.startsWith("beep")) {
    if (rec_msg[0] == '0') {
      emit beeped(false);
      emit beep_failure();
    } else {
      emit beeped(true);
      emit beep_success();
    }
  }

  // TODO: handle PHINF tool info command response
  // else if (lsent.startswith("phinf")) { }
  // TODO: handle PHSR tool status command response (subcommands 00-04)
  // else if (lsent.startswith("phsr")) { }

  // anything else
  else { emit unrecognized(rec_msg); }
}

void NdiManager::initialize_tools() {
  // This implements the flow chart in their documentation

  auto check_send = [this](const QByteArray &to_send, int timeout_ms = 1000) {
    auto reply = send_raw(to_send, timeout_ms);
    if (reply.startsWith("ERROR")) {
      parse_raw(to_send, reply);
      throw ParseError("Got an error in initialize_tools()");
    }
    return reply;
  };

  // get port handles that need to be freed
  auto tools = parse_phsr(check_send("phsr 01"));

  // free them
  for (auto &tool : tools) {
    check_send("phf " + tool.first);
  }

  // keep initializing until there's nothing left to initialize
  while (true) {
    // get port handles that need to be initialized
    tools = parse_phsr(check_send("phsr 02"));

    if (tools.isEmpty()) {
      break;
    } else {
      // initialize them
      for (auto &tool : tools) {
        check_send("pinit " + tool.first, 5000);
      }
    }
  }

  // get port handles that need to be enabled
  tools = parse_phsr(check_send("phsr 03"));

  // enable them
  for (auto &tool : tools) {
    check_send("pena " + tool.first + "D");
  }
}

void NdiManager::start_tracking() {
  initialize_tools();
  
  auto check_send = [this](const QByteArray &to_send, int timeout_ms = 1000) {
    auto reply = send_raw(to_send, timeout_ms);
    if (reply.startsWith("ERROR")) {
      parse_raw(to_send, reply);
      throw ParseError("Got an error in initialize_tools()");
    }
    return reply;
  };

  check_send("tstart", 5000);
  QThread::msleep(50);
}

//
// =Private Methods=
//

} // end of namespace ndi

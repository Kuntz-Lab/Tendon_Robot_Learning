#include "qt_util/StdinIO.h"
#include "qt_util/StdinWatcher.h"

#include <algorithm>
#include <memory>
#include <stdexcept>

#include <cstring>

namespace qt_util {

StdinIO::StdinIO(QObject *parent)
  : QIODevice(parent)
  , _watcher(new StdinWatcher(this))
{
  connect(_watcher, &StdinWatcher::newline,
          this, &StdinIO::receiveNewline);
}

StdinIO::~StdinIO() {
  delete _watcher;
  _watcher = nullptr;
}

qint64 StdinIO::readData(char *data, qint64 maxSize) {
  auto readSize = std::min<qint64>(maxSize, _buf.size());
  std::memcpy(data, _buf.data(), sizeof(char) * readSize);
  _buf = _buf.right(_buf.size() - readSize);
  return readSize;
}

qint64 StdinIO::writeData([[maybe_unused]] const char *data,
                          [[maybe_unused]] qint64 maxSize) {
  throw std::runtime_error("Cannot write to stdin");
}

void StdinIO::receiveNewline(const QByteArray &data) {
  _buf += data;
  emit readyRead();
}

} // end of namesapce qt_util

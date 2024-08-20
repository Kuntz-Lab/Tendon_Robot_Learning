#ifndef STDIN_IO_H
#define STDIN_IO_H

#include "qt_util/StdinWatcher.h"

#include <QByteArray>
#include <QIODevice>

namespace qt_util {

class StdinIO : public QIODevice {
public:
  StdinIO(QObject *parent = nullptr);
  virtual ~StdinIO() override;

  const StdinWatcher* watcher() const { return _watcher; }
  StdinWatcher* watcher() { return _watcher; }

  bool isSequential() const override { return true; }
  qint64 bytesAvailable() const override {
    return _buf.size() + QIODevice::bytesAvailable();
  }

protected:
  virtual qint64 readData(char *data, qint64 maxSize) override;
  virtual qint64 writeData(const char *data, qint64 maxSize) override;

protected slots:
  void receiveNewline(const QByteArray &data);

private:
  StdinWatcher *_watcher;
  QByteArray    _buf;
};

} // end of namespace qt_util

#endif // STDIN_IO_H

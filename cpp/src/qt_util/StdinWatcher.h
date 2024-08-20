#ifndef qt_util_StdinWatcher_h
#define qt_util_StdinWatcher_h

#include <QByteArray>
#include <QObject>
#include <QThread>
#include <QTimer>

#include <functional>

namespace qt_util {

class StdinWatcher : public QObject {
  Q_OBJECT

public:
  StdinWatcher (QObject *parent = nullptr);
  StdinWatcher (bool in_separate_thread, QObject *parent = nullptr);
  ~StdinWatcher();

  void set_prompt_generator(std::function<QString()> &&f) {
    _prompt_generator = f;
  }

signals:
  void newline(const QByteArray &data);

private:
  QThread *_background_thread;
  QTimer _timer;
  std::function<QString()> _prompt_generator;
};

} // end of namespace qt_util

#endif // qt_util_StdinWatcher_h

#include "StdinWatcher.h"

#include <QCoreApplication>
#include <QMetaObject>
#include <QtGlobal>

#include <iostream>
#include <string>
#include <stdexcept>

#include <stdio.h>
#include <readline/readline.h>
#include <readline/history.h>

namespace qt_util {

namespace {

struct EofException : public std::runtime_error
{ using std::runtime_error::runtime_error; };

QByteArray get_input(const char *prompt) {
  // Get a line from the user.
  auto line_read = readline (prompt);
  if (line_read == nullptr) {
    throw EofException("Stdin is now closed");
  }
  QByteArray line (line_read);

  // If the line has any text in it, save it on the history.
  if (line_read && *line_read) {
    add_history (line_read);
  }
  free(line_read);

  return line;
}

} // end of unnamed namespace

StdinWatcher::StdinWatcher (QObject *parent) : StdinWatcher(true, parent) { }

StdinWatcher::StdinWatcher (bool in_separate_thread, QObject *parent)
  : QObject(parent)
{
  if (in_separate_thread) {
    _background_thread = new QThread(this);
    _timer.moveToThread(_background_thread);
    QObject::connect(_background_thread, &QThread::started,
                     &_timer, qOverload<>(&QTimer::start));
    _background_thread->start();
  } else {
    _timer.start();
  }
  _timer.setInterval(1);
  QObject::connect(&_timer, &QTimer::timeout,
      [this, in_separate_thread]() {
        while (true) {
          QString prompt;
          if (_prompt_generator) {
            prompt = _prompt_generator();
          } else {
            //std::cout << "Warning: prompt generator is not set" << std::endl;
            prompt = " > ";
          }

          QByteArray line;
          try {
            line = get_input(qPrintable(prompt));
          } catch (EofException&) {
            QMetaObject::invokeMethod(QCoreApplication::instance(), "quit",
                                      Qt::QueuedConnection);
            QMetaObject::invokeMethod(&_timer, "stop", Qt::QueuedConnection);
            if (in_separate_thread) { _background_thread->quit(); }
            return;
          }

          // this is done on the background thread
          //std::string input;
          //std::getline(std::cin, input);
          auto connect_type = Qt::BlockingQueuedConnection;
          if (!in_separate_thread) { connect_type = Qt::DirectConnection; }
          QMetaObject::invokeMethod(this, "newline", connect_type,
                                    Q_ARG(QByteArray, line));
        }
      });
}

StdinWatcher::~StdinWatcher() {
  if (_background_thread) {
    _background_thread->quit();
    if (!_background_thread->wait(2)) {
      std::cout << "Killing stdin watcher's background thread" << std::endl;
      _background_thread->terminate();
      _background_thread->wait();
    }
  }
}

} // end of namespace qt_util

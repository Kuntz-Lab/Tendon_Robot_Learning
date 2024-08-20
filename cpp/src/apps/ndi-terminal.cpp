#include "cliparser/CliParser.h"
#include "mag-tracker/ndi/NdiManager.h"
#include "mag-tracker/svd_reg.h"
#include "qt_util/StdinWatcher.h"
#include "qt_util/streams.h"
#include "cpptoml/toml_conversions.h"
#include "csv/Csv.h"

#include <QCoreApplication>
#include <QVector3D>
#include <QQuaternion>
#include <QString>
#include <QSerialPortInfo>
#include <QTime>

#include <string>
#include <sstream>
#include <iomanip>
#include <fstream>

#include <cstring>

#include <readline/readline.h>
#include <Eigen/Core>

using csv::CsvWriter;
using ndi::NdiManager;
using qt_util::StdinWatcher;
using qt_util::operator<<;

namespace {

namespace defaults {
  qint32 baud = 921'600;
} // end of namespace defaults

void populate_parser(CliParser &parser) {
  parser.set_program_description(
      "A simple terminal interface for the NDI Aurora machine.  Sends the\n"
      "  characters from stdin to the NDI machine (each time the user\n"
      "  presses enter) and prints a human readable version of the\n"
      "  message from the NDI Aurora to the terminal.");

  parser.add_positional("port");
  parser.set_required("port");
  parser.set_description("port", "serial port for the NDI machine");

  parser.add_argflag("-b", "--baud");
  parser.set_description("--baud",
                      "Set the baud rate of the serial port connection to\n"
      "                this value.  Possible values are\n"
      "                  - 9600\n"
      "                  - 38400\n"
      "                  - 57600\n"
      "                  - 115200\n"
      "                  - 921600\n"
      "                  - 230400\n"
      "                (default is " + std::to_string(defaults::baud) + ")");

  parser.add_flag("-r", "--raw");
  parser.set_description("--raw",
                      "Puts the terminal in raw mode instead of the default\n"
      "                behavior.  By default, the user is presented a menu\n"
      "                of commands and is limited to that interface.\n"
      "                Instead, this raw mode lets you send commands\n"
      "                directly through the serial port to the NDI\n"
      "                equipment.  Instead of printing the exact replies, a\n"
      "                human-readable printout is given to the user.  In\n"
      "                that way, this terminal program differs from using\n"
      "                the serial port directly using something like screen\n"
      "                or picocom.");

  parser.add_flag("-s", "--stream");
  parser.set_description("--stream",
                      "Start tracking and stream it at 40 Hz.  Does not work\n"
      "                with --raw.");

  parser.add_argflag("-o", "--stream-output");
  parser.set_description("--stream-output",
                      "It outputs the\n"
      "                position and orientation information to the given CSV\n"
      "                file.");
  parser.add_argflag("-t", "--transform");
  parser.set_description("--transform",
                      "It reads a transform from a toml file\n");

}

void dispatch_command([[maybe_unused]] const QByteArray command,
                      [[maybe_unused]] NdiManager &manager) {
  // TODO: call the appropriate command to the manager.
  qDebug() << "would now dispatch '" + command + "'";
}

void print_menu() {
  // TODO: print the supported commands to the terminal
  qDebug() << "would print menu";
}

char **command_completion(const char *, int, int);
char *command_generator(const char *, int);

const char *possible_commands[] = {
  "apirev",
  "beep",
  "bx",
  "comm",
  "echo",
  "init",
  "pena",
  "phinf",
  "phsr",
  "pinit",
  "reset",
  "tstart",
  "tstop",
  "tx",
  "vsel",
  NULL
};

char ** command_completion(const char *text,
                           [[maybe_unused]] int start, [[maybe_unused]] int end)
{
    rl_attempted_completion_over = 1;
    return rl_completion_matches(text, command_generator);
}

char * command_generator(const char *text, int state)
{
  static int list_index, len;
  const char *name;

  if (!state) {
    list_index = 0;
    len = strlen(text);
  }

  while ((name = possible_commands[list_index++])) {
    if (strncmp(name, text, len) == 0) {
      return strdup(name);
    }
  }

  return nullptr;
}

} // end of unnamed namespace

int main(int arg_count, char *arg_list[]) {
  // setup tab-completion with commands on GNU readline library
  rl_attempted_completion_function = command_completion;

  QCoreApplication app(arg_count, arg_list);
  CliParser cli_parser;
  populate_parser(cli_parser);
  cli_parser.parse(arg_count, arg_list);

  QString port      = QString::fromStdString(cli_parser["port"]);
  qint32  baud      = cli_parser.get("--baud", defaults::baud);
  bool    raw       = cli_parser.has("--raw");
  bool    streaming = cli_parser.has("--stream");
//                   || cli_parser.has("--stream-output");

  std::unique_ptr<std::ofstream> fout;
  std::unique_ptr<CsvWriter> writer;
  if (cli_parser.has("--stream-output")) {
    fout.reset(new std::ofstream(cli_parser["--stream-output"]));
    writer.reset(new CsvWriter(*fout));
    writer->write_row<std::string>(
        {"time", "id", "frame", "x", "y", "z", "qw", "qx", "qy", "qz"});
  }

  NdiManager manager;

  // handle streaming data
  QObject::connect(&manager, &NdiManager::new_pose,
      [&writer,&cli_parser](const QByteArray &id,
         const QVector3D &pos, const QQuaternion &quat, quint32 frame)
      {
      auto p=pos;
      if(cli_parser.has("--transform")){
          auto RT=cpptoml::from_file<SVD_Reg>(cli_parser["--transform"]);
          auto temp_pos=RT.compute_transformed_points(pos);
          p.setX(temp_pos(0));
          p.setY(temp_pos(1));
          p.setZ(temp_pos(2));

      }
      if (writer) {
          *writer << QTime::currentTime().msecsSinceStartOfDay()
                  << id.constData()
                  <<frame
                  << p.x()
                  << p.y()
                  << p.z()
                  << quat.scalar()
                  << quat.x()
                  << quat.y()
                  << quat.z();
          writer->new_row();
        }
        qDebug() << QString(id) + " frame:       " << frame;
        qDebug() << QString(id) + " position:    " << p;
        qDebug() << QString(id) + " orientation: " << quat;
      });
  QObject::connect(&manager, &NdiManager::error,
      [](const QString &description) {
        qDebug() << "Error: " << description;
      });
  QObject::connect(&manager, &NdiManager::tool_initialized,
      [](const QByteArray &id) {
        qDebug() << "Initialized tool " + QString(id);
      });
  QObject::connect(&manager, &NdiManager::tool_enabled,
      [](const QByteArray &id) {
        qDebug() << "Enabled tool     " + QString(id);
      });

  try {
    manager.init(port, baud);
  } catch(const NdiManager::ConnectionError &e) {
    std::cerr << "Failed to open port " << port
              << ", error: " << e.what() << std::endl;
    auto available_ports = QSerialPortInfo::availablePorts();
    std::cout << "Available ports:\n";
    for (auto &p : available_ports) {
      std::cout << " - " << p.portName() << "\n";
    }
    std::cout << std::flush;
    return 1;
  }
  std::cout << "Successfully connected to " << port << " with baud " << baud
            << std::endl;

  StdinWatcher *watcher = nullptr;
  if (!streaming) {
    manager.initialize_tools();
    qDebug() << "\nTools are now all initialized\n";
    watcher = new StdinWatcher(false, &app);
    watcher->set_prompt_generator([]() -> QString {
          static int i = 1;
          return " " + QString::number(i++) + " ndi> ";
        });
    QObject::connect(watcher, &StdinWatcher::newline,
        [&manager, raw](const QByteArray &data) {
          if (raw) {
            std::cout << std::endl;
            int timeout_ms = 1000;
            auto ldata = data.toLower();
            if (ldata.startsWith("tstart") || ldata.startsWith("pinit")
                || ldata.startsWith("init")) {
              timeout_ms = 5000;
            } else if (ldata.startsWith("reset")) {
              timeout_ms = 12000;
            }
            manager.send_and_parse_raw(data, timeout_ms);
            std::cout << std::endl;
          } else {
            // make my own menu instead of sending raw bytes
            dispatch_command(data, manager);
          }
        });
  }

  // Note: most connections are handled in the NdiManager

  if (!raw && !streaming) { print_menu(); }
  std::cout << std::endl;

  QTimer streaming_timer;
  std::ostringstream buf;
  buf.setf(std::ios::fixed, std:: ios::floatfield);
  buf.precision(7);
  if (streaming) {
    streaming_timer.start(25);
    manager.start_tracking();
    qDebug() << "\nTools are now all initialized and tracking\n\n\n";

    QObject::connect(&streaming_timer, &QTimer::timeout,
        [&manager, &buf]() {
          manager.send_and_parse_raw("bx", 20);
        });
    // turn off qDebug() messages
//    qInstallMessageHandler(
//        [](QtMsgType type,
//           const QMessageLogContext &context,
//           const QString &msg)
//        {
//          // do nothing...
//        });
  }

  return app.exec();
}

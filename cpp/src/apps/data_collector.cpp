#include "serialport/SerialPort.h"
#include "cliparser/CliParser.h"
#include "tip-control/Controller.h"
#include "phys-robot/trajectory_conversion.h"
#include "cpptoml/toml_conversions.h"
#include "csv/Csv.h"
#include "qt_util/streams.h"
#include "util/openfile_check.h"
#include "util/vector_ops.h"
#include "mag-tracker/ndi/NdiManager.h"
#include "mag-tracker/svd_reg.h"
#include "tendon/TendonRobot.h"
#include "csv/Csv.h"

#include<fstream>
#include <vector>

void populate_parser(CliParser &parser) {
    parser.set_program_description(
                "Takes a csv file consisting of configurations, commands it to the robot and reads and outputs the sensor readings to the output csv file. Main purpose is to collect data for learning methods\n"
                );


    parser.add_positional("sensor_port");
    parser.add_positional("robot_port");
    parser.add_positional("tensions_file");
    parser.add_positional("output_file");




}
using ndi::NdiManager;
using csv::CsvWriter;
using qt_util::operator<<;
int main(int argCount,char *argList[]){
    QCoreApplication(argCount,argList);
    CliParser parser;
    populate_parser(parser);
    parser.parse(argCount, argList);




    std::string port_name=parser["robot_port"];
    SerialPort s_port(port_name);

    std::vector<std::string> msg;

    auto robot = cpptoml::from_file<tendon::TendonRobot>("phys_robot_limits.toml");
    trajectory_conversion conv;
    std::vector<double> c  = conv.get_motor_control_constant(robot);
    NdiManager manager;
    auto RT = cpptoml::from_file<SVD_Reg>("transform.toml");
    qint32 baud = 921600;
    QString port      = QString::fromStdString(parser["sensor_port"]);
    bool tool_reading=false;
    std::unique_ptr<std::ofstream> fout;
    std::unique_ptr<CsvWriter> writer;
    fout.reset(new std::ofstream(parser["output_file"]));
    writer.reset(new CsvWriter(*fout));
    writer->write_row<std::string>(
    {"Tension_1","Tension_2","Tension_3","Tension_4","sensed_x", "sensed_y", "sensed_z"});

    QObject::connect(&manager, &NdiManager::new_pose,
                     [&writer, &parser, &RT]
                     ([[maybe_unused]] const QByteArray &id,
                      const QVector3D &pos,
                      [[maybe_unused]] const QQuaternion &quat,
                      [[maybe_unused]] quint32 frame)
    {

        auto trans_pos=RT.compute_transformed_points(pos);



        if (writer) {
            *writer << trans_pos(0)
                    << trans_pos(1)
                    << trans_pos(2);
            writer->new_row();
        }

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
    QObject::connect(&manager, &NdiManager::tool_missing,
                     [&tool_reading](){
        tool_reading=false;
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


    manager.start_tracking();
    qDebug() << "\nTools are now all initialized and tracking\n\n\n";

    std::vector<std::vector<double>> traj;
    {
        std::ifstream in;
        util::openfile_check(in, parser["tensions_file"]);
        csv::CsvReader reader(in);
        csv::CsvRow row;


        while (reader >> row) {
            traj.emplace_back(std::vector{std::stod(row["Tension_1"]),
                                          std::stod(row["Tension_2"]),
                                          std::stod(row["Tension_3"]),
                                          std::stod(row["Tension_4"])
                              });
        }


    }
    sleep(2);
    for (auto& step:traj){
        std::vector<double> motor_control= conv.length_to_motor_control_conversion(robot.shape(step).L_i, c);
        s_port.writeData(conv.message_builder(motor_control).c_str());
        sleep(5);
        if (writer) {
            *writer << step[0]
                    << step[1]
                    << step[2]
                    << step[3];
        }
        manager.send_and_parse_raw("bx",20);
    }



}

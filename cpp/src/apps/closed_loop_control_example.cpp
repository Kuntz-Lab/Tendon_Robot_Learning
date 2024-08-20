#include "cliparser/CliParser.h"
#include "collision/CapsuleSequence.h"
#include "cpptoml/toml_conversions.h"
#include "tendon/TendonRobot.h"
#include "tendon/TendonSpecs.h"
#include "tip-control/Controller.h"
#include "tip-control/tip_control.h"
#include "util/openfile_check.h"  // for openfile_check()
#include "util/vector_ops.h"      // for operator<<()
#include "mag-tracker/ndi/NdiManager.h"
#include "serialport/SerialPort.h"
#include "phys-robot/trajectory_conversion.h"
#include "qt_util/streams.h"
#include "mag-tracker/svd_reg.h"
#include "vistendon/ManualRvizMarkerArrayPublisher.h"

#include <Eigen/Core>
#include <Eigen/LU>
#include <QSerialPortInfo>
#include <3rdparty/libros2qt/qt_executor.h>

#include <rclcpp/rclcpp.hpp>
#include <QCoreApplication>
#include <QObject>
#include <QThread>
#include <QVector3D>

#include <chrono>
#include <fstream>
#include <iomanip>
#include <ios>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>

#include <cmath>
#include <cstdlib>

namespace E = Eigen;



namespace defaults {
const int max_iter = 100;
const double mu_init = 1e-3;
const double eps1 = 1e-9;
const double eps2 = 1e-4;
const double eps3 = 1e-4;
const double fd_delta = 1e-6;
} // end of namespace defaults

using csv::CsvWriter;
using ndi::NdiManager;
using qt_util::operator<<;
void populate_parser(CliParser &parser) {
    parser.add_positional("port");
    parser.set_required("port");
    parser.add_positional("tip-traj");
    parser.set_required("tip-traj");
    parser.add_flag("-c", "--clamped");

    parser.set_description("--clamped", "Use the clamped version of the controller.\n"
                                        "                By default, we use the non-clamped version which does not\n"
                                        "                limit the step size in each iteration.\n");
    parser.set_description("tip-traj", "specify csv file with tip trajectory");
}


int main(int argCount, char* argList[]) {
    QCoreApplication app(argCount, argList);
    rclcpp::init(argCount, argList);
    using util::operator<<;

    CliParser parser;
    populate_parser(parser);
    parser.parse(argCount, argList);
    QString port      = QString::fromStdString(parser["port"]);
    char clamped_char;

    tendon::TendonRobot robot;
    NdiManager manager;
    robot = cpptoml::from_file<tendon::TendonRobot>("phys_robot_limits.toml");
    auto RT=cpptoml::from_file<SVD_Reg>("transform.toml");

    // initial state

    std::vector<double> state(robot.state_size(), 0.0);
    if (parser.has("--clamped")) {
        clamped_char = 'y';
    } else {
        clamped_char = 'n';
    }
    qint32 baud = 921600;
    bool tool_reading=false;
    float tiptol=1e-2;
    //float tol=1e-10;
    std::string port_name="/dev/ttyACM0";
    size_t i = 0;
    SerialPort s_port(port_name);
    sleep(1);
    Controller cont(robot);
    Eigen::Vector3d sensor_tip_pos,prev_sensor_tip_pos(0,0,0);
    trajectory_conversion conv;
    std::vector<double> c  = conv.get_motor_control_constant(robot);
    auto tendon_node = std::make_shared<rclcpp::Node>("publish_tip_pos");
    vistendon::ManualRvizMarkerArrayPublisher tip_publisher(tendon_node,"/map","tip-pos"),des_publisher(tendon_node,"/map","desired-points");
    using Marker = visualization_msgs::msg::Marker;
    Marker samples_marker,des_marker;
    samples_marker.type = Marker::SPHERE_LIST;
    samples_marker.action = Marker::ADD;
    samples_marker.scale.x = 0.001;
    samples_marker.scale.y = 0.001;
    samples_marker.scale.z = 0.001;
    samples_marker.pose.position.x = 0;
    samples_marker.pose.position.y = 0;
    samples_marker.pose.position.z = 0;
    samples_marker.pose.orientation.w = 1;
    samples_marker.pose.orientation.x = 0;
    samples_marker.pose.orientation.y = 0;
    samples_marker.pose.orientation.z = 0;
    des_marker=samples_marker;

    geometry_msgs::msg::Point p,des_p;
    std_msgs::msg::ColorRGBA color,des_color;

    auto calc_error = [&robot](const E::Vector3d &tip_pos, const E::Vector3d &des) {
        double err = (tip_pos - des).norm();
        return err;
    };
    std::vector<std::vector<double>> tip_traj;
    {
        std::ifstream in;
        util::openfile_check(in, parser["tip-traj"]);
        csv::CsvReader reader(in);
        csv::CsvRow row;


        while (reader >> row) {
            tip_traj.emplace_back(std::vector{std::stod(row["x"]),
                                              std::stod(row["y"]),
                                              std::stod(row["z"])
                                  });
        }

    }

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

    QTimer sensor_timer;
    sensor_timer.start(25);


    QObject::connect(&sensor_timer, &QTimer::timeout,
                     [&manager]() {
        manager.send_and_parse_raw("bx", 20);
    });



    QObject::connect(&manager,&NdiManager::new_pose,
                     [&]([[maybe_unused]] const QByteArray &id,
                         const QVector3D &pos,
                         [[maybe_unused]] const QQuaternion &quat,
                         [[maybe_unused]] quint32 frame)
    {

        sensor_tip_pos=RT.compute_transformed_points(pos);
        qDebug()<<"sensor tip pos"<<sensor_tip_pos(0)<<","<<sensor_tip_pos(1)<<","<<sensor_tip_pos(2);
        visualization_msgs::msg::MarkerArray m_array;
        p.x = sensor_tip_pos[0];
        p.y = sensor_tip_pos[1];
        p.z = sensor_tip_pos[2];
        color.r=1.0;
        color.g=1.0;
        color.b=1.0;
        color.a=1.0;
        samples_marker.colors.emplace_back(color);
        samples_marker.points.emplace_back(p);
        m_array.markers.emplace_back(samples_marker);
        tip_publisher.set_marker_array(m_array);
        auto tip_pos=sensor_tip_pos;

        E::Vector3d des_pos;
        des_pos(0)=tip_traj[i][0];
        des_pos(1)=tip_traj[i][1];
        des_pos(2)=tip_traj[i][2];
        auto error=calc_error(des_pos,tip_pos);
        qDebug()<<"Error is"<<error;
        if(error<tiptol){
            qDebug()<<"reached point";
            i++;
        }
        else{
            state=cont.closed_loop_control(state,des_pos,tip_pos,clamped_char);
            std::vector<double> motor_control= conv.length_to_motor_control_conversion(robot.shape(state).L_i, c);
            s_port.writeData(conv.message_builder(motor_control).c_str());
        }
        if(i >= tip_traj.size()){
            sensor_timer.stop();
        }


    });


    QtExecutor executor;
    executor.add_node(tendon_node);
    executor.start();

    auto exit_code = app.exec();
    rclcpp::shutdown();



    return exit_code;
}

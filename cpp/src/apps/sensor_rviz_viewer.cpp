#include "mag-tracker/ndi/NdiManager.h"
#include "mag-tracker/svd_reg.h"
#include "vistendon/ManualRvizMarkerArrayPublisher.h"
#include "3rdparty/libros2qt/qt_executor.h"
#include "util/vector_ops.h"      // for operator<<()
#include "qt_util/streams.h"
#include <rclcpp/rclcpp.hpp>
#include <QCoreApplication>
#include <QObject>
#include <QVector3D>
#include <QSerialPortInfo>
#include <QTimer>


using ndi::NdiManager;
using qt_util::operator<<;
int main(int argCount, char * argList[]){
    QCoreApplication app(argCount,argList);
    rclcpp::init(argCount,argList);
    auto RT=cpptoml::from_file<SVD_Reg>("transform.toml");
    auto tendon_node = std::make_shared<rclcpp::Node>("publish_tip_pos");
    vistendon::ManualRvizMarkerArrayPublisher tip_publisher(tendon_node,"/map","tip-pos"),ws_publisher(tendon_node,"/map","workspace");
    using Marker = visualization_msgs::msg::Marker;
    Marker samples_marker,ws_marker;
    visualization_msgs::msg::MarkerArray ws_array;
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

    ws_marker.type=Marker::CUBE;
    ws_marker.action=Marker::ADD;
    ws_marker.scale.x=0.12;
    ws_marker.scale.y = 0.1;
    ws_marker.scale.z = 0.06;
    ws_marker.pose.position.x = 0;
    ws_marker.pose.position.y = -0.05;
    ws_marker.pose.position.z = 0.16;
    ws_marker.pose.orientation.w = 1;
    ws_marker.pose.orientation.x = 0;
    ws_marker.pose.orientation.y = 0;
    ws_marker.pose.orientation.z = 0;
    ws_publisher.add_marker(ws_marker);
    ws_publisher.set_color(0.0f,0.0f,1.0f,0.2f);
    ws_publisher.publish();





    bool tool_reading=false;
    Eigen::Vector3d sensor_tip_pos,prev_sensor_tip_pos(0,0,0);

    qint32 baud = 921600;

    geometry_msgs::msg::Point p,des_p;
    std_msgs::msg::ColorRGBA color,des_color;
    QString port      = QString::fromStdString("/dev/ttyUSB0");
    NdiManager manager;
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

    QTimer sensor_timer;
    sensor_timer.start(25);
    manager.start_tracking();
    qDebug() << "\nTools are now all initialized and tracking\n\n\n";

    QObject::connect(&sensor_timer, &QTimer::timeout,
                     [&manager]() {
        manager.send_and_parse_raw("bx", 20);
    });
    QObject::connect(&manager, &NdiManager::new_pose,
                     [&samples_marker, &p, &color, &sensor_tip_pos,
                      &prev_sensor_tip_pos, &RT, &tool_reading, &tip_publisher]
                     ([[maybe_unused]] const QByteArray &id,
                      const QVector3D &pos,
                      [[maybe_unused]] const QQuaternion &quat,
                      [[maybe_unused]] quint32 frame)
    {

        tool_reading=true;
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

    });
    QtExecutor executor;
    executor.add_node(tendon_node);
    executor.start();

    auto exit_code = app.exec();
    rclcpp::shutdown();


    return exit_code;

}

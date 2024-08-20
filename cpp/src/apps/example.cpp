#include "vistendon/ManualRvizMarkerArrayPublisher.h"
#include <rclcpp/rclcpp.hpp>
#include <QCoreApplication>

int main(int argCount, char * argList[]){
    QCoreApplication app(argCount,argList);
    rclcpp::init(argCount,argList);
    auto node = std::make_shared<rclcpp::Node>("publish_random_markers");
    vistendon::ManualRvizMarkerArrayPublisher tip_publisher(node,"/map","random_markers");
    using Marker = visualization_msgs::msg::Marker;
    Marker samples_marker;
    samples_marker.type = Marker::CUBE;
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
    samples_marker.color.r=1.0f;
    samples_marker.color.g=0.0f;
    samples_marker.color.b=0.0f;
    samples_marker.color.a=1.0f;


    visualization_msgs::msg::MarkerArray m_array;

    m_array.markers.emplace_back(samples_marker);
    tip_publisher.set_marker_array(m_array);

    rclcpp::spin(node);
    rclcpp::shutdown();

}

#include "cliparser/CliParser.h"
#include "csv/Csv.h"
#include "util/openfile_check.h"
#include "util/time_function_call.h"
#include "util/vector_ops.h"
#include "vistendon/EnvironmentRvizPublisher.h"
#include "vistendon/ManualRvizMarkerArrayPublisher.h"
#include "vistendon/TendonRvizPublisher.h"
#include "vistendon/marker_array_conversions.h"

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Core>

#include <iostream>
#include <string>
#include <chrono>
#include <iterator>
#include <vector>


namespace E = Eigen;

namespace {

void populate_parser(CliParser &parser) {
    parser.set_program_description(
                "View a 1 or 2 trajectories in Rviz.");
    parser.add_positional("traj");
    parser.set_required("traj");
    parser.set_description("traj", "The csv file with the points in the trajectory");

    parser.add_argflag("-t", "--traj2");
    parser.set_description("--traj2",
                           "A CSV file to also visualize as tiny spheres.\n");



}

std::vector<E::Vector3d> load_tip_samples(const std::string &samples_file) {
    std::ifstream in;
    util::openfile_check(in, samples_file);
    csv::CsvReader reader(in);

    std::vector<E::Vector3d> tip_positions;
    csv::CsvRow row;
    while (reader >> row) {
        tip_positions.emplace_back(E::Vector3d{std::stod(row["x"]),
                                               std::stod(row["y"]),
                                               std::stod(row["z"])});
    }


    return tip_positions;
}


} // end of unnamed namespace


int main(int argCount, char* argList[]) {
    rclcpp::init(argCount, argList);
    CliParser parser;
    populate_parser(parser);
    parser.parse(argCount, argList);
    std_msgs::msg::ColorRGBA color;

    auto samples=load_tip_samples(parser["traj"]);



    // blocks.  Press Ctrl-C to cancel

    // initialize visualization
    std::string frame = "/map";

    std::string samples_namespace = "tip-samples";
    auto tendon_node = std::make_shared<rclcpp::Node>("view_tendon");

    vistendon::ManualRvizMarkerArrayPublisher samples_publisher(
                tendon_node, frame, samples_namespace);

    vistendon::ManualRvizMarkerArrayPublisher samples_publisher_2(
                tendon_node, frame, "tip-samples-2");
    vistendon::ManualRvizMarkerArrayPublisher context_publisher(
                tendon_node, frame, "sphere_context"),cuboid_publisher(tendon_node,frame,"cuboid_context");
    vistendon::ManualRvizMarkerArrayPublisher base_publisher(
                tendon_node, frame, "robot_base");



    using Marker = visualization_msgs::msg::Marker;
    Marker samples_marker,samples_marker2,context_marker,cuboid_marker;








    samples_marker.type = Marker::SPHERE_LIST;
    samples_marker.action = Marker::ADD;
    samples_marker.scale.x = 0.004;
    samples_marker.scale.y = 0.004;
    samples_marker.scale.z = 0.004;
    samples_marker.pose.position.x = 0;
    samples_marker.pose.position.y = 0;
    samples_marker.pose.position.z = 0;
    samples_marker.pose.orientation.w = 1;
    samples_marker.pose.orientation.x = 0;
    samples_marker.pose.orientation.y = 0;
    samples_marker.pose.orientation.z = 0;
    //double k=1;



    if(parser.has("--traj2")){
        auto samples2=load_tip_samples(parser["--traj2"]);
        samples_marker2.type = Marker::SPHERE_LIST;
        samples_marker2.action = Marker::ADD;
        samples_marker2.scale.x = 0.005;
        samples_marker2.scale.y = 0.005;
        samples_marker2.scale.z = 0.005;
        samples_marker2.pose.position.x = 0;
        samples_marker2.pose.position.y = 0;
        samples_marker2.pose.position.z = 0;
        samples_marker2.pose.orientation.w = 1;
        samples_marker2.pose.orientation.x = 0;
        samples_marker2.pose.orientation.y = 0;
        samples_marker2.pose.orientation.z = 0;
        for (auto &sample : samples2) {
            geometry_msgs::msg::Point p;
            p.x = sample[0];
            p.y = sample[1];
            p.z = sample[2];
            samples_marker2.points.emplace_back(std::move(p));
        }
        samples_publisher_2.set_color(0.0f, 1.0f, 0.0f, 1.0f); // transparent blue
        samples_publisher_2.add_marker(samples_marker2);

    }



    context_marker.type=Marker::SPHERE;
    context_marker.action=Marker::ADD;
//Min sphere
//    context_marker.scale.x=0.02159945;
//      context_marker.scale.y = 0.02159945;
//      context_marker.scale.z = 0.02159945;
//      context_marker.pose.position.x = 0.0269074;
//      context_marker.pose.position.y = -0.04;
//      context_marker.pose.position.z = 0.14670269;
//Max sphere
          context_marker.scale.x=2.87973446e-02;
    context_marker.scale.y = 2.87973446e-02;
    context_marker.scale.z = 2.87973446e-02;
    context_marker.pose.position.x = -7.26717505e-04 ;
    context_marker.pose.position.y = -4.00000000e-02;
    context_marker.pose.position.z = 1.56427485e-01;
    context_marker.pose.orientation.w = 1;
    context_marker.pose.orientation.x = 0;
    context_marker.pose.orientation.y = 0;
    context_marker.pose.orientation.z = 0;
    context_publisher.set_color(0.0f,0.0f,1.0f,0.2f);
    context_publisher.add_marker(context_marker);


    cuboid_marker.type=Marker::CUBE;
    cuboid_marker.action=Marker::ADD;
        //max cuboid
    cuboid_marker.scale.x=1.347268766425058911e-02;
    cuboid_marker.scale.y =  0.01;
    cuboid_marker.scale.z = 1.366835828113047678e-02;
    cuboid_marker.pose.position.x = -1.445055651539241606e-02;
    cuboid_marker.pose.position.y = -4.000000000000000083e-02;
    cuboid_marker.pose.position.z = 1.665482920735344430e-01;
    //min cuboid
//      cuboid_marker.scale.x=1.008689979173867969e-02;
//      cuboid_marker.scale.y =  0.01;
//      cuboid_marker.scale.z = 1.264747846810999193e-02;
//      cuboid_marker.pose.position.x = -6.030279751097596813e-03;
//      cuboid_marker.pose.position.y = -4.000000000000000083e-02;
//      cuboid_marker.pose.position.z = 1.526370181594503550e-01;
    cuboid_marker.pose.orientation.w = 1;
    cuboid_marker.pose.orientation.x = 0;
    cuboid_marker.pose.orientation.y = 0;
    cuboid_marker.pose.orientation.z = 0;
    cuboid_publisher.set_color(0.0f,0.0f,1.0f,0.2f);
    cuboid_publisher.add_marker(cuboid_marker);




    auto timer = tendon_node->create_wall_timer(std::chrono::seconds(1),
                                                [&]() -> void{
                                                    visualization_msgs::msg::MarkerArray m_array;
                                                    static size_t i=0;
                                                    E::Vector3d sample= samples[i];
                                                    geometry_msgs::msg::Point p;
                                                    p.x = sample[0];
                                                    p.y = sample[1];
                                                    p.z = sample[2];
                                                    color.r=1.0f;
                                                    color.g=0.078f;
                                                    color.b=0.576f;
                                                    color.a=1;
                                                    samples_marker.points.emplace_back(std::move(p));
                                                    samples_marker.colors.emplace_back(color);
                                                    m_array.markers.emplace_back(samples_marker);
                                                    samples_publisher.set_marker_array(m_array);
                                                    i=(i+1)%samples.size();

                                                });


    rclcpp::spin(tendon_node);
    rclcpp::shutdown();

    return 0;
}

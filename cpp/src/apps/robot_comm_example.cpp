 #include "serialport/SerialPort.h"
#include "cliparser/CliParser.h"
#include "tip-control/Controller.h"
#include "phys-robot/trajectory_conversion.h"
#include "cpptoml/toml_conversions.h"
#include "csv/Csv.h"
#include "util/openfile_check.h"
#include "util/vector_ops.h"
#include<fstream>
#include <vector>


void populate_parser(CliParser &parser) {
    parser.set_program_description(
                "Hard codes a trajectory(raw motor controls or tip points) and if it is tip points converts to "
                "motor controls and sends it over serial port,\n"
                );


    parser.add_argflag("-t", "--tip");
    parser.add_argflag("-c", "--config");
    parser.add_argflag("-p","--port");
    parser.add_flag("-r","--raw");
    parser.add_flag("-no-interpolation");
    parser.set_description("-no-interpolation","Turns off tip-space interpolation");
    parser.set_description("-t","Read in tip positions trajectory from a csv file");
    parser.set_description("-r","Execute the default raw motor control trajectory");

}
namespace defaults_1 {
const int max_iter = 100;
const double mu_init = 1e-3;
const double eps1 = 1e-9;
const double eps2 = 1e-4;
const double eps3 = 1e-4;
const double fd_delta = 1e-6;
const double thres=5e-3;
const std::vector<std::string> raw_msg {"<1726.4, 1770.95, 1736.27, 1712.1>",
                                        "<1726.4, 1770.95, 1736.27, 1712.1>",
                                        "<1214.2, 1405.97, 1402.55, 1073.74>",
                                        "<1439.47, 1740.74, 1477.32, 1414.67>",
                                        "<1627.95, 1724.06, 1472.29, 1500.83>",
                                        "<1726.4, 1770.95, 1736.27, 1712.1>",
                                        "<1214.2, 1405.97, 1402.55, 1073.74>",
                                        "<1439.47, 1740.74, 1477.32, 1414.67>",
                                        "<1627.95, 1724.06, 1472.29, 1500.83>",
                                        "<1726.4, 1770.95, 1736.27, 1712.1>"};
const std::vector<std::vector<double>> tip_traj {{0,-0.08,0.197},
                                                 {0.0298,-0.08,0.1672},
                                                 {0,-0.08,0.1374},
                                                 {-0.0298,-0.08,0.1672},
                                                 {0,-0.08,0.197}
                                                };
}

std::vector<std::string> interpolated_controls_trajectory(std::vector<std::vector<double>> tip_trajectory,double threshold,Controller controller,trajectory_conversion conv,tendon::TendonRobot robot,std::vector<double> c,bool inter){
    std::vector<std::vector<double>> interpolated;
    std::vector<std::vector<double>> traj;
    std::vector<std::string> msg;
    //    traj=std::move(tip_trajectory);
    if(inter){
        for(size_t i=0; i<tip_trajectory.size()-1;i++){
            auto range = util::range(tip_trajectory[i], tip_trajectory[i+1],threshold);
            interpolated.insert(interpolated.end(), range.begin(), range.end());
        }
        tip_trajectory=std::move(interpolated);
    }
    for (auto& step:tip_trajectory){
        auto soln = controller.inverse_kinematics(std::vector(controller.robot().state_size(),0.0), Eigen::Vector3d{step[0],step[1],step[2]}, defaults_1::max_iter, defaults_1::mu_init, defaults_1::eps1,
                                                  defaults_1::eps2,defaults_1::eps3, defaults_1::fd_delta);
        qDebug()<<"Config "<<soln.state;

        std::vector<double> motor_control= conv.length_to_motor_control_conversion(conv.tension_to_length_conversion(soln.state,robot),c);

        msg.push_back(conv.message_builder(motor_control));

    }
    return msg;


}


int main (int arg_count, char *arg_list[]) {
    //    QCoreApplication app(arg_count, arg_list);
    CliParser parser;
    populate_parser(parser);
    parser.parse(arg_count, arg_list);




    std::string port_name=parser["-p"];
    SerialPort s_port(port_name);
    sleep(1);
    std::vector<std::string> msg;

    auto robot = cpptoml::from_file<tendon::TendonRobot>("phys_robot_limits.toml");
    Controller controller(robot);
    trajectory_conversion conv;
    std::vector<double> c  = conv.get_motor_control_constant(robot);
    bool inter=true;
    if(parser.has("-no-interpolation")){
        inter=false;
    }
    if(parser.has("-t")){

        std::vector<std::vector<double>> traj;
        {
            std::ifstream in;
            util::openfile_check(in, parser["-t"]);
            csv::CsvReader reader(in);
            csv::CsvRow row;


            while (reader >> row) {
                traj.emplace_back(std::vector{std::stod(row["x"]),
                                              std::stod(row["y"]),
                                              std::stod(row["z"])
                                  });
            }


        }
        msg=interpolated_controls_trajectory(traj,defaults_1::thres,controller,conv,robot,c,inter);
        //        for (auto& step:traj){
        //            auto soln = controller.inverse_kinematics(std::vector(controller.robot().state_size(),0.0), Eigen::Vector3d{step[0],step[1],step[2]}, defaults_1::max_iter, defaults_1::mu_init, defaults_1::eps1,
        //                                                      defaults_1::eps2,defaults_1::eps3, defaults_1::fd_delta);
        //            std::vector<double> motor_control= conv.length_to_motor_control_conversion(conv.tension_to_length_conversion(soln.state,robot),c);

        //            msg.push_back(conv.message_builder(motor_control));

        //        }




    }
    if(parser.has("-c")){

        std::vector<std::vector<double>> traj;

        std::ifstream in;
        util::openfile_check(in, parser["-c"]);
        csv::CsvReader reader(in);
        csv::CsvRow row;


        while (reader >> row) {

            traj.emplace_back(std::vector{std::stod(row["Tension_1"]),
                                          std::stod(row["Tension_2"]),
                                          std::stod(row["Tension_3"]),
                                          std::stod(row["Tension_4"])
                              });

        }





        for (auto& step:traj){
            std::vector<double> motor_control= conv.length_to_motor_control_conversion(conv.tension_to_length_conversion(step,robot),c);
            msg.push_back(conv.message_builder(motor_control));


        }

    }

    else if(parser.has("-r")){
        msg=defaults_1::raw_msg;
    }

    else{
        msg=interpolated_controls_trajectory(defaults_1::tip_traj,defaults_1::thres,controller,conv,robot,c,inter);

    }
    s_port.writeData(msg,5000);
    return 0;
}

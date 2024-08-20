#ifndef TRAJ_CONV
#define TRAJ_CONV

#include "tendon/TendonRobot.h"
#include "tendon/TendonSpecs.h"
#include "csv/Csv.h"
#include "util/openfile_check.h"
#include<fstream>
#include <vector>
#include <QCoreApplication>


struct trajectory_conversion{


    std::vector<std::vector<double>> load_tension_trajectory(const std::string &csv_file) {
      std::ifstream in;
      util::openfile_check(in, csv_file);
      csv::CsvReader reader(in);
      csv::CsvRow row;
      std::vector<std::vector<double>> traj;

      while (reader >> row) {
        traj.emplace_back(std::vector{std::stod(row["tau_1"]),
                                      std::stod(row["tau_2"]),
                                      std::stod(row["tau_3"]),
                                      std::stod(row["tau_4"])});
      }
      return traj;
    }

    std::vector<std::vector<double>> tension_to_length_conversion(const std::vector<std::vector<double>> &traj,tendon::TendonRobot &robot){
      std::vector<std::vector<double>> len_traj;
      for (auto &step : traj){
        len_traj.push_back(robot.shape(step).L_i);

      }
      return len_traj;
    }

    std::vector<double> tension_to_length_conversion(const std::vector<double> &traj,const tendon::TendonRobot &robot){
        std::vector<double> len_traj;
        len_traj=robot.shape(traj).L_i;
        return len_traj;
    }

    std::vector<std::vector<double>> length_to_motor_control_conversion(const std::vector<std::vector<double>> &len_traj,std::vector<double> &c){
      std::vector<std::vector<double>> motor_controls;


      for(auto &step : len_traj){
        motor_controls.push_back(length_to_motor_control_conversion(step,c));
      }
      return motor_controls;
    }

    std::vector<double> length_to_motor_control_conversion(const std::vector<double> &lengths, std::vector<double> &c){
        // projects val onto interval [range.first, range.second]
        auto clamp = [](double val, std::pair<double, double> range) {
            return std::min(range.second, std::max(val, range.first));
        };
        std::vector<double> control_step(lengths.size());
        for(size_t i = 0; i < lengths.size(); ++i){
          auto x = clamp(lengths[i] + c[i], {0.0, 0.053});
          control_step[i] = round(1000+(1000/0.053)*x);
        }
      return control_step;
    }

    std::string message_builder(const std::vector<double> motor_control){
        std::ostringstream msg_builder;
        bool first = true;
        msg_builder<<"<";
        for (auto &val : motor_control) {
          if (!first) { msg_builder << ", "; }
          first = false;
          msg_builder << val;
        }
        msg_builder<<">";
        return msg_builder.str();
    }

    std::vector<double> get_motor_control_constant(tendon::TendonRobot robot){
        auto home_shape = robot.home_shape();
        std::vector<double> L_h=home_shape.L_i;
        double x_h=0.04;
        std::vector<double> c;
        for(auto l : L_h){
          c.push_back(x_h-l);
        }
        return c;
    }
};

#endif

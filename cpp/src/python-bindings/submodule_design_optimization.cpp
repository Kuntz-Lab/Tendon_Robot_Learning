#include "submodule_design_optimization.h"

#include <design-optimization/MultiGoalPlanningCostIK.h>

#include <cpptoml/toml_conversions.h>
#include <tendon/BackboneSpecs.h>
#include <tendon/TendonResult.h>
#include <tendon/TendonRobot.h>
#include <tendon/TendonSpecs.h>

#include <pybind11/eigen.h>    // auto convert between python and eigen types
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>      // auto convert between python and STL types

#include <Eigen/Core>

#include <sstream>
#include <string>
#include <vector>

namespace E = Eigen;
namespace py = pybind11;

namespace {

void def_class_MultiGoalPlanningCostIK(py::module &m) {
  using OptCost = design_optimization::MultiGoalPlanningCostIK;

  py::class_<OptCost>(m, "MultiGoalPlanningCostIK",
      "Cost function for reaching multiple goals with planning")
    .def(py::init<>())

    // attributes
    .def_readwrite("start_config", &OptCost::start_config,
                   "Starting configuration for both IK and planning")
    .def_readwrite("goal_tips", &OptCost::goal_tips,
                   "Set of tip-positions for which the planner will attempt to"
                   " reach")
    .def_readwrite("venv", &OptCost::venv,
                   "Voxel environment containing obstacles to avoid in planning")
    .def_readwrite("tip_threshold", &OptCost::tip_threshold,
                   "How close to a goal tip is close enough")
    .def_readwrite("time_limit_seconds", &OptCost::time_limit_seconds,
                   "Time limit for IK + planning to see how many are reachable")
    .def_readwrite("ik_max_iters", &OptCost::ik_max_iters,
                   "IK param: maximum number of iterations")
    .def_readwrite("ik_mu_init", &OptCost::ik_mu_init,
                   "IK param: initial value of mu")
    .def_readwrite("ik_stop_threshold_JT_err_inf",
                   &OptCost::ik_stop_threshold_JT_err_inf,
                   "IK param: stop threshold for |J^T e|")
    .def_readwrite("ik_stop_thresdhold_Dp", &OptCost::ik_stop_threshold_Dp,
                   "IK param: stop threshold for |Dp|")
    .def_readwrite("ik_finite_difference_delta",
                   &OptCost::ik_finite_difference_delta,
                   "IK param: distance for doing finite different Jacobian")
    .def_readwrite("rrt_grow_distance", &OptCost::rrt_grow_distance,
                   "RRT param: max config distance for each branch growth")
    .def_readwrite("rrt_goal_bias", &OptCost::rrt_goal_bias,
                   "RRT param: value between 0 and 1.  How often to choose the"
                   " goal as the RRT growth target instead of a random sample.")

    // methods
    .def("reachable_goal_tips", &OptCost::reachable_goal_tips,
         py::arg("robot"),
         "Returns list of reachable goal tips using RRT within the specified"
         " time limit")
    .def("__call__", &OptCost::operator(), py::arg("robot"),
         "Returns the number of reachable goal tips using RRT within the"
         " specified time limit.  Basically len(reachable_goal_tips(robot)).")
    .def("to_toml", [](const OptCost &e, const std::string &fname) {
          cpptoml::to_file(e, fname);
        }, py::arg("filepath"), "save this object to a toml file")

    // static methods
    .def_static("from_toml", [](const std::string &fname) {
          return cpptoml::from_file<OptCost>(fname);
        }, py::arg("filepath"),
        "load a MultiGoalPlanningCostIK object from the given toml file")

    // python-specific added methods
    .def("__repr__", [](const OptCost &e) {
          std::ostringstream builder;
          cpptoml::to_stream(builder, e.to_toml());
          return builder.str();
        })

    ;
}

} // end of unnamed namespace

py::module def_submodule_design_optimization(py::module &m) {
  auto submodule = m.def_submodule("design_optimization",
                                   "Pieces for running robot design optimization");
  def_class_MultiGoalPlanningCostIK(submodule);
  return submodule;
}

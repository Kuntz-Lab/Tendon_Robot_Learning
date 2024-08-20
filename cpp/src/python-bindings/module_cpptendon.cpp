#include "submodule_collision.h"
#include "submodule_controller.h"
#include "submodule_motion_planning.h"
#include "submodule_tendon.h"
#include "submodule_design_optimization.h"
#include "submodule_ndi.h"

#include <pybind11/pybind11.h>

#include <sstream>

namespace py = pybind11;

PYBIND11_MODULE(cpptendon, m) {
  def_submodule_collision(m);
  def_submodule_tendon(m);
  def_submodule_controller(m);
  def_submodule_motion_planning(m);
  def_submodule_design_optimization(m);
  def_submodule_ndi(m);
}

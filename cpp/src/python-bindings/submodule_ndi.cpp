#include "submodule_ndi.h"

#include <mag-tracker/ndi/NdiManager.h>

#include <Eigen/Core>     // for E::Vector3d
#include <Eigen/Geometry> // for E::Quaterniond

#include <pybind11/cast.h>
#include <pybind11/numpy.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <memory>
#include <string>

// namespace aliases
namespace py = pybind11;
namespace E  = Eigen;

namespace {

// Wrapper for ToolData that uses std containers instead of Qt types
struct ToolDataWrap {
  std::string     id;           // 2-character hex tool ID
  bool            missing;      // true means the tool is reported missing
  quint32         status;       // each bit means something different
  quint32         frame;        // like a time step
  E::Vector3f     position;     // position of the tool
  E::Vector4f     orientation;  // orientation of the tool
  float           error;        // calculated error (only for 6DOF tools)

  ToolDataWrap() = default;                          // default constructor
  ToolDataWrap(const ToolDataWrap &other) = default; // copy constructor
  ToolDataWrap(ToolDataWrap &&other) = default;      // move constructor
  ToolDataWrap& operator=(const ToolDataWrap &other) = default; // copy assign
  ToolDataWrap& operator=(ToolDataWrap &&other) = default;      // move assign

  ToolDataWrap(const ndi::ToolData &t)
    : id(t.id.toStdString())
    , missing(t.missing)
    , status(t.status)
    , frame(t.frame)
    , position(t.position.x(), t.position.y(), t.position.z())
    , orientation(t.orientation.scalar(),
                  t.orientation.x(),
                  t.orientation.y(),
                  t.orientation.z())
    , error(t.error)
  { }

  //ndi::ToolData unwrap() const {
  //  ndi::ToolData t;
  //  auto &p = this->position;
  //  auto &q = this->orientation;
  //  t.id          = QByteArray::fromStdString(this->id);
  //  t.missing     = this->missing;
  //  t.status      = this->status;
  //  t.frame       = this->frame;
  //  t.position    = QVector3D(p[0], p[1], p[2]);
  //  t.orientation = QQuaternion(q[0], q[1], q[2], q[3]);
  //  t.error       = this->error;
  //}

  static ToolDataWrap wrap(const ndi::ToolData &t) {
    return ToolDataWrap(t);
  }

  static std::vector<ToolDataWrap> wrapvec(const QVector<ndi::ToolData> &ts) {
    std::vector<ToolDataWrap> wrapped(ts.size());
    std::transform(ts.begin(), ts.end(), wrapped.begin(), &ToolDataWrap::wrap);
    return wrapped;
  }
};

QByteArray from_pybytes(const py::bytes &msg) {
  auto p = msg.ptr();
  QByteArray ba(PyBytes_AS_STRING(p), PyBytes_GET_SIZE(p));
  return ba;
}

py::bytes to_pybytes(const QByteArray &ba) {
  return py::bytes(ba.data(), ba.size());
}

void def_class_ToolData(py::module &m) {
  using TD = ToolDataWrap;
  py::class_<TD>(m, "ToolData")
    .def(py::init<>())

    // attributes
    .def_readwrite("id", &TD::id)
    .def_readwrite("missing", &TD::missing)
    .def_readwrite("status", &TD::status)
    .def_readwrite("frame", &TD::frame)
    .def_readwrite("position", &TD::position)
    .def_readwrite("orientation", &TD::orientation)
    .def_readwrite("error", &TD::error)
    ;
}

void def_class_NdiManager(py::module &m) {
  QPair<int, int> p(1, 2);
  using Ndi = ndi::NdiManager;

  py::class_<Ndi, std::shared_ptr<Ndi>>(m, "NdiManager",
      "Manage communication with the NDI magnetic tracker")
    .def(py::init<>())

    // public methods
    .def("init",
        [](Ndi &ndi, const std::string &port, uint32_t baud) {
          ndi.init(QString::fromStdString(port), baud);
        },
        py::arg("port"),
        py::arg("baud") = 921600,
        "Initialize the connection.  May raise an exception.")
    .def("close", &Ndi::close,
        "Closes the connection.  After, you may call init() again.")
    .def("initialize_tools", &Ndi::initialize_tools)
    .def("start_tracking", &Ndi::start_tracking,
        "Initialize and start tracking the tools")
    .def("send_raw",
        [](Ndi &ndi, const py::bytes &msg, const int32_t timeout_ms) -> py::bytes {
          auto ans = ndi.send_raw(from_pybytes(msg), timeout_ms);
          return to_pybytes(ans);
        },
        py::arg("data"),
        py::arg("timeout_ms") = 1000,
        "both send and parse response")

    // static methods
    .def_static("available_bauds",
        []() { return Ndi::available_bauds().toStdVector(); })
    .def_static("is_error",
        [](const py::bytes &received) {
          auto p = received.ptr();
          std::string_view sv(PyBytes_AS_STRING(p), PyBytes_GET_SIZE(p));
          return sv.substr(0, 5) == "ERROR";
        },
        py::arg("received"),
        "Returns True if the received message is an error.  Call error_string()\n"
        "to get a meaningful message.")
    .def_static("error_string",
        [](const py::bytes &received) {
          return Ndi::error_string(from_pybytes(received)).toStdString();
        },
        py::arg("response"),
        "Convert error response into a human-readable message (see is_error())")
    .def_static("parse_tx",
        [](const py::bytes &received) {
          auto ans = Ndi::parse_tx(from_pybytes(received));
          return ToolDataWrap::wrapvec(ans);
        },
        py::arg("response"),
        "Parse and return tool data from the TX command response.")
    .def_static("parse_bx",
        [](const py::bytes &received) {
          auto ans = Ndi::parse_bx(from_pybytes(received));
          return ToolDataWrap::wrapvec(ans);
        },
        py::arg("response"),
        "Parse and return tool data from the BX command response.")
    .def_static("parse_phsr",
        [](const py::bytes &received) {
          auto ans = Ndi::parse_phsr(from_pybytes(received));
          std::vector<std::pair<py::bytes, uint32_t>> converted(ans.size());
          std::transform(ans.begin(), ans.end(), converted.begin(),
              [](const auto &pair) {
                return std::make_pair(to_pybytes(pair.first), pair.second);
              });
          return converted;
        },
        py::arg("response"),
        "Parse and return tool handles from the PHSR command response.")

    ;
}

} // end of unnamed namespace

py::module def_submodule_ndi(py::module &m) {
  auto submodule = m.def_submodule("ndi", "Communicate with NDI tracker");
  def_class_ToolData(submodule);
  def_class_NdiManager(submodule);
  return submodule;
}

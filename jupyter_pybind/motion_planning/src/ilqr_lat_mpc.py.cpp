 #include "ilqr_lat_mpc.h"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py = pybind11;
using namespace pnc::control;

PYBIND11_MODULE(ilqr_lat_mpc, m) {

  m.doc() = "m";
  py::class_<LatMpcIlqr>(m, "LatMpcIlqr")
      .def(py::init<>())
      .def("Init", &LatMpcIlqr::Init)
      .def("Update", &LatMpcIlqr::Update)
      .def("GetMpcOutput", &LatMpcIlqr::GetMpcOutput);
}
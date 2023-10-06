#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "apa_planner/dubins_lib/dubins_lib.h"

namespace py = pybind11;
using namespace pnc::dubins_lib;

static DubinsLibrary *pBase = nullptr;

int Init() {
  pBase = new DubinsLibrary();
  return 0;
}

template <class T>
inline T BytesToProto(py::bytes &bytes) {
  T proto_obj;
  py::buffer buf(bytes);
  py::buffer_info input_info = buf.request();
  char *input_ptr = static_cast<char *>(input_info.ptr);
  std::string input_s(input_ptr, input_info.size);

  T input;
  input.ParseFromString(input_s);
  return input;
}

int Update(double x_start, double y_start, double heading_start,
           double x_target, double y_target, double heading_target,
           double radius, uint8_t dubins_type, uint8_t case_type) {
  DubinsLibrary::Input input;
  input.radius = radius;
  input.heading1 = heading_start;
  input.heading2 = heading_target;
  input.p1 << x_start, y_start;
  input.p2 << x_target, y_target;

  pBase->SetInput(input);
  pBase->Solve(dubins_type, case_type);

  return 0;
}

Eigen::Vector2d GetABCenter() { return pBase->GetOutput().arc_AB.circle_info.center; }
Eigen::Vector2d GetCDCenter() { return pBase->GetOutput().arc_CD.circle_info.center; }
Eigen::Vector2d GetpB() { return pBase->GetOutput().line_BC.pA; }
Eigen::Vector2d GetpC() { return pBase->GetOutput().line_BC.pB; }

PYBIND11_MODULE(dubins_lib_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
   .def("Update", &Update)
   .def("GetABCenter", &GetABCenter)
   .def("GetCDCenter", &GetCDCenter)
   .def("GetpB", &GetpB)
   .def("GetpC", &GetpC);
}

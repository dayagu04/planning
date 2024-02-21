#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <iostream>
#include <vector>

#include "dubins_lib.h"

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
           double radius, uint8_t dubins_type, uint8_t case_type, double ds,
           bool is_complete_path) {
  DubinsLibrary::Input input;
  input.radius = radius;
  input.heading1 = heading_start;
  input.heading2 = heading_target;
  input.p1 << x_start, y_start;
  input.p2 << x_target, y_target;

  pBase->SetInput(input);
  pBase->Solve(dubins_type, case_type);
  pBase->Sampling(ds, is_complete_path);

  pBase->PrintOutput();

  return 0;
}

int UpdateLineArc(double x_start, double y_start, double heading_start,
                  double x_target, double y_target, double heading_target,
                  double radius, uint8_t line_arc_type, double ds,
                  bool is_complete_path) {
  DubinsLibrary::Input input;
  input.radius = radius;
  input.heading1 = heading_start;
  input.heading2 = heading_target;
  input.p1 << x_start, y_start;
  input.p2 << x_target, y_target;

  std::cout << "---------------" << std::endl;
  pBase->SetInput(input);
  pBase->Solve(line_arc_type);
  pBase->Sampling(ds, is_complete_path);

  pBase->PrintOutput();

  return 0;
}

Eigen::Vector2d GetABCenter() {
  return pBase->GetOutput().arc_AB.circle_info.center;
}

Eigen::Vector2d GetCDCenter() {
  return pBase->GetOutput().arc_CD.circle_info.center;
}

std::vector<double> GetPathEle(size_t index) {
  return pBase->GetPathEle(index);
}

Eigen::Vector2d GetpB() { return pBase->GetOutput().line_BC.pA; }
Eigen::Vector2d GetpC() { return pBase->GetOutput().line_BC.pB; }
Eigen::Vector2d GetpD() { return pBase->GetOutput().arc_CD.pB; }
bool GetPathAvailiable() { return pBase->GetOutput().path_available; }
double GetLength() { return pBase->GetOutput().length; }
std::vector<uint8_t> GetGearCmdVec() { return pBase->GetOutput().gear_cmd_vec; }
uint8_t GetGearChangeCount() { return pBase->GetOutput().gear_change_count; }
double GetThetaBC() { return pBase->GetThetaBC(); }
double GetThetaD() { return pBase->GetThetaD(); }
double GetRadius() { return pBase->GetOutput().arc_AB.circle_info.radius; }

PYBIND11_MODULE(dubins_lib_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("Update", &Update)
      .def("UpdateLineArc", &UpdateLineArc)
      .def("GetABCenter", &GetABCenter)
      .def("GetCDCenter", &GetCDCenter)
      .def("GetpB", &GetpB)
      .def("GetpC", &GetpC)
      .def("GetpD", &GetpD)
      .def("GetThetaBC", &GetThetaBC)
      .def("GetThetaD", &GetThetaD)
      .def("GetPathAvailiable", &GetPathAvailiable)
      .def("GetLength", &GetLength)
      .def("GetGearCmdVec", &GetGearCmdVec)
      .def("GetGearChangeCount", &GetGearChangeCount)
      .def("GetRadius", &GetRadius)
      .def("GetPathEle", &GetPathEle);
}

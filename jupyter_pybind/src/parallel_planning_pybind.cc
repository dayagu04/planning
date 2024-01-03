#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <iostream>
#include <vector>

#include "apa_plan_base.h"
#include "parallel_park_in_planner.h"
#include "parallel_path_planner.h"

namespace py = pybind11;
using namespace planning::apa_planner;

static planning::apa_planner::ParallelPathPlanner *pBase = nullptr;

int Init() {
  pBase = new ParallelPathPlanner();
  pBase->Reset();
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

static ParallelPathPlanner::DebugInfo debuginfo;
static std::vector<double> res;

int Update(double ego_x, double ego_y, double ego_heading, double tlane_p0_x,
           double tlane_p0_y, double tlane_p1_x, double tlane_p1_y,
           double tlane_pt_x, double tlane_pt_y, double channel_x, double ds,
           bool is_complete_path, bool is_plan_first) {
  planning::apa_planner::ParallelPathPlanner::Input input;

  input.ego_pose.pos << ego_x, ego_y;
  input.ego_pose.heading = ego_heading;

  input.tlane.p0 << tlane_p0_x, tlane_p0_y;
  input.tlane.p1 << tlane_p1_x, tlane_p1_y;
  input.tlane.pt << tlane_pt_x, tlane_pt_y;
  input.tlane.channel_x = channel_x;
  input.sample_ds = ds;
  input.is_complete_path = is_complete_path;
  input.is_replan_first = is_plan_first;

  // std::cout << "---------------" << std::endl;
  pBase->SetInput(input);

  pBase->Update();
  // pBase->PrintOutputSegmentsInfo();

  pBase->SampleCurrentPathSeg();

  return 0;
}

std::vector<double> GetPathEle(size_t index) {
  return pBase->GetPathEle(index);
}

PYBIND11_MODULE(parallel_planning_py, m) {
  m.doc() = "m";

  m.def("Init", &Init).def("Update", &Update).def("GetPathEle", &GetPathEle);
}
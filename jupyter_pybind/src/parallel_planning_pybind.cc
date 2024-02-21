#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <iostream>
#include <vector>

#include "apa_plan_base.h"
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

int Update(double ego_x, double ego_y, double ego_heading, double p_outside_x,
           double p_outside_y, double p_inside_x, double p_inside_y,
           double p_target_x, double p_target_y, double channel_width,
           double ds, bool is_complete_path, bool is_plan_first,
           bool set_left_side) {
  planning::apa_planner::ParallelPathPlanner::Input input;

  input.ego_pose.pos << ego_x, ego_y;
  input.ego_pose.heading = ego_heading;

  input.tlane.pt_outside << p_outside_x, p_outside_y;
  input.tlane.pt_inside << p_inside_x, p_inside_y;
  input.tlane.pt_terminal << p_target_x, p_target_y;

  input.tlane.slot_side = (set_left_side ? ApaPlannerBase::SLOT_SIDE_LEFT
                                         : ApaPlannerBase::SLOT_SIDE_RIGHT);

  input.tlane.channel_width = channel_width;
  input.tlane.channel_length = 10.0;
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
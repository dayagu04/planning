#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <iostream>
#include <vector>

#include "apa_plan_base.h"
#include "perpendicular_park_in_planner.h"
#include "perpendicular_path_planner.h"

namespace py = pybind11;
using namespace planning::apa_planner;

static planning::apa_planner::PerpendicularPathPlanner *pBase = nullptr;

int Init() {
  pBase = new PerpendicularPathPlanner();
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

static PerpendicularPathPlanner::DebugInfo debuginfo;
static std::vector<double> res;

int Update(double ego_x, double ego_y, double ego_heading, double tlane_p0_x,
           double tlane_p0_y, double tlane_p1_x, double tlane_p1_y,
           double tlane_pt_x, double tlane_pt_y, double channel_x, double ds,
           bool is_complete_path) {
  planning::apa_planner::PerpendicularPathPlanner::Input input;

  input.ego_pose.pos << ego_x, ego_y;
  input.ego_pose.heading = ego_heading;

  input.tlane.pt_outside << tlane_p0_x, tlane_p0_y;
  input.tlane.pt_inside << tlane_p1_x, tlane_p1_y;
  input.tlane.pt_terminal_pos << tlane_pt_x, tlane_pt_y;
  input.is_complete_path = is_complete_path;
  input.is_replan_first = false;

  // std::cout << "---------------" << std::endl;
  pBase->SetInput(input);

  pBase->Update();
  // pBase->PrintOutputSegmentsInfo();

  pBase->SampleCurrentPathSeg();

  return 0;
}

Eigen::Vector2d GetTagPoint() { return debuginfo.tag_point; }

double GetHeadingB() { return debuginfo.headingB; }

std::vector<double> GetPathEle(size_t index) {
  return pBase->GetPathEle(index);
}

std::vector<double> GetMinSafeCircle() { return pBase->GetMinSafeCircle(); }

Eigen::Vector2d GetpA() {
  if (pBase->GetOutput().path_segment_vec[0].seg_type ==
      pnc::geometry_lib::SEG_TYPE_LINE) {
    return pBase->GetOutput().path_segment_vec[0].line_seg.pB;
  } else {
    return pBase->GetOutput().path_segment_vec[0].line_seg.pA;
  }
}

Eigen::Vector2d GetpB() {
  if (pBase->GetOutput().path_segment_vec[0].seg_type ==
      pnc::geometry_lib::SEG_TYPE_LINE) {
    return pBase->GetOutput().path_segment_vec[1].arc_seg.pB;
  } else {
    return pBase->GetOutput().path_segment_vec[0].line_seg.pB;
  }
}

Eigen::Vector2d GetABCenter() {
  if (pBase->GetOutput().path_available &&
      pBase->GetOutput().path_segment_vec.size() > 1) {
    if (pBase->GetOutput().path_segment_vec[1].seg_type ==
        pnc::geometry_lib::SEG_TYPE_ARC) {
      return pBase->GetOutput().path_segment_vec[1].arc_seg.circle_info.center;
    } else {
      return pBase->GetOutput().path_segment_vec[0].arc_seg.circle_info.center;
    }
  } else {
    return Eigen::Vector2d();
  }
}

PYBIND11_MODULE(perpendicular_planning_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("Update", &Update)
      .def("GetPathEle", &GetPathEle)
      .def("GetMinSafeCircle", &GetMinSafeCircle)
      .def("GetpA", &GetpA)
      .def("GetpB", &GetpB)
      .def("GetHeadingB", &GetHeadingB)
      .def("GetTagPoint", &GetTagPoint)
      .def("GetABCenter", &GetABCenter);
}
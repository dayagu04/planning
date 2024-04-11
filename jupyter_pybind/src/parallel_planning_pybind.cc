#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <iostream>
#include <vector>

#include "apa_plan_base.h"
#include "collision_detection.h"
#include "geometry_math.h"
#include "parallel_path_planner.h"

namespace py = pybind11;
using namespace planning::apa_planner;

static planning::apa_planner::ParallelPathPlanner *pBase = nullptr;
static planning::CollisionDetector col_det;

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

static std::vector<double> res;

int UpdateObstacles(double p_outside_x, double p_outside_y, double p_inside_x,
                    double p_inside_y, double p_target_x, double p_target_y,
                    double channel_width, double channel_length,
                    double curb_offset, bool set_pin_obs, bool set_pout_obs,
                    bool set_pin_line_obs, bool set_pout_line_obs,
                    bool set_pin_parallel_line_obs, bool set_left_side) {
  // set obstacles

  double p0_x = p_outside_x;
  double p0_y = p_outside_y;
  double p1_x = p_inside_x;
  double p1_y = p_inside_y;

  const double slot_width = 2.0;
  const double slot_side_sgn = (set_left_side ? -1.0 : 1.0);
  double channel_y = slot_side_sgn * (0.5 * slot_width + channel_width);

  // set limit obstacles in clockwise direction
  const Eigen::Vector2d A(p_outside_x, channel_y);
  const Eigen::Vector2d B(p_outside_x + channel_length, channel_y);

  const Eigen::Vector2d C(B.x(), p_inside_y);
  const Eigen::Vector2d D(p_inside_x, p_inside_y);

  const Eigen::Vector2d E(D.x(), -slot_side_sgn * curb_offset);

  const Eigen::Vector2d G(p_outside_x, p_outside_y);
  const Eigen::Vector2d F(G.x(), E.y());

  pnc::geometry_lib::LineSegment channel_line;
  std::vector<pnc::geometry_lib::LineSegment> channel_line_vec;
  channel_line.SetPoints(A, B);
  channel_line_vec.emplace_back(channel_line);

  channel_line.SetPoints(B, C);
  channel_line_vec.emplace_back(channel_line);

  double ds = apa_param.GetParam().obstacle_ds;
  std::vector<Eigen::Vector2d> point_set;
  std::vector<Eigen::Vector2d> channel_obstacle_vec;
  channel_obstacle_vec.clear();
  channel_obstacle_vec.reserve(100);
  for (const auto &line : channel_line_vec) {
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, line, ds);
    channel_obstacle_vec.reserve(channel_obstacle_vec.size() +
                                 point_set.size());
    channel_obstacle_vec.insert(channel_obstacle_vec.end(), point_set.begin(),
                                point_set.end());
  }
  col_det.SetObstacles(channel_obstacle_vec);

  pnc::geometry_lib::LineSegment tlane_line;
  std::vector<pnc::geometry_lib::LineSegment> tlane_line_vec;

  // curb
  tlane_line.SetPoints(E, F);
  tlane_line_vec.emplace_back(tlane_line);

  if (set_pin_line_obs) {
    tlane_line.SetPoints(D, E);
    tlane_line_vec.emplace_back(tlane_line);
  }

  if (set_pout_line_obs) {
    tlane_line.SetPoints(F, G);
    tlane_line_vec.emplace_back(tlane_line);
  }

  if (set_pin_parallel_line_obs) {
    tlane_line.SetPoints(C, D);
    tlane_line_vec.emplace_back(tlane_line);
  }

  std::vector<Eigen::Vector2d> tlane_obstacle_vec;
  tlane_obstacle_vec.clear();
  tlane_obstacle_vec.reserve(100);
  if (tlane_line_vec.size() > 0) {
    for (const auto &line : tlane_line_vec) {
      pnc::geometry_lib::SamplePointSetInLineSeg(point_set, line, ds);
      for (const auto &obs_pt : point_set) {
        col_det.AddObstacles(obs_pt);
      }
    }
  }

  if (set_pin_obs) {
    col_det.AddObstacles(D);
  }
  if (set_pout_obs) {
    col_det.AddObstacles(G);
  }

  return 0;
}

int Update(double ego_x, double ego_y, double ego_heading, double p_outside_x,
           double p_outside_y, double p_inside_x, double p_inside_y,
           double p_target_x, double p_target_y, double channel_width,
           double channel_length, double curb_offset, double ds,
           bool is_complete_path, bool is_plan_first, bool set_left_side,
           bool set_pin_obs, bool set_pout_obs, bool set_pin_line_obs,
           bool set_pout_line_obs, bool set_pin_parallel_line_obs,
           bool ref_gear_drive, bool ref_steer_left) {
  col_det.Reset();
  planning::apa_planner::ParallelPathPlanner::Input input;

  input.ego_pose.pos << ego_x, ego_y;
  input.ego_pose.heading = ego_heading;

  input.tlane.pt_outside << p_outside_x, p_outside_y;
  input.tlane.pt_inside << p_inside_x, p_inside_y;
  input.tlane.pt_terminal_pos << p_target_x, p_target_y;

  input.tlane.slot_side = (set_left_side ? pnc::geometry_lib::SLOT_SIDE_LEFT
                                         : pnc::geometry_lib::SLOT_SIDE_RIGHT);

  input.tlane.channel_width = channel_width;
  input.tlane.channel_length = channel_length;
  input.sample_ds = ds;
  input.is_complete_path = is_complete_path;
  input.is_replan_first = is_plan_first;

  input.ref_arc_steer = (ref_steer_left ? pnc::geometry_lib::SEG_STEER_LEFT
                                        : pnc::geometry_lib::SEG_STEER_RIGHT);

  input.ref_gear = (ref_gear_drive ? pnc::geometry_lib::SEG_GEAR_DRIVE
                                   : pnc::geometry_lib::SEG_GEAR_REVERSE);

  // std::cout << "---------------" << std::endl;
  pBase->SetInput(input);

  UpdateObstacles(p_outside_x, p_outside_y, p_inside_x, p_inside_y, p_target_x,
                  p_target_y, channel_width, channel_length, curb_offset,
                  set_pin_obs, set_pout_obs, set_pin_line_obs,
                  set_pout_line_obs, set_pin_parallel_line_obs, set_left_side);
  std::shared_ptr<planning::CollisionDetector> obs_det_ptr =
      std::make_shared<planning::CollisionDetector>(col_det);

  pBase->Update(obs_det_ptr);
  // pBase->PrintOutputSegmentsInfo();

  pBase->SampleCurrentPathSeg();

  return 0;
}

std::vector<double> GetPathEle(size_t index) {
  return pBase->GetPathEle(index);
}

std::vector<double> GetObsX() {
  std::vector<double> obs_x_vec;
  obs_x_vec.reserve(200);
  for (const auto &obs_pt : col_det.GetObstacles()) {
    obs_x_vec.emplace_back(obs_pt.x());
  }
  return obs_x_vec;
}

std::vector<double> GetObsY() {
  std::vector<double> obs_y_vec;
  obs_y_vec.reserve(200);
  for (const auto &obs_pt : col_det.GetObstacles()) {
    obs_y_vec.emplace_back(obs_pt.y());
  }
  return obs_y_vec;
}

std::vector<double> GetInverseArcVec() {
  std::vector<double> res;
  res.reserve(80);

  for (const auto &path_seg : pBase->GetPlannerParams().park_out_path_in_slot) {
    if (path_seg.seg_type == pnc::geometry_lib::PathSegType::SEG_TYPE_ARC) {
      const auto arc = path_seg.arc_seg;
      res.emplace_back(arc.circle_info.center.x());
      res.emplace_back(arc.circle_info.center.y());
      res.emplace_back(arc.circle_info.radius);

      res.emplace_back(arc.pA.x());
      res.emplace_back(arc.pA.y());

      res.emplace_back(arc.pB.x());
      res.emplace_back(arc.pB.y());
    }
  }
  return res;
}

PYBIND11_MODULE(parallel_planning_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("Update", &Update)
      .def("GetPathEle", &GetPathEle)
      .def("UpdateObstacles", &UpdateObstacles)
      .def("GetObsX", &GetObsX)
      .def("GetObsY", &GetObsY)
      .def("GetInverseArcVec", &GetInverseArcVec);
}
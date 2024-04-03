#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <iostream>
#include <vector>

#include "apa_plan_base.h"
#include "collision_detection.h"
#include "debug_info_log.h"
#include "geometry_math.h"
#include "parallel_park_in_planner.h"
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

static std::vector<double> res;

int UpdateObstacles(double ego_x, double ego_y, double ego_heading,
                    double slot_occupied_ratio, double obs_p_outside_x,
                    double obs_p_outside_y, double obs_p_inside_x,
                    double obs_p_inside_y, double p_target_x, double p_target_y,
                    double channel_y, double channel_x_limit, double curb_y,
                    double obs_ds, bool set_left_side) {
  const double slot_side_sgn = (set_left_side ? -1.0 : 1.0);
  // set obstacles. eg: right side
  //  channel pt1 ----------------------------------- channel pt2
  //                         ^ y                      |
  //                         |       ego-------->     |
  //            A -----------B pout        E---------F
  //                         |-->x         |  pin
  //                         c-------------D

  const Eigen::Vector2d B(obs_p_outside_x, obs_p_outside_y);
  const Eigen::Vector2d A(obs_p_outside_x - 1.0, B.y());
  const Eigen::Vector2d E(obs_p_inside_x, obs_p_inside_y);

  if (slot_occupied_ratio > 0.7) {
    curb_y -= slot_side_sgn * 0.1;
  }
  const Eigen::Vector2d C(B.x(), curb_y);
  const Eigen::Vector2d D(E.x(), curb_y);
  const Eigen::Vector2d F(channel_x_limit, E.y());

  // channel
  const Eigen::Vector2d channel_point_1(A.x(), channel_y);
  const Eigen::Vector2d channel_point_2(F.x(), channel_y);

  pnc::geometry_lib::LineSegment channel_line;
  std::vector<pnc::geometry_lib::LineSegment> channel_line_vec;
  channel_line.SetPoints(channel_point_1, channel_point_2);
  channel_line_vec.emplace_back(channel_line);

  channel_line.SetPoints(channel_point_2, F);
  channel_line_vec.emplace_back(channel_line);

  // sample channel
  std::vector<Eigen::Vector2d> point_set;
  std::vector<Eigen::Vector2d> channel_obstacle_vec;
  const double obs_sample_ds = obs_ds;

  for (const auto &line : channel_line_vec) {
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, line, obs_sample_ds);
    channel_obstacle_vec.insert(channel_obstacle_vec.end(), point_set.begin(),
                                point_set.end());
  }
  col_det.SetObstacles(channel_obstacle_vec);

  // set tlane obs
  pnc::geometry_lib::LineSegment tlane_line;
  std::vector<pnc::geometry_lib::LineSegment> tlane_line_vec;
  // currently channel and curb are set as obstacles
  tlane_line.SetPoints(A, B);
  tlane_line_vec.emplace_back(tlane_line);

  tlane_line.SetPoints(B, C);
  tlane_line_vec.emplace_back(tlane_line);

  tlane_line.SetPoints(C, D);
  tlane_line_vec.emplace_back(tlane_line);

  tlane_line.SetPoints(D, E);
  tlane_line_vec.emplace_back(tlane_line);

  tlane_line.SetPoints(E, F);
  tlane_line_vec.emplace_back(tlane_line);

  std::vector<Eigen::Vector2d> tlane_obstacle_vec;
  tlane_obstacle_vec.clear();
  tlane_obstacle_vec.reserve(100);

  for (const auto &line : tlane_line_vec) {
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, line, obs_sample_ds);
    tlane_obstacle_vec.insert(tlane_obstacle_vec.end(), point_set.begin(),
                              point_set.end());
  }
  pnc::geometry_lib::PathPoint ego_pose;
  ego_pose.Set(Eigen::Vector2d(ego_x, ego_y), ego_heading);
  double safe_dist = 0.08;
  if (slot_occupied_ratio < 0.018) {
    safe_dist = 0.58;
    DEBUG_PRINT("safe dist =" << safe_dist);
  }

  for (const auto &obs_pos : tlane_obstacle_vec) {
    if (!col_det.IsObstacleInCar(obs_pos, ego_pose, safe_dist)) {
      col_det.AddObstacles(obs_pos);
      // std::cout << "obs_pos = " << obs_pos.transpose() << std::endl;
    }
  }
  return 0;
}

int Update(double ego_x, double ego_y, double ego_heading, double obs_pt_in_x,
           double obs_pt_in_y, double obs_pt_out_x, double obs_pt_out_y,
           double p_target_x, double p_target_y, double p_outside_x,
           double p_outside_y, double p_inside_x, double p_inside_y,
           double channel_max_x, double channel_y, double curb_y,
           double slot_width, double ds, double obs_ds, bool is_complete_path,
           bool is_plan_first, bool set_left_side, bool ref_gear_drive,
           bool ref_steer_left) {
  col_det.Reset();

  // preprocess
  const double slot_side_sgn = (set_left_side ? -1.0 : 1.0);
  obs_pt_in_y = slot_side_sgn * std::fabs(obs_pt_in_y);
  obs_pt_out_y = slot_side_sgn * std::fabs(obs_pt_out_y);
  channel_y = slot_side_sgn * std::fabs(channel_y);
  curb_y = -slot_side_sgn * std::fabs(curb_y);

  planning::apa_planner::ParallelPathPlanner::Input input;

  input.ego_pose.pos << ego_x, ego_y;
  input.ego_pose.heading = ego_heading;

  input.tlane.obs_pt_inside << obs_pt_in_x, obs_pt_in_y;
  input.tlane.obs_pt_outside << obs_pt_out_x, obs_pt_out_y;
  input.tlane.pt_terminal_pos << p_target_x, p_target_y;

  input.tlane.pt_inside << p_inside_x, p_inside_y;
  input.tlane.pt_outside << p_outside_x, p_outside_y;

  input.tlane.curb_y = curb_y;

  input.tlane.channel_y = channel_y;
  input.tlane.channel_x_limit = channel_max_x;

  input.tlane.slot_side = (set_left_side ? pnc::geometry_lib::SLOT_SIDE_LEFT
                                         : pnc::geometry_lib::SLOT_SIDE_RIGHT);

  input.sample_ds = ds;
  input.is_complete_path = is_complete_path;
  input.is_replan_first = is_plan_first;

  input.ref_arc_steer = (ref_steer_left ? pnc::geometry_lib::SEG_STEER_LEFT
                                        : pnc::geometry_lib::SEG_STEER_RIGHT);

  input.ref_gear = (ref_gear_drive ? pnc::geometry_lib::SEG_GEAR_DRIVE
                                   : pnc::geometry_lib::SEG_GEAR_REVERSE);

  // std::cout << "---------------" << std::endl;
  pBase->SetInput(input);

  const Eigen::Vector2d terminal_err =
      input.ego_pose.pos - Eigen::Vector2d(p_target_x, p_target_y);

  ParallelParInPlanner park_planner;
  const double slot_occupied_ratio = park_planner.CalcSlotOccupiedRatio(
      terminal_err, 0.5 * slot_width, !set_left_side);

  UpdateObstacles(ego_x, ego_y, ego_heading, slot_occupied_ratio, obs_pt_out_x,
                  obs_pt_out_y, obs_pt_in_x, obs_pt_in_y, p_target_x,
                  p_target_y, channel_y, channel_max_x, curb_y, obs_ds,
                  set_left_side);

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
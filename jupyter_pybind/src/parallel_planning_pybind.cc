#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <stddef.h>

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <vector>

#include "src/modules/apa_function/parking_scenario/parking_scenario.h"
#include "apa_plan_interface.h"
#include "collision_detection/collision_detection.h"
#include "config_context.h"
#include "debug_info_log.h"
#include "geometry_math.h"
#include "ifly_time.h"
#include "math_lib.h"
#include "parallel_park_in_scenario.h"
#include "parallel_path_generator.h"
#include "slot_manager.h"
#include "slot_management_info.pb.h"

namespace py = pybind11;
using namespace planning::apa_planner;

static planning::apa_planner::ParallelPathGenerator *pBase = nullptr;
static planning::apa_planner::ApaPlanInterface*pApaPlanInterface= nullptr;
static planning::apa_planner::CollisionDetector col_det;

static planning::apa_planner::ParallelParkInScenario parallel_park_planner;

int Init() {
  (void)planning::common::ConfigurationContext::Instance();
  pBase = new ParallelPathGenerator();
  pBase->Reset();

  pApaPlanInterface= new planning::apa_planner::ApaPlanInterface();
  SyncParkingParameters(true);
  return 0;
}

static std::vector<double> res;

static std::vector<std::vector<double>> debug_paths_x;
static std::vector<std::vector<double>> debug_paths_y;

int UpdateObstacles(double ego_x, double ego_y, double ego_heading,
                    double slot_occupied_ratio, double obs_p_outside_x,
                    double obs_p_outside_y, double obs_p_inside_x,
                    double obs_p_inside_y, double p_target_x, double p_target_y,
                    double channel_y, double channel_x_limit, double curb_y,
                    double obs_ds, bool set_left_side) {
  col_det.ClearObstacles();

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
  col_det.SetObstacles(channel_obstacle_vec, CollisionDetector::CHANNEL_OBS);

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
    safe_dist = 0.3;
    DEBUG_PRINT("safe dist =" << safe_dist);
  }

  for (const auto &obs_pos : tlane_obstacle_vec) {
    if (!col_det.IsObstacleInCar(obs_pos, ego_pose, safe_dist)) {
      col_det.AddObstacles(obs_pos, CollisionDetector::TLANE_OBS);
      // std::cout << "obs_pos = " << obs_pos.transpose() << std::endl;
    }
  }
  return 0;
}

int UpdateByJson(std::vector<double> obs_x_vec, std::vector<double> obs_y_vec,
                 double slot_width, double slot_length, double ego_x,
                 double ego_y, double ego_heading, double path_ds) {
  parallel_park_planner.Reset();
  DEBUG_PRINT("0");
  std::shared_ptr<ApaWorld> apa_world_ptr = std::make_shared<ApaWorld>();
  apa_world_ptr->GetApaDataPtr()->simu_param.sample_ds = path_ds;
  apa_world_ptr->GetApaDataPtr()->simu_param.is_complete_path = true;

  apa_world_ptr->GetApaDataPtr()->measurement_data.pos << ego_x, ego_y;
  apa_world_ptr->GetApaDataPtr()->measurement_data.heading = ego_heading;
  apa_world_ptr->GetApaDataPtr()->measurement_data.heading_vec =
      geometry_lib::GetUnitTangVecByHeading(ego_heading);

  DEBUG_PRINT("1");
  planning::common::SlotInfo select_slot_filter;
  const std::vector<double> slot_x_vec = {slot_length, 0.0, slot_length, 0.0};
  const std::vector<double> slot_y_vec = {0.5 * slot_width, 0.5 * slot_width,
                                          -0.5 * slot_width, -0.5 * slot_width};
  DEBUG_PRINT("2");
  for (size_t i = 0; i < slot_x_vec.size(); i++) {
    auto corner_pt =
        select_slot_filter.mutable_corner_points()->add_corner_point();
    corner_pt->set_x(slot_x_vec[i]);
    corner_pt->set_y(slot_y_vec[i]);
  }
  SlotManager::Frame slm_frame;
  slm_frame.ego_slot_info.ego_pos_slot << ego_x, ego_y;
  slm_frame.ego_slot_info.ego_heading_slot = ego_heading;
  slm_frame.ego_slot_info.select_slot_filter = select_slot_filter;
  slm_frame.ego_slot_info.slot_side = pnc::geometry_lib::SLOT_SIDE_RIGHT;
  slm_frame.ego_slot_info.slot_origin_pos << 0.0, 0.5 * slot_width;
  slm_frame.ego_slot_info.slot_origin_heading = 0.0;
  slm_frame.ego_slot_info.ego_heading_slot_vec << 1.0, 0.0;

  for (size_t i = 0; i < obs_x_vec.size(); i++) {
    slm_frame.obs_pt_vec.emplace_back(
        Eigen::Vector2d(obs_x_vec[i], obs_y_vec[i]));
  }
  DEBUG_PRINT("3");

  apa_world_ptr->GetSlotManagerPtr()->SetFrame(slm_frame);
  parallel_park_planner.SetApaWorldPtr(apa_world_ptr);
  DEBUG_PRINT("after l_park_planner.SetApaWorldPtr(apa_wo");

  parallel_park_planner.UpdateEgoSlotInfo();
  parallel_park_planner.GenTlane();
  parallel_park_planner.GenTBoundaryObstacles();
  DEBUG_PRINT("after GenTBoundaryObstacles");
  parallel_park_planner.PathPlanOnce();

  ParallelPathGenerator::Input path_planner_input;
  path_planner_input.tlane = parallel_park_planner.GetTlane();
  path_planner_input.sample_ds = path_ds;
  path_planner_input.is_replan_first = true;
  path_planner_input.is_complete_path = true;

  const auto &ego_slot_info = parallel_park_planner.GetFrame().ego_slot_info;
  path_planner_input.slot_occupied_ratio = ego_slot_info.slot_occupied_ratio;
  path_planner_input.ego_pose.Set(ego_slot_info.ego_pos_slot,
                                  ego_slot_info.ego_heading_slot);

  pBase->SetInput(path_planner_input);

  const double path_plan_start_time = IflyTime::Now_ms();

  const bool path_plan_success =
      pBase->Update(apa_world_ptr->GetCollisionDetectorPtr());

  DEBUG_PRINT("path planner cost time(ms) = " << IflyTime::Now_ms() -
                                                     path_plan_start_time);
  // const auto& path_planner_output = parallel_path_planner_.GetOutput();

  auto path_planner_output = pBase->GetOutput();
  path_planner_output.path_seg_index.first = 0;
  path_planner_output.path_seg_index.second =
      path_planner_output.path_segment_vec.size() - 1;
  if (path_plan_success) {
    pBase->SampleCurrentPathSeg();
  }

  DEBUG_PRINT(
      "path points size = " << pBase->GetOutput().path_point_vec.size());

  return static_cast<int>(path_plan_success);
}

std::vector<std::vector<double>> GetParkPlannerObs() {
  auto obs_map = parallel_park_planner.GetApaWorldPtr()
                     ->GetCollisionDetectorPtr()
                     ->GetObstaclesMap();
  std::vector<double> obs_x_vec;
  std::vector<double> obs_y_vec;
  for (const auto obs_pair : obs_map) {
    if (obs_pair.first == CollisionDetector::VIRTUAL_OBS) {
      continue;
    }
    for (const auto pt : obs_pair.second) {
      obs_x_vec.emplace_back(pt.x());
      obs_y_vec.emplace_back(pt.y());
    }
  }
  return {obs_x_vec, obs_y_vec};
}

std::vector<std::vector<double>> GenTraTBoundary(double slot_length) {
  auto obs_map = parallel_park_planner.GetApaWorldPtr()
                     ->GetCollisionDetectorPtr()
                     ->GetObstaclesMap();
  double channel_min_y = 20.0;
  double min_x = 10.0;
  double max_x = 0.0;
  for (const auto &pt : obs_map[CollisionDetector::CHANNEL_OBS]) {
    if (pt.x() < 12.0 && pt.x() > 0.0) {
      channel_min_y = std::min(channel_min_y, pt.y());
    }

    min_x = std::min(min_x, pt.x());
    max_x = std::max(max_x, pt.x());
  }
  const Eigen::Vector2d channel_pt0(min_x, channel_min_y);
  const Eigen::Vector2d channel_pt1(max_x, channel_min_y);

  double rear_max_y = -10.0;
  double rear_max_x = -10.0;

  double front_max_y = -10.0;
  double front_min_x = 100.0;

  for (const auto &pt : obs_map[CollisionDetector::TLANE_BOUNDARY_OBS]) {
    bool is_rear_tb = mathlib::IsInBound(pt.x(), -2.0, 0.3) &&
                      mathlib::IsInBound(pt.y(), 0.0, 2.2);
    if (is_rear_tb) {
      rear_max_x = std::max(rear_max_x, pt.x());
      rear_max_y = std::max(rear_max_y, pt.y());
      continue;
    }

    bool is_front_tb = mathlib::IsInBound(pt.x(), slot_length - 0.3, 20.0) &&
                       mathlib::IsInBound(pt.y(), 0.0, 2.2);
    if (is_front_tb) {
      front_min_x = std::min(front_min_x, pt.x());
      front_max_y = std::max(front_max_y, pt.y());
    }
  }
  double curb_min_y = -1.5;
  // double curb_min_y = 100.0;
  // for (const auto &pt : obs_map[CollisionDetector::TLANE_OBS]) {
  //   curb_min_y = std::min(curb_min_y, pt.y());
  // }

  const Eigen::Vector2d A(channel_pt0.x(), rear_max_y);
  const Eigen::Vector2d B(rear_max_x, rear_max_y);
  const Eigen::Vector2d C(rear_max_x, curb_min_y);
  const Eigen::Vector2d D(front_min_x, curb_min_y);
  const Eigen::Vector2d E(front_min_x, front_max_y);
  const Eigen::Vector2d F(channel_pt1.x(), front_max_y);
  std::vector<double> x_vec = {A.x(), B.x(), C.x(),           D.x(),
                               E.x(), F.x(), channel_pt1.x(), channel_pt0.x()};

  std::vector<double> y_vec = {A.y(), B.y(), C.y(),           D.y(),
                               E.y(), F.y(), channel_pt1.y(), channel_pt0.y()};
  return {x_vec, y_vec};
}

int Update(double ego_x, double ego_y, double ego_heading, double obs_pt_in_x,
           double obs_pt_in_y, double obs_pt_out_x, double obs_pt_out_y,
           double slot_length, double slot_width, double p_target_x,
           double p_target_y, double channel_max_x, double channel_y,
           double curb_y, double ds, double obs_ds, bool is_complete_path,
           bool is_plan_first, bool set_left_side, bool ref_gear_drive,
           bool ref_steer_left) {
  col_det.Reset();

  // preprocess
  const double slot_side_sgn = (set_left_side ? -1.0 : 1.0);
  obs_pt_in_y = slot_side_sgn * std::fabs(obs_pt_in_y);
  obs_pt_out_y = slot_side_sgn * std::fabs(obs_pt_out_y);
  channel_y = slot_side_sgn * std::fabs(channel_y);
  curb_y = -slot_side_sgn * std::fabs(curb_y);

  planning::apa_planner::ParallelPathGenerator::Input input;

  input.ego_pose.pos << ego_x, ego_y;
  input.ego_pose.heading = ego_heading;

  input.tlane.obs_pt_inside << obs_pt_in_x, obs_pt_in_y;
  input.tlane.obs_pt_outside << obs_pt_out_x, obs_pt_out_y;
  input.tlane.pt_terminal_pos << p_target_x, p_target_y;

  input.tlane.pt_inside << slot_length, 0.5 * slot_side_sgn * slot_width;
  input.tlane.pt_outside << 0.0, 0.5 * slot_side_sgn * slot_width;

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

  ParallelParkInScenario park_planner;
  const double slot_occupied_ratio = park_planner.CalcSlotOccupiedRatio(
      terminal_err, 0.5 * slot_width, !set_left_side);

  UpdateObstacles(ego_x, ego_y, ego_heading, slot_occupied_ratio, obs_pt_out_x,
                  obs_pt_out_y, obs_pt_in_x, obs_pt_in_y, p_target_x,
                  p_target_y, channel_y, channel_max_x, curb_y, obs_ds,
                  set_left_side);

  std::shared_ptr<CollisionDetector> obs_det_ptr =
      std::make_shared<CollisionDetector>(col_det);

  pBase->Update(obs_det_ptr);
  // pBase->PrintOutputSegmentsInfo();

  pBase->SampleCurrentPathSeg();

  return 0;
}

void SampleAllDebugPaths() {
  debug_paths_x.clear();
  debug_paths_y.clear();

  for (const auto &path : pBase->GetDebugInfo().debug_all_path_vec) {
    std::vector<pnc::geometry_lib::PathPoint> path_point_vec;
    if (pBase->SamplePathSeg(path_point_vec, path.path_segment_vec)) {
      std::vector<double> path_point_vec_x;
      std::vector<double> path_point_vec_y;
      for (const auto &path_point : path_point_vec) {
        path_point_vec_x.emplace_back(path_point.pos.x());
        path_point_vec_y.emplace_back(path_point.pos.y());
        debug_paths_x.emplace_back(path_point_vec_x);
        debug_paths_y.emplace_back(path_point_vec_y);
      }
    }
  }
}

std::vector<std::vector<double>> GetTraSearchOutPath() {
  std::vector<double> path_x_vec;
  std::vector<double> path_y_vec;
  std::vector<pnc::geometry_lib::PathPoint> path_point_vec;

  pBase->SamplePathSeg(path_point_vec,
                       pBase->GetDebugInfo().tra_search_out_res);
  for (const auto point : path_point_vec) {
    path_x_vec.emplace_back(point.pos.x());
    path_y_vec.emplace_back(point.pos.y());
  }
  std::vector<std::vector<double>> res;
  res.emplace_back(path_x_vec);
  res.emplace_back(path_y_vec);
  return res;
}

std::vector<std::vector<double>> GetDebugPathsX() { return debug_paths_x; }
std::vector<std::vector<double>> GetDebugPathsY() { return debug_paths_y; }

std::vector<double> GetPathEle(size_t index) {
  return pBase->GetPathEle(index);
}

std::vector<double> GetObsX() {
  std::vector<double> obs_x_vec;
  obs_x_vec.reserve(200);
  for (const auto &obs_pt_pair : col_det.GetObstaclesMap()) {
    for (const auto &obs_pt : obs_pt_pair.second) {
      obs_x_vec.emplace_back(obs_pt.x());
    }
  }
  return obs_x_vec;
}

std::vector<double> GetObsY() {
  std::vector<double> obs_y_vec;
  obs_y_vec.reserve(200);
  for (const auto &obs_pt_pair : col_det.GetObstaclesMap()) {
    for (const auto &obs_pt : obs_pt_pair.second) {
      obs_y_vec.emplace_back(obs_pt.y());
    }
  }
  return obs_y_vec;
}

std::vector<double> GetVirtualObsX() {
  std::vector<double> obs_x_vec;
  obs_x_vec.reserve(15);
  for (const auto &virtual_obs : pBase->GetVirtualObs()) {
    obs_x_vec.emplace_back(virtual_obs.x());
  }
  return obs_x_vec;
}

std::vector<double> GetVirtualObsY() {
  std::vector<double> obs_y_vec;
  obs_y_vec.reserve(15);
  for (const auto &virtual_obs : pBase->GetVirtualObs()) {
    obs_y_vec.emplace_back(virtual_obs.y());
  }
  return obs_y_vec;
}

std::vector<double> GetInverseArcVec() {
  std::vector<double> res;
  res.reserve(80);

  // for (size_t i = 0; i <
  // pBase->GetPlannerParams().park_out_path_in_slot.size();
  //      i++) {
  //   // tmp debug for last arc
  //   if (i != pBase->GetPlannerParams().park_out_path_in_slot.size() - 1) {
  //     continue;
  //   }
  //   const auto &path_seg =
  //   pBase->GetPlannerParams().park_out_path_in_slot[i];

  //   if (path_seg.seg_type == pnc::geometry_lib::PathSegType::SEG_TYPE_ARC)
  //   {
  //     const auto arc = path_seg.arc_seg;
  //     res.emplace_back(arc.circle_info.center.x());
  //     res.emplace_back(arc.circle_info.center.y());
  //     res.emplace_back(arc.circle_info.radius);

  //     res.emplace_back(arc.pA.x());
  //     res.emplace_back(arc.pA.y());

  //     res.emplace_back(arc.pB.x());
  //     res.emplace_back(arc.pB.y());
  //   }
  // }

  for (const auto &arc : pBase->GetDebugInfo().debug_arc_vec) {
    res.emplace_back(arc.circle_info.center.x());
    res.emplace_back(arc.circle_info.center.y());
    res.emplace_back(arc.circle_info.radius);

    res.emplace_back(arc.pA.x());
    res.emplace_back(arc.pA.y());

    res.emplace_back(arc.pB.x());
    res.emplace_back(arc.pB.y());
  }
  return res;
}

PYBIND11_MODULE(parallel_planning_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("UpdateByJson", &UpdateByJson)
      .def("GetParkPlannerObs", &GetParkPlannerObs)
      .def("GenTraTBoundary", &GenTraTBoundary)
      .def("Update", &Update)
      .def("SampleAllDebugPaths", &SampleAllDebugPaths)
      .def("GetTraSearchOutPath", &GetTraSearchOutPath)
      .def("GetDebugPathsX", &GetDebugPathsX)
      .def("GetDebugPathsY", &GetDebugPathsY)
      .def("GetPathEle", &GetPathEle)
      .def("UpdateObstacles", &UpdateObstacles)
      .def("GetObsX", &GetObsX)
      .def("GetObsY", &GetObsY)
      .def("GetVirtualObsX", &GetVirtualObsX)
      .def("GetVirtualObsY", &GetVirtualObsY)
      .def("GetInverseArcVec", &GetInverseArcVec);
}
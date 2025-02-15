#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <stddef.h>

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include "apa_plan_interface.h"
#include "collision_detection/collision_detection.h"
#include "config_context.h"
#include "debug_info_log.h"
#include "geometry_math.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "math_lib.h"
#include "parallel_park_in_scenario.h"
#include "parallel_path_generator.h"
#include "slot_management_info.pb.h"
#include "slot_manager.h"
#include "src/modules/apa_function/parking_scenario/parking_scenario.h"

using namespace planning::apa_planner;

static planning::apa_planner::ParallelPathGenerator *pBase = nullptr;
static planning::apa_planner::ApaPlanInterface *pApaPlanInterface = nullptr;
static planning::apa_planner::CollisionDetector col_det;

static planning::apa_planner::ParallelParkInScenario parallel_park_planner;

int Init() {
  planning::FilePath::SetName("preparing_step_in_py");
  planning::InitGlog(planning::FilePath::GetName().c_str());
  (void)planning::common::ConfigurationContext::Instance();
  pBase = new ParallelPathGenerator();
  pBase->Reset();

  pApaPlanInterface = new planning::apa_planner::ApaPlanInterface();
  SyncParkingParameters(true);
  return 0;
}

static std::vector<double> res;
static std::vector<double> debug_paths_x;
static std::vector<double> debug_paths_y;
static std::vector<double> debug_paths_heading;

int UpdateObstacles(double ego_x, double ego_y, double ego_heading,
                    double slot_occupied_ratio, double obs_p_outside_x,
                    double obs_p_outside_y, double obs_p_inside_x,
                    double obs_p_inside_y, double p_target_x, double p_target_y,
                    double channel_y, double channel_x_limit, double curb_y,
                    double obs_ds, bool set_left_side) {
  col_det.ClearObstacles();
  col_det.SetParam(CollisionDetector::Paramters(0.2, true));

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
    ILOG_INFO << "safe dist =" << safe_dist;
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
                 double ego_y, double ego_heading, double path_ds,
                 double prepare_line_x, double prepare_line_y,
                 double prepare_heading) {
  parallel_park_planner.Reset();

  ILOG_INFO << "---------------------------- start simulation in pybind "
               "----------------------------";
  ILOG_INFO << "obs_x_vec.size() = " << obs_x_vec.size();
  ILOG_INFO << "obs_y_vec.size() = " << obs_y_vec.size();

  std::shared_ptr<ApaWorld> apa_world_ptr = std::make_shared<ApaWorld>();
  SimulationParam simu_param;
  simu_param.sample_ds = path_ds;
  simu_param.is_simulation = true;
  simu_param.is_complete_path = true;
  apa_world_ptr->SetSimuParam(simu_param);

  apa_world_ptr->GetMeasureDataManagerPtr()->SetPose(
      Eigen::Vector2d(ego_x, ego_y), ego_heading);

  planning::common::SlotInfo select_slot_filter;
  const std::vector<double> slot_x_vec = {slot_length, 0.0, slot_length, 0.0};
  const std::vector<double> slot_y_vec = {0.5 * slot_width, 0.5 * slot_width,
                                          -0.5 * slot_width, -0.5 * slot_width};
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
    const Eigen::Vector2d obs_pt(obs_x_vec[i], obs_y_vec[i]);
    slm_frame.obs_pt_vec.emplace_back(obs_pt);
    slm_frame.ego_slot_info.obs_pt_vec_slot.emplace_back(obs_pt);
  }

  apa_world_ptr->GetSlotManagerPtr()->SetFrame(slm_frame);
  parallel_park_planner.SetApaWorldPtr(apa_world_ptr);
  parallel_park_planner.UpdateEgoSlotInfo();
  parallel_park_planner.GetMutableFrame()->ego_slot_info.obs_pt_vec_slot =
      slm_frame.obs_pt_vec;

  parallel_park_planner.GenTlane();
  parallel_park_planner.GenTBoundaryObstacles();

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
  apa_world_ptr->GetCollisionDetectorPtr()->SetParam(
      CollisionDetector::Paramters(0.2, true));
  pBase->SetColPtr(apa_world_ptr->GetCollisionDetectorPtr());

  debug_paths_x.clear();
  debug_paths_y.clear();
  debug_paths_heading.clear();
  ILOG_INFO << "-------------CalPreparingPaths DEBUG -------------------";

  const Eigen::Vector2d ego_pos(ego_x, ego_y);
  const pnc::geometry_lib::PathPoint ego_pose(ego_pos, ego_heading);

  const Eigen::Vector2d preparing_pos(prepare_line_x, prepare_line_y);
  const pnc::geometry_lib::LineSegment preparing_line =
      pnc::geometry_lib::BuildLineSegByPose(preparing_pos, prepare_heading);

  std::vector<pnc::geometry_lib::PathSegment> ego_to_prepare_seg_vec;
  const bool success = pBase->PlanToPreparingLine(ego_to_prepare_seg_vec,
                                                  ego_pose, preparing_line);
  if (ego_to_prepare_seg_vec.back().seg_type ==
      pnc::geometry_lib::SEG_TYPE_LINE) {
    ego_to_prepare_seg_vec.pop_back();
  }
  pnc::geometry_lib::PrintSegmentsVecInfo(ego_to_prepare_seg_vec);
  if (success && ego_to_prepare_seg_vec.size() > 0) {
    const auto path_point_vec =
        pnc::geometry_lib::SamplePathSegVec(ego_to_prepare_seg_vec, path_ds);
    std::cout << "path_point_vec size = " << path_point_vec.size() << std::endl;

    std::vector<double> path_point_vec_x;
    std::vector<double> path_point_vec_y;
    std::vector<double> path_point_vec_heading;
    for (const auto &path_point : path_point_vec) {
      debug_paths_x.emplace_back(path_point.pos.x());
      debug_paths_y.emplace_back(path_point.pos.y());
      debug_paths_heading.emplace_back(path_point.heading);
    }
  }

  return success;
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

std::vector<double> GetDebugPathsX() { return debug_paths_x; }
std::vector<double> GetDebugPathsY() { return debug_paths_y; }
std::vector<double> GetDebugPathsHeading() { return debug_paths_heading; }

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

PYBIND11_MODULE(parallel_preparing_planning_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("UpdateByJson", &UpdateByJson)
      .def("GetParkPlannerObs", &GetParkPlannerObs)
      .def("GetDebugPathsX", &GetDebugPathsX)
      .def("GetDebugPathsY", &GetDebugPathsY)
      .def("GetDebugPathsHeading", &GetDebugPathsHeading)
      .def("GetPathEle", &GetPathEle)
      .def("UpdateObstacles", &UpdateObstacles)
      .def("GetObsX", &GetObsX)
      .def("GetObsY", &GetObsY)
      .def("GetVirtualObsX", &GetVirtualObsX)
      .def("GetVirtualObsY", &GetVirtualObsY)
      .def("GetInverseArcVec", &GetInverseArcVec);
}
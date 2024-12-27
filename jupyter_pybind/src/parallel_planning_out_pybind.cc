#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <stddef.h>

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <vector>

#include "apa_plan_interface.h"
#include "collision_detection/collision_detection.h"
#include "config_context.h"
#include "debug_info_log.h"
#include "geometry_math.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "math_lib.h"
#include "parallel_out_path_generator.h"
#include "parallel_park_out_scenario.h"
#include "parallel_path_generator.h"
#include "slot_management_info.pb.h"
#include "slot_manager.h"
#include "src/modules/apa_function/parking_scenario/parking_scenario.h"

using namespace planning::apa_planner;

static planning::apa_planner::ParallelOutPathGenerator *pBase = nullptr;
static planning::apa_planner::ApaPlanInterface *pApaPlanInterface = nullptr;
static planning::apa_planner::CollisionDetector col_det;

static planning::apa_planner::ParallelParkOutScenario parallel_park_planner;

int Init() {
  planning::FilePath::SetName("parallel_parking_Out_py");
  planning::InitGlog(planning::FilePath::GetName().c_str());
  (void)planning::common::ConfigurationContext::Instance();
  pBase = new ParallelOutPathGenerator();
  pBase->Reset();

  pApaPlanInterface = new planning::apa_planner::ApaPlanInterface();
  SyncParkingParameters(true);
  return 0;
}

static std::vector<double> res;
static std::vector<std::vector<double>> debug_paths_x;
static std::vector<std::vector<double>> debug_paths_y;

int UpdateByJson(std::vector<double> obs_x_vec, std::vector<double> obs_y_vec,
                 double slot_width, double slot_length, double ego_x,
                 double ego_y, double ego_heading, double path_ds, int ref_gear,
                 int is_dirve_out_left) {
  parallel_park_planner.Reset();
  ILOG_INFO << "---------------------------- start simulation in pybind "
               "----------------------------";
  ILOG_INFO << "obs_x_vec.size() = " << obs_x_vec.size();
  ILOG_INFO << "obs_y_vec.size() = " << obs_y_vec.size();

  std::shared_ptr<ApaWorld> apa_world_ptr = std::make_shared<ApaWorld>();

  // set apa data
  SimulationParam simu_param;
  simu_param.sample_ds = path_ds;
  simu_param.is_simulation = true;
  simu_param.is_complete_path = true;
  apa_world_ptr->SetSimuParam(simu_param);

  // set function state mechine
  if (is_dirve_out_left) {
    apa_world_ptr->GetStateMachineManagerPtr()->SetParkOutDirection(
        ApaParkOutDirection::LEFT_FRONT);
  } else {
    apa_world_ptr->GetStateMachineManagerPtr()->SetParkOutDirection(
        ApaParkOutDirection::RIGHT_FRONT);
  }

  // set measurement
  apa_world_ptr->GetMeasureDataManagerPtr()->SetPose(
      Eigen::Vector2d(ego_x, ego_y), ego_heading);

  // slot management
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

  auto &park_planner_frame = parallel_park_planner.SetFrame();
  park_planner_frame.ego_slot_info.obs_pt_vec_slot = slm_frame.obs_pt_vec;

  if (ref_gear == 0) {
    park_planner_frame.current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
  } else if (ref_gear == 1) {
    park_planner_frame.current_gear = pnc::geometry_lib::SEG_GEAR_DRIVE;
  } else {
    park_planner_frame.current_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
  }
  parallel_park_planner.GenTlane();
  parallel_park_planner.GenTBoundaryObstacles();
  const double path_plan_start_time = IflyTime::Now_ms();
  // const bool success = parallel_park_planner.PathPlanOnce();

  ParallelOutPathGenerator::Input path_planner_input;
  path_planner_input.tlane = parallel_park_planner.GetTlane();
  path_planner_input.sample_ds =
      apa_world_ptr->GetSimuParam().sample_ds;
  path_planner_input.is_replan_first = (ref_gear == 0);
  path_planner_input.is_complete_path = true;

  const auto &ego_slot_info = parallel_park_planner.GetFrame().ego_slot_info;
  path_planner_input.slot_occupied_ratio = ego_slot_info.slot_occupied_ratio;
  path_planner_input.ego_pose.Set(ego_slot_info.ego_pos_slot,
                                  ego_slot_info.ego_heading_slot);

  auto &path_planner = parallel_park_planner.SetPathPlanner();
  path_planner.SetInput(path_planner_input);

  const bool path_plan_success = path_planner.Update(
      parallel_park_planner.GetApaWorldPtr()->GetCollisionDetectorPtr());
  if (path_plan_success) {
    path_planner.SetCurrentPathSegIndex();
    path_planner.SampleCurrentPathSeg();
  }

  return path_plan_success;
}

std::vector<std::vector<double>> GetParkPlannerObs() {
  ILOG_INFO << "GetParkPlannerObs";
  auto obs_map = parallel_park_planner.GetApaWorldPtr()
                     ->GetCollisionDetectorPtr()
                     ->GetObstaclesMap();
  std::vector<double> obs_x_vec;
  std::vector<double> obs_y_vec;
  for (const auto obs_pair : obs_map) {
    if (obs_pair.first == CollisionDetector::VIRTUAL_OBS) {
      continue;
    }
    if (obs_pair.first == 1) {
      ILOG_INFO << "channel size = " << obs_pair.second.size();
    }

    for (const auto pt : obs_pair.second) {
      obs_x_vec.emplace_back(pt.x());
      obs_y_vec.emplace_back(pt.y());
    }
  }
  return {obs_x_vec, obs_y_vec};
}

std::vector<double> GetPathEle(size_t index) {
  return parallel_park_planner.GetPathPlanner().GetPathEle(index);
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

PYBIND11_MODULE(parallel_planning_out_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("UpdateByJson", &UpdateByJson)
      .def("GetParkPlannerObs", &GetParkPlannerObs)
      .def("GetPathEle", &GetPathEle)
      .def("GetObsX", &GetObsX)
      .def("GetObsY", &GetObsY);
}
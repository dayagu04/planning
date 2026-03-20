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

  ApaObstacle apa_obs;
  std::vector<Eigen::Vector2d> obs_vec;
  obs_vec.resize(obs_x_vec.size());
  for (size_t i = 0; i < obs_x_vec.size(); i++) {
    obs_vec[i] << obs_x_vec[i], obs_y_vec[i];
  }
  apa_obs.SetPtClout2dGlobal(obs_vec);
  apa_obs.SetObsMovementType(ApaObsMovementType::STATIC);
  apa_obs.SetObsAttributeType(ApaObsAttributeType::FUSION_POINT_CLOUD);
  apa_obs.SetId(0);
  std::unordered_map<size_t, ApaObstacle> &obstacles =
      apa_world_ptr->GetObstacleManagerPtr()->GetMutableObstacles();
  obstacles[0] = apa_obs;

  // set function state mechine
  if (is_dirve_out_left) {
    apa_world_ptr->GetStateMachineManagerPtr()->SetParkOutDirection(
        ApaParkOutDirection::LEFT_FRONT);

  } else {
    apa_world_ptr->GetStateMachineManagerPtr()->SetParkOutDirection(
        ApaParkOutDirection::RIGHT_FRONT);
  }

  SimulationParam simu_param;
  simu_param.sample_ds = path_ds;
  simu_param.is_complete_path = true;
  apa_world_ptr->SetSimuParam(simu_param);

  // set measurement
  apa_world_ptr->GetMeasureDataManagerPtr()->SetPose(
      Eigen::Vector2d(ego_x, ego_y), ego_heading);

  EgoInfoUnderSlot &ego_info_under_slot =
      apa_world_ptr->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();
  ego_info_under_slot.id = 1;

  const double half_slot_width = 0.5 * slot_width;

  ego_info_under_slot.slot.origin_corner_coord_global_.pt_0 << slot_length,
      half_slot_width;
  ego_info_under_slot.slot.origin_corner_coord_global_.pt_1 << 0.0,
      half_slot_width;
  ego_info_under_slot.slot.origin_corner_coord_global_.pt_2 << slot_length,
      -half_slot_width;
  ego_info_under_slot.slot.origin_corner_coord_global_.pt_3 << 0.0,
      -half_slot_width;
  ego_info_under_slot.slot.origin_corner_coord_global_.CalExtraCoord();
  if (ref_gear == 0) {
    parallel_park_planner.GetMutableFrame()->current_gear =
        pnc::geometry_lib::SEG_GEAR_INVALID;
  } else if (ref_gear == 1) {
    parallel_park_planner.GetMutableFrame()->current_gear =
        pnc::geometry_lib::SEG_GEAR_DRIVE;
  } else {
    parallel_park_planner.GetMutableFrame()->current_gear =
        pnc::geometry_lib::SEG_GEAR_REVERSE;
  }

  parallel_park_planner.SetApaWorldPtr(apa_world_ptr);
  if (!parallel_park_planner.UpdateEgoSlotInfo()) {
    ILOG_INFO << "UpdateEgoSlotInfo failed!";
    return false;
  }
  parallel_park_planner.GenTlane();
  parallel_park_planner.GenTBoundaryObstacles();

  GeometryPathInput path_planner_input;
  path_planner_input.tlane = parallel_park_planner.GetTlane();
  path_planner_input.sample_ds = apa_world_ptr->GetSimuParam().sample_ds;
  path_planner_input.is_replan_first = (ref_gear == 0);
  path_planner_input.is_complete_path = true;
  path_planner_input.ego_info_under_slot = ego_info_under_slot;

  auto path_planner = parallel_park_planner.GetMutablePathPlanner();
  path_planner->SetInput(path_planner_input);

  const bool path_plan_success =
      path_planner->Update(apa_world_ptr->GetCollisionDetectorPtr());
  if (path_plan_success) {
    path_planner->SetCurrentPathSegIndex();
    path_planner->SampleCurrentPathSeg();
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
    // if (obs_pair.first == CollisionDetector::VIRTUAL_OBS) {
    //   continue;
    // }

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
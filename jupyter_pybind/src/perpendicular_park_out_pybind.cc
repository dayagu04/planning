#include <gflags/gflags.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>

#include "Eigen/Core"
#include "apa_param_config.h"
#include "apa_plan_interface.h"
#include "apa_slot.h"
#include "apa_world.h"
#include "collision_detection/collision_detection.h"
#include "common_c.h"
#include "config_context.h"
#include "geometry_math.h"
#include "geometry_path_generator.h"
#include "hmi_inner_c.h"
#include "local_view.h"
#include "log_glog.h"
#include "math_lib.h"
#include "perpendicular_head_out_scenario.h"
#include "src/modules/apa_function/parking_scenario/parking_scenario.h"

using namespace planning::apa_planner;
using namespace pnc;
planning::LocalView g_local_view;
std::shared_ptr<ApaWorld> g_apa_world_ptr;
std::shared_ptr<ParkingScenario> g_scenario_ptr;
static planning::apa_planner::ApaPlanInterface* pApaPlanInterface = nullptr;
void Init() {
  planning::FilePath::SetName("slant_simulation_pybind");
  planning::InitGlog(planning::FilePath::GetName().c_str());
  (void)planning::common::ConfigurationContext::Instance();
  pApaPlanInterface = new planning::apa_planner::ApaPlanInterface();
  pApaPlanInterface->Init(true);

  g_apa_world_ptr = std::make_shared<ApaWorld>();
  g_scenario_ptr =
      std::make_shared<PerpendicularHeadOutScenario>(g_apa_world_ptr);
}

void UpdateLocalization(Eigen::Vector3d pose) {
  ILOG_INFO << "\n\n\n UpdateLocalization";
  g_local_view.localization.position.position_boot.x = pose.x();
  g_local_view.localization.position.position_boot.y = pose.y();
  g_local_view.localization.orientation.euler_boot.yaw =
      geometry_lib::NormalizeAngle(pose.z());
}

void UpdateSlot(std::vector<Eigen::Vector2d> pt_vec) {
  ILOG_INFO << "UpdateSlot";
  auto& park_slot = g_local_view.parking_fusion_info;
  park_slot.parking_fusion_slot_lists_size = 1;
  park_slot.select_slot_id = 1;
  auto& slot = park_slot.parking_fusion_slot_lists[0];
  slot.id = 1;
  slot.allow_parking = iflyauto::ALLOW_PARKING;

  g_apa_world_ptr->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot().id =
      slot.id;

  Eigen::Vector2d pt_01_vec = (pt_vec[1] - pt_vec[0]).normalized();
  Eigen::Vector2d pt_23mid_01mid_vec =
      ((pt_vec[1] + pt_vec[0] - pt_vec[2] - pt_vec[3]) * 0.5).normalized();
  double angle = std::fabs(
      geometry_lib::GetAngleFromTwoVec(pt_01_vec, pt_23mid_01mid_vec) *
      kRad2Deg);
  if (angle > 80.0 && angle < 100.0) {
    slot.type = iflyauto::PARKING_SLOT_TYPE_VERTICAL;
    g_apa_world_ptr->GetSlotManagerPtr()
        ->GetMutableEgoInfoUnderSlot()
        .slot_type = SlotType::PERPENDICULAR;
    ILOG_INFO << "PERPENDICULAR SLOT";
  } else {
    slot.type = iflyauto::PARKING_SLOT_TYPE_SLANTING;
    g_apa_world_ptr->GetSlotManagerPtr()
        ->GetMutableEgoInfoUnderSlot()
        .slot_type = SlotType::SLANT;
    ILOG_INFO << "SLANT SLOT";
  }
  for (size_t i = 0; i < pt_vec.size() && i < 4; ++i) {
    slot.corner_points[i].x = pt_vec[i].x();
    slot.corner_points[i].y = pt_vec[i].y();
  }
}

void UpdateObs(std::vector<Eigen::Vector2d> obs_vec) {
  ILOG_INFO << "UpdateObs";
  auto& fusion_obs = g_local_view.fusion_occupancy_objects_info;
  fusion_obs.fusion_object_size = 1;
  auto& obs = fusion_obs.fusion_object[0];
  obs.additional_occupancy_info.polygon_points_size = obs_vec.size();
  for (size_t i = 0; i < obs_vec.size() && i < 1132; ++i) {
    obs.additional_occupancy_info.polygon_points[i].x = obs_vec[i].x();
    obs.additional_occupancy_info.polygon_points[i].y = obs_vec[i].y();
  }
}

void UpdateStateMachine() {
  ILOG_INFO << "UpdateStateMachine";
  g_local_view.function_state_machine_info.current_state =
      iflyauto::FunctionalState_PARK_GUIDANCE;
  g_local_view.function_state_machine_info.parking_req.apa_work_mode =
      iflyauto::APA_WORK_MODE_PARKING_OUT;
}

void UpdateApaParkOutDirection(int direction_index) {
  ILOG_INFO << "UpdateApaParkOutDirection";
  switch (direction_index) {
    case 0:
      g_local_view.function_state_machine_info.parking_req
          .apa_park_out_direction = iflyauto::PRK_OUT_TO_FRONT_RIGHT_CROSS;
      break;
    case 1:
      g_local_view.function_state_machine_info.parking_req
          .apa_park_out_direction = iflyauto::PRK_OUT_TO_FRONT_OUT;
      break;
    case 2:
      g_local_view.function_state_machine_info.parking_req
          .apa_park_out_direction = iflyauto::PRK_OUT_TO_FRONT_RIGHT_CROSS;
      break;
  }
}

void Update() {
  iflyauto::PlanningOutput planning_output;
  g_apa_world_ptr->Update(&g_local_view, planning_output);
  g_scenario_ptr->Reset();
  g_scenario_ptr->ScenarioRunning();
}

std::vector<Eigen::Vector4d> GetCompletePlanPath() {
  std::vector<Eigen::Vector4d> path_vec;
  path_vec.reserve(g_scenario_ptr->GetCompletePlanPathPt().size() + 6);
  for (const auto& pt : g_scenario_ptr->GetCompletePlanPathPt()) {
    path_vec.emplace_back(
        Eigen::Vector4d(pt.pos.x(), pt.pos.y(), pt.heading, pt.lat_buffer));
  }
  return path_vec;
}

std::vector<Eigen::Vector4d> GetCurrentGearPlanPath() {
  std::vector<Eigen::Vector4d> path_vec;
  path_vec.reserve(g_scenario_ptr->GetCurrentGearPlanPathPt().size() + 6);
  for (const auto& pt : g_scenario_ptr->GetCurrentGearPlanPathPt()) {
    path_vec.emplace_back(
        Eigen::Vector4d(pt.pos.x(), pt.pos.y(), pt.heading, pt.lat_buffer));
  }
  return path_vec;
}

std::vector<Eigen::Vector2d> GetObsVec() {
  std::vector<Eigen::Vector2d> obs_vec;
  const std::unordered_map<size_t, std::vector<Eigen::Vector2d>>&
      obstacles_map =
          g_apa_world_ptr->GetCollisionDetectorPtr()->GetObstaclesMap();

  const geometry_lib::LocalToGlobalTf& l2g_tf =
      g_apa_world_ptr->GetSlotManagerPtr()->GetEgoInfoUnderSlot().l2g_tf;

  for (const auto& obs_pair : obstacles_map) {
    for (const auto& obstacle : obs_pair.second) {
      obs_vec.emplace_back(l2g_tf.GetPos(obstacle));
    }
  }
  return obs_vec;
}

PYBIND11_MODULE(perpendicular_parking_out_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("Update", &Update)
      .def("UpdateLocalization", &UpdateLocalization)
      .def("UpdateStateMachine", &UpdateStateMachine)
      .def("UpdateApaParkOutDirection", &UpdateApaParkOutDirection)
      .def("UpdateSlot", &UpdateSlot)
      .def("UpdateObs", &UpdateObs)
      .def("GetCurrentGearPlanPath", &GetCurrentGearPlanPath)
      .def("GetCompletePlanPath", &GetCompletePlanPath)
      .def("GetObsVec", &GetObsVec);
  ;
}

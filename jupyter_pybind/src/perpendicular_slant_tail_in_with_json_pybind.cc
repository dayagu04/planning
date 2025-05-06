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
#include "apa_obstacle.h"
#include "apa_param_config.h"
#include "apa_plan_interface.h"
#include "apa_slot.h"
#include "apa_slot_manager.h"
#include "apa_world.h"
#include "collision_detection/collision_detection.h"
#include "common_c.h"
#include "config_context.h"
#include "geometry_math.h"
#include "geometry_path_generator.h"
#include "local_view.h"
#include "log_glog.h"
#include "math_lib.h"
#include "perpendicular_tail_in_scenario.h"
#include "src/modules/apa_function/parking_scenario/parking_scenario.h"

using namespace planning::apa_planner;
using namespace pnc;
planning::LocalView g_local_view;
SimulationParam g_simu_param;
std::shared_ptr<ApaWorld> g_apa_world_ptr;
std::shared_ptr<ParkingScenario> g_scenario_ptr;
static planning::apa_planner::ApaPlanInterface* pApaPlanInterface = nullptr;
void Init() {
  planning::FilePath::SetName("perpendicular_slant_tail_in_with_json_pybind");
  planning::InitGlog(planning::FilePath::GetName().c_str());
  (void)planning::common::ConfigurationContext::Instance();
  pApaPlanInterface = new planning::apa_planner::ApaPlanInterface();
  pApaPlanInterface->Init(true);

  g_apa_world_ptr = std::make_shared<ApaWorld>();
  g_scenario_ptr =
      std::make_shared<PerpendicularTailInScenario>(g_apa_world_ptr);
}

void UpdateSimuParams(int is_path_optimization, int is_cilqr_enable,
                      int is_complete_path, int use_average_obs_dist,
                      int force_mid_process_plan, double sample_ds, int set_obs,
                      double right_obj_dx, double right_obj_dy,
                      double left_obj_dx, double left_obj_dy,
                      double channel_width) {
  ILOG_INFO << "\n\n\n UpdateSimuParams";
  g_simu_param.is_simulation = true;
  g_simu_param.is_path_optimization = is_path_optimization;
  g_simu_param.is_cilqr_optimization = is_cilqr_enable;
  g_simu_param.is_complete_path = is_complete_path;
  g_simu_param.use_average_obs_dist = use_average_obs_dist;
  g_simu_param.force_mid_process_plan = force_mid_process_plan;
  g_simu_param.sample_ds = sample_ds;

  ApaParameters& muable_param = apa_param.SetPram();
  SyncParkingParameters(true);
  if (set_obs == 1) {
    muable_param.channel_width = channel_width;
    muable_param.virtual_obs_left_x_pos = left_obj_dx;
    muable_param.virtual_obs_left_y_pos = left_obj_dy;
    muable_param.virtual_obs_right_x_pos = right_obj_dx;
    muable_param.virtual_obs_right_y_pos = right_obj_dy;
  }
}

void UpdateLocalization(Eigen::Vector3d pose) {
  ILOG_INFO << "UpdateLocalization";
  g_local_view.localization.position.position_boot.x = pose.x();
  g_local_view.localization.position.position_boot.y = pose.y();
  g_local_view.localization.orientation.euler_boot.yaw =
      geometry_lib::NormalizeAngle(pose.z());
}

void UpdateSlot(
    int select_id, std::vector<int> id_vec, std::vector<int> type_vec,
    std::vector<std::vector<Eigen::Vector2d>> corner_points_vec,
    std::vector<std::vector<std::vector<Eigen::Vector2d>>> limiter_points_vec) {
  ILOG_INFO << "UpdateSlot";
  auto& park_slot = g_local_view.parking_fusion_info;
  park_slot.parking_fusion_slot_lists_size = id_vec.size();
  park_slot.select_slot_id = select_id;
  g_apa_world_ptr->GetSlotManagerPtr()->ego_info_under_slot_.id = select_id;

  for (size_t i = 0; i < id_vec.size(); ++i) {
    auto& slot = park_slot.parking_fusion_slot_lists[i];
    slot.id = id_vec[i];
    slot.type = static_cast<iflyauto::ParkingSlotType>(type_vec[i]);
    if (slot.id == select_id) {
      switch (slot.type) {
        case iflyauto::PARKING_SLOT_TYPE_VERTICAL:
          g_apa_world_ptr->GetSlotManagerPtr()->ego_info_under_slot_.slot_type =
              SlotType::PERPENDICULAR;
          break;
        case iflyauto::PARKING_SLOT_TYPE_SLANTING:
          g_apa_world_ptr->GetSlotManagerPtr()->ego_info_under_slot_.slot_type =
              SlotType::SLANT;
          break;
        case ::iflyauto::PARKING_SLOT_TYPE_HORIZONTAL:
          g_apa_world_ptr->GetSlotManagerPtr()->ego_info_under_slot_.slot_type =
              SlotType::PARALLEL;
          break;
        default:
          break;
      }
    }
    slot.allow_parking = iflyauto::ALLOW_PARKING;

    for (size_t j = 0; j < corner_points_vec[i].size(); ++j) {
      slot.corner_points[j].x = corner_points_vec[i][j].x();
      slot.corner_points[j].y = corner_points_vec[i][j].y();
    }

    // 每个车位的限位器 可能 2 1 0
    std::vector<std::vector<Eigen::Vector2d>> limit = limiter_points_vec[i];
    slot.limiters_size = limit.size();
    for (size_t j = 0; j < limit.size(); ++j) {
      slot.limiters[j].end_points[0].x = limit[j][0].x();
      slot.limiters[j].end_points[0].y = limit[j][0].y();
      slot.limiters[j].end_points[1].x = limit[j][1].x();
      slot.limiters[j].end_points[1].y = limit[j][1].y();
    }
  }
}

void UpdateFusionObs(std::vector<std::vector<Eigen::Vector2d>> obs_vec) {
  ILOG_INFO << "UpdateFusionObs";
  auto& fusion_obs = g_local_view.fusion_occupancy_objects_info;
  fusion_obs.fusion_object_size = obs_vec.size();
  for (size_t i = 0; i < obs_vec.size(); ++i) {
    auto& obs = fusion_obs.fusion_object[i];
    obs.additional_occupancy_info.polygon_points_size = obs_vec[i].size();
    for (size_t j = 0; j < obs_vec[i].size(); ++j) {
      obs.additional_occupancy_info.polygon_points[j].x = obs_vec[i][j].x();
      obs.additional_occupancy_info.polygon_points[j].y = obs_vec[i][j].y();
    }
  }
}

void UpdateGroundLineObs(std::vector<std::vector<Eigen::Vector2d>> obs_vec) {
  ILOG_INFO << "UpdateGroundLineObs";
  auto& gl_obs = g_local_view.ground_line_perception;
  gl_obs.groundline_size = obs_vec.size();
  for (size_t i = 0; i < obs_vec.size(); ++i) {
    auto& obs = gl_obs.groundline[i];
    obs.groundline_point_size = obs_vec[i].size();
    for (size_t j = 0; j < obs_vec[i].size(); ++j) {
      obs.groundline_point[j].x = obs_vec[i][j].x();
      obs.groundline_point[j].y = obs_vec[i][j].y();
    }
  }
}

void UpdateUssPerceptionObs(std::vector<std::vector<Eigen::Vector2d>> obs_vec) {
  ILOG_INFO << "UpdateUssPerceptionObs";
  auto& uss_obs = g_local_view.uss_percept_info;
  for (size_t i = 0; i < obs_vec.size(); ++i) {
    auto& obs = uss_obs.out_line_dataori[i];
    obs.obj_pt_cnt = obs_vec[i].size();
    for (size_t j = 0; j < obs_vec[i].size(); ++j) {
      obs.obj_pt_global[j].x = obs_vec[i][j].x();
      obs.obj_pt_global[j].y = obs_vec[i][j].y();
    }
  }
}

void UpdateStateMachine() {
  ILOG_INFO << "UpdateStateMachine";
  g_local_view.function_state_machine_info.current_state =
      iflyauto::FunctionalState_PARK_GUIDANCE;
  g_local_view.function_state_machine_info.parking_req.apa_work_mode =
      iflyauto::APA_WORK_MODE_PARKING_IN;
  g_local_view.function_state_machine_info.parking_req.apa_parking_direction =
      iflyauto::BACK_END_PARKING_DIRECTION;
}

void Update() {
  iflyauto::PlanningOutput planning_output;
  g_apa_world_ptr->GetParkingTaskInterfacePtr()->Reset();
  g_apa_world_ptr->SetSimuParam(g_simu_param);
  g_apa_world_ptr->Update(&g_local_view, planning_output);
  g_scenario_ptr->Reset();
  g_scenario_ptr->ScenarioRunning();
}

std::vector<std::vector<Eigen::Vector4d>> GetPerferredPlanPath() {
  std::vector<std::vector<Eigen::Vector4d>> path_vec_vec;
  if (g_scenario_ptr->GetPlannerStates().planning_status ==
      ParkingScenario::ParkingStatus::PARKING_FAILED) {
    return path_vec_vec;
  }
  path_vec_vec.reserve(g_scenario_ptr->GetPerferredGeometryPathVec().size() +
                       6);
  const auto& l2g_tf =
      g_apa_world_ptr->GetSlotManagerPtr()->ego_info_under_slot_.l2g_tf;
  for (auto& geometry_path : g_scenario_ptr->GetPerferredGeometryPathVec()) {
    geometry_path.Sample(g_simu_param.sample_ds);
    std::vector<Eigen::Vector4d> path_vec;
    for (const auto& pt : geometry_path.path_pt_vec) {
      path_vec.emplace_back(
          Eigen::Vector4d(l2g_tf.GetPos(pt.pos).x(), l2g_tf.GetPos(pt.pos).y(),
                          l2g_tf.GetHeading(pt.heading), pt.lat_buffer));
    }
    path_vec_vec.emplace_back(path_vec);
  }
  return path_vec_vec;
}

std::vector<Eigen::Vector4d> GetCompletePlanPath() {
  std::vector<Eigen::Vector4d> path_vec;
  if (g_scenario_ptr->GetPlannerStates().planning_status ==
      ParkingScenario::ParkingStatus::PARKING_FAILED) {
    return path_vec;
  }
  path_vec.reserve(g_scenario_ptr->GetCompletePlanPathPt().size() + 6);
  for (const auto& pt : g_scenario_ptr->GetCompletePlanPathPt()) {
    path_vec.emplace_back(
        Eigen::Vector4d(pt.pos.x(), pt.pos.y(), pt.heading, pt.lat_buffer));
  }
  return path_vec;
}

std::vector<Eigen::Vector4d> GetCurrentGearPlanPath() {
  std::vector<Eigen::Vector4d> path_vec;
  if (g_scenario_ptr->GetPlannerStates().planning_status ==
      ParkingScenario::ParkingStatus::PARKING_FAILED) {
    return path_vec;
  }
  path_vec.reserve(g_scenario_ptr->GetCurrentGearPlanPathPt().size() + 6);
  for (const auto& pt : g_scenario_ptr->GetCurrentGearPlanPathPt()) {
    path_vec.emplace_back(
        Eigen::Vector4d(pt.pos.x(), pt.pos.y(), pt.heading, pt.lat_buffer));
  }
  return path_vec;
}

std::vector<Eigen::Vector2d> GetObsVec() {
  std::vector<Eigen::Vector2d> obs_vec;
  if (g_scenario_ptr->GetPlannerStates().planning_status ==
      ParkingScenario::ParkingStatus::PARKING_FAILED) {
    return obs_vec;
  }
  const geometry_lib::LocalToGlobalTf& l2g_tf =
      g_apa_world_ptr->GetSlotManagerPtr()->ego_info_under_slot_.l2g_tf;

  const std::unordered_map<size_t, ApaObstacle>& obs =
      g_apa_world_ptr->GetObstacleManagerPtr()->GetObstacles();

  for (const auto& pair : obs) {
    if (pair.second.GetObsAttributeType() !=
        ApaObsAttributeType::VIRTUAL_POINT_CLOUD) {
      continue;
    }
    for (const Eigen::Vector2d& pt : pair.second.GetPtClout2dLocal()) {
      obs_vec.emplace_back(l2g_tf.GetPos(pt));
    }
  }
  return obs_vec;
}

PYBIND11_MODULE(perpendicular_slant_tail_in_with_json_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("Update", &Update)
      .def("UpdateSimuParams", &UpdateSimuParams)
      .def("UpdateLocalization", &UpdateLocalization)
      .def("UpdateStateMachine", &UpdateStateMachine)
      .def("UpdateSlot", &UpdateSlot)
      .def("UpdateFusionObs", &UpdateFusionObs)
      .def("UpdateGroundLineObs", &UpdateGroundLineObs)
      .def("UpdateUssPerceptionObs", &UpdateUssPerceptionObs)
      .def("GetCurrentGearPlanPath", &GetCurrentGearPlanPath)
      .def("GetCompletePlanPath", &GetCompletePlanPath)
      .def("GetObsVec", &GetObsVec)
      .def("GetPerferredPlanPath", &GetPerferredPlanPath);
  ;
}

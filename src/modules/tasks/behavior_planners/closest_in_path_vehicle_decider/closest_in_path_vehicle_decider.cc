#include "closest_in_path_vehicle_decider.h"

#include <cmath>
#include <limits>

#include "debug_info_log.h"
#include "environmental_model_manager.h"
#include "log.h"
#include "st_graph/st_graph_helper.h"
#include "vehicle_config_context.h"

namespace planning {
namespace {
constexpr int32_t kInvalidId = -1;
constexpr double kDefaultTTC = 100.0;
constexpr double KSpeedDiffThr = 0.1;
constexpr double dangerous_level_ttc_thr_1 = 1.0;
constexpr double dangerous_level_ttc_thr_2 = 0.8;
constexpr double dangerous_level_ttc_thr_3 = 0.5;
constexpr double kLargeAgentLengthM = 8.0;
}  // namespace

ClosestInPathVehicleDecider::ClosestInPathVehicleDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  name_ = "ClosestInPathVehicleDecider";
}

bool ClosestInPathVehicleDecider::Execute() {
  Reset();
  auto res = CipvDecision();
  if (!res) {
    JSON_DEBUG_VALUE("cipv_id_st", -1.0)
    return false;
  }
  res = DetermineIfConeBucketCIPV();
  const auto cipv_decider_output =
      session_->planning_context().cipv_decider_output();
  DetermineCIPVInfoForHMI();
  JSON_DEBUG_VALUE("cipv_id_st", cipv_decider_output.cipv_id())
  JSON_DEBUG_VALUE("cipv_vel_fusion", cipv_decider_output.v_fusion_frenet());
  JSON_DEBUG_VALUE("cipv_acc", cipv_decider_output.acceleration())
  JSON_DEBUG_VALUE("cipv_acc_fusion", cipv_decider_output.acceleration_fusion())
  return true;
}

bool ClosestInPathVehicleDecider::CipvDecision() {
  const auto ptr_st_graph_helper =
      session_->planning_context().st_graph_helper();
  if (nullptr == ptr_st_graph_helper) {
    return false;
  }
  const auto &st_boundaries = ptr_st_graph_helper->GetAllStBoundaries();
  const auto agent_manager =
      session_->environmental_model().get_agent_manager();
  double min_s = std::numeric_limits<double>::max();
  int32_t id = kInvalidId;
  double releative_s = 0.0;
  double cipv_v_frenet = 0.0;
  double cipv_v_fusion_frenet = 0.0;
  double cipv_acc = 0.0;
  double cipv_acc_fusion = 0.0;
  double cipv_ttc = kDefaultTTC;
  int32_t dangerous_level = -1;
  bool is_virtual = false;
  bool is_large = false;
  auto &mutable_cipv_decider_output =
      session_->mutable_planning_context()->mutable_cipv_decider_output();
  if (st_boundaries.empty()) {
    Reset(&id, &releative_s, &cipv_v_frenet, &cipv_ttc, &dangerous_level,
          &is_virtual, &is_large);
    mutable_cipv_decider_output.set_cipv_id(id);
    mutable_cipv_decider_output.set_relative_s(releative_s);
    mutable_cipv_decider_output.set_v_frenet(cipv_v_frenet);
    mutable_cipv_decider_output.set_v_fusion_frenet(cipv_v_fusion_frenet);
    mutable_cipv_decider_output.set_acceleration(cipv_acc);
    mutable_cipv_decider_output.set_acceleration_fusion(cipv_acc_fusion);
    mutable_cipv_decider_output.set_ttc(cipv_ttc);
    mutable_cipv_decider_output.set_dangerous_level(dangerous_level);
    mutable_cipv_decider_output.set_is_virtual(is_virtual);
    mutable_cipv_decider_output.set_is_large(is_large);
  } else {
    for (const auto &item : st_boundaries) {
      const auto *ptr_st_boundary = item.second.get();
      if (nullptr == ptr_st_boundary) {
        continue;
      }
      speed::STPoint lower_point;
      speed::STPoint upper_point;
      if (!ptr_st_boundary->GetBoundaryBounds(0.0, &lower_point,
                                              &upper_point)) {
        continue;
      }
      double lower_s = lower_point.s();
      if (lower_s < 0.0) {
        continue;
      }
      if (lower_s < min_s) {
        min_s = lower_s;
        id = lower_point.agent_id();
      }
      agents_distance_id_map_[lower_s] = std::make_pair(
          agent_manager->GetAgent(lower_point.agent_id())->type() ==
              agent::AgentType::VIRTUAL,
          lower_point.agent_id());
      if (kInvalidId == id) {
        mutable_cipv_decider_output.Reset();
      } else {
        MakeCipvInfo(id, &releative_s, &cipv_v_frenet, &cipv_v_fusion_frenet,
                     &cipv_acc, &cipv_acc_fusion, &cipv_ttc, &dangerous_level,
                     &is_virtual, &is_large);
        mutable_cipv_decider_output.set_cipv_id(id);
        mutable_cipv_decider_output.set_relative_s(releative_s);
        mutable_cipv_decider_output.set_v_frenet(cipv_v_frenet);
        mutable_cipv_decider_output.set_v_fusion_frenet(cipv_v_fusion_frenet);
        mutable_cipv_decider_output.set_acceleration(cipv_acc);
        mutable_cipv_decider_output.set_acceleration_fusion(cipv_acc_fusion);
        mutable_cipv_decider_output.set_ttc(cipv_ttc);
        mutable_cipv_decider_output.set_dangerous_level(dangerous_level);
        mutable_cipv_decider_output.set_is_virtual(is_virtual);
        mutable_cipv_decider_output.set_is_large(is_large);
      }
    }
  }
  return true;
}

void ClosestInPathVehicleDecider::MakeCipvInfo(
    const int32_t cipv_id, double *const relative_s, double *const v_frenet,
    double *const v_fusion_frenet, double *acc, double *acc_fusion,
    double *const cipv_ttc, int32_t *const dangerous_level,
    bool *const is_virtual, bool *const is_large) {
  const auto agent_manager =
      session_->environmental_model().get_agent_manager();
  const auto &ego_vehi_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  if (nullptr == agent_manager) {
    return;
  }
  auto agent = agent_manager->GetAgent(cipv_id);
  if (nullptr == agent) {
    return;
  }
  const auto &planned_kd_path =
      session_->planning_context().motion_planner_output().lateral_path_coord;
  const auto ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const double ego_v =
      ego_state_manager->planning_init_point().lon_init_state.v();
  double min_s = std::numeric_limits<double>::max();
  for (const auto &point : agent->box().GetAllCorners()) {
    double s = 0.0;
    double l = 0.0;
    if (!planned_kd_path->XYToSL(point.x(), point.y(), &s, &l)) {
      ILOG_ERROR << "----planned_kd_path err---";
      return;
    }
    min_s = std::fmin(min_s, s);
  }
  // consider back bumper to center distance of ego
  constexpr double kEgoCenterToBackCenter = 1.1;
  // *relative_s = min_s - kEgoCenterToBackCenter;
  *relative_s = min_s - ego_vehi_param.front_edge_to_rear_axle;
  if (session_->is_rads_scene()) {
    *relative_s = min_s - ego_vehi_param.rear_edge_to_rear_axle;
  }

  double center_s = 0.0;
  double center_l = 0.0;
  const auto &center = agent->box().center();
  if (!planned_kd_path->XYToSL(center.x(), center.y(), &center_s, &center_l)) {
    ILOG_ERROR << "----planned_kd_path err---";
    return;
  }
  auto matched_point = planned_kd_path->GetPathPointByS(center_s);
  double heading_diff = agent->box().heading() - matched_point.theta();

  *v_frenet = agent->speed() * std::cos(heading_diff);
  *v_fusion_frenet = agent->speed_fusion() * std::cos(heading_diff);
  *acc = agent->accel();
  *acc_fusion = agent->accel_fusion();
  *is_large = agent->length() > kLargeAgentLengthM;
  if (ego_v - *v_frenet > KSpeedDiffThr) {
    *cipv_ttc = *relative_s / (ego_v - *v_frenet);
  } else {
    *cipv_ttc = kDefaultTTC;
  }

  if (*cipv_ttc <= dangerous_level_ttc_thr_3) {
    *dangerous_level = 3;
  } else if (*cipv_ttc > dangerous_level_ttc_thr_3 &&
             *cipv_ttc <= dangerous_level_ttc_thr_2) {
    *dangerous_level = 2;
  } else if (*cipv_ttc <= dangerous_level_ttc_thr_1 &&
             *cipv_ttc > dangerous_level_ttc_thr_2) {
    *dangerous_level = 1;
  } else {
    *dangerous_level = -1;
  }
  *is_virtual = agent->type() == agent::AgentType::VIRTUAL;
}

bool ClosestInPathVehicleDecider::DetermineIfConeBucketCIPV() {
  const auto dynamic_world =
      session_->environmental_model().get_dynamic_world();
  const auto agent_manager =
      session_->environmental_model().get_dynamic_world()->agent_manager();
  if (nullptr == dynamic_world || nullptr == agent_manager) {
    ILOG_DEBUG << "dynamic_world or agent_manager is nullptr";
    return true;
  }
  const auto *st_graph_helper = session_->planning_context().st_graph_helper();
  if (nullptr == st_graph_helper) {
    ILOG_DEBUG << "st_graph_helper is nullptr";
    return false;
  }
  const auto &all_agents = agent_manager->GetAllCurrentAgents();
  auto *mutable_helper = const_cast<speed::StGraphHelper *>(st_graph_helper);
  mutable_helper->DetermineIfConeBucketCIPV(all_agents);
  return true;
}

void ClosestInPathVehicleDecider::DetermineCIPVInfoForHMI() const {
  auto hmi_info =
      session_->mutable_planning_context()->mutable_planning_hmi_info();
  for (const auto &[agent_cur_distance_to_ego, isvirtual_agentid] :
       agents_distance_id_map_) {
    if (isvirtual_agentid.first) {  // if virtual, skip
      continue;
    } else {
      hmi_info->cipv_info.cipv_id = isvirtual_agentid.second;
      hmi_info->cipv_info.has_cipv = true;
      JSON_DEBUG_VALUE("cipv_id_hmi", hmi_info->cipv_info.cipv_id)
      return;
    }
  }
  hmi_info->cipv_info.cipv_id = -1;
  hmi_info->cipv_info.has_cipv = false;
  JSON_DEBUG_VALUE("cipv_id_hmi", hmi_info->cipv_info.cipv_id)
  return;
}

void ClosestInPathVehicleDecider::Reset(
    int32_t *const cipv_id, double *const relative_s, double *const v_frenet,
    double *const cipv_ttc, int32_t *const dangerous_level,
    bool *const is_virtual, bool *const is_large) {
  *cipv_id = kInvalidId;
  *relative_s = std::numeric_limits<double>::max();
  *v_frenet = std::numeric_limits<double>::max();
  *cipv_ttc = std::numeric_limits<double>::max();
  *dangerous_level = -1;
  *is_virtual = true;
  *is_large = false;
}

void ClosestInPathVehicleDecider::Reset() {
  auto &mutable_cipv_decider_output =
      session_->mutable_planning_context()->mutable_cipv_decider_output();
  mutable_cipv_decider_output.set_cipv_id(kInvalidId);
  mutable_cipv_decider_output.set_relative_s(
      std::numeric_limits<double>::max());
  mutable_cipv_decider_output.set_v_frenet(std::numeric_limits<double>::max());
  mutable_cipv_decider_output.set_v_fusion_frenet(0.0);
  mutable_cipv_decider_output.set_acceleration(0.0);
  mutable_cipv_decider_output.set_acceleration_fusion(0.0);
  mutable_cipv_decider_output.set_ttc(std::numeric_limits<double>::max());
  mutable_cipv_decider_output.set_dangerous_level(-1);
  mutable_cipv_decider_output.set_is_virtual(true);
  mutable_cipv_decider_output.set_is_large(false);
  agents_distance_id_map_.clear();
}
}  // namespace planning

#include "dynamic_agent_emergence_avoid_lane_change_request.h"

#include <glog/logging.h>
#include <math.h>

#include <cassert>
#include <cmath>
#include <complex>
#include <limits>

#include "common.pb.h"
#include "common_c.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "define/geometry.h"
#include "ego_state_manager.h"
#include "ifly_time.h"
#include "log.h"
#include "planning_context.h"
#include "tasks/behavior_planners/lane_change_decider/lane_change_requests/lane_change_request.h"
#include "tracked_object.h"
#include "virtual_lane_manager.h"

namespace planning {

namespace {
constexpr double kEmergencyAvoidanceLateralSafeDistanceThreshold = 0.2;
constexpr double kEmergencyAvoidanceHalfDistance = 0.65;
constexpr double kEmergencyAvoidancelongitudinalDistanceThreshold = 100.0;
constexpr int kInvalidAgentId = -1;
constexpr double kSplitTriggleDistance = 3000.0;
constexpr double kMaxVelocityTriggerEmergenceAvoidRequest = 80;
constexpr int kHighRiskEmergencySituationDurationCountThr = 3;
constexpr int kLowRiskEmergencySituationDurationCountThr = 6;
constexpr double kIntrusionLateralDistanceThr = 0.25;
constexpr double kNearLateralDistanceThr = -0.15;
constexpr double kNearLaneLateralDistanceThr = 0.1;
constexpr double kNearEgoLongDistanceBuffer = 1;

}  // namespace
// class: DynamicAgentEmergenceAvoidRequest
DynamicAgentEmergenceAvoidRequest::DynamicAgentEmergenceAvoidRequest(
    planning::framework::Session* session,
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
    std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr)
    : LaneChangeRequest(session, virtual_lane_mgr, lane_change_lane_mgr) {
  base_frenet_coord_ = std::make_shared<planning_math::KDPath>();
  emergency_lane_change_avoid_speed_ysteresis_.SetThreValue(
      kMaxVelocityTriggerEmergenceAvoidRequest + 5,
      kMaxVelocityTriggerEmergenceAvoidRequest - 5);
}

void DynamicAgentEmergenceAvoidRequest::Update(int lc_status) {
  ILOG_DEBUG << "DynamicAgentEmergenceAvoidRequest::Update::coming emergence "
                "avoid lane "
                "change request";
  lc_request_cancel_reason_ = IntCancelReasonType::NO_CANCEL;
  // trigger EA lane change when lane keep status.
  if (lc_status != kLaneKeeping && lc_status != kLaneChangePropose) {
    ILOG_DEBUG << "DynamicAgentEmergenceAvoidRequest::Update: ego not in lane "
                  "keeping!";
    return;
  }

  // intersection surpression
  // if (EgoInIntersection()) {
  //   // 路口抑制变道
  //   Reset();
  //   Finish();
  //   return;
  // }

  Init();

  UpdateDynamicAgentEmergencyAvoidanceSituation();

  if (!is_dynamic_agent_emergency_avoidance_situation_) {
    if (request_type_ != NO_CHANGE) {
      Finish();
      set_target_lane_virtual_id(origin_lane_virtual_id_);
      ILOG_DEBUG
          << "[DynamicAgentEmergenceAvoidRequest::update] finish request";
    }
    return;
  }

  GenerateLaneChangeDirection();
  JSON_DEBUG_VALUE("dynamic_agent_emergency_lane_change_direction",
                   static_cast<uint32_t>(lane_change_direction_));
  CheckLaneChangeDirection(lc_status);

  return;
}

void DynamicAgentEmergenceAvoidRequest::Init() {
  lateral_obstacle_ = session_->environmental_model().get_lateral_obstacle();
  lane_tracks_manager_ =
      session_->environmental_model().get_lane_tracks_manager();
  const int current_lane_virtual_id =
      virtual_lane_mgr_->current_lane_virtual_id();
  if (lane_change_lane_mgr_->has_origin_lane()) {
    auto origin_lane = lane_change_lane_mgr_->olane();
    origin_lane_virtual_id_ = origin_lane->get_virtual_id();
  } else {
    origin_lane_virtual_id_ = current_lane_virtual_id;
  }
  planning_init_point_ = session_->environmental_model()
                             .get_ego_state_manager()
                             ->planning_init_point();
  recommend_dynamic_agent_emergency_avoidance_direction_ = NO_CHANGE;
  risk_level_ = RiskLevel::NO_RISK;
}

void DynamicAgentEmergenceAvoidRequest::
    UpdateDynamicAgentEmergencyAvoidanceSituation() {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  emergency_lane_change_avoid_speed_ysteresis_.SetIsValidByValue(
      ego_state->ego_v() * 3.6);
  const bool enable_emergency_avoid =
      !emergency_lane_change_avoid_speed_ysteresis_.IsValid();
  if (!enable_emergency_avoid) {
      // 是否需要限制功能
    ILOG_DEBUG << "DynamicAgentEmergenceAvoidRequest:: not "
                  "enable_emergency_avoid becase of high speed";
    return;
  }
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto& rlane = virtual_lane_mgr_->get_right_lane();
  const auto& llane = virtual_lane_mgr_->get_left_lane();
  const int current_lane_virtual_id =
      virtual_lane_mgr_->current_lane_virtual_id();
  int base_lane_virtual_id = current_lane_virtual_id;
  auto base_lane =
      virtual_lane_mgr_->get_lane_with_virtual_id(base_lane_virtual_id);
  if (base_lane == nullptr) {
    ILOG_DEBUG << "DynamicAgentEmergenceAvoidRequest::base lane not exist";
    Reset();
    return;
  }
  std::shared_ptr<ReferencePath> origin_refline =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->get_reference_path_by_lane(base_lane_virtual_id, false);
  if (origin_refline == nullptr) {
    return;
  }
  base_frenet_coord_ = origin_refline->get_frenet_coord();
  if (base_frenet_coord_ == nullptr) {
    return;
  }
  Point2D ego_frenet_point;
  Point2D ego_cart_point{planning_init_point_.lat_init_state.x(),
                         planning_init_point_.lat_init_state.y()};
  if (!base_frenet_coord_->XYToSL(ego_cart_point, ego_frenet_point)) {
    ILOG_DEBUG
        << "DynamicAgentEmergenceAvoidRequest::fail to get ego position on base lane";
    Reset();
    return;
  }
  right_lane_nums_ = 0;
  left_lane_nums_ = 0;
  auto lane_nums_msg = base_lane->get_lane_nums();
  auto iter =
      std::find_if(lane_nums_msg.begin(), lane_nums_msg.end(),
                   [&ego_frenet_point](const iflyauto::LaneNumMsg& lane_num) {
                     return lane_num.begin <= ego_frenet_point.x &&
                            lane_num.end > ego_frenet_point.x;
                   });
  if (iter != lane_nums_msg.end()) {
    left_lane_nums_ = iter->left_lane_num;
    right_lane_nums_ = iter->right_lane_num;
  } else {
    left_lane_nums_ = llane ? 1 : 0;
    right_lane_nums_ = rlane ? 1 : 0;
  }

  // 是否存在需要紧急变道的动态障碍物
  bool is_exit_emergency_dynamic_agent = CheckEmergencyDynamicAgent();
  auto kEmergencySituationDurationCountThr = kLowRiskEmergencySituationDurationCountThr;
  if (risk_level_ == RiskLevel::HIGH_RISK) {
    kEmergencySituationDurationCountThr = kHighRiskEmergencySituationDurationCountThr;
  }
  if (is_exit_emergency_dynamic_agent) {
    // 存在危险障碍物
    dynamic_agent_emergency_situation_timetstamp_ =
        std::min(dynamic_agent_emergency_situation_timetstamp_ + 1,
                 kEmergencySituationDurationCountThr + 5);
  } else {
    dynamic_agent_emergency_situation_timetstamp_ =
        std::max(dynamic_agent_emergency_situation_timetstamp_ - 1, 0);
  }
  if (dynamic_agent_emergency_situation_timetstamp_ >=
      kEmergencySituationDurationCountThr) {
    is_dynamic_agent_emergency_avoidance_situation_ = true;
  } else {
    is_dynamic_agent_emergency_avoidance_situation_ = false;
  }
  ILOG_DEBUG << "DynamicAgentEmergenceAvoidRequest::Update: "
                "is_emergency_avoidance_situation "
             << is_dynamic_agent_emergency_avoidance_situation_;
  JSON_DEBUG_VALUE("is_dynamic_agent_emergency_avoidance_situation_",
                   is_dynamic_agent_emergency_avoidance_situation_);
  JSON_DEBUG_VALUE("recommend_dynamic_agent_emergency_avoidance_direction",
                   static_cast<uint32_t>(
                       recommend_dynamic_agent_emergency_avoidance_direction_));
  JSON_DEBUG_VALUE("risk_level", static_cast<uint32_t>(risk_level_));
  JSON_DEBUG_VALUE("dynamic_agent_emergency_situation_timetstamp",
                   dynamic_agent_emergency_situation_timetstamp_);
}

bool DynamicAgentEmergenceAvoidRequest::CheckEmergencyDynamicAgent() {
  bool is_emergency_base_side_dynamic_agent = CheckEmergencyDynamicSideAgentBaseRisk();
  bool is_emergency_base_last_emergency_avoid = CheckEmergencyBaseLastEmergencyAvoid();
  return is_emergency_base_side_dynamic_agent || is_emergency_base_last_emergency_avoid;
}

bool DynamicAgentEmergenceAvoidRequest::CheckEmergencyDynamicSideAgentBaseRisk() {
  const int current_lane_virtual_id =
      virtual_lane_mgr_->current_lane_virtual_id();
  std::shared_ptr<ReferencePath> origin_refline =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->get_reference_path_by_lane(current_lane_virtual_id, false);
  if (origin_refline == nullptr) {
    return false;
  }
  const auto& current_lane =
      virtual_lane_mgr_->get_lane_with_virtual_id(current_lane_virtual_id);
  if (current_lane == nullptr) {
    return false;
  }
  const auto& potential_dangerous_agent_decider_output =
      session_->planning_context().potential_dangerous_agent_decider_output();
  const auto& obstacles_map = origin_refline->get_obstacles_map();
  const auto ego_frenet_boundary = origin_refline->get_ego_frenet_boundary();
  const auto half_lane_width = current_lane->width() * 0.5;

  if (!potential_dangerous_agent_decider_output.dangerous_agent_info.empty() &&
      potential_dangerous_agent_decider_output.dangerous_agent_info.front()
              .recommended_maneuver.lateral_maneuver !=
          LateralManeuver ::IGNORE) {
    const auto obstacle_id =
        potential_dangerous_agent_decider_output.dangerous_agent_info.front()
            .id;
    auto obstacle_iter = obstacles_map.find(obstacle_id);
    if (obstacle_iter == obstacles_map.end()) {
      return false;
    }
    // 是否在侧方,过滤非侧方障碍物,障碍物一定要超过自车尾部1米
    bool is_overlap_side =
        std::max(obstacle_iter->second->frenet_obstacle_boundary().s_start,
                 ego_frenet_boundary.s_start + kNearEgoLongDistanceBuffer) <
        std::min(obstacle_iter->second->frenet_obstacle_boundary().s_end,
                 ego_frenet_boundary.s_end);
    if (!is_overlap_side || obstacle_iter->second->is_static()) {
      return false;
    }
    risk_level_ =
        potential_dangerous_agent_decider_output.dangerous_agent_info.front()
            .risk_level;
    if (risk_level_ == RiskLevel::LOW_RISK ||
        risk_level_ == RiskLevel::NO_RISK) {
      // 判断自车的位置是否已经进行了大幅度避让，离车道边界线较近
      // 可能会存在一种可能，自车避让压线时，会切换车道，为了避免频繁这种情况，
      // 此时判断切换车道的时间需要久一点，以及考虑障碍物侵入车道的距离
      double v_lat = obstacle_iter->second->frenet_velocity_lateral();
      double intrusion_distance = 0;
      bool is_near_lane_boundary = false;  // 自车是否在车道边界附近
      bool is_in_lat_range = false;        // 障碍物是否侵入自车道
      double extra_lat_buffer_with_no_risk = 0.2;  // 没有风险等级的障碍物，条件更加严格
      if (risk_level_ == RiskLevel::LOW_RISK) {
        extra_lat_buffer_with_no_risk = 0;
      }
      if (obstacle_iter->second->d_max_cpath() < 0) {
        intrusion_distance =
            half_lane_width - std::fabs(obstacle_iter->second->d_max_cpath());
      } else if (obstacle_iter->second->d_min_cpath() > 0) {
        intrusion_distance =
            half_lane_width - obstacle_iter->second->d_min_cpath();
      }
      // 需要考虑障碍物的横向速度
      if (v_lat < -0.3) {
        // 障碍物具有明显靠近自车的横向速度
        is_in_lat_range =
            intrusion_distance >
            kNearLateralDistanceThr + extra_lat_buffer_with_no_risk;
      } else {
        is_in_lat_range =
            intrusion_distance >
            kIntrusionLateralDistanceThr + extra_lat_buffer_with_no_risk;
      }
      is_near_lane_boundary = half_lane_width - ego_frenet_boundary.l_end <
                                  kNearLaneLateralDistanceThr ||
                              half_lane_width + ego_frenet_boundary.l_start <
                                  kNearLaneLateralDistanceThr;
      if (is_near_lane_boundary && is_in_lat_range) {
        if (obstacle_iter->second->frenet_l() >=
            origin_refline->get_frenet_ego_state().l()) {
          recommend_dynamic_agent_emergency_avoidance_direction_ = RIGHT_CHANGE;
        } else {
          recommend_dynamic_agent_emergency_avoidance_direction_ = LEFT_CHANGE;
        }
        return true;
      } else {
        return false;
      }
    } else if (risk_level_ == RiskLevel::HIGH_RISK) {
      // 侧方高风险等级，直接判断为需要变道避让
      if (obstacle_iter->second->frenet_l() >=
          origin_refline->get_frenet_ego_state().l()) {
        recommend_dynamic_agent_emergency_avoidance_direction_ = RIGHT_CHANGE;
      } else {
        recommend_dynamic_agent_emergency_avoidance_direction_ = LEFT_CHANGE;
      }
      return true;
    }
  } else {
    for (const auto& [obs_id, obs_ptr] : obstacles_map) {
      if (!obs_ptr) {
        continue;
      }
      const auto& frenet_obs = *obs_ptr;
      bool is_overlap_side =
          std::max(frenet_obs.frenet_obstacle_boundary().s_start,
                  ego_frenet_boundary.s_start + kNearEgoLongDistanceBuffer) <
          std::min(frenet_obs.frenet_obstacle_boundary().s_end,
                  ego_frenet_boundary.s_end);
      if (!is_overlap_side || frenet_obs.is_static()) {
        continue;
      }
      double v_lat = frenet_obs.frenet_velocity_lateral();
      double intrusion_distance = 0;
      bool is_near_lane_boundary = false;  // 自车是否在车道边界附近
      bool is_in_lat_range = false;        // 障碍物是否侵入自车道
      double extra_lat_buffer_with_no_risk = 0.2;  // 没有风险等级的障碍物，条件更加严格
      if (frenet_obs.d_max_cpath() < 0) {
        intrusion_distance =
            half_lane_width - std::fabs(frenet_obs.d_max_cpath());
      } else if (frenet_obs.d_min_cpath() > 0) {
        intrusion_distance =
            half_lane_width - frenet_obs.d_min_cpath();
      }
      // 需要考虑障碍物的横向速度
      if (v_lat < -0.3) {
        // 障碍物具有明显靠近自车的横向速度
        is_in_lat_range =
            intrusion_distance >
            kNearLateralDistanceThr + extra_lat_buffer_with_no_risk;
      } else {
        is_in_lat_range =
            intrusion_distance >
            kIntrusionLateralDistanceThr + extra_lat_buffer_with_no_risk;
      }
      is_near_lane_boundary = half_lane_width - ego_frenet_boundary.l_end <
                                  kNearLaneLateralDistanceThr ||
                              half_lane_width + ego_frenet_boundary.l_start <
                                  kNearLaneLateralDistanceThr;
      if (is_near_lane_boundary && is_in_lat_range) {
        if (frenet_obs.frenet_l() >=
            origin_refline->get_frenet_ego_state().l()) {
          recommend_dynamic_agent_emergency_avoidance_direction_ = RIGHT_CHANGE;
        } else {
          recommend_dynamic_agent_emergency_avoidance_direction_ = LEFT_CHANGE;
        }
        return true;
      } else {
        continue;
      }
    }
  }
  return false;
}

bool DynamicAgentEmergenceAvoidRequest::CheckEmergencyBaseLastEmergencyAvoid() {
  if (recommend_dynamic_agent_emergency_avoidance_direction_ != NO_CHANGE) {
    return false;
  }
  const auto &lat_obstacle_position = session_->planning_context()
                                          .lateral_obstacle_decider_output()
                                          .lateral_obstacle_history_info;
  const int current_lane_virtual_id =
      virtual_lane_mgr_->current_lane_virtual_id();
  std::shared_ptr<ReferencePath> origin_refline =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->get_reference_path_by_lane(current_lane_virtual_id, false);
  if (origin_refline == nullptr) {
    return false;
  }
  const auto& current_lane =
      virtual_lane_mgr_->get_lane_with_virtual_id(current_lane_virtual_id);
  if (current_lane == nullptr) {
    return false;
  }
  const auto& potential_dangerous_agent_decider_output =
      session_->planning_context().potential_dangerous_agent_decider_output();
  const auto& obstacles_map = origin_refline->get_obstacles_map();
  const auto ego_frenet_boundary = origin_refline->get_ego_frenet_boundary();
  const auto half_lane_width = current_lane->width() * 0.5;
  // 接受上一帧道内紧急避让释放的id
  int emergency_avoid_num = 0;
  for (const auto& [obs_id, obs_ptr] : obstacles_map) {
    if (!obs_ptr) {
      continue;
    }
    const auto& frenet_obs = *obs_ptr;
    if (frenet_obs.is_static()) {
      continue;
    }
    if (lat_obstacle_position.find(frenet_obs.id()) ==
        lat_obstacle_position.end()) {
      continue;
    }
    if (!lat_obstacle_position.find(frenet_obs.id())->second.emergency_avoid) {
      continue;
    }
    emergency_avoid_num += 1;
    if (emergency_avoid_num >= 2) {
      recommend_dynamic_agent_emergency_avoidance_direction_ = NO_CHANGE;
      break;
    }
    double v_lat = frenet_obs.frenet_velocity_lateral();
    double intrusion_distance = 0;
    double extra_lat_buffer_with_emergency_avoid = -0.1;  // 没有风险等级的障碍物，条件更加严格
    bool is_in_lat_range = false;        // 障碍物是否侵入自车道
    if (frenet_obs.d_max_cpath() < 0) {
      intrusion_distance =
          half_lane_width - std::fabs(frenet_obs.d_max_cpath());
    } else if (frenet_obs.d_min_cpath() > 0) {
      intrusion_distance =
          half_lane_width - frenet_obs.d_min_cpath();
    }
    // 需要考虑障碍物的横向速度
    if (v_lat < -0.5) {
      // 障碍物具有明显靠近自车的横向速度
      is_in_lat_range =
          intrusion_distance >
          kNearLateralDistanceThr + extra_lat_buffer_with_emergency_avoid;
    } else {
      is_in_lat_range =
          intrusion_distance >
          kIntrusionLateralDistanceThr + extra_lat_buffer_with_emergency_avoid;
    }
    if (is_in_lat_range) {
      if (frenet_obs.frenet_l() >=
          origin_refline->get_frenet_ego_state().l()) {
        recommend_dynamic_agent_emergency_avoidance_direction_ = RIGHT_CHANGE;
      } else {
        recommend_dynamic_agent_emergency_avoidance_direction_ = LEFT_CHANGE;
      }
    } else {
      continue;
    }
  }
  if (emergency_avoid_num == 1 &&
      recommend_dynamic_agent_emergency_avoidance_direction_ != NO_CHANGE) {
    return true;
  }
  return false;
}

void DynamicAgentEmergenceAvoidRequest::GenerateLaneChangeDirection() {
  const auto& route_info_output =
      session_->environmental_model().get_route_info()->get_route_info_output();
  std::shared_ptr<ReferencePathManager> reference_path_mgr =
      session_->mutable_environmental_model()->get_reference_path_manager();
  const auto& rlane = virtual_lane_mgr_->get_right_lane();
  const auto& llane = virtual_lane_mgr_->get_left_lane();
  const auto& function_info = session_->environmental_model().function_info();

  if (llane != nullptr) {
    left_reference_path_ = reference_path_mgr->get_reference_path_by_lane(
        llane->get_virtual_id(), false);
    ILOG_DEBUG << "DynamicAgentEmergenceAvoidRequest::Update: for left_lane: update "
               << llane->get_virtual_id();
  } else {
    left_reference_path_ = nullptr;
  }

  if (rlane != nullptr) {
    right_reference_path_ = reference_path_mgr->get_reference_path_by_lane(
        rlane->get_virtual_id(), false);
    ILOG_DEBUG << "DynamicAgentEmergenceAvoidRequest::Update: for right_lane: update "
               << rlane->get_virtual_id();
  } else {
    right_reference_path_ = nullptr;
  }

  // 过滤对向车道
  bool enable_left = llane && left_reference_path_ &&
                     llane->get_lane_type() != iflyauto::LANETYPE_OPPOSITE;
  bool enable_right = rlane && right_reference_path_ &&
                      rlane->get_lane_type() != iflyauto::LANETYPE_OPPOSITE;

  const bool is_left_lane_change_safe =
      enable_left &&
      !IsRoadBorderSurpressDuringLaneChange(
          LEFT_CHANGE, origin_lane_virtual_id_, llane->get_virtual_id());
  // (enable_left && ComputeLcValid(LEFT_CHANGE));
  const bool is_right_lane_change_safe =
      enable_right &&
      !IsRoadBorderSurpressDuringLaneChange(
          RIGHT_CHANGE, origin_lane_virtual_id_, rlane->get_virtual_id());

  const auto& merge_point_info = route_info_output.merge_point_info;
  double distance_to_first_road_split = NL_NMAX;
  double dis_to_first_merge = NL_NMAX;
  double dis_to_merge_point = NL_NMAX;
  if (function_info.function_mode() == common::DrivingFunctionInfo::NOA) {
    dis_to_merge_point = merge_point_info.dis_to_merge_fp;
    const auto& split_region_info_list =
        route_info_output.split_region_info_list;
    const auto& merge_region_info_list =
        route_info_output.merge_region_info_list;
    if (!split_region_info_list.empty()) {
      if (split_region_info_list[0].is_valid) {
        distance_to_first_road_split =
            split_region_info_list[0].distance_to_split_point;
      }
    }
    if (!merge_region_info_list.empty()) {
      if (merge_region_info_list[0].is_valid) {
        dis_to_first_merge = merge_region_info_list[0].distance_to_split_point;
      }
    }
  }

  const auto& feasible_lane_sequence =
      route_info_output.mlc_decider_route_info.feasible_lane_sequence;
  bool left_lane_is_on_navigation_route = true;
  bool right_lane_is_on_navigation_route = true;
  if (distance_to_first_road_split < 500.0 || dis_to_first_merge < 500.0 ||
      dis_to_merge_point < 200.0) {
    if (feasible_lane_sequence.size() > 0) {
      int current_lane_order_num = left_lane_nums_ + 1;
      int target_lane_order_num = current_lane_order_num - 1;
      if (std::find(feasible_lane_sequence.begin(),
                    feasible_lane_sequence.end(),
                    target_lane_order_num) == feasible_lane_sequence.end()) {
        left_lane_is_on_navigation_route = false;
      }
    }

    if (feasible_lane_sequence.size() > 0) {
      int current_lane_order_num = left_lane_nums_ + 1;
      int target_lane_order_num = current_lane_order_num + 1;
      if (std::find(feasible_lane_sequence.begin(),
                    feasible_lane_sequence.end(),
                    target_lane_order_num) == feasible_lane_sequence.end()) {
        right_lane_is_on_navigation_route = false;
      }
    }
  }

  lane_change_direction_ = NO_CHANGE;
  if (is_left_lane_change_safe && left_lane_is_on_navigation_route &&
      recommend_dynamic_agent_emergency_avoidance_direction_ == LEFT_CHANGE) {
    ILOG_DEBUG << "emergency avoidence alc left!!!";
    lane_change_direction_ = LEFT_CHANGE;
    return;
  } else if (is_right_lane_change_safe && right_lane_is_on_navigation_route &&
             recommend_dynamic_agent_emergency_avoidance_direction_ ==
                 RIGHT_CHANGE) {
    ILOG_DEBUG << "emergency avoidence alc left!!!";
    lane_change_direction_ = RIGHT_CHANGE;
    return;
  } else {
    // default
    lane_change_direction_ = NO_CHANGE;
    return;
  }
}

void DynamicAgentEmergenceAvoidRequest::CheckLaneChangeDirection(int lc_status) {
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  int olane_virtual_id = lane_change_decider_output.origin_lane_virtual_id;
  auto olane = virtual_lane_mgr_->get_lane_with_virtual_id(olane_virtual_id);
  int target_lane_virtual_id_tmp{origin_lane_virtual_id_};
  if (lane_change_direction_ == LEFT_CHANGE) {
    // 获取左车道线型
    iflyauto::LaneBoundaryType left_boundary_type =
        MakesureCurrentBoundaryType(LEFT_CHANGE, origin_lane_virtual_id_);
    if (request_type_ != LEFT_CHANGE) {
      target_lane_virtual_id_tmp = origin_lane_virtual_id_ - 1;
      GenerateRequest(LEFT_CHANGE);
      set_target_lane_virtual_id(target_lane_virtual_id_tmp);
      ILOG_DEBUG << "[DynamicAgentEmergenceAvoidRequest::update] Ask for emergency "
                    "avoidence changing lane to left";
    }
    if (request_type_ != NO_CHANGE &&
        (lc_status == kLaneChangeCancel &&
         (lane_change_lane_mgr_->has_origin_lane() &&
          lane_change_lane_mgr_->is_ego_on(olane)))) {
      Finish();
      set_target_lane_virtual_id(target_lane_virtual_id_tmp);
      ILOG_DEBUG << "[DynamicAgentEmergenceAvoidRequest::update] " << __FUNCTION__ << ":"
                 << __LINE__ << " finish request, dash not enough";
    }
    if (trigger_lane_change_cancel_) {
      lc_request_cancel_reason_ = IntCancelReasonType::MANUAL_CANCEL;
      Finish();
      Reset();
      set_target_lane_virtual_id(target_lane_virtual_id_tmp);
    }
  } else if (lane_change_direction_ == RIGHT_CHANGE) {
    // 获取右车道线型
    iflyauto::LaneBoundaryType right_boundary_type =
        MakesureCurrentBoundaryType(RIGHT_CHANGE, origin_lane_virtual_id_);
    if (request_type_ != RIGHT_CHANGE) {
      target_lane_virtual_id_tmp = origin_lane_virtual_id_ + 1;
      GenerateRequest(RIGHT_CHANGE);
      set_target_lane_virtual_id(target_lane_virtual_id_tmp);
      ILOG_DEBUG << "[DynamicAgentEmergenceAvoidRequest::update] Ask for emergency "
                    "avoidence changing lane to right";
    }
    if (request_type_ != NO_CHANGE &&
        (lc_status == kLaneChangeCancel &&
         (lane_change_lane_mgr_->has_origin_lane() &&
          lane_change_lane_mgr_->is_ego_on(olane)))) {
      Finish();
      set_target_lane_virtual_id(target_lane_virtual_id_tmp);
      ILOG_DEBUG << "[DynamicAgentEmergenceAvoidRequest::update] " << __FUNCTION__ << ":"
                 << __LINE__ << " finish request, dash not enough";
    }
    if (trigger_lane_change_cancel_) {
      lc_request_cancel_reason_ = IntCancelReasonType::MANUAL_CANCEL;
      Finish();
      Reset();
      set_target_lane_virtual_id(target_lane_virtual_id_tmp);
    }
  } else {
    if (request_type_ != NO_CHANGE &&
        (lane_change_lane_mgr_->has_origin_lane() &&
         lane_change_lane_mgr_->is_ego_on(olane))) {
      Finish();
      set_target_lane_virtual_id(target_lane_virtual_id_tmp);
      ILOG_DEBUG << "[DynamicAgentEmergenceAvoidRequest::update] " << __FUNCTION__ << ":"
                 << __LINE__
                 << " !trigger_left_overtake and !trigger_right_overtake";
    }
  }
}

void DynamicAgentEmergenceAvoidRequest::Reset() {
  dynamic_agent_emergency_situation_timetstamp_ = 0;
  is_dynamic_agent_emergency_avoidance_situation_ = false;
  lane_change_direction_ = NO_CHANGE;
  right_lane_nums_ = 0;
  left_lane_nums_ = 0;
}

}  // namespace planning
#include "emergence_avoid_lane_change_request.h"

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
#include "tasks/behavior_planners/lane_change_decider/lateral_behavior_object_selector.h"
#include "tracked_object.h"
#include "virtual_lane_manager.h"

namespace planning {

namespace {
constexpr double kEmergencyAvoidanceLateralSafeDistanceThreshold = 0.33;
constexpr double kEmergencyAvoidancelongitudinalDistanceThreshold = 100.0;
constexpr int kInvalidAgentId = -1;
constexpr double kEmergencySituationDuration = 0.4;
constexpr double kSplitTriggleDistance = 3000.0;
}  // namespace
// class: EmergenceAvoidRequest
EmergenceAvoidRequest::EmergenceAvoidRequest(
    planning::framework::Session* session,
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
    std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr)
    : LaneChangeRequest(session, virtual_lane_mgr, lane_change_lane_mgr) {
  base_frenet_coord_ = std::make_shared<KDPath>();
}

void EmergenceAvoidRequest::Update(int lc_status) {
  std::cout << "EmergenceAvoidRequest::Update::coming emergence avoid lane "
               "change request"
            << std::endl;

  // trigger EA lane change when lane keep status.
  if (lc_status != kLaneKeeping && lc_status != kLaneChangePropose) {
    LOG_DEBUG("EmergenceAvoidRequest::Update: ego not in lane keeping!");
    return;
  }
  lateral_obstacle_ = session_->environmental_model().get_lateral_obstacle();
  lane_tracks_manager_ =
      session_->environmental_model().get_lane_tracks_manager();
  const auto& tracks = lateral_obstacle_->all_tracks();
  tracks_map_.clear();
  for (auto track : tracks) {
    tracks_map_[track.track_id] = track;
  }

  const int current_lane_virtual_id =
      virtual_lane_mgr_->current_lane_virtual_id();

  if (lane_change_lane_mgr_->has_origin_lane()) {
    auto origin_lane = lane_change_lane_mgr_->olane();
    origin_lane_virtual_id_ = origin_lane->get_virtual_id();
  } else {
    origin_lane_virtual_id_ = current_lane_virtual_id;
  }
  int target_lane_virtual_id_tmp{origin_lane_virtual_id_};
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  planning_init_point_ = ego_state->planning_init_point();
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  int fix_lane_virtual_id = lane_change_decider_output.fix_lane_virtual_id;
  int olane_virtual_id = lane_change_decider_output.origin_lane_virtual_id;
  int target_lane_virtual_id =
      lane_change_decider_output.target_lane_virtual_id;
  std::shared_ptr<ReferencePathManager> reference_path_mgr =
      session_->mutable_environmental_model()->get_reference_path_manager();
  auto flane = virtual_lane_mgr_->get_lane_with_virtual_id(fix_lane_virtual_id);
  auto olane = virtual_lane_mgr_->get_lane_with_virtual_id(olane_virtual_id);
  auto tlane =
      virtual_lane_mgr_->get_lane_with_virtual_id(target_lane_virtual_id);
  const auto& clane = virtual_lane_mgr_->get_current_lane();
  const auto& llane = virtual_lane_mgr_->get_left_lane();
  const auto& rlane = virtual_lane_mgr_->get_right_lane();
  const auto& route_info_output = session_->
      environmental_model().get_route_info()->get_route_info_output();

  UpdateEmergencyAvoidanceSituation(lc_status);
  LOG_DEBUG(
      "EmergenceAvoidRequest::Update: is_emergency_avoidance_situation_ %d",
      is_emergency_avoidance_situation_);
  JSON_DEBUG_VALUE("is_emergency_avoidance_situation_",
                   is_emergency_avoidance_situation_);

  if (!is_emergency_avoidance_situation_) {
    if (request_type_ != NO_CHANGE) {
      Finish();
      set_target_lane_virtual_id(target_lane_virtual_id_tmp);
      LOG_DEBUG("[EmergenceAvoidRequest::update] finish request\n");
    }
    return;
  }

  if (llane != nullptr) {
    left_reference_path_ = reference_path_mgr->get_reference_path_by_lane(
        llane->get_virtual_id(), false);
    LOG_DEBUG("EmergenceAvoidRequest::Update: for left_lane: update %d\n",
              llane->get_virtual_id());
  } else {
    left_reference_path_ = nullptr;
  }

  if (rlane != nullptr) {
    right_reference_path_ = reference_path_mgr->get_reference_path_by_lane(
        rlane->get_virtual_id(), false);
    LOG_DEBUG("EmergenceAvoidRequest::Update: for right_lane: update %d\n",
              rlane->get_virtual_id());
  } else {
    right_reference_path_ = nullptr;
  }
  bool enable_left = llane && left_reference_path_;
  bool enable_right = rlane && right_reference_path_;
  const bool is_left_lane_change_safe =
      (enable_left && compute_lc_valid_info(LEFT_CHANGE));
  const bool is_right_lane_change_safe =
      (enable_right && compute_lc_valid_info(RIGHT_CHANGE));
  const bool emergency_avoidance_valid =
      (is_left_lane_change_safe || is_right_lane_change_safe);
  bool lane_change_to_left = true;
  bool curr_direct_exist = (clane->get_lane_marks() ==
                            iflyauto::LaneDrivableDirection_DIRECTION_STRAIGHT);

  if (is_left_lane_change_safe && is_right_lane_change_safe) {
    bool ramp_on_Right = false;
    bool is_on_highway = route_info_output.is_ego_on_expressway;
    if (is_on_highway) {
      ramp_on_Right =
          route_info_output.ramp_direction == RampDirection::RAMP_ON_RIGHT
              ? true
              : false;
      const double distance_to_next_ramp = route_info_output.dis_to_ramp;
      if (distance_to_next_ramp < kSplitTriggleDistance) {
        lane_change_to_left = ramp_on_Right ? false : true;
      } else {
        lane_change_to_left = true;
      }
    }
  } else if (is_left_lane_change_safe) {
    lane_change_to_left = true;
  } else if (is_right_lane_change_safe) {
    lane_change_to_left = false;
  }

  if (emergency_avoidance_valid) {
    if (lane_change_to_left) {
      // 获取左车道线型
      iflyauto::LaneBoundaryType left_boundary_type =
          MakesureCurrentBoundaryType(LEFT_CHANGE, origin_lane_virtual_id_);
      if (request_type_ != LEFT_CHANGE) {
        target_lane_virtual_id_tmp = origin_lane_virtual_id_ - 1;
        GenerateRequest(LEFT_CHANGE);
        set_target_lane_virtual_id(target_lane_virtual_id_tmp);
        LOG_DEBUG(
            "[EmergenceAvoidRequest::update] Ask for emergency avoidence "
            "changing lane to left "
            "\n");
      }
      if (request_type_ != NO_CHANGE &&
          (lc_status == kLaneKeeping || lc_status == kLaneChangePropose ||
           (lc_status == kLaneChangeCancel &&
            (lane_change_lane_mgr_->has_origin_lane() &&
             lane_change_lane_mgr_->is_ego_on(olane))))) {
        Finish();
        set_target_lane_virtual_id(target_lane_virtual_id_tmp);
        LOG_DEBUG(
            "[EmergenceAvoidRequest::update] %s:%d finish request, dash not "
            "enough \n",
            __FUNCTION__, __LINE__);
      }
    } else {
      // 获取右车道线型
      iflyauto::LaneBoundaryType right_boundary_type =
          MakesureCurrentBoundaryType(RIGHT_CHANGE, origin_lane_virtual_id_);
      if (request_type_ != RIGHT_CHANGE) {
        target_lane_virtual_id_tmp = origin_lane_virtual_id_ + 1;
        GenerateRequest(RIGHT_CHANGE);
        set_target_lane_virtual_id(target_lane_virtual_id_tmp);
        LOG_DEBUG(
            "[EmergenceAvoidRequest::update] Ask for emergency avoidence "
            "changing lane to right "
            "\n");
      }
      if (request_type_ != NO_CHANGE &&
          (lc_status == kLaneKeeping || lc_status == kLaneChangePropose ||
           (lc_status == kLaneChangeCancel &&
            (lane_change_lane_mgr_->has_origin_lane() &&
             lane_change_lane_mgr_->is_ego_on(olane))))) {
        Finish();
        set_target_lane_virtual_id(target_lane_virtual_id_tmp);
        LOG_DEBUG(
            "[EmergenceAvoidRequest::update] %s:%d finish request, dash not "
            "enough \n",
            __FUNCTION__, __LINE__);
      }
    }
  } else if (request_type_ != NO_CHANGE &&
             (lane_change_lane_mgr_->has_origin_lane() &&
              lane_change_lane_mgr_->is_ego_on(olane))) {
    Finish();
    set_target_lane_virtual_id(target_lane_virtual_id_tmp);
    LOG_DEBUG(
        "[EmergenceAvoidRequest::update] %s:%d finish request, "
        "!trigger_left_overtake and !trigger_right_overtake\n",
        __FUNCTION__, __LINE__);
  } else {
    return;
  }
}

void EmergenceAvoidRequest::UpdateEmergencyAvoidanceSituation(int lc_status) {
  const int current_lane_virtual_id =
      virtual_lane_mgr_->current_lane_virtual_id();
  int base_lane_virtual_id{current_lane_virtual_id};

  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  auto base_lane =
      virtual_lane_mgr_->get_lane_with_virtual_id(base_lane_virtual_id);
  if (base_lane == nullptr) {
    LOG_DEBUG("EmergenceAvoidRequest::base lane not exist");
    Reset();
    return;
  }

  // const double default_velocity_trigger_emergence_avoid_request = 13.88;

  // if (ego_state->ego_v() < default_velocity_trigger_emergence_avoid_request)
  // {
  //   LOG_DEBUG("EmergenceAvoidRequest::ego_v < 50km/h");
  //   return;
  // }

  std::shared_ptr<ReferencePath> origin_refline =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->get_reference_path_by_lane(base_lane_virtual_id, false);

  base_frenet_coord_ = origin_refline->get_frenet_coord();
  Point2D ego_frenet_point;
  Point2D ego_cart_point{planning_init_point_.lat_init_state.x(),
                         planning_init_point_.lat_init_state.y()};
  if (!base_frenet_coord_->XYToSL(ego_cart_point, ego_frenet_point)) {
    LOG_DEBUG("EmergenceAvoidRequest::fail to get ego position on base lane");
    Reset();
    return;
  }
  const double ego_left_edge = vehicle_param.max_width * 0.5;
  const double ego_right_edge = vehicle_param.max_width * 0.5;
  const double lateral_left_offset =
      ego_left_edge + kEmergencyAvoidanceLateralSafeDistanceThreshold;
  const double lateral_right_offset =
      ego_right_edge + kEmergencyAvoidanceLateralSafeDistanceThreshold;
  bool has_emergency_leading_vehicle = false;
  int leading_vehicle_id_ = -1;
  double leading_vehicle_speed = std::numeric_limits<double>::max();
  double long_gap = std::numeric_limits<double>::max();
  const std::vector<TrackedObject>& front_obstacles_array =
      lateral_obstacle_->front_tracks();
  for (const auto& front_obstacle : front_obstacles_array) {
    auto front_vehicle_iter = tracks_map_.find(front_obstacle.track_id);
    if (front_vehicle_iter != tracks_map_.end()) {
      if (front_vehicle_iter->second.track_id == kInvalidAgentId) {
        continue;
      }
      // bool object_type_static =
      //     front_vehicle_iter->second.motion_pattern_current ==
      //     iflyauto::ObjectMotionType::OBJECT_MOTION_TYPE_STATIC;
      // if ((!object_type_static ||
      //      (front_vehicle_iter->second.type !=
      //           Common::ObjectType::OBJECT_TYPE_COUPE &&
      //       front_vehicle_iter->second.type !=
      //           Common::ObjectType::OBJECT_TYPE_TRUCK) ||
      //       function_info.function_mode() ==
      //       common::DrivingFunctionInfo::NOA) &&
      // 对静止车暂时不做处理
      if (front_vehicle_iter->second.type !=
              Common::ObjectType::OBJECT_TYPE_TRAFFIC_CONE &&
          front_vehicle_iter->second.type !=
              Common::ObjectType::OBJECT_TYPE_WATER_SAFETY_BARRIER &&
          front_vehicle_iter->second.type !=
              Common::ObjectType::OBJECT_TYPE_CRASH_BARREL &&
          front_vehicle_iter->second.type !=
              Common::ObjectType::OBJECT_TYPE_TRAFFIC_TEM_SIGN) {
        continue;
      }
      const double long_dis = front_vehicle_iter->second.d_rel;
      if (long_dis > kEmergencyAvoidancelongitudinalDistanceThreshold) {
        continue;
      }
      const double half_width = front_vehicle_iter->second.width * 0.5;
      const double front_track_left_boundary_l =
          front_vehicle_iter->second.l + half_width;
      const double front_track_right_boundary_l =
          front_vehicle_iter->second.l - half_width;
      const bool is_out_target_area =
          (front_track_right_boundary_l > lateral_left_offset ||
           front_track_left_boundary_l < -lateral_right_offset);
      if (!is_out_target_area) {
        has_emergency_leading_vehicle = true;
        leading_vehicle_id_ = front_vehicle_iter->first;
        leading_vehicle_speed = front_vehicle_iter->second.v;
        long_gap = long_dis;
        break;
      }
    }
  }

  if (has_emergency_leading_vehicle) {
    double current_timestamp = IflyTime::Now_s();
    if (emergency_situation_timetstamp_ == std::numeric_limits<double>::max()) {
      emergency_situation_timetstamp_ = current_timestamp;
    }
    if (current_timestamp - emergency_situation_timetstamp_ >
        kEmergencySituationDuration) {
      is_emergency_avoidance_situation_ = true;
      LOG_DEBUG(
          "leading_vehicle_speed: %f, long_gap: %f "
          "Front Leading track_id: %d \n",
          leading_vehicle_speed, long_gap, leading_vehicle_id_);
      JSON_DEBUG_VALUE("leading_vehicle_id_", leading_vehicle_id_);
    }
  } else {
    emergency_situation_timetstamp_ = std::numeric_limits<double>::max();
  }
}

void EmergenceAvoidRequest::Reset() {
  emergency_situation_timetstamp_ = std::numeric_limits<double>::max();
  leading_vehicle_id_ = -1;
  is_emergency_avoidance_situation_ = false;
}

}  // namespace planning
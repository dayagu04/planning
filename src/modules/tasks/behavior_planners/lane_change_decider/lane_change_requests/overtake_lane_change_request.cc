#include "overtake_lane_change_request.h"

#include <glog/logging.h>
#include <math.h>

#include <cassert>
#include <cmath>
#include <complex>
#include <limits>

#include "common.pb.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "define/geometry.h"
#include "ego_state_manager.h"
#include "log.h"
#include "planning_context.h"
#include "tasks/behavior_planners/lane_change_decider/lane_change_requests/lane_change_request.h"
#include "tasks/behavior_planners/lane_change_decider/lateral_behavior_object_selector.h"
#include "tracked_object.h"
#include "virtual_lane_manager.h"

namespace planning {

namespace {
constexpr double kODDRouteDistanceThreshold = 500.0;
constexpr double kStaticVehicleThreshold = 1.0;
constexpr int kOvertakeCountThreshold = 200;
constexpr int kOvertakeRightLaneCountThreshold = 300;
constexpr int kExtraOvertakeCountForRainMode = 150;
constexpr double kDisancebetweenRoadSplitAndRampAllowError = 5.0;
constexpr double kSplitTriggleDistance = 3000.0;
constexpr double kOvertakeUpdateCountTtcThreshold = 24.0;
constexpr double kOvertakeMaintainCountTtcThreshold = 48.0;
constexpr double kOvertakeLeadingVehicleDistanceThreshold = 180.0;
constexpr double kOvertakeEgoHighSpeedThreshold = 25.00;           // 90km/h
constexpr double kOvertakeHighSpeedDiffThreshold = 1.39;           // 5km/h
constexpr double kOvertakeHighSpeedDiffThresholdRainMode = 3.33;   // 12km/h
constexpr double kOvertakeLowSpeedDiffThreshold = 1.39;            // 5km/h
constexpr double kOvertakeLowSpeedDiffThresholdRainMode = 2.5;     // 9km/h
constexpr double kOvertakeMaintainCountSpeedDiffThreshold = 1.39;  // 5km/h
constexpr double kOvertakeUpdateCountRatioThreshold = 0.10;
constexpr double kOvertakeUpdateCountRatioThresholdRainMode = 0.15;
constexpr double kOvertakeMaintainCountRatioThreshold = 0.05;
constexpr double kOvertakeUpdateCountSpeedRatioThreshold = 0.2;
constexpr int kOvertakeUpdateCountTruckTypeThreshold = 4;
constexpr int kOvertakeUpdateCountCarTypeThreshold = 2;
constexpr double kOvertakeLeadingVehicleHighSpeedThreshold = 22.22;  // 80km/h
constexpr double kOvertakeLeadingVehicleLowSpeedThreshold = 16.67;   // 60km/h
constexpr double kOvertakeLeadingVehicleHighSpeedDiffThreshold = 1.94;  // 7km/h
constexpr double kOvertakeLeadingVehicleLowSpeedDiffThreshold = 3.89;  // 14km/h
constexpr double kOvertakeRightTurnExtraSpeedThreshold = 1.39;         // 5km/h
constexpr int kOvertakeInhibitExtraSpeedTotalLaneNum = 3;
constexpr double kOvertakeMinSpeedDiffThreshold = 0.01;
constexpr double kLaneChangeSafetyDistanceBuffer = 2.0;
constexpr double kPotensialObjectLonRange = 6.0;
constexpr double kPotensialObjectLatRange = 0.6;
constexpr int kPotensialObjectNum = 2;
constexpr int kBoundaryConeNumThres = 3;
constexpr int kLaneConeNumThres = 2;
constexpr double kLaneConeRangeThres = 0.6;
constexpr double kLaneBoundaryConeRangeThres = 0.5;
constexpr double kMinConeDistanceToLaneChange = 20.0;
constexpr double kSuppressionRebounceTime = 1.0;
constexpr double kSuppressionDistanceThres = 10;                // meter
constexpr double kDefaultFrontObstacleDistance = 300;           // m
constexpr double kCancelOverTakeLnChgLeadVehLatSpdThold = 0.4;  // mps
constexpr double kCancelOverTakeLnChgEgoVehLatDstThold = 0.4;   // m
constexpr double kCancelOverTakeLnChgFrtLeadVehLonDst = 140;    // m
constexpr double kCancelOverTakeLnChgLeadVehLatDstThold = 1.1;  // m
constexpr double kDeadEndAheadLaneChangeMaxDistance = 600.0;
constexpr double kDeadEndAheadLaneChangeTimeBuffer = 20.0;
constexpr double kCancelOverTakeLnChgTargetLaneVehDftSpd = 50;  // mps,180kph
constexpr double kTriggerOvertakeDuration = 0.5;
constexpr double kCancelOverTakeLnChgTargetLaneVehDstThold = 180;   // m
constexpr double kCancelOverTakeLnChgTargetLaneVehHighDst = 140;    // m
constexpr double kCancelOverTakeLnChgTargetLaneVehLowDst = 30;      // m
constexpr double kCancelOverTakeLnChgTargetLaneVehHighSpdDiff = 0;  // mps
constexpr double kCancelOverTakeLnChgTargetLaneVehLowSpdDiff =
    -3;  // mps,10.8km/h
constexpr double kLaneChangeSafetyForwardTime = 4.0;
constexpr double kLaneChangeSafetyBackwardTime = 4.0;
constexpr double kMinLengthForTruck = 7.0;
constexpr double kMinWidthForTruck = 2.0;
constexpr double kExtraLatBufferForTruck = 0.5;
constexpr double kLaneChangeSafetyLongDistance = 6.0;
constexpr double kMaxLaneChangeSafetyLongDistance = 20.0;
constexpr double kBackwardLaneChangeSafetyLongDistance = 6.0;
constexpr double kBackwardMaxLaneChangeSafetyLongDistance = 13.0;
constexpr double kDecInCheckLaneChangeSafety = -0.6;
constexpr double kConsiderTimestampInCheckLaneChangeSafety = 4.0;
constexpr double kSearchRangeInCheckLaneChangeSafety = 150.0;
constexpr int kSearchObsNumInCheckLaneChangeSafety = 1;
constexpr double kLateralBufferInCheckLaneChangeSafety = 0.7;
constexpr double kLowestSpeedInCheckLaneChangeSafety = 13.889;  // 50km/h
constexpr double kHighestSpeedInCheckLaneChangeSafety = 33.333;

}  // namespace
// class: OvertakeRequest
OvertakeRequest::OvertakeRequest(
    planning::framework::Session* session,
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
    std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr)
    : LaneChangeRequest(session, virtual_lane_mgr, lane_change_lane_mgr) {
  base_frenet_coord_ = std::make_shared<KDPath>();
}

void OvertakeRequest::Update(int lc_status) {
  std::cout << "OvertakeRequest::Update::coming overtake lane change request"
            << std::endl;

  lateral_obstacle_ = session_->environmental_model().get_lateral_obstacle();
  lane_tracks_manager_ =
      session_->environmental_model().get_lane_tracks_manager();
  const auto tracks = lateral_obstacle_->all_tracks();
  tracks_map_.clear();
  for (auto track : tracks) {
    tracks_map_[track.track_id] = track;
  }

  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  planning_init_point_ = ego_state->planning_init_point();
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  int current_lane_virtual_id = virtual_lane_mgr_->current_lane_virtual_id();
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
  if (llane != nullptr) {
    left_reference_path_ = reference_path_mgr->get_reference_path_by_lane(
        llane->get_virtual_id(), false);
    LOG_DEBUG("OvertakeRequest::Update: for left_lane: update %d\n",
              llane->get_virtual_id());
  } else {
    left_reference_path_ = nullptr;
  }

  if (rlane != nullptr) {
    right_reference_path_ = reference_path_mgr->get_reference_path_by_lane(
        rlane->get_virtual_id(), false);
    LOG_DEBUG("OvertakeRequest::Update: for right_lane: update %d\n",
              rlane->get_virtual_id());
  } else {
    right_reference_path_ = nullptr;
  }

  double current_timestamp = IflyTime::Now_s();
  enable_l_ = llane && left_reference_path_;
  enable_r_ = rlane && right_reference_path_;
  const bool is_lane_changing =
      lc_status == kLaneChangeExecution || lc_status == kLaneChangeComplete;

  if (enable_l_) {
    bool left_valid =
        is_lane_changing ? true
                         : checkLeftLaneChangeValid(left_reference_path_, true);
    if (!left_valid) {
      enable_l_ = false;
      left_invalid_timestamp_ = current_timestamp;
    } else {
      if (current_timestamp - left_invalid_timestamp_ <
          kSuppressionRebounceTime) {
        LOG_DEBUG("Left LC invalid for rebounce time!");
        enable_l_ = false;
      }
    }
  }
  if (enable_r_) {
    bool right_valid = is_lane_changing ? true
                                        : checkRightLaneChangeValid(
                                              right_reference_path_, false);
    if (!right_valid) {
      enable_r_ = false;
      right_invalid_timestamp_ = current_timestamp;
    } else {
      if (current_timestamp - right_invalid_timestamp_ <
          kSuppressionRebounceTime) {
        LOG_DEBUG("Left LC invalid for rebounce time!");
        enable_r_ = false;
      }
    }
  }
  JSON_DEBUG_VALUE("enable_l_", enable_l_);
  JSON_DEBUG_VALUE("enable_r_", enable_r_);

  updateLaneChangeSafety(left_reference_path_, right_reference_path_);
  JSON_DEBUG_VALUE("is_left_lane_change_safe_", is_left_lane_change_safe_);
  JSON_DEBUG_VALUE("is_right_lane_change_safe_", is_right_lane_change_safe_);
  setLaneChangeRequestByFrontSlowVehcile(lc_status);
  LOG_DEBUG("request_type_: [%d] turn_signal_: [%d]\n", request_type_,
            turn_signal_);
}

void OvertakeRequest::setLaneChangeRequestByFrontSlowVehcile(int lc_status) {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const int current_lane_virtual_id =
      virtual_lane_mgr_->current_lane_virtual_id();
  int base_lane_virtual_id{current_lane_virtual_id};

  if (lane_change_lane_mgr_->has_origin_lane()) {
    auto origin_lane = lane_change_lane_mgr_->olane();
    origin_lane_virtual_id_ = origin_lane->get_virtual_id();
  } else {
    origin_lane_virtual_id_ = current_lane_virtual_id;
  }
  int target_lane_virtual_id_tmp{current_lane_virtual_id};

  const auto olane = lane_change_lane_mgr_->olane();
  const auto flane = lane_change_lane_mgr_->flane();
  const auto tlane = lane_change_lane_mgr_->tlane();

  const auto& clane = virtual_lane_mgr_->get_current_lane();
  const auto& llane = virtual_lane_mgr_->get_left_lane();
  const auto& rlane = virtual_lane_mgr_->get_right_lane();

  bool curr_direct_exist = (clane->get_lane_marks() ==
                            iflyauto::LaneDrivableDirection_DIRECTION_STRAIGHT);
  double v_ego = ego_state->ego_v();

  const std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes =
      virtual_lane_mgr_->get_virtual_lanes();
  int lane_nums = relative_id_lanes.size();
  int lane_index = virtual_lane_mgr_->get_lane_index(clane);
  int right_lane_nums = std::max((int)lane_nums - lane_index - 1, 0);
  int left_lane_nums = lane_index;

  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();

  TrackedObject* lead_one = lateral_obstacle_->leadone();

  // 无效的track_id暂时赋值为-1
  if ((lead_one != nullptr && lead_one->track_id == -1) ||
      lead_one == nullptr) {
    LOG_DEBUG("not exist stable leading vehicle");
    overtake_count_ = 0;
    Finish();
    return;
  }

  // HACK：由于目前无法识别红绿灯路口
  // 暂时将速度低于10km/h的目标物看作静止目标物
  double active_lane_change_min_object_speed_threshold = 2.778;
  if (lead_one->v <= active_lane_change_min_object_speed_threshold) {
    LOG_DEBUG("lead_one is static object");
    overtake_count_ = 0;
    Finish();
    return;
  }

  auto base_lane =
      virtual_lane_mgr_->get_lane_with_virtual_id(base_lane_virtual_id);

  if (base_lane == nullptr) {
    LOG_DEBUG("base lane not exist");
    Finish();
    overtake_count_ = 0;
    return;
  }

  std::shared_ptr<ReferencePath> origin_refline =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->get_reference_path_by_lane(base_lane_virtual_id, false);

  base_frenet_coord_ = origin_refline->get_frenet_coord();
  Point2D ego_frenet_point;
  Point2D ego_cart_point{planning_init_point_.lat_init_state.x(),
                         planning_init_point_.lat_init_state.y()};
  if (!base_frenet_coord_->XYToSL(ego_cart_point, ego_frenet_point)) {
    LOG_DEBUG("fail to get ego position on base lane");
    Finish();
    overtake_count_ = 0;
    return;
  }

  const double ego_front_edge = vehicle_param.front_edge_to_rear_axle;
  const double long_diff =
      lead_one->d_rel - ego_front_edge - lead_one->length * 0.5;

  const double ego_speed = ego_state->ego_v();
  const double reference_speed = ego_state->ego_v_cruise();
  const bool is_rain_mode =
      false;  // hack：当前planning中没有区分下雨天、不下雨场景
  const bool is_satisfy_update_condition =
      isSatisfyOvertakeCountUpdateCondition(
          lead_one, ego_speed, reference_speed, long_diff, is_rain_mode);
  const bool is_satisfy_maintain_condition =
      isSatisfyOvertakeCountMaintainCondition(lead_one, reference_speed,
                                              long_diff, is_rain_mode);

  std::cout << "reference_speed: " << reference_speed
            << "is_satisfy_update_condition: " << is_satisfy_update_condition
            << "is_satisfy_maintain_condition: "
            << is_satisfy_maintain_condition << std::endl;

  int left_count_thres = kOvertakeCountThreshold;
  int right_count_thres = kOvertakeRightLaneCountThreshold;
  if (is_rain_mode) {
    left_count_thres += kExtraOvertakeCountForRainMode;
    right_count_thres += kExtraOvertakeCountForRainMode;
  }
  if (is_satisfy_update_condition) {
    updateOvertakeCount(lead_one, ego_speed, reference_speed,
                        right_count_thres);
  } else if (!is_satisfy_maintain_condition) {
    overtake_count_ = 0;
  }

  LOG_DEBUG("overtake_count_: %d", overtake_count_);
  JSON_DEBUG_VALUE("overtake_count_", overtake_count_);

  if (overtake_count_ < left_count_thres) {
    return;
  }

  double left_route_traffic_speed = 0.0;
  double right_route_traffic_speed = 0.0;

  updateRouteTrafficSpeed(true, &left_route_traffic_speed);
  updateRouteTrafficSpeed(false, &right_route_traffic_speed);
  const double leading_vehicle_speed = lead_one->v;

  const bool is_left_overtake =
      enable_l_
          ? isCouldOvertakeByRoute(
                origin_refline, llane, left_route_traffic_speed,
                leading_vehicle_speed, left_lane_nums, right_lane_nums, true)
          : false;
  const bool is_right_overtake =
      enable_r_
          ? isCouldOvertakeByRoute(
                origin_refline, rlane, right_route_traffic_speed,
                leading_vehicle_speed, left_lane_nums, right_lane_nums, false)
          : false;
  JSON_DEBUG_VALUE("is_left_overtake", is_left_overtake);
  JSON_DEBUG_VALUE("is_right_overtake", is_right_overtake);

  const double current_time = IflyTime::Now_s();
  auto checkOvertakeTrigger = [](const double time, const bool is_trigger,
                                 double* overtake_t) -> bool {
    if (!is_trigger) {
      *overtake_t = std::numeric_limits<double>::max();
      return false;
    } else {
      if (*overtake_t == std::numeric_limits<double>::max()) {
        *overtake_t = time;
      }
      if (time - *overtake_t > kTriggerOvertakeDuration) {
        return true;
      } else {
        return false;
      }
    }
  };

  const bool trigger_left_overtake = checkOvertakeTrigger(
      current_time, is_left_overtake && is_left_lane_change_safe_,
      &left_overtake_valid_timestamp_);

  const bool trigger_right_overtake = checkOvertakeTrigger(
      current_time, is_right_overtake && is_right_lane_change_safe_,
      &right_overtake_valid_timestamp_);

  JSON_DEBUG_VALUE("trigger_left_overtake", trigger_left_overtake);
  JSON_DEBUG_VALUE("trigger_right_overtake", trigger_right_overtake);
  if (trigger_left_overtake) {
    if (request_type_ != LEFT_CHANGE && compute_lc_valid_info(LEFT_CHANGE)) {
      target_lane_virtual_id_tmp = origin_lane_virtual_id_ - 1;
      GenerateRequest(LEFT_CHANGE);
      set_target_lane_virtual_id(target_lane_virtual_id_tmp);
      LOG_DEBUG(
          "[OvertakeRequest::update] Ask for overtake changing lane to left "
          "\n");
    }
    if (!IsDashEnoughForRepeatSegments(LEFT_CHANGE, origin_lane_virtual_id_) &&
        curr_direct_exist && request_type_ != NO_CHANGE &&
        (lc_status == kLaneKeeping || lc_status == kLaneChangePropose ||
         (lc_status == kLaneChangeCancel &&
          (lane_change_lane_mgr_->has_origin_lane() &&
           lane_change_lane_mgr_->is_ego_on(olane))))) {
      Finish();
      set_target_lane_virtual_id(current_lane_virtual_id);
      LOG_DEBUG(
          "[OvertakeRequest::update] %s:%d finish request, dash not enough \n",
          __FUNCTION__, __LINE__);
    } else {
      overtake_vehicle_id_ = lead_one->track_id;
      overtake_vehicle_speed_ = leading_vehicle_speed;
      LOG_DEBUG("overtake_vehicle_id_: [%d] overtake_vehicle_speed_: [%f] \n",
                overtake_vehicle_id_, overtake_vehicle_speed_);
    }
  } else if (trigger_right_overtake && overtake_count_ >= right_count_thres) {
    if (request_type_ != RIGHT_CHANGE && compute_lc_valid_info(RIGHT_CHANGE)) {
      target_lane_virtual_id_tmp = origin_lane_virtual_id_ + 1;
      GenerateRequest(RIGHT_CHANGE);
      set_target_lane_virtual_id(target_lane_virtual_id_tmp);
      LOG_DEBUG(
          "[OvertakeRequest::update] Ask for overtake changing lane to right "
          "\n");
    }
    if (!IsDashEnoughForRepeatSegments(RIGHT_CHANGE, origin_lane_virtual_id_) &&
        curr_direct_exist && request_type_ != NO_CHANGE &&
        (lc_status == kLaneKeeping || lc_status == kLaneChangePropose ||
         (lc_status == kLaneChangeCancel &&
          (lane_change_lane_mgr_->has_origin_lane() &&
           lane_change_lane_mgr_->is_ego_on(olane))))) {
      Finish();
      set_target_lane_virtual_id(current_lane_virtual_id);
      LOG_DEBUG(
          "[OvertakeRequest::update] %s:%d finish request, dash not enough \n",
          __FUNCTION__, __LINE__);
    } else {
      overtake_vehicle_id_ = lead_one->track_id;
      overtake_vehicle_speed_ = leading_vehicle_speed;
      LOG_DEBUG("overtake_vehicle_id_: [%d] overtake_vehicle_speed_: [%f] \n",
                overtake_vehicle_id_, overtake_vehicle_speed_);
    }
  } else if (request_type_ != NO_CHANGE &&
             (lane_change_lane_mgr_->has_origin_lane() &&
              lane_change_lane_mgr_->is_ego_on(olane))) {
    Finish();
    set_target_lane_virtual_id(current_lane_virtual_id);
    LOG_DEBUG(
        "[OvertakeRequest::update] %s:%d finish request, "
        "!trigger_left_overtake and !trigger_right_overtake\n",
        __FUNCTION__, __LINE__);
  }
}

bool OvertakeRequest::isSatisfyOvertakeCountUpdateCondition(
    const TrackedObject* leading_vehicle, const double ego_speed,
    const double reference_speed, const double leading_vehicle_dist,
    const bool rain_mode) {
  if (leading_vehicle->track_id == -1) {
    return false;
  }

  if (leading_vehicle_dist > kOvertakeLeadingVehicleDistanceThreshold) {
    return false;
  }

  const double speed_diff = std::max(reference_speed - leading_vehicle->v,
                                     kOvertakeMinSpeedDiffThreshold);
  const double front_ttc = (speed_diff > 0.0)
                               ? (leading_vehicle_dist / speed_diff)
                               : std::numeric_limits<double>::max();
  std::cout << "leading_vehicle_speed: " << leading_vehicle->v
            << "ego_speed: " << ego_speed << "speed_diff: " << speed_diff
            << "long_distance: " << leading_vehicle_dist
            << "front_ttc: " << front_ttc << std::endl;
  const int high_speed_diff_thres =
      rain_mode ? kOvertakeHighSpeedDiffThresholdRainMode
                : kOvertakeHighSpeedDiffThreshold;
  if (ego_speed >= kOvertakeEgoHighSpeedThreshold &&
      speed_diff > high_speed_diff_thres) {
    return true;
  }
  const int low_speed_diff_thres = rain_mode
                                       ? kOvertakeLowSpeedDiffThresholdRainMode
                                       : kOvertakeLowSpeedDiffThreshold;
  if (ego_speed < kOvertakeEgoHighSpeedThreshold &&
      speed_diff > low_speed_diff_thres) {
    return true;
  }
  const double speed_ratio =
      (0.0 != reference_speed) ? (speed_diff / reference_speed) : 0.0;
  const double update_count_ratio_thres =
      rain_mode ? kOvertakeUpdateCountRatioThresholdRainMode
                : kOvertakeUpdateCountRatioThreshold;
  if (speed_ratio > update_count_ratio_thres) {
    return true;
  }
  return false;
}

bool OvertakeRequest::isSatisfyOvertakeCountMaintainCondition(
    const TrackedObject* leading_vehicle, const double reference_speed,
    const double leading_vehicle_dist, const bool rain_mode) {
  if (leading_vehicle->track_id == -1) {
    return false;
  }

  if (leading_vehicle_dist > kOvertakeLeadingVehicleDistanceThreshold) {
    return false;
  }
  const double speed_diff = std::max(reference_speed - leading_vehicle->v,
                                     kOvertakeMinSpeedDiffThreshold);
  const double front_ttc = (speed_diff > 0.0)
                               ? (leading_vehicle_dist / speed_diff)
                               : std::numeric_limits<double>::max();
  if (front_ttc > kOvertakeMaintainCountTtcThreshold) {
    return false;
  }

  if (speed_diff > kOvertakeMaintainCountSpeedDiffThreshold) {
    return true;
  }
  const double speed_ratio =
      (0.0 != reference_speed) ? (speed_diff / reference_speed) : 0.0;
  if (speed_ratio > kOvertakeMaintainCountRatioThreshold) {
    return true;
  }
  return false;
}

void OvertakeRequest::updateOvertakeCount(const TrackedObject* leading_vehicle,
                                          const double ego_speed,
                                          const double reference_speed,
                                          const int max_count_thres) {
  int type_value = kOvertakeUpdateCountCarTypeThreshold;
  if (leading_vehicle->type == iflyauto::OBJECT_TYPE_TRUCK) {
    type_value = kOvertakeUpdateCountTruckTypeThreshold;
  }
  const int curr_count = round((reference_speed - leading_vehicle->v) *
                               kOvertakeUpdateCountSpeedRatioThreshold) +
                         type_value;
  overtake_count_ += curr_count;
  overtake_count_ = std::min(overtake_count_, max_count_thres);
}

void OvertakeRequest::updateRouteTrafficSpeed(const bool is_left,
                                              double* route_traffic_speed) {
  if (!route_traffic_speed) {
    return;
  }
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();

  *route_traffic_speed = 0.0;
  const std::vector<TrackedObject>& front_tracks_l =
      lane_tracks_manager_->front_tracks_l();
  const std::vector<TrackedObject>& front_tracks_r =
      lane_tracks_manager_->front_tracks_r();

  std::vector<TrackedObject> side_front_obstacle_array;
  if (is_left) {
    for (auto& tr : front_tracks_l) {
      side_front_obstacle_array.push_back(tr);
    }
  } else {
    for (auto& tr : front_tracks_r) {
      side_front_obstacle_array.push_back(tr);
    }
  }

  double first_vehicle_speed = std::numeric_limits<double>::max();
  double second_vehicle_speed = std::numeric_limits<double>::max();
  for (const auto& front_obstacle : side_front_obstacle_array) {
    if (std::numeric_limits<double>::max() == first_vehicle_speed) {
      first_vehicle_speed = front_obstacle.v;
    } else if (std::numeric_limits<double>::max() == second_vehicle_speed) {
      second_vehicle_speed = front_obstacle.v;
      break;
    }
  }

  double refer_speed = ego_state->ego_v_cruise();
  double final_speed = std::min(
      std::min(first_vehicle_speed, second_vehicle_speed), refer_speed);

  *route_traffic_speed = final_speed;
}

bool OvertakeRequest::isCouldOvertakeByRoute(
    const std::shared_ptr<ReferencePath>& base_ref_line,
    const std::shared_ptr<VirtualLane>& target_lane,
    const double lane_traffic_speed, const double leading_vehicle_speed,
    const int left_lane_nums, const int right_lane_nums, const bool is_left) {
  std::shared_ptr<ReferencePath> target_ref_line =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->get_reference_path_by_lane(target_lane->get_virtual_id(), false);
  if (!base_ref_line || !target_ref_line) {
    return false;
  }
  bool ramp_on_left = false;
  bool ramp_on_Right = false;
  bool is_on_highway = session_->environmental_model().is_on_highway();
  if (is_on_highway) {
    ramp_on_left =
        virtual_lane_mgr_->ramp_direction() == RampDirection::RAMP_ON_LEFT
            ? true
            : false;
    ramp_on_Right =
        virtual_lane_mgr_->ramp_direction() == RampDirection::RAMP_ON_RIGHT
            ? true
            : false;
    const double distance_to_next_ramp = virtual_lane_mgr_->dis_to_ramp();
    const double distance_to_next_split =
        virtual_lane_mgr_->distance_to_first_road_split();
    double dis_between_first_road_split_and_ramp =
        distance_to_next_split - distance_to_next_ramp;
    if (distance_to_next_ramp < kSplitTriggleDistance &&
        dis_between_first_road_split_and_ramp <
            kDisancebetweenRoadSplitAndRampAllowError) {
      if ((is_left && ramp_on_Right) || (!is_left && ramp_on_left)) {
        return false;
      }
    }
    // TODO:靠近分流口，分流口类型不是匝道，考虑反方向变道抑制(城区智能驾驶领航)

    if (target_lane->get_lane_merge_split_point().merge_split_point_data_size >
        0) {
      const auto& target_lane_merge_info =
          target_lane->get_lane_merge_split_point().merge_split_point_data[0];
      if (!target_lane_merge_info.is_split) {
        LOG_DEBUG(
            "Not trigger overtake since target route has merge lane, is left "
            "lane: %d",
            is_left);
        return false;
      }
    }
  }
  double speed_threshold = kOvertakeLeadingVehicleHighSpeedDiffThreshold;
  if (leading_vehicle_speed >= kOvertakeLeadingVehicleHighSpeedThreshold) {
    speed_threshold = kOvertakeLeadingVehicleHighSpeedDiffThreshold;
  } else if (leading_vehicle_speed <=
             kOvertakeLeadingVehicleLowSpeedThreshold) {
    speed_threshold = kOvertakeLeadingVehicleLowSpeedDiffThreshold;
  } else {
    speed_threshold = planning_math::lerp(
        kOvertakeLeadingVehicleLowSpeedDiffThreshold,
        kOvertakeLeadingVehicleLowSpeedThreshold,
        kOvertakeLeadingVehicleHighSpeedDiffThreshold,
        kOvertakeLeadingVehicleHighSpeedThreshold, leading_vehicle_speed);
  }
  const int total_lane_nums = left_lane_nums + right_lane_nums + 1;
  const bool inhibit_extra_speed =
      (total_lane_nums >= kOvertakeInhibitExtraSpeedTotalLaneNum &&
       0 == left_lane_nums);
  if (!is_left && !inhibit_extra_speed) {
    speed_threshold += kOvertakeRightTurnExtraSpeedThreshold;
  }

  // 当总车道数不少于3时，抑制向最右侧车道触发超车变道
  if (total_lane_nums >= kOvertakeInhibitExtraSpeedTotalLaneNum && !is_left &&
      1 == right_lane_nums) {
    return false;
  }
  const double left_speed_diff = lane_traffic_speed - leading_vehicle_speed;
  const bool is_overtake = (left_speed_diff > speed_threshold);
  return is_overtake;
}

void OvertakeRequest::updateLaneChangeSafety(
    const std::shared_ptr<ReferencePath>& left_ref_line,
    const std::shared_ptr<ReferencePath>& right_ref_line) {
  is_left_lane_change_safe_ = false;
  is_right_lane_change_safe_ = false;

  std::vector<int> risk_agent_id_for_lane_change;
  const double safety_extra_forward_distance = 0.0;
  const double safety_extra_backward_distance = kLaneChangeSafetyDistanceBuffer;
  const double safety_forward_time = kLaneChangeSafetyForwardTime;
  const double safety_backward_time = kLaneChangeSafetyBackwardTime;
  const double safety_ratio = 1.0;

  double front_required_space = 0.0;
  double rear_required_space = 0.0;
  if (enable_l_ && left_ref_line) {
    is_left_lane_change_safe_ = checkLaneChangeSafety(
        left_ref_line, safety_extra_forward_distance,
        safety_extra_backward_distance, safety_forward_time,
        safety_backward_time, true, safety_ratio, safety_ratio, true,
        &risk_agent_id_for_lane_change, &front_required_space,
        &rear_required_space);
  }
  if (enable_r_ && right_ref_line) {
    is_right_lane_change_safe_ = checkLaneChangeSafety(
        right_ref_line, safety_extra_forward_distance,
        safety_extra_backward_distance, safety_forward_time,
        safety_backward_time, true, safety_ratio, safety_ratio, true,
        &risk_agent_id_for_lane_change, &front_required_space,
        &rear_required_space);
  }
}

bool OvertakeRequest::checkLeftLaneChangeValid(
    const std::shared_ptr<ReferencePath>& ref_line, const bool is_left) {
  if (!checkLeftLaneChangeValidByObjects(ref_line)) {
    LOG_DEBUG("left invalid since objects");
    return false;
  }
  if (!checkLaneChangeValidBySuprsSignal(is_left)) {
    LOG_DEBUG("left invalid since suppression");
    return false;
  }
  return true;
}

bool OvertakeRequest::checkRightLaneChangeValid(
    const std::shared_ptr<ReferencePath>& ref_line, const bool is_left) {
  if (!checkRightLaneChangeValidByObjects(ref_line)) {
    LOG_DEBUG("right invalid since objects");
    return false;
  }
  if (!checkLaneChangeValidBySuprsSignal(is_left)) {
    LOG_DEBUG("right invalid since suppression");
    return false;
  }
  return true;
}

bool OvertakeRequest::checkLeftLaneChangeValidByObjects(
    const std::shared_ptr<ReferencePath>& ref_line) {
  if (!ref_line) {
    return false;
  }

  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& target_lane_coord_ptr = ref_line->get_frenet_coord();
  Point2D ego_frenet_point_in_left_lane;
  Point2D ego_cart_point{planning_init_point_.lat_init_state.x(),
                         planning_init_point_.lat_init_state.y()};
  if (!target_lane_coord_ptr->XYToSL(ego_cart_point,
                                     ego_frenet_point_in_left_lane)) {
    LOG_DEBUG("Enmergency error! Ego Pose Cart2SL in left lane failed!");
  }
  const double potensial_max_l =
      -ego_frenet_point_in_left_lane.y + kPotensialObjectLatRange;
  int potensial_counter = 0;

  auto target_lane = virtual_lane_mgr_->get_left_lane();
  auto current_lane = virtual_lane_mgr_->get_current_lane();
  double lane_width = current_lane->width();

  const std::vector<int>& left_potensial_objects =
      target_lane->get_reference_path()->get_lane_obstacles_ids();
  const std::vector<int>& front_potensial_objects =
      current_lane->get_reference_path()->get_lane_obstacles_ids();
  const std::vector<TrackedObject>& front_tracks =
      lateral_obstacle_->front_tracks();

  if (!left_potensial_objects.size()) {
    return true;
  }
  for (const auto& id : left_potensial_objects) {
    if (tracks_map_[id].d_rel < -kPotensialObjectLonRange ||
        tracks_map_[id].d_rel > kPotensialObjectLonRange ||
        tracks_map_[id].l > potensial_max_l || tracks_map_[id].l < 0.0) {
      continue;
    }
    ++potensial_counter;
    if (potensial_counter >= kPotensialObjectNum) {
      return false;
    }
  }

  std::vector<TrackedObject> left_front_target_tracks;
  std::vector<TrackedObject> ego_front_target_tracks;
  std::vector<TrackedObject> left_lane_cone_distribution_set;
  std::vector<TrackedObject> current_lane_cone_distribution_set;
  std::vector<TrackedObject> left_boundary_cone_distribution_set;
  std::vector<TrackedObject> left_boundary_cone_distribution_ordered_set;

  for (auto& obstacle : front_tracks) {
    if (std::count(left_potensial_objects.begin(), left_potensial_objects.end(),
                   obstacle.track_id) > 0) {
      left_front_target_tracks.push_back(obstacle);
    }
    if (std::count(front_potensial_objects.begin(),
                   front_potensial_objects.end(), obstacle.track_id) > 0) {
      ego_front_target_tracks.push_back(obstacle);
    }
  }

  for (const auto& obj : ego_front_target_tracks) {
    if (obj.type == iflyauto::OBJECT_TYPE_TRAFFIC_CONE) {
      if (std::fabs(obj.l) < kLaneConeRangeThres) {
        current_lane_cone_distribution_set.push_back(obj);
      }
      if ((obj.l > kLaneConeRangeThres) &&
          std::fabs(obj.l - lane_width * 0.5) < kLaneBoundaryConeRangeThres) {
        left_boundary_cone_distribution_set.push_back(obj);
      }
    }
  }
  for (const auto& obj : left_front_target_tracks) {
    if (obj.type == iflyauto::OBJECT_TYPE_TRAFFIC_CONE) {
      if (std::fabs(obj.l - lane_width) < kLaneConeRangeThres) {
        left_lane_cone_distribution_set.push_back(obj);
      }
      if ((obj.l < lane_width - kLaneConeRangeThres) &&
          std::fabs(obj.l - lane_width * 0.5) < kLaneBoundaryConeRangeThres) {
        left_boundary_cone_distribution_set.push_back(obj);
      }
    }
  }
  // 针对左侧车道边界的锥桶，安排距离自车由近及远排序
  for (const auto& tr : left_boundary_cone_distribution_set) {
    if (tr.d_rel >= 0.0) {
      auto it = left_boundary_cone_distribution_ordered_set.begin();
      while (it != left_boundary_cone_distribution_ordered_set.end() &&
             it->d_rel < tr.d_rel) {
        ++it;
      }

      if (it != left_boundary_cone_distribution_ordered_set.end()) {
        left_boundary_cone_distribution_ordered_set.insert(it, tr);
      } else {
        left_boundary_cone_distribution_ordered_set.push_back(tr);
      }
    }
  }

  // 根据自车道以及目标车道的锥桶分布情况来判断左侧变道是否有效
  if (left_boundary_cone_distribution_ordered_set.size() >=
          kBoundaryConeNumThres &&
      current_lane_cone_distribution_set.size() >= kLaneConeNumThres &&
      left_lane_cone_distribution_set.size() == 0 &&
      current_lane_cone_distribution_set[0].s >
          left_boundary_cone_distribution_ordered_set[0].s) {
    return true;
  }
  if (current_lane_cone_distribution_set.size() >= kLaneConeNumThres &&
      left_lane_cone_distribution_set.size() >= kLaneConeNumThres) {
    if (left_lane_cone_distribution_set[0].s -
            current_lane_cone_distribution_set[0].s >
        kMinConeDistanceToLaneChange) {
      return true;
    } else {
      return false;
    }
  }
  if (left_lane_cone_distribution_set.size() +
          left_boundary_cone_distribution_ordered_set.size() >=
      kBoundaryConeNumThres) {
    return false;
  }
  return true;
}

bool OvertakeRequest::checkRightLaneChangeValidByObjects(
    const std::shared_ptr<ReferencePath>& ref_line) {
  if (!ref_line) {
    return false;
  }

  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& target_lane_coord_ptr = ref_line->get_frenet_coord();
  Point2D ego_frenet_point_in_right_lane;
  Point2D ego_cart_point{planning_init_point_.lat_init_state.x(),
                         planning_init_point_.lat_init_state.y()};
  if (!target_lane_coord_ptr->XYToSL(ego_cart_point,
                                     ego_frenet_point_in_right_lane)) {
    LOG_DEBUG("Enmergency error! Ego Pose Cart2SL in right lane failed!");
  }
  const double potensial_min_l =
      -ego_frenet_point_in_right_lane.y - kPotensialObjectLatRange;
  int potensial_counter = 0;

  auto target_lane = virtual_lane_mgr_->get_right_lane();
  auto current_lane = virtual_lane_mgr_->get_current_lane();
  double lane_width = current_lane->width();

  const std::vector<int>& right_potensial_objects =
      target_lane->get_reference_path()->get_lane_obstacles_ids();
  const std::vector<int>& front_potensial_objects =
      current_lane->get_reference_path()->get_lane_obstacles_ids();
  const std::vector<TrackedObject>& front_tracks =
      lateral_obstacle_->front_tracks();

  if (!right_potensial_objects.size()) {
    return true;
  }
  for (const auto& id : right_potensial_objects) {
    if (tracks_map_[id].d_rel < -kPotensialObjectLonRange ||
        tracks_map_[id].d_rel > kPotensialObjectLonRange ||
        tracks_map_[id].l < potensial_min_l || tracks_map_[id].l > 0.0) {
      continue;
    }
    ++potensial_counter;
    if (potensial_counter >= kPotensialObjectNum) {
      return false;
    }
  }

  std::vector<TrackedObject> right_front_target_tracks;
  std::vector<TrackedObject> ego_front_target_tracks;
  std::vector<TrackedObject> right_lane_cone_distribution_set;
  std::vector<TrackedObject> current_lane_cone_distribution_set;
  std::vector<TrackedObject> right_boundary_cone_distribution_set;
  std::vector<TrackedObject> right_boundary_cone_distribution_ordered_set;

  for (auto& obstacle : lateral_obstacle_->front_tracks()) {
    if (std::count(right_potensial_objects.begin(),
                   right_potensial_objects.end(), obstacle.track_id) > 0) {
      right_front_target_tracks.push_back(obstacle);
    }
    if (std::count(front_potensial_objects.begin(),
                   front_potensial_objects.end(), obstacle.track_id) > 0) {
      ego_front_target_tracks.push_back(obstacle);
    }
  }

  for (const auto& obj : ego_front_target_tracks) {
    if (obj.type == iflyauto::OBJECT_TYPE_TRAFFIC_CONE) {
      if (std::fabs(obj.l) < kLaneConeRangeThres) {
        current_lane_cone_distribution_set.push_back(obj);
      }
      if ((obj.l < -kLaneConeRangeThres) &&
          std::fabs(obj.l + lane_width * 0.5) < kLaneBoundaryConeRangeThres) {
        right_boundary_cone_distribution_set.push_back(obj);
      }
    }
  }
  for (const auto& obj : right_front_target_tracks) {
    if (obj.type == iflyauto::OBJECT_TYPE_TRAFFIC_CONE) {
      if (std::fabs(obj.l + lane_width) < kLaneConeRangeThres) {
        right_lane_cone_distribution_set.push_back(obj);
      }
      if ((obj.l > -lane_width + kLaneConeRangeThres) &&
          std::fabs(obj.l + lane_width * 0.5) < kLaneBoundaryConeRangeThres) {
        right_boundary_cone_distribution_set.push_back(obj);
      }
    }
  }
  // 针对右侧车道边界的锥桶，安排距离自车由近及远排序
  for (const auto& tr : right_boundary_cone_distribution_set) {
    if (tr.d_rel >= 0.0) {
      auto it = right_boundary_cone_distribution_ordered_set.begin();
      while (it != right_boundary_cone_distribution_ordered_set.end() &&
             it->d_rel < tr.d_rel) {
        ++it;
      }

      if (it != right_boundary_cone_distribution_ordered_set.end()) {
        right_boundary_cone_distribution_ordered_set.insert(it, tr);
      } else {
        right_boundary_cone_distribution_ordered_set.push_back(tr);
      }
    }
  }

  // 根据自车道以及目标车道的锥桶分布情况来判断右侧变道是否有效
  if (right_boundary_cone_distribution_ordered_set.size() >=
          kBoundaryConeNumThres &&
      current_lane_cone_distribution_set.size() >= kLaneConeNumThres &&
      right_lane_cone_distribution_set.size() == 0 &&
      current_lane_cone_distribution_set[0].s >
          right_boundary_cone_distribution_ordered_set[0].s) {
    return true;
  }
  if (current_lane_cone_distribution_set.size() >= kLaneConeNumThres &&
      right_lane_cone_distribution_set.size() >= kLaneConeNumThres) {
    if (right_lane_cone_distribution_set[0].s -
            current_lane_cone_distribution_set[0].s >
        kMinConeDistanceToLaneChange) {
      return true;
    } else {
      return false;
    }
  }
  if (right_lane_cone_distribution_set.size() +
          right_boundary_cone_distribution_ordered_set.size() >=
      kBoundaryConeNumThres) {
    return false;
  }
  return true;
}

bool OvertakeRequest::checkLaneChangeValidBySuprsSignal(const bool is_left) {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  std::vector<int> target_side_objects_id_set;
  Point2D ego_pose{planning_init_point_.lat_init_state.x(),
                   planning_init_point_.lat_init_state.y()};
  if (is_left) {
    auto target_lane = virtual_lane_mgr_->get_left_lane();
    target_side_objects_id_set =
        target_lane->get_reference_path()->get_lane_obstacles_ids();
  } else {
    auto target_lane = virtual_lane_mgr_->get_right_lane();
    target_side_objects_id_set =
        target_lane->get_reference_path()->get_lane_obstacles_ids();
  }
  if (!target_side_objects_id_set.size()) {
    return true;
  }
  bool has_moving_obs = false;
  for (const auto& id : target_side_objects_id_set) {
    auto iter = tracks_map_.find(id);
    if (iter == tracks_map_.end()) {
      continue;
    }
    const double dis =
        std::hypot(tracks_map_[id].center_x, tracks_map_[id].center_y);
    // 障碍物type为卡车或者轿车的需要抑制变道
    if (tracks_map_[id].v > kStaticVehicleThreshold &&
        dis < kSuppressionDistanceThres &&
        (tracks_map_[id].type == iflyauto::OBJECT_TYPE_TRUCK ||
         tracks_map_[id].type == iflyauto::OBJECT_TYPE_COUPE)) {
      has_moving_obs = true;
      break;
    }
  }
  return !has_moving_obs;
}

bool OvertakeRequest::checkLaneChangeSafety(
    const std::shared_ptr<ReferencePath>& target_ref_line,
    const double extra_front_distance_buffer,
    const double extra_rear_distance_buffer, const double safety_forward_time,
    const double safety_backward_time, const bool is_left,
    const double lon_safety_ratio, const double lat_safety_ratio,
    const bool use_dynamic_safety_distance,
    std::vector<int>* not_safe_agent_ids, double* front_required_space,
    double* rear_required_space) {
  assert(not_safe_agent_ids != nullptr && rear_required_space != nullptr &&
         front_required_space != nullptr);

  const auto& target_lane_coord_ptr = target_ref_line->get_frenet_coord();
  Point2D ego_frenet_point_in_target_lane;
  Point2D ego_cart_point{planning_init_point_.lat_init_state.x(),
                         planning_init_point_.lat_init_state.y()};
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();

  if (!target_lane_coord_ptr->XYToSL(ego_cart_point,
                                     ego_frenet_point_in_target_lane)) {
    LOG_DEBUG("ego on ref lane failed!");
    return false;
  }

  const double ego_front_edge = vehicle_param.front_edge_to_rear_axle;
  const double ego_rear_edge = vehicle_param.rear_edge_to_rear_axle;
  const double ego_half_width = vehicle_param.width * 0.5;
  *front_required_space = 0.0;
  *rear_required_space = 0.0;
  const double lateral_buffer =
      kLateralBufferInCheckLaneChangeSafety * lat_safety_ratio;

  std::vector<TrackedObject> target_side_front_tracks;
  if (is_left) {
    target_side_front_tracks = lane_tracks_manager_->front_tracks_l();
  } else {
    target_side_front_tracks = lane_tracks_manager_->front_tracks_r();
  }
  std::vector<int> front_tracks_ids;

  selectTargetObstacleIds(
      target_lane_coord_ptr, ego_cart_point, target_side_front_tracks,
      kSearchRangeInCheckLaneChangeSafety, kSearchObsNumInCheckLaneChangeSafety,
      ego_half_width, lateral_buffer, kLateralBufferInCheckLaneChangeSafety,
      false, &front_tracks_ids);

  std::vector<TrackedObject> target_side_rear_tracks;
  if (is_left) {
    target_side_rear_tracks = lane_tracks_manager_->side_tracks_l();
  } else {
    target_side_rear_tracks = lane_tracks_manager_->side_tracks_r();
  }
  std::vector<int> rear_tracks_ids;
  selectTargetObstacleIds(
      target_lane_coord_ptr, ego_cart_point, target_side_rear_tracks,
      kSearchRangeInCheckLaneChangeSafety, kSearchObsNumInCheckLaneChangeSafety,
      ego_half_width, lateral_buffer, kLateralBufferInCheckLaneChangeSafety,
      false, &rear_tracks_ids);

  auto front_track_iter = front_tracks_ids.empty()
                              ? tracks_map_.end()
                              : tracks_map_.find(front_tracks_ids.front());
  auto rear_track_iter = rear_tracks_ids.empty()
                             ? tracks_map_.end()
                             : tracks_map_.find(rear_tracks_ids.front());
  const bool has_front_obs = front_track_iter != tracks_map_.end();
  const bool has_rear_obs = rear_track_iter != tracks_map_.end();

  int not_safety_agent_id = -1;
  if (has_front_obs && has_rear_obs) {
    const double front_to_ego_dis = std::hypot(
        front_track_iter->second.center_x, front_track_iter->second.center_y);
    const double rear_to_ego_dis = std::hypot(rear_track_iter->second.center_x,
                                              rear_track_iter->second.center_y);
    not_safety_agent_id = front_to_ego_dis < rear_to_ego_dis
                              ? front_track_iter->first
                              : rear_track_iter->first;
  } else if (has_front_obs) {
    not_safety_agent_id = front_track_iter->first;
  } else if (has_rear_obs) {
    not_safety_agent_id = rear_track_iter->first;
  }

  if (has_front_obs) {
    const auto& ego_state =
        session_->environmental_model().get_ego_state_manager();
    double ego_fx = std::cos(ego_state->ego_pose_raw().theta);
    double ego_fy = std::sin(ego_state->ego_pose_raw().theta);
    const double long_dis = front_track_iter->second.d_rel;
    Point2D obs_cart_point{0.0, 0.0};
    obs_cart_point.x = ego_cart_point.x +
                       front_track_iter->second.center_x * ego_fx -
                       front_track_iter->second.center_y * ego_fy;
    obs_cart_point.y = ego_cart_point.y +
                       front_track_iter->second.center_x * ego_fy +
                       front_track_iter->second.center_y * ego_fx;

    Point2D obs_target_frenet_point;
    if (!target_lane_coord_ptr->XYToSL(obs_cart_point,
                                       obs_target_frenet_point)) {
      LOG_DEBUG("front obs on ref lane failed!");
      return false;
    }
    double front_safety_threshold_at_present = 0.0;
    const double front_safety_distance =
        getSafetyDistance(kLaneChangeSafetyLongDistance,
                          kMaxLaneChangeSafetyLongDistance, ego_state->ego_v(),
                          use_dynamic_safety_distance) +
        extra_front_distance_buffer;
    const bool front_safety_at_present = checkFrontSafetyAtPresent(
        long_dis, ego_state->ego_v(), front_track_iter->second.v,
        kDecInCheckLaneChangeSafety, front_track_iter->second.a, ego_front_edge,
        0.5 * front_track_iter->second.length, front_safety_distance,
        lon_safety_ratio, &front_safety_threshold_at_present);
    if (!front_safety_at_present) {
      not_safe_agent_ids->emplace_back(not_safety_agent_id);
      LOG_DEBUG(
          "checkLaneChangeSafety(): Not safety for front vehicle at present!");
      return false;
    }
    const double front_safety_at_future = checkFrontSafetyAtFuture(
        long_dis, ego_state->ego_v(), front_track_iter->second.v,
        kDecInCheckLaneChangeSafety, front_track_iter->second.a, ego_front_edge,
        0.5 * front_track_iter->second.length, front_safety_distance,
        lon_safety_ratio, kConsiderTimestampInCheckLaneChangeSafety);
    if (!front_safety_at_future) {
      not_safe_agent_ids->emplace_back(not_safety_agent_id);
      LOG_DEBUG(
          "checkLaneChangeSafety(): Not safety for front vehicle at future!");
      return false;
    }
    *front_required_space = front_safety_threshold_at_present;
  }

  if (has_rear_obs) {
    const auto& ego_state =
        session_->environmental_model().get_ego_state_manager();
    double ego_fx = std::cos(ego_state->ego_pose_raw().theta);
    double ego_fy = std::sin(ego_state->ego_pose_raw().theta);
    const double long_dis = rear_track_iter->second.d_rel;
    Point2D obs_cart_point{0.0, 0.0};
    obs_cart_point.x = ego_cart_point.x +
                       rear_track_iter->second.center_x * ego_fx -
                       rear_track_iter->second.center_y * ego_fy;
    obs_cart_point.y = ego_cart_point.y +
                       rear_track_iter->second.center_x * ego_fy +
                       rear_track_iter->second.center_y * ego_fx;

    Point2D obs_target_frenet_point;
    if (!target_lane_coord_ptr->XYToSL(obs_cart_point,
                                       obs_target_frenet_point)) {
      LOG_DEBUG("rear obs on ref lane failed!");
      return false;
    }
    double rear_safety_threshold_at_present = 0.0;
    const double rear_safety_distance =
        getSafetyDistance(kBackwardLaneChangeSafetyLongDistance,
                          kBackwardMaxLaneChangeSafetyLongDistance,
                          rear_track_iter->second.v,
                          use_dynamic_safety_distance) +
        extra_rear_distance_buffer;
    const bool rear_safety_at_present = checkRearSafetyAtPresent(
        long_dis, ego_state->ego_v(), rear_track_iter->second.v,
        kDecInCheckLaneChangeSafety, rear_track_iter->second.a, ego_rear_edge,
        0.5 * rear_track_iter->second.length, rear_safety_distance,
        safety_backward_time, lon_safety_ratio,
        &rear_safety_threshold_at_present);
    if (!rear_safety_at_present) {
      not_safe_agent_ids->emplace_back(not_safety_agent_id);
      LOG_DEBUG(
          "checkLaneChangeSafety(): Not safety for rear vehicle at present!");
      return false;
    }
    const double rear_safety_at_future = checkRearSafetyAtFuture(
        long_dis, ego_state->ego_v(), rear_track_iter->second.v,
        kDecInCheckLaneChangeSafety, rear_track_iter->second.a, ego_rear_edge,
        0.5 * rear_track_iter->second.length, rear_safety_distance,
        lon_safety_ratio,
        kConsiderTimestampInCheckLaneChangeSafety * lon_safety_ratio);
    if (!rear_safety_at_future) {
      not_safe_agent_ids->emplace_back(not_safety_agent_id);
      LOG_DEBUG(
          "checkLaneChangeSafety(): Not safety for front vehicle at future!");
      return false;
    }
    *rear_required_space = rear_safety_threshold_at_present;
  }
  return true;
}

void OvertakeRequest::selectTargetObstacleIds(
    const std::shared_ptr<KDPath>& ref_line, const Point2D ego_cart_point,
    const std::vector<TrackedObject> candidate_obs_info,
    const double search_range, const int max_target_num,
    const double ego_half_width, const double l_buffer,
    const double max_l_buffer, const bool order_reverse,
    std::vector<int>* target_tracks_ids) {
  if (!target_tracks_ids || candidate_obs_info.empty()) {
    return;
  }

  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  double ego_fx = std::cos(ego_state->ego_pose_raw().theta);
  double ego_fy = std::sin(ego_state->ego_pose_raw().theta);

  int begin_index = 0;
  int end_index = candidate_obs_info.size();
  int step = 1;
  if (order_reverse) {
    begin_index = candidate_obs_info.size() - 1;
    end_index = -1;
    step = -1;
  }
  target_tracks_ids->clear();

  auto selectObsByLateralDistance = [&](const double obs_center_l,
                                        const double obs_half_width,
                                        const double buffer) -> bool {
    const double obs_left_l = obs_center_l + obs_half_width;
    const double obs_right_l = obs_center_l - obs_half_width;
    if (obs_left_l * obs_right_l <= 0.0) {
      return true;
    } else if (obs_right_l > 0.0) {
      return obs_right_l - ego_half_width < buffer;
    } else {
      return obs_left_l + ego_half_width > -buffer;
    }
  };

  auto isTruck = [&](const TrackedObject& veh) -> bool {
    return veh.type == iflyauto::OBJECT_TYPE_TRUCK &&
           veh.length > kMinLengthForTruck && veh.width > kMinWidthForTruck;
  };

  auto getHalfWidthOnLane = [&](const TrackedObject& veh,
                                double* half_width) -> bool {
    if (!half_width) {
      return false;
    }
    Point2D obs_cart_point{0.0, 0.0};
    obs_cart_point.x =
        ego_cart_point.x + veh.center_x * ego_fx - veh.center_y * ego_fy;
    obs_cart_point.y =
        ego_cart_point.y + veh.center_x * ego_fy + veh.center_y * ego_fx;

    Point2D new_frenet_point;
    double new_lane_theta = 0.0;
    if (!ref_line->XYToSL(obs_cart_point, new_frenet_point)) {
      LOG_DEBUG("obs on ref lane failed!");
      return false;
    }
    new_lane_theta = ref_line->GetPathCurveHeading(new_frenet_point.x);
    const double delta_angle =
        std::abs(NormalizeAngle(new_lane_theta) - NormalizeAngle(veh.theta));
    *half_width = std::abs(std::cos(delta_angle)) * 0.5 * veh.width +
                  std::abs(std::sin(delta_angle)) * 0.5 * veh.length;
    return true;
  };

  for (int i = begin_index; i != end_index; i += step) {
    auto obs_info = candidate_obs_info.at(i);
    const auto id = obs_info.track_id;
    if (tracks_map_.find(id) == tracks_map_.end()) {
      continue;
    }
    double veh_half_width = 1.0;
    if (!getHalfWidthOnLane(obs_info, &veh_half_width)) {
      veh_half_width = 0.5 * obs_info.width;
    }
    const bool is_truck = isTruck(obs_info);
    const double buffer = std::min(
        max_l_buffer, is_truck ? l_buffer + kExtraLatBufferForTruck : l_buffer);
    const bool is_select =
        selectObsByLateralDistance(obs_info.l, veh_half_width, buffer) &&
        is_truck;

    if (!is_select) {
      continue;
    }

    const double distance = std::hypot(obs_info.center_x, obs_info.center_y);
    if (distance < search_range) {
      target_tracks_ids->emplace_back(id);
    }
    if (target_tracks_ids->size() == max_target_num) {
      return;
    }
  }
}

double OvertakeRequest::getSafetyDistance(
    const double low_speed_safety_distance,
    const double high_speed_safety_distance, const double vehicle_speed,
    const bool use_dynamic_safety_distance) {
  double safety_distance = low_speed_safety_distance;
  if (use_dynamic_safety_distance) {
    if (vehicle_speed <= kLowestSpeedInCheckLaneChangeSafety) {
      safety_distance = low_speed_safety_distance;
    } else if (vehicle_speed >= kHighestSpeedInCheckLaneChangeSafety) {
      safety_distance = high_speed_safety_distance;
    } else {
      safety_distance = planning_math::lerp(
          low_speed_safety_distance, kLowestSpeedInCheckLaneChangeSafety,
          high_speed_safety_distance, kHighestSpeedInCheckLaneChangeSafety,
          vehicle_speed);
    }
  }

  return safety_distance;
}

bool OvertakeRequest::checkFrontSafetyAtPresent(
    const double long_dis, const double ego_v, const double veh_v,
    const double ego_a, const double veh_a, const double ego_front_edge,
    const double veh_half_length, const double base_distance_buffer,
    const double safety_ratio, double* front_required_space) {
  const double long_diff = long_dis - veh_half_length - ego_front_edge;
  double safety_threshold = base_distance_buffer;
  if (ego_v > veh_v) {
    const double v_diff = ego_v - veh_v;
    safety_threshold += v_diff * v_diff * 0.5 / std::max(0.01, fabs(ego_a));
  }
  safety_threshold *= safety_ratio;
  if (front_required_space) {
    *front_required_space = safety_threshold;
  }
  LOG_DEBUG(
      "ego v: %f, front v: %f "
      "base dis buff: %f "
      "safety_threshold: %f "
      "long_diff: %f \n",
      ego_v, veh_v, base_distance_buffer, safety_threshold, long_diff);
  if (long_diff < safety_threshold) {
    return false;
  }
  return true;
}

bool OvertakeRequest::checkFrontSafetyAtFuture(
    const double long_dis, const double ego_v, const double veh_v,
    const double ego_a, const double veh_a, const double ego_front_edge,
    const double veh_half_length, const double base_distance_buffer,
    const double safety_ratio, const double future_time) {
  double ego_future_v = 0.0;
  const double ego_future_move_dis =
      getDrivingDistance(ego_v, ego_a, future_time, &ego_future_v);
  double veh_future_v = 0.0;
  const double veh_future_move_dis =
      getDrivingDistance(veh_v, veh_a, future_time, &veh_future_v);

  const double future_long_diff = long_dis + veh_future_move_dis -
                                  ego_future_move_dis - veh_half_length -
                                  ego_front_edge;
  const double safety_threshold = base_distance_buffer * safety_ratio;

  if (future_long_diff < safety_threshold) {
    return false;
  }
  return true;
}

bool OvertakeRequest::checkRearSafetyAtPresent(
    const double long_dis, const double ego_v, const double veh_v,
    const double ego_a, const double veh_a, const double ego_rear_edge,
    const double veh_half_length, const double base_distance_buffer,
    const double safety_backward_time, const double safety_ratio,
    double* rear_required_space) {
  const double long_diff = std::abs(long_dis) - veh_half_length - ego_rear_edge;
  double safety_threshold = base_distance_buffer;
  if (veh_v > ego_v) {
    safety_threshold += safety_backward_time * (veh_v - ego_v);
  }
  safety_threshold *= safety_ratio;
  if (rear_required_space) {
    *rear_required_space = safety_threshold;
  }

  if (long_diff < safety_threshold) {
    return false;
  }
  return true;
}

bool OvertakeRequest::checkRearSafetyAtFuture(
    const double long_dis, const double ego_v, const double veh_v,
    const double ego_a, const double veh_a, const double ego_rear_edge,
    const double veh_half_length, const double base_distance_buffer,
    const double safety_ratio, const double future_time) {
  const double ego_future_move_s = ego_v * future_time;
  const double veh_future_move_s =
      getDrivingDistance(veh_v, veh_a, future_time, nullptr);

  double safety_threshold = base_distance_buffer * safety_ratio;
  const double future_long_diff = std::abs(long_dis) + ego_future_move_s -
                                  veh_future_move_s - veh_half_length -
                                  ego_rear_edge;

  if (future_long_diff < safety_threshold) {
    return false;
  }
  return true;
}

double OvertakeRequest::getDrivingDistance(const double v, const double a,
                                           const double t, double* v_out) {
  double s = 0.0;
  double v_t = 0.0;
  if (v + a * t > 0.0) {
    s = v * t + 0.5 * a * t * t;
    v_t = v + a * t;
  } else {
    if (fabs(a - 0.0) < 1e-3) {
      s = v * t;
      v_t = v;
    } else {
      s = v * v * 0.5 / fabs(a);
      v_t = 0.0;
    }
  }
  if (v_out) {
    *v_out = v_t;
  }
  return s;
}

bool OvertakeRequest::isCancelOverTakingLaneChange(int lc_state) {
  auto leading_vehicle_iter = tracks_map_.find(overtake_vehicle_id_);
  if (leading_vehicle_iter != tracks_map_.end()) {
    const double overtake_lane_change_vehicle_speed =
        leading_vehicle_iter->second.v;
    const double lateral_distance = planning_init_point_.frenet_state.r;
    double front_leading_vehivle_long_distance = kDefaultFrontObstacleDistance;
    const auto lane_tracks_manager =
        session_->environmental_model().get_lane_tracks_manager();
    std::vector<TrackedObject> ego_front_vehicle_id_array;
    ego_front_vehicle_id_array = lane_tracks_manager->front_tracks_c();
    double leading_vehicle_lateral_speed = leading_vehicle_iter->second.v_lat;
    double leading_vehicle_lateral_dis = leading_vehicle_iter->second.l;

    if (ego_front_vehicle_id_array.size() >= 1) {
      int ego_front_vehicle_id = ego_front_vehicle_id_array.at(0).track_id;
      if (ego_front_vehicle_id != -1) {
        auto front_leading_vehicle_iter =
            tracks_map_.find(ego_front_vehicle_id_array.at(0).track_id);
        if (overtake_vehicle_id_ == ego_front_vehicle_id) {
          if (ego_front_vehicle_id_array.size() == 1) {
            front_leading_vehicle_iter = tracks_map_.end();
          } else {
            front_leading_vehicle_iter =
                tracks_map_.find(ego_front_vehicle_id_array.at(1).track_id);
          }
        }
        if (front_leading_vehicle_iter != tracks_map_.end()) {
          front_leading_vehivle_long_distance =
              front_leading_vehicle_iter->second.d_rel;
          LOG_DEBUG("ego vehicle front_leading_vehicle id: %d \n",
                    front_leading_vehicle_iter->second.track_id);
        }
      }
    }

    if (((lc_state == kLaneChangePropose) &&
         ((request_type_ == LEFT_CHANGE &&
           leading_vehicle_lateral_speed >
               kCancelOverTakeLnChgLeadVehLatSpdThold &&
           leading_vehicle_lateral_dis >=
               kCancelOverTakeLnChgLeadVehLatDstThold) ||
          (request_type_ == RIGHT_CHANGE &&
           leading_vehicle_lateral_speed >
               kCancelOverTakeLnChgLeadVehLatSpdThold &&
           leading_vehicle_lateral_dis <=
               -kCancelOverTakeLnChgLeadVehLatDstThold))) ||
        (((lc_state == kLaneChangeExecution ||
           lc_state == kLaneChangeComplete) &&
          front_leading_vehivle_long_distance >
              kCancelOverTakeLnChgFrtLeadVehLonDst) &&
         ((request_type_ == LEFT_CHANGE &&
           lateral_distance <= kCancelOverTakeLnChgEgoVehLatDstThold &&
           leading_vehicle_lateral_speed >
               kCancelOverTakeLnChgLeadVehLatSpdThold &&
           leading_vehicle_lateral_dis >=
               kCancelOverTakeLnChgLeadVehLatDstThold &&
           !false) ||
          (request_type_ == RIGHT_CHANGE &&
           lateral_distance >= -kCancelOverTakeLnChgEgoVehLatDstThold &&
           leading_vehicle_lateral_speed >
               kCancelOverTakeLnChgLeadVehLatSpdThold &&
           leading_vehicle_lateral_dis <=
               -kCancelOverTakeLnChgLeadVehLatDstThold &&
           !false)))) {
      // HACK：待前轮是否跨线接口ready来替代上述false
      LOG_DEBUG(
          "Cancel OverTakeLaneChange Dir: %d, LaneChangeProgress: %d "
          "OverTake Vehicle LatSpd: %f "
          "Ego Lat Distance: %f "
          "Front Leading Vehicle Distance: %f \n",
          request_type_, lc_state, leading_vehicle_lateral_speed,
          lateral_distance, front_leading_vehivle_long_distance);
      return true;
    }
    std::vector<TrackedObject> ego_left_front_vehicle_array;
    std::vector<TrackedObject> ego_right_front_vehicle_array;
    double ego_left_front_leading_vehicle_min_speed =
        kCancelOverTakeLnChgTargetLaneVehDftSpd;
    double ego_left_front_leading_vehivle_long_distance =
        kDefaultFrontObstacleDistance;
    double ego_right_front_leading_vehicle_min_speed =
        kCancelOverTakeLnChgTargetLaneVehDftSpd;
    double ego_right_front_leading_vehivle_long_distance =
        kDefaultFrontObstacleDistance;
    ego_left_front_vehicle_array = lane_tracks_manager_->front_tracks_l();
    ego_right_front_vehicle_array = lane_tracks_manager_->front_tracks_r();
    if (ego_left_front_vehicle_array.size() >= 1) {
      int ego_left_front_vehicle_id =
          ego_left_front_vehicle_array.at(0).track_id;
      auto left_front_leading_vehicle_iter =
          tracks_map_.find(ego_left_front_vehicle_array.at(0).track_id);
      if (overtake_vehicle_id_ == ego_left_front_vehicle_id) {
        if (ego_left_front_vehicle_array.size() == 1) {
          left_front_leading_vehicle_iter = tracks_map_.end();
        } else {
          left_front_leading_vehicle_iter =
              tracks_map_.find(ego_left_front_vehicle_array.at(1).track_id);
        }
      }
      if (left_front_leading_vehicle_iter != tracks_map_.end()) {
        ego_left_front_leading_vehicle_min_speed =
            left_front_leading_vehicle_iter->second.v;
        ego_left_front_leading_vehivle_long_distance =
            left_front_leading_vehicle_iter->second.d_rel;
        LOG_DEBUG("ego_left_front_leading_vehivle_long_distance: %f ",
                  ego_left_front_leading_vehivle_long_distance);
      }
    }
    if (request_type_ == LEFT_CHANGE) {
      double speed_threshold = kCancelOverTakeLnChgTargetLaneVehHighSpdDiff;
      if (ego_left_front_leading_vehivle_long_distance >=
          kCancelOverTakeLnChgTargetLaneVehHighDst) {
        speed_threshold = kCancelOverTakeLnChgTargetLaneVehLowSpdDiff;
      } else if (ego_left_front_leading_vehivle_long_distance <=
                 kCancelOverTakeLnChgTargetLaneVehLowDst) {
        speed_threshold = kCancelOverTakeLnChgTargetLaneVehHighSpdDiff;
      } else {
        speed_threshold =
            planning_math::lerp(kCancelOverTakeLnChgTargetLaneVehHighSpdDiff,
                                kCancelOverTakeLnChgTargetLaneVehLowDst,
                                kCancelOverTakeLnChgTargetLaneVehLowSpdDiff,
                                kCancelOverTakeLnChgTargetLaneVehHighDst,
                                ego_left_front_leading_vehivle_long_distance);
      }
      if ((ego_left_front_leading_vehicle_min_speed <
           overtake_lane_change_vehicle_speed + speed_threshold) &&
          ego_left_front_leading_vehivle_long_distance <=
              kCancelOverTakeLnChgTargetLaneVehDstThold) {
        target_lane_exist_slow_front_veh_frame_num_ += 1;
      } else {
        target_lane_exist_slow_front_veh_frame_num_ = 0;
      }
    }
    if (request_type_ == LEFT_CHANGE &&
        target_lane_exist_slow_front_veh_frame_num_ >= 2 && !false) {
      // HACK：false待前轮压线接口ready后替代
      LOG_DEBUG(
          "Cancel OverTakeLaneChange Dir: %d, LaneChangeProgress: %d "
          "OverTake Vehicle LonSpd: %f, Left Lane Target Speed: %f "
          "Ego Lat Distance: %f, left lane exist slow front veh frame num: %d "
          "\n",
          request_type_, lc_state, overtake_lane_change_vehicle_speed,
          ego_left_front_leading_vehicle_min_speed, lateral_distance,
          target_lane_exist_slow_front_veh_frame_num_);
      return true;
    }

    if (ego_right_front_vehicle_array.size() >= 1) {
      int ego_right_front_vehicle_id =
          ego_right_front_vehicle_array.at(0).track_id;
      auto right_front_leading_vehicle_iter =
          tracks_map_.find(ego_right_front_vehicle_array.at(0).track_id);
      if (overtake_vehicle_id_ == ego_right_front_vehicle_id) {
        if (1 == ego_right_front_vehicle_array.size()) {
          right_front_leading_vehicle_iter = tracks_map_.end();
        } else {
          right_front_leading_vehicle_iter =
              tracks_map_.find(ego_right_front_vehicle_array.at(1).track_id);
        }
      }
      if (right_front_leading_vehicle_iter != tracks_map_.end()) {
        ego_right_front_leading_vehicle_min_speed =
            right_front_leading_vehicle_iter->second.v;
        ego_right_front_leading_vehivle_long_distance =
            right_front_leading_vehicle_iter->second.d_rel;
        LOG_DEBUG("right front leading vehicle long distance: %f ",
                  ego_right_front_leading_vehivle_long_distance);
      }
    }
    if (request_type_ == RIGHT_CHANGE) {
      double speed_threshold = kCancelOverTakeLnChgTargetLaneVehHighSpdDiff;
      if (ego_right_front_leading_vehivle_long_distance >=
          kCancelOverTakeLnChgTargetLaneVehHighDst) {
        speed_threshold = kCancelOverTakeLnChgTargetLaneVehLowSpdDiff;
      } else if (ego_right_front_leading_vehivle_long_distance <=
                 kCancelOverTakeLnChgTargetLaneVehLowDst) {
        speed_threshold = kCancelOverTakeLnChgTargetLaneVehHighSpdDiff;
      } else {
        speed_threshold =
            planning_math::lerp(kCancelOverTakeLnChgTargetLaneVehHighSpdDiff,
                                kCancelOverTakeLnChgTargetLaneVehLowDst,
                                kCancelOverTakeLnChgTargetLaneVehLowSpdDiff,
                                kCancelOverTakeLnChgTargetLaneVehHighDst,
                                ego_right_front_leading_vehivle_long_distance);
      }
      if ((ego_right_front_leading_vehicle_min_speed <
           overtake_lane_change_vehicle_speed + speed_threshold) &&
          ego_right_front_leading_vehivle_long_distance <=
              kCancelOverTakeLnChgTargetLaneVehDstThold) {
        target_lane_exist_slow_front_veh_frame_num_ += 1;
      } else {
        target_lane_exist_slow_front_veh_frame_num_ = 0;
      }
    }
    if (request_type_ == RIGHT_CHANGE && !false &&
        (target_lane_exist_slow_front_veh_frame_num_ >= 2)) {
      LOG_DEBUG(
          "Cancel OverTakeLaneChange Dir: %d, LaneChangeProgress: %d "
          "OverTake Vehicle LonSpd: %f, Left Lane Target Speed: %f "
          "Ego Lat Distance: %f, right lane exist slow front veh frame num: %d "
          "\n",
          request_type_, lc_state, overtake_lane_change_vehicle_speed,
          ego_left_front_leading_vehicle_min_speed, lateral_distance,
          target_lane_exist_slow_front_veh_frame_num_);
      return true;
    }
  }
  return false;
}

void OvertakeRequest::Reset() {
  overtake_vehicle_id_ = -1;  // 初始化为有关无效的障碍物track_id
  overtake_vehicle_speed_ = 0.0;
  overtake_count_ = 0;
  target_lane_exist_slow_front_veh_frame_num_ = 0;
}

}  // namespace planning
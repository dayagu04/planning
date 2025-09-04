#include "overtake_lane_change_request.h"

#include <glog/logging.h>
#include <math.h>

#include <cassert>
#include <cmath>
#include <complex>
#include <limits>
#include <vector>

#include "agent/agent.h"
#include "common.pb.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "define/geometry.h"
#include "ego_state_manager.h"
#include "log.h"
#include "planning_context.h"
#include "tasks/behavior_planners/lane_change_decider/lane_change_requests/lane_change_request.h"
#include "tracked_object.h"
#include "virtual_lane_manager.h"

namespace planning {
using namespace planning_math;

namespace {
constexpr double kStaticVehicleThreshold = 1.0;
constexpr int kOvertakeCountThreshold = 200;
constexpr int kOvertakeRightLaneCountThreshold = 300;
constexpr int kExtraOvertakeCountForRainMode = 150;
constexpr double kDisancebetweenRoadSplitAndRampAllowError = 5.0;
constexpr double kSplitTriggleDistance = 3000.0;
constexpr double kOvertakeMaintainCountTtcThreshold = 48.0;
constexpr double kOvertakeLeadingVehicleDistanceThreshold = 130.0;
constexpr double kOvertakeEgoHighSpeedThreshold = 25.00;           // 90km/h
constexpr double kOvertakeHighSpeedDiffThreshold = 1.39;           // 5km/h
constexpr double kOvertakeHighSpeedDiffThresholdRainMode = 3.33;   // 12km/h
constexpr double kOvertakeLowSpeedDiffThreshold = 1.39;            // 5km/h
constexpr double kOvertakeLowSpeedDiffThresholdRainMode = 2.5;     // 9km/h
constexpr double kOvertakeMaintainCountSpeedDiffThreshold = 1.39;  // 5km/h
constexpr double kOvertakeUpdateCountRatioThreshold = 0.10;
constexpr double kOvertakeUpdateCountRatioThresholdRainMode = 0.15;
constexpr double kOvertakeMaintainCountRatioThreshold = 0.05;
constexpr double kOvertakeUpdateCountSpeedRatioThreshold = 0.4;
constexpr int kOvertakeUpdateCountTruckTypeThreshold = 8;
constexpr int kOvertakeUpdateCountCarTypeThreshold = 4;
constexpr double kOvertakeLeadingVehicleHighSpeedThreshold = 22.22;  // 80km/h
constexpr double kOvertakeLeadingVehicleLowSpeedThreshold = 16.67;   // 60km/h
constexpr double kOvertakeLeadingVehicleHighSpeedDiffThreshold = 1.94;  // 7km/h
constexpr double kOvertakeLeadingVehicleLowSpeedDiffThreshold = 3.89;  // 14km/h
constexpr double kOvertakeLeadingVehicleRadicalLowSpeedDiffThreshold = 2.78;  // 10km/h

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
constexpr double kDefaultLeadOneConsiderRange = 120.0;

}  // namespace
// class: OvertakeRequest
OvertakeRequest::OvertakeRequest(
    const EgoPlanningConfigBuilder* config_builder,
    planning::framework::Session* session,
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
    std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr)
    : LaneChangeRequest(session, virtual_lane_mgr, lane_change_lane_mgr) {
  base_frenet_coord_ = std::make_shared<KDPath>();
  config_ = config_builder->cast<EgoPlanningConfig>();
}

void OvertakeRequest::Update(int lc_status) {
  std::cout << "OvertakeRequest::Update::coming overtake lane change request"
            << std::endl;

  lateral_obstacle_ = session_->environmental_model().get_lateral_obstacle();
  lane_tracks_manager_ =
      session_->environmental_model().get_lane_tracks_manager();


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
  const auto& llane = virtual_lane_mgr_->get_left_lane();
  const auto& rlane = virtual_lane_mgr_->get_right_lane();
  if (llane != nullptr) {
    left_reference_path_ = reference_path_mgr->get_reference_path_by_lane(
        llane->get_virtual_id(), false);
    ILOG_DEBUG << "OvertakeRequest::Update: for left_lane: update:" << llane->get_virtual_id();
  } else {
    left_reference_path_ = nullptr;
  }

  if (rlane != nullptr) {
    right_reference_path_ = reference_path_mgr->get_reference_path_by_lane(
        rlane->get_virtual_id(), false);
    ILOG_DEBUG << "OvertakeRequest::Update: for right_lane: update " << rlane->get_virtual_id();
  } else {
    right_reference_path_ = nullptr;
  }

  double current_timestamp = IflyTime::Now_s();
  enable_l_ = llane && left_reference_path_ && llane->get_lane_type() != iflyauto::LANETYPE_OPPOSITE;
  enable_r_ = rlane && right_reference_path_ && rlane->get_lane_type() != iflyauto::LANETYPE_OPPOSITE;
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
        ILOG_DEBUG << "Left LC invalid for rebounce time!";
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
        ILOG_DEBUG << "Left LC invalid for rebounce time!";
        enable_r_ = false;
      }
    }
  }
  JSON_DEBUG_VALUE("enable_l_", enable_l_);
  JSON_DEBUG_VALUE("enable_r_", enable_r_);

  // updateLaneChangeSafety(left_reference_path_, right_reference_path_);
  // JSON_DEBUG_VALUE("is_left_lane_change_safe_", is_left_lane_change_safe_);
  // JSON_DEBUG_VALUE("is_right_lane_change_safe_", is_right_lane_change_safe_);
  setLaneChangeRequestByFrontSlowVehcile(lc_status);
  ILOG_DEBUG << "request_type_:" << request_type_ << "turn_signal:" << turn_signal_;
}

void OvertakeRequest::setLaneChangeRequestByFrontSlowVehcile(int lc_status) {
  const auto& route_info_output =
      session_->environmental_model().get_route_info()->get_route_info_output();
  const auto& feasible_lane_sequence = route_info_output.mlc_decider_route_info.feasible_lane_sequence;
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const int current_lane_virtual_id =
      virtual_lane_mgr_->current_lane_virtual_id();
  const double min_distance_nearby_split_to_surpress_specific_direction_overtake =
      config_
          .minimum_distance_nearby_split_to_surpress_specific_direction_overtake;
  // const double max_pass_merge_distance_to_surpress_overtake_lane_change =
  //     virtual_lane_mgr_->dis_threshold_to_last_merge_point();
  double distance_to_first_road_split = NL_NMAX;
  SplitDirection first_split_direction = SplitDirection::SPLIT_NONE;
  double dis_to_first_merge = NL_NMAX;
  SplitDirection first_merge_direction = SplitDirection::SPLIT_NONE;
  // double sum_dis_to_last_merge_point =
  //     route_info_output.sum_dis_to_last_merge_point;
  const double dis_threshold_to_merged_point =
      virtual_lane_mgr_->dis_threshold_to_merged_point();
  const auto& split_region_info_list = route_info_output.split_region_info_list;
  const auto& merge_region_info_list = route_info_output.merge_region_info_list;
  if (!split_region_info_list.empty()) {
    if (split_region_info_list[0].is_valid) {
      distance_to_first_road_split = split_region_info_list[0].distance_to_split_point;
      first_split_direction = split_region_info_list[0].split_direction;
      if (distance_to_first_road_split <= config_.minimum_distance_nearby_ramp_to_surpress_overtake_lane_change &&
          split_region_info_list[0].is_ramp_split) {
        return;
      }
    }
  }
  if (!merge_region_info_list.empty()) {
    if (merge_region_info_list[0].is_valid) {
      dis_to_first_merge = merge_region_info_list[0].distance_to_split_point;
      first_merge_direction = merge_region_info_list[0].split_direction;
    }
  }
  if (distance_to_first_road_split >= dis_to_first_merge) {
    distance_to_first_road_split = NL_NMAX;
    first_split_direction = SplitDirection::SPLIT_NONE;
  } else {
    dis_to_first_merge = NL_NMAX;
    first_merge_direction = SplitDirection::SPLIT_NONE;
  }
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

  auto lane_nums_msg = clane->get_lane_nums();
  int right_lane_nums = 0;
  int left_lane_nums = 0;
  Point2D ego_frenet;
  if (clane != nullptr) {
    const auto& cur_lane_frenet_coord = clane->get_lane_frenet_coord();
    if (cur_lane_frenet_coord != nullptr) {
      if (cur_lane_frenet_coord->XYToSL(
              {ego_state->ego_pose().x, ego_state->ego_pose().y}, ego_frenet)) {
        auto iter =
            std::find_if(lane_nums_msg.begin(), lane_nums_msg.end(),
                        [&ego_frenet](const iflyauto::LaneNumMsg& lane_num) {
                          return lane_num.begin <= ego_frenet.x && lane_num.end > ego_frenet.x;
                        });
        if (iter != lane_nums_msg.end()) {
          left_lane_nums = iter->left_lane_num;
          right_lane_nums = iter->right_lane_num;
        } else {
          return;
        }
      }
    } else {
      return;
    }
  } else {
    return;
  }

  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();

  const auto& cipv_info = session_->planning_context().cipv_decider_output();
  const int32_t cipv_id = cipv_info.cipv_id();
  const auto& agent_manager =
      session_->environmental_model().get_dynamic_world()->agent_manager();
  const agent::Agent* agent = agent_manager->GetAgent(cipv_id);

  // 无效的track_id暂时赋值为-1
  if ((agent != nullptr && agent->agent_id() == -1) || agent == nullptr ||
      cipv_info.relative_s() > kDefaultLeadOneConsiderRange) {
    ILOG_DEBUG << "not exist stable leading vehicle";
    overtake_count_ = 0;
    Finish();
    return;
  }
  auto base_lane =
      virtual_lane_mgr_->get_lane_with_virtual_id(base_lane_virtual_id);

  if (base_lane == nullptr) {
    ILOG_DEBUG << "base lane not exist";
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
    ILOG_DEBUG << "fail to get ego position on base lane";
    Finish();
    overtake_count_ = 0;
    return;
  }

  const double ego_front_edge = vehicle_param.front_edge_to_rear_axle;
  const double long_diff =
      cipv_info.relative_s() - ego_front_edge - agent->length() * 0.5;

  const double ego_speed = ego_state->ego_v();
  const double reference_speed = ego_state->ego_v_cruise();
  const bool is_rain_mode =
      false;  // hack：当前planning中没有区分下雨天、不下雨场景
  const bool is_satisfy_update_condition =
      isSatisfyOvertakeCountUpdateCondition(agent, ego_speed, reference_speed,
                                            long_diff, is_rain_mode);
  const bool is_satisfy_maintain_condition =
      isSatisfyOvertakeCountMaintainCondition(agent, reference_speed, long_diff,
                                              is_rain_mode);

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
    updateOvertakeCount(agent, ego_speed, reference_speed, right_count_thres);
  } else if (!is_satisfy_maintain_condition) {
    overtake_count_ = 0;
  }

  ILOG_DEBUG << "overtake_count :" << overtake_count_;
  JSON_DEBUG_VALUE("overtake_count_", overtake_count_);

  if (overtake_count_ < left_count_thres) {
    return;
  }

  double left_route_traffic_speed = 0.0;
  double right_route_traffic_speed = 0.0;

  updateRouteTrafficSpeed(true, &left_route_traffic_speed);
  updateRouteTrafficSpeed(false, &right_route_traffic_speed);
  const double leading_vehicle_speed = agent->speed();

  JSON_DEBUG_VALUE("left_route_traffic_speed", left_route_traffic_speed);
  JSON_DEBUG_VALUE("right_route_traffic_speed", right_route_traffic_speed);

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

  bool left_lane_is_on_navigation_route = true;
  if (feasible_lane_sequence.size() > 0) {
    int current_lane_order_num = left_lane_nums + 1;
    int target_lane_order_num = current_lane_order_num - 1;
    if (std::find(feasible_lane_sequence.begin(), feasible_lane_sequence.end(), target_lane_order_num) == feasible_lane_sequence.end()) {
      left_lane_is_on_navigation_route = false;
    }
  }

  bool right_lane_is_on_navigation_route = true;
  if (feasible_lane_sequence.size() > 0) {
    int current_lane_order_num = left_lane_nums + 1;
    int target_lane_order_num = current_lane_order_num + 1;
    if (std::find(feasible_lane_sequence.begin(), feasible_lane_sequence.end(), target_lane_order_num) == feasible_lane_sequence.end()) {
      right_lane_is_on_navigation_route = false;
    }
  }

#ifdef X86
  bool trigger_left_overtake = false;
  bool trigger_right_overtake = false;
  // const bool is_trigger_left = (is_left_overtake && is_left_lane_change_safe_);
  const bool is_trigger_left =
      is_left_overtake && left_lane_is_on_navigation_route;

  static int counter_left = 0;
  if (!is_trigger_left) {
    counter_left = 0;
  } else {
    counter_left++;
  }
  if (counter_left >= 5) {
    trigger_left_overtake = true;
  }

  // const bool is_trigger_right =
  //     (is_right_overtake && is_right_lane_change_safe_);
  const bool is_trigger_right =
      is_right_overtake && right_lane_is_on_navigation_route;
  static int counter_right = 0;
  if (!is_trigger_right) {
    counter_right = 0;
  } else {
    counter_right++;
  }
  if (counter_right >= 5) {
    trigger_right_overtake = true;
  }
#else
  const bool trigger_left_overtake = checkOvertakeTrigger(
      current_time, is_left_overtake && left_lane_is_on_navigation_route,
      &left_overtake_valid_timestamp_);

  const bool trigger_right_overtake = checkOvertakeTrigger(
      current_time, is_right_overtake && right_lane_is_on_navigation_route,
      &right_overtake_valid_timestamp_);
#endif

  JSON_DEBUG_VALUE("left_lane_is_on_navigation_route", left_lane_is_on_navigation_route);
  JSON_DEBUG_VALUE("right_lane_is_on_navigation_route", right_lane_is_on_navigation_route);

  JSON_DEBUG_VALUE("trigger_left_overtake", trigger_left_overtake);
  JSON_DEBUG_VALUE("trigger_right_overtake", trigger_right_overtake);
  if (trigger_left_overtake) {
    if (request_type_ != LEFT_CHANGE) {
      target_lane_virtual_id_tmp = origin_lane_virtual_id_ - 1;
      GenerateRequest(LEFT_CHANGE);
      set_target_lane_virtual_id(target_lane_virtual_id_tmp);
      LOG_DEBUG(
          "[OvertakeRequest::update] Ask for overtake changing lane to left "
          "\n");
    }
    if (!IsDashEnoughForRepeatSegments(
            LEFT_CHANGE, origin_lane_virtual_id_,
            static_cast<StateMachineLaneChangeStatus>(lc_status)) &&
        request_type_ != NO_CHANGE &&
        (lc_status == kLaneKeeping || lc_status == kLaneChangePropose ||
         (lc_status == kLaneChangeCancel &&
          (lane_change_lane_mgr_->has_origin_lane() &&
           lane_change_lane_mgr_->is_ego_on(olane))))) {
      Finish();
      set_target_lane_virtual_id(current_lane_virtual_id);
      ILOG_DEBUG << "[OvertakeRequest::update] " << __FUNCTION__ << ":" << __LINE__
                 << " finish request, dash not enough";
    } else {
      overtake_vehicle_id_ = agent->agent_id();
      overtake_vehicle_speed_ = leading_vehicle_speed;
      ILOG_DEBUG << "overtake_vehicle_id_: " << overtake_vehicle_id_ << "overtake_vehicle_speed_: " << overtake_vehicle_speed_;
    }
  } else if (trigger_right_overtake && overtake_count_ >= right_count_thres) {
    if (request_type_ != RIGHT_CHANGE) {
      target_lane_virtual_id_tmp = origin_lane_virtual_id_ + 1;
      GenerateRequest(RIGHT_CHANGE);
      set_target_lane_virtual_id(target_lane_virtual_id_tmp);
      LOG_DEBUG(
          "[OvertakeRequest::update] Ask for overtake changing lane to right "
          "\n");
    }
    if (!IsDashEnoughForRepeatSegments(
            RIGHT_CHANGE, origin_lane_virtual_id_,
            static_cast<StateMachineLaneChangeStatus>(lc_status)) &&
        request_type_ != NO_CHANGE &&
        (lc_status == kLaneKeeping || lc_status == kLaneChangePropose ||
         (lc_status == kLaneChangeCancel &&
          (lane_change_lane_mgr_->has_origin_lane() &&
           lane_change_lane_mgr_->is_ego_on(olane))))) {
      Finish();
      set_target_lane_virtual_id(current_lane_virtual_id);
      ILOG_DEBUG << "[OvertakeRequest::update] " << __FUNCTION__ << ":" << __LINE__ << " finish request, dash not enough";
    } else {
      overtake_vehicle_id_ = agent->agent_id();
      overtake_vehicle_speed_ = leading_vehicle_speed;
      ILOG_DEBUG << "overtake_vehicle_id_: " << overtake_vehicle_id_ << "overtake_vehicle_speed_: " << overtake_vehicle_speed_;
    }
  } else if (request_type_ != NO_CHANGE &&
             (lane_change_lane_mgr_->has_origin_lane() &&
              lane_change_lane_mgr_->is_ego_on(olane))) {
    Finish();
    set_target_lane_virtual_id(current_lane_virtual_id);
    ILOG_DEBUG << "[OvertakeRequest::update] " << __FUNCTION__ << ":" << __LINE__ << " finish request, !trigger_left_overtake and !trigger_right_overtake";
  }
}

bool OvertakeRequest::isSatisfyOvertakeCountUpdateCondition(
    const agent::Agent* leading_agent, const double ego_speed,
    const double reference_speed, const double leading_vehicle_dist,
    const bool rain_mode) {
  if (leading_agent->agent_id() == -1) {
    return false;
  }

  if (leading_vehicle_dist > kOvertakeLeadingVehicleDistanceThreshold) {
    return false;
  }

  const double speed_diff = std::max(reference_speed - leading_agent->speed(),
                                     kOvertakeMinSpeedDiffThreshold);
  const double front_ttc = (speed_diff > 0.0)
                               ? (leading_vehicle_dist / speed_diff)
                               : std::numeric_limits<double>::max();
  std::cout << "leading_vehicle_speed: " << leading_agent->speed()
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
    const agent::Agent* leading_agent, const double reference_speed,
    const double leading_vehicle_dist, const bool rain_mode) {
  if (leading_agent->agent_id() == -1) {
    return false;
  }

  if (leading_vehicle_dist > kOvertakeLeadingVehicleDistanceThreshold) {
    return false;
  }
  const double speed_diff = std::max(reference_speed - leading_agent->speed(),
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

void OvertakeRequest::updateOvertakeCount(const agent::Agent* leading_agent,
                                          const double ego_speed,
                                          const double reference_speed,
                                          const int max_count_thres) {
  int type_value = kOvertakeUpdateCountCarTypeThreshold;
  if (leading_agent->type() == agent::AgentType::TRUCK) {
    type_value = kOvertakeUpdateCountTruckTypeThreshold;
  }
  const int curr_count = round((reference_speed - leading_agent->speed()) *
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
  const auto& front_tracks_l =
      lane_tracks_manager_->front_tracks_l();
  const auto& front_tracks_r =
      lane_tracks_manager_->front_tracks_r();

  std::vector<std::shared_ptr<FrenetObstacle>> side_front_obstacle_array;
  if (is_left) {
    for (const auto& tr : front_tracks_l) {
      if (!(tr->obstacle()->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
        continue;
      }
      if (tr->d_s_rel() > kDefaultLeadOneConsiderRange) {
        continue;
      }
      side_front_obstacle_array.push_back(tr);
    }
  } else {
    for (const auto& tr : front_tracks_r) {
      if (!(tr->obstacle()->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
        continue;
      }
      if (tr->d_s_rel() > kDefaultLeadOneConsiderRange) {
        continue;
      }
      side_front_obstacle_array.push_back(tr);
    }
  }

  double first_vehicle_speed = std::numeric_limits<double>::max();
  double second_vehicle_speed = std::numeric_limits<double>::max();
  for (const auto& front_obstacle : side_front_obstacle_array) {
    if (std::numeric_limits<double>::max() == first_vehicle_speed) {
      first_vehicle_speed = front_obstacle->velocity();
    } else if (std::numeric_limits<double>::max() == second_vehicle_speed) {
      second_vehicle_speed = front_obstacle->velocity();
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
  const auto &lane_change_style = session_->environmental_model()
                                  .get_local_view()
                                  .function_state_machine_info.pilot_req.lane_change_style;
  double speed_threshold = config_.overtake_standard_left_lane_change_speed_threshold;
  if (lane_change_style == iflyauto::LANE_CHANGE_STYLE_ASSISTIVE) {
    speed_threshold = config_.overtake_soft_lane_change_speed_threshold;
  } else if (lane_change_style == iflyauto::LANE_CHANGE_STYLE_AGILE) {
    speed_threshold = config_.overtake_radical_lane_change_speed_threshold;
  } else {
    if (is_left) {
      speed_threshold = config_.overtake_standard_left_lane_change_speed_threshold;
    } else {
      speed_threshold = config_.overtake_standard_right_lane_change_speed_threshold;
    }
  }
  // if (leading_vehicle_speed >= kOvertakeLeadingVehicleHighSpeedThreshold) {
  //   speed_threshold = kOvertakeLeadingVehicleHighSpeedDiffThreshold;
  // } else if (leading_vehicle_speed <=
  //           kOvertakeLeadingVehicleLowSpeedThreshold) {
  //   speed_threshold = kOvertakeLeadingVehicleLowSpeedDiffThreshold;
  // } else {
  //   speed_threshold = planning_math::lerp(
  //       kOvertakeLeadingVehicleLowSpeedDiffThreshold,
  //       kOvertakeLeadingVehicleLowSpeedThreshold,
  //       kOvertakeLeadingVehicleHighSpeedDiffThreshold,
  //       kOvertakeLeadingVehicleHighSpeedThreshold, leading_vehicle_speed);
  // }
  const int total_lane_nums = left_lane_nums + right_lane_nums + 1;
  const bool inhibit_extra_speed =
      (total_lane_nums >= kOvertakeInhibitExtraSpeedTotalLaneNum &&
       0 == left_lane_nums);
  // if (!is_left && !inhibit_extra_speed) {
  //   speed_threshold += kOvertakeRightTurnExtraSpeedThreshold;
  // }
  JSON_DEBUG_VALUE("speed_threshold", speed_threshold);

  // 当总车道数不少于3时，抑制向最右侧车道触发超车变道
  if (total_lane_nums >= kOvertakeInhibitExtraSpeedTotalLaneNum && !is_left &&
      1 == right_lane_nums) {
    return false;
  }
  const double left_speed_diff = lane_traffic_speed - leading_vehicle_speed;
  const bool is_overtake = (left_speed_diff > speed_threshold);
  return is_overtake;
}

// void OvertakeRequest::updateLaneChangeSafety(
//     const std::shared_ptr<ReferencePath>& left_ref_line,
//     const std::shared_ptr<ReferencePath>& right_ref_line) {
//   is_left_lane_change_safe_ = false;
//   is_right_lane_change_safe_ = false;

//   std::vector<int> risk_agent_id_for_lane_change;
//   const double safety_extra_forward_distance = 0.0;
//   const double safety_extra_backward_distance = kLaneChangeSafetyDistanceBuffer;
//   const double safety_forward_time = kLaneChangeSafetyForwardTime;
//   const double safety_backward_time = kLaneChangeSafetyBackwardTime;
//   const double safety_ratio = 1.0;

//   double front_required_space = 0.0;
//   double rear_required_space = 0.0;
//   if (enable_l_ && left_ref_line) {
//     is_left_lane_change_safe_ = checkLaneChangeSafety(
//         left_ref_line, safety_extra_forward_distance,
//         safety_extra_backward_distance, safety_forward_time,
//         safety_backward_time, true, safety_ratio, safety_ratio, true,
//         &risk_agent_id_for_lane_change, &front_required_space,
//         &rear_required_space);
//   }
//   if (enable_r_ && right_ref_line) {
//     is_right_lane_change_safe_ = checkLaneChangeSafety(
//         right_ref_line, safety_extra_forward_distance,
//         safety_extra_backward_distance, safety_forward_time,
//         safety_backward_time, true, safety_ratio, safety_ratio, true,
//         &risk_agent_id_for_lane_change, &front_required_space,
//         &rear_required_space);
//   }
// }

bool OvertakeRequest::checkLeftLaneChangeValid(
    const std::shared_ptr<ReferencePath>& ref_line, const bool is_left) {
  if (!checkLeftLaneChangeValidByObjects(ref_line)) {
    ILOG_DEBUG << "left invalid since objects";
    return false;
  }
  // if (!checkLaneChangeValidBySuprsSignal(is_left)) {
  //   return false;
  // }
  return true;
}

bool OvertakeRequest::checkRightLaneChangeValid(
    const std::shared_ptr<ReferencePath>& ref_line, const bool is_left) {
  if (!checkRightLaneChangeValidByObjects(ref_line)) {
    ILOG_DEBUG << "right invalid since objects";
    return false;
  }
  // if (!checkLaneChangeValidBySuprsSignal(is_left)) {
  //   return false;
  // }
  return true;
}

bool OvertakeRequest::checkLeftLaneChangeValidByObjects(
    const std::shared_ptr<ReferencePath>& ref_line) {
  if (!ref_line) {
    return false;
  }
  const auto& tracks_map = lateral_obstacle_->tracks_map();
  const auto& target_lane_coord_ptr = ref_line->get_frenet_coord();
  Point2D ego_frenet_point_in_left_lane;
  Point2D ego_cart_point{planning_init_point_.lat_init_state.x(),
                         planning_init_point_.lat_init_state.y()};
  if (!target_lane_coord_ptr->XYToSL(ego_cart_point,
                                     ego_frenet_point_in_left_lane)) {
    ILOG_DEBUG << "Enmergency error! Ego Pose Cart2SL in left lane failed!";
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
  const std::vector<std::shared_ptr<planning::FrenetObstacle>>& front_tracks =
      lateral_obstacle_->front_tracks();

  // if (!left_potensial_objects.size()) {
  //   return true;
  // }
  // for (const auto& id : left_potensial_objects) {
  //   if (tracks_map_[id].d_rel < -kPotensialObjectLonRange ||
  //       tracks_map_[id].d_rel > kPotensialObjectLonRange ||
  //       tracks_map_[id].l > potensial_max_l || tracks_map_[id].l < 0.0) {
  //     continue;
  //   }
  //   ++potensial_counter;
  //   if (potensial_counter >= kPotensialObjectNum) {
  //     return false;
  //   }
  // }

  std::vector<std::shared_ptr<FrenetObstacle>> left_front_target_tracks;
  std::vector<std::shared_ptr<FrenetObstacle>> ego_front_target_tracks;
  std::vector<std::shared_ptr<FrenetObstacle>> left_lane_cone_distribution_set;
  std::vector<std::shared_ptr<FrenetObstacle>> current_lane_cone_distribution_set;
  std::vector<std::shared_ptr<FrenetObstacle>> left_boundary_cone_distribution_set;
  std::vector<std::shared_ptr<FrenetObstacle>> left_boundary_cone_distribution_ordered_set;

  for (auto& obstacle : front_tracks) {
    if (std::count(left_potensial_objects.begin(), left_potensial_objects.end(),
                   obstacle->id()) > 0) {
      left_front_target_tracks.emplace_back(obstacle);
    }
    if (std::count(front_potensial_objects.begin(),
                   front_potensial_objects.end(), obstacle->id()) > 0) {
      ego_front_target_tracks.emplace_back(obstacle);
    }
  }

  for (const auto& obj : ego_front_target_tracks) {
    if (obj->type() == iflyauto::OBJECT_TYPE_TRAFFIC_CONE) {
      if (std::fabs(obj->frenet_l()) < kLaneConeRangeThres) {
        current_lane_cone_distribution_set.emplace_back(obj);
      }
      if ((obj->frenet_l() > kLaneConeRangeThres) &&
          std::fabs(obj->frenet_l() - lane_width * 0.5) < kLaneBoundaryConeRangeThres) {
        left_boundary_cone_distribution_set.emplace_back(obj);
      }
    }
  }
  for (const auto& obj : left_front_target_tracks) {
    if (obj->type() == iflyauto::OBJECT_TYPE_TRAFFIC_CONE) {
      if (std::fabs(obj->frenet_l() - lane_width) < kLaneConeRangeThres) {
        left_lane_cone_distribution_set.emplace_back(obj);
      }
      if ((obj->frenet_l() < lane_width - kLaneConeRangeThres) &&
          std::fabs(obj->frenet_l() - lane_width * 0.5) < kLaneBoundaryConeRangeThres) {
        left_boundary_cone_distribution_set.emplace_back(obj);
      }
    }
  }
  // 针对左侧车道边界的锥桶，安排距离自车由近及远排序
  for (const auto& tr : left_boundary_cone_distribution_set) {
    if (tr->d_s_rel() >= 0.0) {
      auto it = left_boundary_cone_distribution_ordered_set.begin();
      while (it != left_boundary_cone_distribution_ordered_set.end() &&
             (*it)->d_s_rel() < tr->d_s_rel()) {
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
      current_lane_cone_distribution_set[0]->frenet_s() >
          left_boundary_cone_distribution_ordered_set[0]->frenet_s()) {
    return true;
  }
  if (current_lane_cone_distribution_set.size() >= kLaneConeNumThres &&
      left_lane_cone_distribution_set.size() >= kLaneConeNumThres) {
    if (left_lane_cone_distribution_set[0]->frenet_s() -
            current_lane_cone_distribution_set[0]->frenet_s() >
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
  const auto& tracks_map = lateral_obstacle_->tracks_map();
  const auto& target_lane_coord_ptr = ref_line->get_frenet_coord();
  Point2D ego_frenet_point_in_right_lane;
  Point2D ego_cart_point{planning_init_point_.lat_init_state.x(),
                         planning_init_point_.lat_init_state.y()};
  if (!target_lane_coord_ptr->XYToSL(ego_cart_point,
                                     ego_frenet_point_in_right_lane)) {
    ILOG_DEBUG << "Enmergency error! Ego Pose Cart2SL in right lane failed!";
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

  if (!right_potensial_objects.size()) {
    return true;
  }
  // for (const auto& id : right_potensial_objects) {
  //   if (tracks_map_[id].d_rel < -kPotensialObjectLonRange ||
  //       tracks_map_[id].d_rel > kPotensialObjectLonRange ||
  //       tracks_map_[id].l < potensial_min_l || tracks_map_[id].l > 0.0) {
  //     continue;
  //   }
  //   ++potensial_counter;
  //   if (potensial_counter >= kPotensialObjectNum) {
  //     return false;
  //   }
  // }

  std::vector<std::shared_ptr<FrenetObstacle>> right_front_target_tracks;
  std::vector<std::shared_ptr<FrenetObstacle>> ego_front_target_tracks;
  std::vector<std::shared_ptr<FrenetObstacle>> right_lane_cone_distribution_set;
  std::vector<std::shared_ptr<FrenetObstacle>> current_lane_cone_distribution_set;
  std::vector<std::shared_ptr<FrenetObstacle>> right_boundary_cone_distribution_set;
  std::vector<std::shared_ptr<FrenetObstacle>> right_boundary_cone_distribution_ordered_set;

  const std::vector<std::shared_ptr<planning::FrenetObstacle>>& front_tracks =
      lateral_obstacle_->front_tracks();

  for (const auto& obstacle : front_tracks) {
    if (std::count(right_potensial_objects.begin(),
                   right_potensial_objects.end(), obstacle->id()) > 0) {
      right_front_target_tracks.emplace_back(obstacle);
    }
    if (std::count(front_potensial_objects.begin(),
                   front_potensial_objects.end(), obstacle->id()) > 0) {
      ego_front_target_tracks.emplace_back(obstacle);
    }
  }

  for (const auto& obj : ego_front_target_tracks) {
    if (obj->type() == iflyauto::OBJECT_TYPE_TRAFFIC_CONE) {
      if (std::fabs(obj->frenet_l()) < kLaneConeRangeThres) {
        current_lane_cone_distribution_set.emplace_back(obj);
      }
      if ((obj->frenet_l() < -kLaneConeRangeThres) &&
          std::fabs(obj->frenet_l() + lane_width * 0.5) < kLaneBoundaryConeRangeThres) {
        right_boundary_cone_distribution_set.emplace_back(obj);
      }
    }
  }
  for (const auto& obj : right_front_target_tracks) {
    if (obj->type() == iflyauto::OBJECT_TYPE_TRAFFIC_CONE) {
      if (std::fabs(obj->frenet_l() + lane_width) < kLaneConeRangeThres) {
        right_lane_cone_distribution_set.emplace_back(obj);
      }
      if ((obj->frenet_l() > -lane_width + kLaneConeRangeThres) &&
          std::fabs(obj->frenet_l() + lane_width * 0.5) < kLaneBoundaryConeRangeThres) {
        right_boundary_cone_distribution_set.emplace_back(obj);
      }
    }
  }
  // 针对右侧车道边界的锥桶，安排距离自车由近及远排序
  for (const auto& tr : right_boundary_cone_distribution_set) {
    if (tr->d_s_rel() >= 0.0) {
      auto it = right_boundary_cone_distribution_ordered_set.begin();
      while (it != right_boundary_cone_distribution_ordered_set.end() &&
             (*it)->d_s_rel() < tr->d_s_rel()) {
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
      current_lane_cone_distribution_set[0]->frenet_s() >
          right_boundary_cone_distribution_ordered_set[0]->frenet_s()) {
    return true;
  }
  if (current_lane_cone_distribution_set.size() >= kLaneConeNumThres &&
      right_lane_cone_distribution_set.size() >= kLaneConeNumThres) {
    if (right_lane_cone_distribution_set[0]->frenet_s() -
            current_lane_cone_distribution_set[0]->frenet_s() >
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
  const auto& tracks_map = lateral_obstacle_->tracks_map();
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
    auto iter = tracks_map.find(id);
    if (iter == tracks_map.end()) {
      continue;
    }
    const double dis =
        std::hypot(tracks_map.at(id)->obstacle()->x_center() - ego_pose.x, tracks_map.at(id)->obstacle()->y_center() - ego_pose.y);
    // 障碍物type为卡车或者轿车的需要抑制变道
    if (tracks_map.at(id)->velocity() > kStaticVehicleThreshold &&
        dis < kSuppressionDistanceThres &&
        (tracks_map.at(id)->type() == iflyauto::OBJECT_TYPE_TRUCK ||
         tracks_map.at(id)->type() == iflyauto::OBJECT_TYPE_COUPE)) {
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
    ILOG_DEBUG << "ego on ref lane failed!";
    return false;
  }

  const double ego_front_edge = vehicle_param.front_edge_to_rear_axle;
  const double ego_rear_edge = vehicle_param.rear_edge_to_rear_axle;
  const double ego_half_width = vehicle_param.width * 0.5;
  *front_required_space = 0.0;
  *rear_required_space = 0.0;
  const double lateral_buffer =
      kLateralBufferInCheckLaneChangeSafety * lat_safety_ratio;

  std::vector<std::shared_ptr<FrenetObstacle>> target_side_front_tracks;
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

  std::vector<std::shared_ptr<FrenetObstacle>> target_side_rear_tracks;
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
  const auto& tracks_map = lateral_obstacle_->tracks_map();
  auto front_track_iter = front_tracks_ids.empty()
                              ? tracks_map.end()
                              : tracks_map.find(front_tracks_ids.front());
  auto rear_track_iter = rear_tracks_ids.empty()
                             ? tracks_map.end()
                             : tracks_map.find(rear_tracks_ids.front());
  const bool has_front_obs = front_track_iter != tracks_map.end();
  const bool has_rear_obs = rear_track_iter != tracks_map.end();

  int not_safety_agent_id = -1;
  if (has_front_obs && has_rear_obs) {
    const double front_to_ego_dis = std::hypot(
        front_track_iter->second->obstacle()->x_center(), front_track_iter->second->obstacle()->y_center());
    const double rear_to_ego_dis = std::hypot(rear_track_iter->second->obstacle()->x_center(),
                                              rear_track_iter->second->obstacle()->y_center());
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
    const double long_dis = front_track_iter->second->d_s_rel();
    Point2D obs_cart_point{0.0, 0.0};
    obs_cart_point.x = ego_cart_point.x +
                       front_track_iter->second->obstacle()->x_center() * ego_fx -
                       front_track_iter->second->obstacle()->y_center() * ego_fy;
    obs_cart_point.y = ego_cart_point.y +
                       front_track_iter->second->obstacle()->x_center() * ego_fy +
                       front_track_iter->second->obstacle()->x_center() * ego_fx;

    Point2D obs_target_frenet_point;
    if (!target_lane_coord_ptr->XYToSL(obs_cart_point,
                                       obs_target_frenet_point)) {
      ILOG_DEBUG << "front obs on ref lane failed!";
      return false;
    }
    double front_safety_threshold_at_present = 0.0;
    const double front_safety_distance =
        getSafetyDistance(kLaneChangeSafetyLongDistance,
                          kMaxLaneChangeSafetyLongDistance, ego_state->ego_v(),
                          use_dynamic_safety_distance) +
        extra_front_distance_buffer;
    const bool front_safety_at_present = checkFrontSafetyAtPresent(
        long_dis, ego_state->ego_v(), front_track_iter->second->velocity(),
        kDecInCheckLaneChangeSafety, front_track_iter->second->obstacle()->acceleration(), ego_front_edge,
        0.5 * front_track_iter->second->length(), front_safety_distance,
        lon_safety_ratio, &front_safety_threshold_at_present);
    if (!front_safety_at_present) {
      not_safe_agent_ids->emplace_back(not_safety_agent_id);
      ILOG_DEBUG << "checkLaneChangeSafety(): Not safety for front vehicle at present!";
      return false;
    }
    const double front_safety_at_future = checkFrontSafetyAtFuture(
        long_dis, ego_state->ego_v(), front_track_iter->second->velocity(),
        kDecInCheckLaneChangeSafety, front_track_iter->second->obstacle()->acceleration(), ego_front_edge,
        0.5 * front_track_iter->second->length(), front_safety_distance,
        lon_safety_ratio, kConsiderTimestampInCheckLaneChangeSafety);
    if (!front_safety_at_future) {
      not_safe_agent_ids->emplace_back(not_safety_agent_id);
      ILOG_DEBUG << "checkLaneChangeSafety(): Not safety for front vehicle at future!";
      return false;
    }
    *front_required_space = front_safety_threshold_at_present;
  }

  if (has_rear_obs) {
    const auto& ego_state =
        session_->environmental_model().get_ego_state_manager();
    double ego_fx = std::cos(ego_state->ego_pose_raw().theta);
    double ego_fy = std::sin(ego_state->ego_pose_raw().theta);
    const double long_dis = rear_track_iter->second->d_s_rel();
    Point2D obs_cart_point{0.0, 0.0};
    obs_cart_point.x = ego_cart_point.x +
                       rear_track_iter->second->obstacle()->x_center() * ego_fx -
                       rear_track_iter->second->obstacle()->y_center() * ego_fy;
    obs_cart_point.y = ego_cart_point.y +
                       rear_track_iter->second->obstacle()->x_center() * ego_fy +
                       rear_track_iter->second->obstacle()->y_center() * ego_fx;

    Point2D obs_target_frenet_point;
    if (!target_lane_coord_ptr->XYToSL(obs_cart_point,
                                       obs_target_frenet_point)) {
      ILOG_DEBUG << "rear obs on ref lane failed!";
      return false;
    }
    double rear_safety_threshold_at_present = 0.0;
    const double rear_safety_distance =
        getSafetyDistance(kBackwardLaneChangeSafetyLongDistance,
                          kBackwardMaxLaneChangeSafetyLongDistance,
                          rear_track_iter->second->velocity(),
                          use_dynamic_safety_distance) +
        extra_rear_distance_buffer;
    const bool rear_safety_at_present = checkRearSafetyAtPresent(
        long_dis, ego_state->ego_v(), rear_track_iter->second->velocity(),
        kDecInCheckLaneChangeSafety, rear_track_iter->second->obstacle()->acceleration(), ego_rear_edge,
        0.5 * rear_track_iter->second->length(), rear_safety_distance,
        safety_backward_time, lon_safety_ratio,
        &rear_safety_threshold_at_present);
    if (!rear_safety_at_present) {
      not_safe_agent_ids->emplace_back(not_safety_agent_id);
      ILOG_DEBUG << "checkLaneChangeSafety(): Not safety for rear vehicle at present!";
      return false;
    }
    const double rear_safety_at_future = checkRearSafetyAtFuture(
        long_dis, ego_state->ego_v(), rear_track_iter->second->velocity(),
        kDecInCheckLaneChangeSafety, rear_track_iter->second->obstacle()->acceleration(), ego_rear_edge,
        0.5 * rear_track_iter->second->length(), rear_safety_distance,
        lon_safety_ratio,
        kConsiderTimestampInCheckLaneChangeSafety * lon_safety_ratio);
    if (!rear_safety_at_future) {
      not_safe_agent_ids->emplace_back(not_safety_agent_id);
      ILOG_DEBUG << "checkLaneChangeSafety(): Not safety for front vehicle at future!";
      return false;
    }
    *rear_required_space = rear_safety_threshold_at_present;
  }
  return true;
}

void OvertakeRequest::selectTargetObstacleIds(
    const std::shared_ptr<KDPath>& ref_line, const Point2D ego_cart_point,
    const std::vector<std::shared_ptr<FrenetObstacle>> candidate_obs_info,
    const double search_range, const int max_target_num,
    const double ego_half_width, const double l_buffer,
    const double max_l_buffer, const bool order_reverse,
    std::vector<int>* target_tracks_ids) {
  if (!target_tracks_ids || candidate_obs_info.empty()) {
    return;
  }

  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& tracks_map = lateral_obstacle_->tracks_map();

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

  auto isTruck = [&](const std::shared_ptr<FrenetObstacle>& veh) -> bool {
    return veh->type() == iflyauto::OBJECT_TYPE_TRUCK &&
           veh->length() > kMinLengthForTruck && veh->width() > kMinWidthForTruck;
  };

  auto getHalfWidthOnLane = [&](const std::shared_ptr<FrenetObstacle>& veh,
                                double* half_width) -> bool {
    if (!half_width) {
      return false;
    }
    Point2D obs_cart_point{0.0, 0.0};
    obs_cart_point.x =
        ego_cart_point.x + veh->obstacle()->x_center() * ego_fx - veh->obstacle()->y_center() * ego_fy;
    obs_cart_point.y =
        ego_cart_point.y + veh->obstacle()->x_center() * ego_fy + veh->obstacle()->y_center() * ego_fx;

    Point2D new_frenet_point;
    double new_lane_theta = 0.0;
    if (!ref_line->XYToSL(obs_cart_point, new_frenet_point)) {
      ILOG_DEBUG << "obs on ref lane failed!";
      return false;
    }

    new_lane_theta = ref_line->GetPathCurveHeading(new_frenet_point.x);
    double veh_theta = veh->obstacle()->heading_angle() - ego_state->ego_pose().theta;
    const double delta_angle =
        std::abs(NormalizeAngle(new_lane_theta) - NormalizeAngle(veh_theta));
    *half_width = std::abs(std::cos(delta_angle)) * 0.5 * veh->width() +
                  std::abs(std::sin(delta_angle)) * 0.5 * veh->length();
    return true;
  };

  for (int i = begin_index; i != end_index; i += step) {
    auto obs_info = candidate_obs_info.at(i);
    if (!(obs_info->obstacle()->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }
    const auto id = obs_info->id();
    if (tracks_map.find(id) == tracks_map.end()) {
      continue;
    }
    double veh_half_width = 1.0;
    if (!getHalfWidthOnLane(obs_info, &veh_half_width)) {
      veh_half_width = 0.5 * obs_info->width();
    }
    const bool is_truck = isTruck(obs_info);
    const double buffer = std::min(
        max_l_buffer, is_truck ? l_buffer + kExtraLatBufferForTruck : l_buffer);
    const bool is_select =
        selectObsByLateralDistance(obs_info->frenet_l(), veh_half_width, buffer) &&
        is_truck;

    if (!is_select) {
      continue;
    }

    const double distance = std::hypot(obs_info->obstacle()->x_center(), obs_info->obstacle()->y_center());
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
  ILOG_DEBUG << "ego v:" << ego_v
             << "front v:" << veh_v
             << "base dis buff:" << base_distance_buffer
             << "safety_threshold:" << safety_threshold
             << "long_diff:" << long_diff;
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
  const auto& tracks_map = lateral_obstacle_->tracks_map();
  auto leading_vehicle_iter = tracks_map.find(overtake_vehicle_id_);
  if (leading_vehicle_iter != tracks_map.end()) {
    const double overtake_lane_change_vehicle_speed =
        leading_vehicle_iter->second->velocity();
    const double lateral_distance = planning_init_point_.frenet_state.r;
    double front_leading_vehivle_long_distance = kDefaultFrontObstacleDistance;
    const auto lane_tracks_manager =
        session_->environmental_model().get_lane_tracks_manager();
    std::vector<std::shared_ptr<planning::FrenetObstacle>> ego_front_vehicle_id_array;
    ego_front_vehicle_id_array = lane_tracks_manager->front_tracks_c();
    double leading_vehicle_lateral_speed = leading_vehicle_iter->second->frenet_velocity_l();
    double leading_vehicle_lateral_dis = leading_vehicle_iter->second->frenet_l();

    if (ego_front_vehicle_id_array.size() >= 1) {
      int ego_front_vehicle_id = ego_front_vehicle_id_array.at(0)->id();
      if (ego_front_vehicle_id != -1) {
        auto front_leading_vehicle_iter =
            tracks_map.find(ego_front_vehicle_id_array.at(0)->id());
        if (overtake_vehicle_id_ == ego_front_vehicle_id) {
          if (ego_front_vehicle_id_array.size() == 1) {
            front_leading_vehicle_iter = tracks_map.end();
          } else {
            front_leading_vehicle_iter =
                tracks_map.find(ego_front_vehicle_id_array.at(1)->id());
          }
        }
        if (front_leading_vehicle_iter != tracks_map.end()) {
          front_leading_vehivle_long_distance =
              front_leading_vehicle_iter->second->d_s_rel();
          ILOG_DEBUG << "ego vehicle front_leading_vehicle id:" << front_leading_vehicle_iter->second->id();
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
      ILOG_DEBUG << "Cancel OverTakeLaneChange Dir: " << request_type_
             << "LaneChangeProgress:" << lc_state
             << "OverTake Vehicle LatSpd:" << leading_vehicle_lateral_speed
             << "Ego Lat Distance:" << lateral_distance
             << "Front Leading Vehicle Distance:" << front_leading_vehivle_long_distance;
      return true;
    }

    double ego_left_front_leading_vehicle_min_speed =
        kCancelOverTakeLnChgTargetLaneVehDftSpd;
    double ego_left_front_leading_vehivle_long_distance =
        kDefaultFrontObstacleDistance;
    double ego_right_front_leading_vehicle_min_speed =
        kCancelOverTakeLnChgTargetLaneVehDftSpd;
    double ego_right_front_leading_vehivle_long_distance =
        kDefaultFrontObstacleDistance;
    const auto &ego_left_front_vehicle_array = lane_tracks_manager_->front_tracks_l();
    const auto &ego_right_front_vehicle_array = lane_tracks_manager_->front_tracks_r();
    if (ego_left_front_vehicle_array.size() >= 1) {
      int ego_left_front_vehicle_id =
          ego_left_front_vehicle_array.at(0)->id();
      auto left_front_leading_vehicle_iter =
          tracks_map.find(ego_left_front_vehicle_array.at(0)->id());
      if (overtake_vehicle_id_ == ego_left_front_vehicle_id) {
        if (ego_left_front_vehicle_array.size() == 1) {
          left_front_leading_vehicle_iter = tracks_map.end();
        } else {
          left_front_leading_vehicle_iter =
              tracks_map.find(ego_left_front_vehicle_array.at(1)->id());
        }
      }
      if (left_front_leading_vehicle_iter != tracks_map.end()) {
        ego_left_front_leading_vehicle_min_speed =
            left_front_leading_vehicle_iter->second->velocity();
        ego_left_front_leading_vehivle_long_distance =
            left_front_leading_vehicle_iter->second->d_s_rel();
        ILOG_DEBUG << "ego_left_front_leading_vehivle_long_distance:" << ego_left_front_leading_vehivle_long_distance;
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
      ILOG_DEBUG << "Cancel OverTakeLaneChange Dir: " << request_type_
             << "LaneChangeProgress:" << lc_state
             << "OverTake Vehicle LonSpd:" << overtake_lane_change_vehicle_speed
             << "Left Lane Target Speed:" << ego_left_front_leading_vehicle_min_speed
             << "Ego Lat Distance:" << lateral_distance
             << "left lane exist slow front veh frame num:" << target_lane_exist_slow_front_veh_frame_num_;
      return true;
    }

    if (ego_right_front_vehicle_array.size() >= 1) {
      int ego_right_front_vehicle_id =
          ego_right_front_vehicle_array.at(0)->id();
      auto right_front_leading_vehicle_iter =
          tracks_map.find(ego_right_front_vehicle_array.at(0)->id());
      if (overtake_vehicle_id_ == ego_right_front_vehicle_id) {
        if (1 == ego_right_front_vehicle_array.size()) {
          right_front_leading_vehicle_iter = tracks_map.end();
        } else {
          right_front_leading_vehicle_iter =
              tracks_map.find(ego_right_front_vehicle_array.at(1)->id());
        }
      }
      if (right_front_leading_vehicle_iter != tracks_map.end()) {
        ego_right_front_leading_vehicle_min_speed =
            right_front_leading_vehicle_iter->second->velocity();
        ego_right_front_leading_vehivle_long_distance =
            right_front_leading_vehicle_iter->second->d_s_rel();
        ILOG_DEBUG << "right front leading vehicle long distance:" << ego_right_front_leading_vehivle_long_distance;
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
      ILOG_DEBUG << "Cancel OverTakeLaneChange Dir: " << request_type_
             << "LaneChangeProgress:" << lc_state
             << "OverTake Vehicle LonSpd:" << overtake_lane_change_vehicle_speed
             << "Left Lane Target Speed:" << ego_left_front_leading_vehicle_min_speed
             << "Ego Lat Distance:" << lateral_distance
             << "left lane exist slow front veh frame num:" << target_lane_exist_slow_front_veh_frame_num_;
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
#include "lane_change_request_manager.h"

#include "adas_function/mrc_condition.h"
#include "basic_types.pb.h"
#include "behavior_planners/lane_change_decider/lane_change_requests/cone_lane_change_request.h"
#include "common_platform_type_soc.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "define/geometry.h"
#include "display_state_types.h"
#include "ego_planning_config.h"
#include "lane_change_requests/emergence_avoid_lane_change_request.h"
#include "lane_change_requests/overtake_lane_change_request.h"
#include "tasks/behavior_planners/lane_change_decider/lane_change_requests/overtake_lane_change_request.h"

namespace planning {
namespace {
constexpr double kMaxSpeedTriggerInteractiveLaneChangeRequest = 33.333;
constexpr double kMinSpeedTriggerInteractiveLaneChangeRequest = 11.111;
constexpr double kCoolingDownPeriodForIntCancel = 4.0;

}  // namespace
// class: LaneChangeRequestManager
LaneChangeRequestManager::LaneChangeRequestManager(
    framework::Session* session, const EgoPlanningConfigBuilder* config_builder,
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
    std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr)
    : int_request_(session, virtual_lane_mgr, lane_change_lane_mgr),
      map_request_(session, config_builder, virtual_lane_mgr,
                   lane_change_lane_mgr),
      overtake_request_(config_builder, session, virtual_lane_mgr,
                        lane_change_lane_mgr),
      emergence_avoid_request_(session, virtual_lane_mgr, lane_change_lane_mgr),
      cone_change_request_(session, virtual_lane_mgr, lane_change_lane_mgr),
      merge_change_request_(session, virtual_lane_mgr, lane_change_lane_mgr),
      virtual_lane_mgr_(virtual_lane_mgr),
      session_(session) {
  config_ = config_builder->cast<EgoPlanningConfig>();
}

void LaneChangeRequestManager::FinishRequest() {
  int_request_.finish_and_clear();
  map_request_.Finish();
  overtake_request_.Finish();
  overtake_request_.Reset();
  emergence_avoid_request_.Finish();
  emergence_avoid_request_.Reset();
  cone_change_request_.Finish();
  cone_change_request_.Reset();
  merge_change_request_.Finish();
  merge_change_request_.Reset();

  request_ = NO_CHANGE;
  request_source_ = NO_REQUEST;
  gen_turn_signal_ = NO_CHANGE;
  lane_change_cmd_ = LaneChangeRequest::TurnSwitchState::NONE;
  trigger_lane_change_cancel_ = false;
  int_request_is_allowed_lc_in_cone_scene_ = true;
}

bool LaneChangeRequestManager::Update(int lc_status, const bool hd_map_valid) {
  ILOG_INFO << "LaneChangeRequestManager.Update()";
  // MDEBUG_JSON_BEGIN_DICT(LaneChangeRequestManager)
  // TBD： 后续考虑json形式进行数据存储
  const auto& route_info_output =
      session_->environmental_model().get_route_info()->get_route_info_output();
  const auto& reference_path_mgr =
      session_->mutable_environmental_model()->get_reference_path_manager();
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto& intersection_state = virtual_lane_manager->GetIntersectionState();
  auto mrc_condition = session_->mutable_planning_context()->mrc_condition();
  const bool location_valid = session_->environmental_model().location_valid();
  const double k_default_lane_change_cooling_duration = 3.0;
  bool const enable_mrc_pull_over = mrc_condition->enable_mrc_pull_over();
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const double trigger_overtake_min_ego_speed_threshold = 5.56;  // 20km/h
  double minimum_ego_cruise_speed_for_active_lane_change =
      config_.minimum_ego_cruise_speed_for_active_lane_change;
  // const double kOvertakeTriggerCruiseSpeedMinThreshold = 16.67;  // 60km/h
  bool EnableGenerateOvertakeQequestByFrontSlowVehicle = true;
  const bool use_overtake_lane_change_request =
      config_
          .use_overtake_lane_change_request_instead_of_active_lane_change_request;
  const bool enable_use_speed_limit_to_suppress_interactive_lane_change =
      config_.enable_use_speed_limit_to_suppress_interactive_lane_change;
  double minimum_distance_nearby_ramp_to_surpress_overtake_lane_change =
      config_.minimum_distance_nearby_ramp_to_surpress_overtake_lane_change;
  const double odd_route_distance_threshold = 500.0;
  const double distance_nearby_merge_point_to_surpress_merge_request = 50.0;
  const bool enable_use_emergency_avoidence_lc_request =
      config_.enable_use_emergency_avoidence_lane_change_request;
  const bool enable_use_cone_change_request =
      config_.enable_use_cone_change_request;
  const bool enable_use_merge_lc_request =
      config_.enable_use_merge_change_request;
  const auto& function_info = session_->environmental_model().function_info();
  const int origin_relative_id_zero_nums =
      virtual_lane_mgr_->origin_relative_id_zero_nums();
  const auto& boundary_merge_point_valid =
      session_->planning_context()
          .ego_lane_road_right_decider_output()
          .boundary_merge_point_valid;
  const auto& boundary_merge_point = session_->planning_context()
                                         .ego_lane_road_right_decider_output()
                                         .boundary_merge_point;
  const auto& cur_lane = virtual_lane_mgr_->get_current_lane();
  const bool is_merge_region = session_->planning_context()
                                   .ego_lane_road_right_decider_output()
                                   .is_merge_region;
  double ego_distance_to_boundary_merge = 100.0;
  const double default_distance_threshld_to_stop_line = 30.0;
  const double dis_to_stopline = session_->environmental_model()
                                     .get_virtual_lane_manager()
                                     ->GetEgoDistanceToStopline();
  const double ego_v =
      session_->environmental_model().get_ego_state_manager()->ego_v();

  if (cur_lane != nullptr && is_merge_region) {
    const auto& curr_reference_path =
        reference_path_mgr->get_reference_path_by_lane(
            cur_lane->get_virtual_id(), false);
    if (curr_reference_path != nullptr && boundary_merge_point_valid) {
      const auto& refline = curr_reference_path->get_frenet_coord();
      Point2D boundary_merge_frenet_point;
      if (!refline->XYToSL(boundary_merge_point, boundary_merge_frenet_point)) {
        ILOG_DEBUG << "LaneChangeRequestManager::fail to get ego position on "
                      "current lane";
      }
      ego_distance_to_boundary_merge =
          boundary_merge_frenet_point.x -
          curr_reference_path->get_frenet_ego_state().s();
    }
  }

  int state = lane_change_decider_output.curr_state;
  double curr_time = IflyTime::Now_s();

  const auto& ego_blinker = session_->mutable_environmental_model()
                                ->get_ego_state_manager()
                                ->ego_blinker();
  ProcessBlinkState(
      ego_blinker, static_cast<StateMachineLaneChangeStatus>(lc_status),
      static_cast<RequestType>(lane_change_decider_output.lc_request));
  int_request_cancel_reason_ = NO_CANCEL;
  // todo(ldh): 使用工厂模式管理变道请求。
  int_request_.SetLaneChangeCmd(lane_change_cmd_);
  int_request_.SetLaneChangeCancelFromTrigger(trigger_lane_change_cancel_);
  map_request_.SetLaneChangeCmd(lane_change_cmd_);
  map_request_.SetLaneChangeCancelFromTrigger(trigger_lane_change_cancel_);
  overtake_request_.SetLaneChangeCmd(lane_change_cmd_);
  overtake_request_.SetLaneChangeCancelFromTrigger(trigger_lane_change_cancel_);
  emergence_avoid_request_.SetLaneChangeCmd(lane_change_cmd_);
  emergence_avoid_request_.SetLaneChangeCancelFromTrigger(
      trigger_lane_change_cancel_);
  cone_change_request_.SetLaneChangeCmd(lane_change_cmd_);
  cone_change_request_.SetLaneChangeCancelFromTrigger(
      trigger_lane_change_cancel_);
  merge_change_request_.SetLaneChangeCmd(lane_change_cmd_);
  merge_change_request_.SetLaneChangeCancelFromTrigger(
      trigger_lane_change_cancel_);
  int_lane_change_cmd_ = LaneChangeRequest::TurnSwitchState::NONE;
  int_request_is_allowed_lc_in_cone_scene_ = true;
  if (int_request_.enable_int_request() || enable_mrc_pull_over) {
    int_request_.Update(lc_status);
    int_request_is_allowed_lc_in_cone_scene_ =
        int_request_.ConeSituationJudgement(
            virtual_lane_mgr_->get_lane_with_virtual_id(target_lane_virtual_id_));
    int_lane_change_cmd_ = int_request_.get_lane_change_cmd();
    // int_request_cancel_reason_ = int_request_.lc_request_cancel_reason();
    ilc_virtual_request_ = int_request_.get_ilc_virtual_req();
  } else {
    int_request_.reset_int_cnt();
  }

  if (int_request_.request_type() == NO_CHANGE) {
    if (enable_use_cone_change_request &&
        request_source_ != EMERGENCE_AVOID_REQUEST) {
      cone_change_request_.Update(lc_status);
    }
    if (enable_use_emergency_avoidence_lc_request &&
        request_source_ != CONE_REQUEST) {
      emergence_avoid_request_.Update(lc_status);
    }
    if (enable_use_merge_lc_request && origin_relative_id_zero_nums == 1 &&
        (curr_time >
         int_request_.tfinish() + k_default_lane_change_cooling_duration)) {
      merge_change_request_.Update(lc_status);
      is_near_merge_region_ =
          merge_change_request_.is_merge_lane_change_situation();
    }

    if (hd_map_valid &&
        (curr_time >
         int_request_.tfinish() + k_default_lane_change_cooling_duration) &&
        request_source_ != MERGE_REQUEST) {
      map_request_.Update(lc_status, map_request_.tfinish(), request_source_);
    }

    if (location_valid && use_overtake_lane_change_request) {
      // lcc功能抑制超车变道
      if (function_info.function_mode() != common::DrivingFunctionInfo::NOA) {
        overtake_request_.Reset();
        ILOG_INFO
            << "cann't generate overtake lane change in non-NOA functions";
        EnableGenerateOvertakeQequestByFrontSlowVehicle = false;
      }

      if (route_info_output.is_on_ramp) {
        overtake_request_.Reset();
        ILOG_INFO << "cann't generate overtake lane change on ramp or near "
                     "ramp or near merge";
        EnableGenerateOvertakeQequestByFrontSlowVehicle = false;
      }

      if (route_info_output.distance_to_route_end <
          odd_route_distance_threshold) {
        overtake_request_.Reset();
        ILOG_INFO << "cann't generate overtake lane change nearby odd boundary";
        EnableGenerateOvertakeQequestByFrontSlowVehicle = false;
      }

      const auto& ego_state =
          session_->environmental_model().get_ego_state_manager();
      if (ego_state->ego_v() < trigger_overtake_min_ego_speed_threshold) {
        ILOG_INFO << "cann't generate overtake lane change since ego speed is "
                     "less than min speed threshold";
        overtake_request_.Reset();
        EnableGenerateOvertakeQequestByFrontSlowVehicle = false;
      }
      if (curr_time <
          int_request_.tfinish() + k_default_lane_change_cooling_duration) {
        overtake_request_.Reset();
        EnableGenerateOvertakeQequestByFrontSlowVehicle = false;
      }
      if (trigger_lane_change_cancel_) {
        EnableGenerateOvertakeQequestByFrontSlowVehicle = false;
        overtake_request_.Finish();
        overtake_request_.Reset();
        ILOG_INFO << "cann't generate overtake lane change since cancel!";
      }

      // TODO:添加至操作时间域的距离小于一定值时将overtake_count_=0
      // trigger overtake lane change when lane keep status.
      if (lc_status != kLaneKeeping && lc_status != kLaneChangePropose &&
          lc_status != kLaneChangeHold) {
        ILOG_INFO << "cann't generate overtake lane change when not lane keep!";
        // overtake_request_.Reset();
        EnableGenerateOvertakeQequestByFrontSlowVehicle = false;
      }
      if (EnableGenerateOvertakeQequestByFrontSlowVehicle) {
        overtake_request_.Update(lc_status);
      }
    }
  }

  ILOG_INFO << "[LaneChangeRequestManager::update] int_request:"
            << int_request_.request_type()
            << "map_request:" << map_request_.request_type()
            << "overtake_request:" << overtake_request_.request_type()
            << "emergence_avoid_request:"
            << emergence_avoid_request_.request_type()
            << "cone_change_request:" << cone_change_request_.request_type()
            << "int_cancel_reason:" << int_request_cancel_reason_
            << "turn_signal:" << gen_turn_signal_;

  if (int_request_cancel_reason_ == MANUAL_CANCEL &&
      gen_turn_signal_ != NO_CHANGE &&
      target_lane_virtual_id_ != virtual_lane_mgr_->current_lane_virtual_id() &&
      request_source_ == MAP_REQUEST) {
    if (gen_turn_signal_ == LEFT_CHANGE) {
      int_request_.set_left_cancel_freeze_cnt(
          DisplayStateConfig::DefaultCancelFreezeCnt);
    } else if (gen_turn_signal_ == RIGHT_CHANGE) {
      int_request_.set_right_cancel_freeze_cnt(
          DisplayStateConfig::DefaultCancelFreezeCnt);
    }
    map_request_.Finish();
    ILOG_DEBUG << "[LaneChangeRequestManager::update] manual cancel finish dd "
                  "or map request!";
  }
  std::cout << "\n int request type is: " << int_request_.request_type()
            << std::endl;

  if (int_request_.request_type() != NO_CHANGE) {
    if (emergence_avoid_request_.request_type() != NO_CHANGE) {
      emergence_avoid_request_.Finish();
      emergence_avoid_request_.Reset();
    }
    if (cone_change_request_.request_type() != NO_CHANGE) {
      cone_change_request_.Finish();
      cone_change_request_.Reset();
    }
    if (map_request_.request_type() != NO_CHANGE) {
      map_request_.Finish();
    }
    if (merge_change_request_.request_type() != NO_CHANGE) {
      merge_change_request_.Finish();
      merge_change_request_.Reset();
    }
    if (overtake_request_.request_type() != NO_CHANGE) {
      overtake_request_.Finish();
      overtake_request_.Reset();
    }
    request_ = int_request_.request_type();
    request_source_ = INT_REQUEST;
    target_lane_virtual_id_ = int_request_.target_lane_virtual_id();
  } else if (cone_change_request_.request_type() != NO_CHANGE) {
    if (emergence_avoid_request_.request_type() != NO_CHANGE) {
      emergence_avoid_request_.Finish();
      emergence_avoid_request_.Reset();
    }
    if (map_request_.request_type() != NO_CHANGE) {
      map_request_.Finish();
    }
    if (merge_change_request_.request_type() != NO_CHANGE) {
      merge_change_request_.Finish();
      merge_change_request_.Reset();
    }
    if (overtake_request_.request_type() != NO_CHANGE) {
      overtake_request_.Finish();
      overtake_request_.Reset();
    }
    request_ = cone_change_request_.request_type();
    request_source_ = CONE_REQUEST;
    target_lane_virtual_id_ = cone_change_request_.target_lane_virtual_id();
  } else if (emergence_avoid_request_.request_type() != NO_CHANGE) {
    if (map_request_.request_type() != NO_CHANGE) {
      map_request_.Finish();
    }
    if (merge_change_request_.request_type() != NO_CHANGE) {
      merge_change_request_.Finish();
      merge_change_request_.Reset();
    }
    if (overtake_request_.request_type() != NO_CHANGE) {
      overtake_request_.Finish();
      overtake_request_.Reset();
    }
    request_ = emergence_avoid_request_.request_type();
    request_source_ = EMERGENCE_AVOID_REQUEST;
    target_lane_virtual_id_ = emergence_avoid_request_.target_lane_virtual_id();
  } else if (merge_change_request_.request_type() != NO_CHANGE) {
    if (map_request_.request_type() != NO_CHANGE) {
      map_request_.Finish();
    }
    if (overtake_request_.request_type() != NO_CHANGE) {
      overtake_request_.Finish();
      overtake_request_.Reset();
    }
    request_ = merge_change_request_.request_type();
    request_source_ = MERGE_REQUEST;
    target_lane_virtual_id_ = merge_change_request_.target_lane_virtual_id();
  } else if (map_request_.request_type() != NO_CHANGE) {
    if (overtake_request_.request_type() != NO_CHANGE) {
      overtake_request_.Finish();
      overtake_request_.Reset();
    }
    request_ = map_request_.request_type();
    request_source_ = MAP_REQUEST;
    target_lane_virtual_id_ = map_request_.target_lane_virtual_id();
  } else {
    // ILOG_DEBUG << "overtake_request_.request_type():" <<
    // overtake_request_.request_type();
    request_ = overtake_request_.request_type();
    request_source_ = (request_ != NO_CHANGE) ? OVERTAKE_REQUEST : NO_REQUEST;
    target_lane_virtual_id_ =
        (request_ != NO_CHANGE) ? overtake_request_.target_lane_virtual_id()
                                : virtual_lane_mgr_->current_lane_virtual_id();
    if (request_ != NO_CHANGE && request_source_ == OVERTAKE_REQUEST) {
      int overtake_vehicle_id = overtake_request_.GetOvertakeVehicleId();
      JSON_DEBUG_VALUE("overtake_vehicle_id", overtake_vehicle_id);
    }
  }

  if (request_source_ == OVERTAKE_REQUEST && request_ != NO_CHANGE &&
      overtake_request_.isCancelOverTakingLaneChange(state)) {
    overtake_request_.Finish();
    ILOG_DEBUG << "Front Vehicle Cutout->isCancelOverTakingLaneChange";
  }

  const bool is_exist_interactive_select_split =
    virtual_lane_manager->get_is_exist_interactive_select_split();
  const bool split_lane_on_left_side_before_interactive =
      virtual_lane_manager->get_split_lane_on_left_side_before_interactive();
  const bool split_lane_on_right_side_before_interactive =
      virtual_lane_manager->get_split_lane_on_right_side_before_interactive();
  const bool other_split_lane_left_side = virtual_lane_manager->get_other_split_lane_left_side();
  const bool other_split_lane_right_side = virtual_lane_manager->get_other_split_lane_right_side();
  if(((other_split_lane_left_side && request_ == LEFT_CHANGE) ||
      (other_split_lane_right_side && request_ == RIGHT_CHANGE) ||
      (split_lane_on_left_side_before_interactive && request_ == LEFT_CHANGE) ||
      (split_lane_on_right_side_before_interactive && request_ == RIGHT_CHANGE) ||
      is_exist_interactive_select_split) && request_source_ == INT_REQUEST) {
    int_request_.Finish();
    request_ = NO_CHANGE;
    request_source_ = NO_REQUEST;
    target_lane_virtual_id_ = virtual_lane_manager->current_lane_virtual_id();
  }
  if (!is_exist_interactive_select_split && last_frame_is_exist_interactive_select_split_) {
    int_request_.finish_and_clear();
    request_ = NO_CHANGE;
    request_source_ = NO_REQUEST;
    target_lane_virtual_id_ = virtual_lane_manager->current_lane_virtual_id();
    lane_change_cmd_ = LaneChangeRequest::TurnSwitchState::NONE;
    int_lane_change_cmd_ = LaneChangeRequest::TurnSwitchState::NONE;
  }
  last_frame_is_exist_interactive_select_split_ = is_exist_interactive_select_split;

  // if (virtual_lane_mgr_->get_lane_with_virtual_id(target_lane_virtual_id_)) {
  //   int target_lane_order_id =
  //       virtual_lane_mgr_->get_lane_with_virtual_id(target_lane_virtual_id_)
  //           ->get_order_id();
  //   ILOG_DEBUG << "[LCRequestManager::update] final :target_lane_order_id: "
  //              << target_lane_order_id
  //              << " target_lane_virtual_id:" << target_lane_virtual_id_;
  // } else {
  //   request_ = NO_CHANGE;
  //   request_source_ = NO_REQUEST;
  //   target_lane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
  //   ILOG_DEBUG << "[LCRequestManager::update] Target lane lost !!! final "
  //                 "target_lane_virtual_id:"
  //              << target_lane_virtual_id_;
  // }
  if (trigger_lane_change_cancel_) {
    int_request_cancel_reason_ = MANUAL_CANCEL;
  }
  GenerateHMIInfo();

  ILOG_DEBUG << "[LCRequestManager::update] ===cur_state: " << lc_status
             << " === gen_turn_signal_:" << gen_turn_signal_;
  // JSON_DEBUG_VALUE("cur_state", lc_status)
  return true;
}

void LaneChangeRequestManager::GenerateHMIInfo() {
  if (request_ == NO_CHANGE) {
    ILOG_DEBUG << "[LCRequestManager::update] request: None";
    // MDEBUG_JSON_ADD_ITEM(request_shape, "========", LaneChangeRequestManager)
  } else if (request_ == LEFT_CHANGE) {
    ILOG_DEBUG
        << "[LCRequestManager::update] request: Left Change <<<<<<<<<<<<<<<<<";
    ILOG_DEBUG << "[LCRequestManager::update] source:" << request_source_;
  } else {
    ILOG_DEBUG
        << "[LCRequestManager::update] request: Right Change >>>>>>>>>>>>>>>>";
    ILOG_DEBUG << "[LCRequestManager::update] source:" << request_source_;
  }

  // auto ad_info = &(session_->mutable_planning_context()
  //                      ->mutable_planning_hmi_info()
  //                      ->ad_info);
  // ad_info->lane_change_intent = iflyauto::NO_INTENT;
  // ad_info->lane_change_source = iflyauto::LC_SOURCE_NONE;
  if (request_source_ == MAP_REQUEST) {
    gen_turn_signal_ = map_request_.turn_signal();
    // auto current_lane = virtual_lane_mgr_->get_current_lane();
    // int lc_map_decision = virtual_lane_mgr_->lc_map_decision(current_lane);
    // if (lc_map_decision > 0) {
    //   ad_info->lane_change_intent = iflyauto::OUT_INTENT;
    // } else if (lc_map_decision < 0) {
    //   ad_info->lane_change_intent = iflyauto::IN_INTENT;
    // }
    // ad_info->lane_change_source = iflyauto::LC_SOURCE_MAP;
  } else if (request_source_ == OVERTAKE_REQUEST) {
    gen_turn_signal_ = overtake_request_.turn_signal();
    // ad_info->lane_change_intent = iflyauto::SLOWING_INTENT;
    // ad_info->lane_change_source = iflyauto::LC_SOURCE_ACT;
  } else if (request_source_ == INT_REQUEST) {
    gen_turn_signal_ = NO_CHANGE;
    // ad_info->lane_change_intent = iflyauto::BLINKSWITCH_INTENT;
    // ad_info->lane_change_source = iflyauto::LC_SOURCE_INT;
  } else {
    gen_turn_signal_ = NO_CHANGE;
  }
}

double LaneChangeRequestManager::GetReqStartTime(int source) const {
  if (source == INT_REQUEST) {
    return int_request_.tstart();
  } else if (source == MAP_REQUEST) {
    return map_request_.tstart();
  } else if (source == OVERTAKE_REQUEST) {
    return overtake_request_.tstart();
  } else if (source == EMERGENCE_AVOID_REQUEST) {
    return emergence_avoid_request_.tstart();
  } else if (source == CONE_REQUEST) {
    return cone_change_request_.tstart();
  } else if (source == MERGE_REQUEST) {
    return merge_change_request_.tstart();
  }
  return DBL_MAX;
}

double LaneChangeRequestManager::GetReqFinishTime(int source) const {
  if (source == INT_REQUEST) {
    return int_request_.tfinish();
  } else if (source == MAP_REQUEST) {
    return map_request_.tfinish();
  } else if (source == OVERTAKE_REQUEST) {
    return overtake_request_.tfinish();
  } else if (source == EMERGENCE_AVOID_REQUEST) {
    return emergence_avoid_request_.tfinish();
  } else if (source == CONE_REQUEST) {
    return cone_change_request_.tfinish();
  } else if (source == MERGE_REQUEST) {
    return merge_change_request_.tfinish();
  }
  return DBL_MAX;
}

void LaneChangeRequestManager::ProcessBlinkState(
    const uint ego_blinker, const StateMachineLaneChangeStatus& lc_status,
    const RequestType& cur_req) {
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const bool is_exist_interactive_select_split =
    virtual_lane_manager->get_is_exist_interactive_select_split();
  static int32_t cancel_freeze_count = 11;
  bool is_allowed_cancel_state =
      (lc_status == StateMachineLaneChangeStatus::kLaneChangePropose ||
       lc_status == StateMachineLaneChangeStatus::kLaneChangeExecution ||
       lc_status == StateMachineLaneChangeStatus::kLaneChangeHold ||
       lc_status == StateMachineLaneChangeStatus::kLaneChangeCancel ||
       is_exist_interactive_select_split);
  bool trigger_left_lane_change =
      (lc_status == StateMachineLaneChangeStatus::kLaneKeeping &&
       cur_req == RequestType::NO_CHANGE) &&
      ((last_frame_blinker_ == LaneChangeRequest::TurnSwitchState::NONE ||
        last_frame_blinker_ == LaneChangeRequest::LaneChangeRequest::
                                   TurnSwitchState::LEFT_LIGHTLY_TOUCH ||
        last_frame_blinker_ ==
            LaneChangeRequest::TurnSwitchState::RIGHT_LIGHTLY_TOUCH) &&
       ego_blinker == LaneChangeRequest::TurnSwitchState::LEFT_FIRMLY_TOUCH);
  bool trigger_right_lane_change =
      (lc_status == StateMachineLaneChangeStatus::kLaneKeeping &&
       cur_req == RequestType::NO_CHANGE) &&
      ((last_frame_blinker_ == LaneChangeRequest::TurnSwitchState::NONE ||
        last_frame_blinker_ ==
            LaneChangeRequest::TurnSwitchState::LEFT_LIGHTLY_TOUCH ||
        last_frame_blinker_ ==
            LaneChangeRequest::TurnSwitchState::RIGHT_LIGHTLY_TOUCH) &&
       ego_blinker == LaneChangeRequest::TurnSwitchState::RIGHT_FIRMLY_TOUCH);
  bool trigger_left_lane_change_cancel =
      is_allowed_cancel_state &&
      ((cur_req == RequestType::LEFT_CHANGE || lane_change_cmd_ == LaneChangeRequest::TurnSwitchState::LEFT_FIRMLY_TOUCH) &&
       (ego_blinker ==
            LaneChangeRequest::TurnSwitchState::RIGHT_LIGHTLY_TOUCH ||
        ego_blinker == LaneChangeRequest::TurnSwitchState::RIGHT_FIRMLY_TOUCH));
  bool trigger_right_lane_change_cancel =
      is_allowed_cancel_state &&
      ((cur_req == RequestType::RIGHT_CHANGE || lane_change_cmd_ == LaneChangeRequest::TurnSwitchState::RIGHT_FIRMLY_TOUCH) &&
       (ego_blinker == LaneChangeRequest::TurnSwitchState::LEFT_LIGHTLY_TOUCH ||
        ego_blinker == LaneChangeRequest::TurnSwitchState::LEFT_FIRMLY_TOUCH));
  if (trigger_left_lane_change && cancel_freeze_count > 10) {
    trigger_lane_change_cancel_ = false;
    lane_change_cmd_ = LaneChangeRequest::TurnSwitchState::LEFT_FIRMLY_TOUCH;
  } else if (trigger_right_lane_change && cancel_freeze_count > 10) {
    trigger_lane_change_cancel_ = false;
    lane_change_cmd_ = LaneChangeRequest::TurnSwitchState::RIGHT_FIRMLY_TOUCH;
  }
  cancel_freeze_count++;
  if (cancel_freeze_count > 11) {
    trigger_lane_change_cancel_ = false;
    cancel_freeze_count = 11;
  } else {
    trigger_lane_change_cancel_ = true;
  }
  static int lane_change_cancel_freeze_cnt = 40;

  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  bool is_interactive_lane_change_cancel =
      static_cast<RequestSource>(
          lane_change_decider_output.lc_request_source) == INT_REQUEST;
  if (trigger_left_lane_change_cancel || trigger_right_lane_change_cancel) {
    if (is_interactive_lane_change_cancel) {
      lane_change_cancel_freeze_cnt = 35;
    } else {
      lane_change_cancel_freeze_cnt = 0;
    }
    lane_change_cmd_ = LaneChangeRequest::TurnSwitchState::NONE;
    int_lane_change_cmd_ = LaneChangeRequest::TurnSwitchState::NONE;
    trigger_lane_change_cancel_ = true;
    cancel_freeze_count = 0;
  }
  last_frame_blinker_ = ego_blinker;
  if (lane_change_cancel_freeze_cnt < 40) {
    trigger_lane_change_cancel_ = true;
    ++lane_change_cancel_freeze_cnt;
    // lane_change_cmd_ = LaneChangeRequest::TurnSwitchState::NONE;
  } else {
    lane_change_cancel_freeze_cnt = 40;
    trigger_lane_change_cancel_ = false;
  }
}

}  // namespace planning
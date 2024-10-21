#include "lane_change_request_manager.h"

#include "adas_function/mrc_condition.h"
#include "basic_types.pb.h"
#include "behavior_planners/lane_change_decider/lane_change_requests/cone_lane_change_request.h"
#include "common_platform_type_soc.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "ego_planning_config.h"
#include "lane_change_requests/emergence_avoid_lane_change_request.h"
#include "lane_change_requests/overtake_lane_change_request.h"
#include "tasks/behavior_planners/lane_change_decider/lane_change_requests/overtake_lane_change_request.h"

namespace planning {

// class: LaneChangeRequestManager
LaneChangeRequestManager::LaneChangeRequestManager(
    framework::Session* session, const EgoPlanningConfigBuilder* config_builder,
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
    std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr)
    : int_request_(session, virtual_lane_mgr, lane_change_lane_mgr),
      map_request_(session, config_builder, virtual_lane_mgr,
                   lane_change_lane_mgr),
      act_request_(session, virtual_lane_mgr, lane_change_lane_mgr),
      overtake_request_(session, virtual_lane_mgr, lane_change_lane_mgr),
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
}

bool LaneChangeRequestManager::Update(
    int lc_status, const bool hd_map_valid) {
  LOG_DEBUG("LaneChangeRequestManager.Update() \n");
  // MDEBUG_JSON_BEGIN_DICT(LaneChangeRequestManager)
  // TBD： 后续考虑json形式进行数据存储
  auto mrc_condition = session_->mutable_planning_context()->mrc_condition();
  const bool location_valid = session_->environmental_model().location_valid();
  bool const enable_mrc_pull_over = mrc_condition->enable_mrc_pull_over();
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const double kOvertakeTriggerEgoSpeedMinThreshold = 5.56;  // 20km/h
  double minimum_ego_cruise_speed_for_active_lane_change =
      config_.minimum_ego_cruise_speed_for_active_lane_change;
  // const double kOvertakeTriggerCruiseSpeedMinThreshold = 16.67;  // 60km/h
  bool EnableGenerateOvertakeQequestByFrontSlowVehicle = true;
  const bool use_overtake_lane_change_request =
      config_
          .use_overtake_lane_change_request_instead_of_active_lane_change_request;
  double minimum_distance_nearby_ramp_to_surpress_overtake_lane_change =
      config_.minimum_distance_nearby_ramp_to_surpress_overtake_lane_change;
  const double max_pass_merge_distance_to_surpress_overtake_lane_change =
      virtual_lane_mgr_->dis_threshold_to_last_merge_point();
  double sum_dis_to_last_merge_point =
      virtual_lane_mgr_->sum_dis_to_last_merge_point();
  const double odd_route_distance_threshold = 500.0;
  const bool enable_use_emergency_avoidence_lc_request =
      config_.enable_use_emergency_avoidence_lane_change_request;
  const bool enable_use_cone_change_request =
      config_.enable_use_cone_change_request;
  const bool enable_use_merge_lc_request =
      config_.enable_use_merge_change_request;
  const auto& function_info = session_->environmental_model().function_info();
  const double dis_threshold_to_merged_point =
      virtual_lane_mgr_->dis_threshold_to_merged_point();
  const double dis_to_first_merge =
      virtual_lane_mgr_->distance_to_first_road_merge();
  const int origin_relative_id_zero_nums = virtual_lane_mgr_->origin_relative_id_zero_nums();

  int state = lane_change_decider_output.curr_state;
  if (int_request_.enable_int_request() || enable_mrc_pull_over) {
    int_request_.Update(lc_status);
    int_request_cancel_reason_ = int_request_.request_cancel_reason();
    ilc_virtual_request_ = int_request_.get_ilc_virtual_req();
  } else {
    int_request_.reset_int_cnt();
  }
  if (int_request_.request_type() == NO_CHANGE) {
    if (enable_use_cone_change_request) {
      cone_change_request_.Update(lc_status);
    }
    if (enable_use_emergency_avoidence_lc_request &&
        cone_change_request_.request_type() == RequestType::NO_CHANGE) {
      emergence_avoid_request_.Update(lc_status);
    }
    if (hd_map_valid) {
      map_request_.update(lc_status, map_request_.tfinish());
    }
    if (enable_use_merge_lc_request && request_source_ != MAP_REQUEST &&
        origin_relative_id_zero_nums == 1) {
      merge_change_request_.Update(lc_status);
      is_near_merge_region_ =
          merge_change_request_.is_merge_lane_change_situation();
    }
    if (location_valid && use_overtake_lane_change_request) {
      // lcc功能抑制超车变道
      if (function_info.function_mode() != common::DrivingFunctionInfo::NOA) {
        overtake_request_.Reset();
        LOG_DEBUG("cann't generate overtake lane change in non-NOA functions");
        EnableGenerateOvertakeQequestByFrontSlowVehicle = false;
      }

      if (virtual_lane_mgr_->is_on_ramp() ||
          virtual_lane_mgr_->dis_to_ramp() <=
              minimum_distance_nearby_ramp_to_surpress_overtake_lane_change ||
          sum_dis_to_last_merge_point <
              max_pass_merge_distance_to_surpress_overtake_lane_change ||
          dis_to_first_merge < dis_threshold_to_merged_point) {
        overtake_request_.Reset();
        LOG_DEBUG(
            "cann't generate overtake lane change on ramp or near ramp or near "
            "merge");
        EnableGenerateOvertakeQequestByFrontSlowVehicle = false;
      }

      if (virtual_lane_mgr_->get_distance_to_route_end() <
          odd_route_distance_threshold) {
        overtake_request_.Reset();
        LOG_DEBUG("cann't generate overtake lane change nearby odd boundary");
        EnableGenerateOvertakeQequestByFrontSlowVehicle = false;
      }

      const auto& ego_state =
          session_->environmental_model().get_ego_state_manager();
      if (ego_state->ego_v() < kOvertakeTriggerEgoSpeedMinThreshold ||
          ego_state->ego_v_cruise() <
              minimum_ego_cruise_speed_for_active_lane_change) {
        LOG_DEBUG(
            "cann't generate overtake lane change since ego speed is less than "
            "min speed threshold");
        overtake_request_.Reset();
        EnableGenerateOvertakeQequestByFrontSlowVehicle = false;
      }

      // TODO:添加至操作时间域的距离小于一定值时将overtake_count_=0

      // trigger overtake lane change when lane keep status.
      if (lc_status != kLaneKeeping && lc_status != kLaneChangePropose) {
        LOG_DEBUG("cann't generate overtake lane change when not lane keep!");
        overtake_request_.Reset();
        EnableGenerateOvertakeQequestByFrontSlowVehicle = false;
      }
      if (EnableGenerateOvertakeQequestByFrontSlowVehicle) {
        overtake_request_.Update(lc_status);
      }
    }
  }

  LOG_DEBUG(
      "[LaneChangeRequestManager::update] int_request: %d, map_request: %d, "
      "overtake_request: %d, emergence_avoid_request: %d, "
      "cone_change_request: %d, "
      "int_cancel_reason: %d, turn_signal: %d \n",
      int_request_.request_type(), map_request_.request_type(),
      overtake_request_.request_type(),
      emergence_avoid_request_.request_type(),
      cone_change_request_.request_type(), int_request_cancel_reason_,
      gen_turn_signal_);

  if (int_request_cancel_reason_ == MANUAL_CANCEL &&
      gen_turn_signal_ != NO_CHANGE &&
      target_lane_virtual_id_ !=
          virtual_lane_mgr_->current_lane_virtual_id() &&
      request_source_ == MAP_REQUEST) {
    if (gen_turn_signal_ == LEFT_CHANGE) {
      int_request_.set_left_cancel_freeze_cnt(
          DisplayStateConfig::DefaultCancelFreezeCnt);
    } else if (gen_turn_signal_ == RIGHT_CHANGE) {
      int_request_.set_right_cancel_freeze_cnt(
          DisplayStateConfig::DefaultCancelFreezeCnt);
    }
    map_request_.Finish();
    LOG_DEBUG(
        "[LaneChangeRequestManager::update] manual cancel finish dd or map "
        "request! \n");
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
    target_lane_virtual_id_ =
        emergence_avoid_request_.target_lane_virtual_id();
  } else if (map_request_.request_type() != NO_CHANGE) {
    if (merge_change_request_.request_type() != NO_CHANGE) {
      merge_change_request_.Finish();
      merge_change_request_.Reset();
    }
    if (overtake_request_.request_type() != NO_CHANGE) {
      overtake_request_.Finish();
      overtake_request_.Reset();
    }
    request_ = map_request_.request_type();
    request_source_ = MAP_REQUEST;
    target_lane_virtual_id_ = map_request_.target_lane_virtual_id();
  } else if (merge_change_request_.request_type() != NO_CHANGE) {
    if (overtake_request_.request_type() != NO_CHANGE) {
      overtake_request_.Finish();
      overtake_request_.Reset();
    }
    request_ = merge_change_request_.request_type();
    request_source_ = MERGE_REQUEST;
    target_lane_virtual_id_ = merge_change_request_.target_lane_virtual_id();
  } else {
    LOG_DEBUG("overtake_request_.request_type(): %d",
              overtake_request_.request_type());
    request_ = overtake_request_.request_type();
    request_source_ = (request_ != NO_CHANGE) ? OVERTAKE_REQUEST : NO_REQUEST;
    target_lane_virtual_id_ =
        (request_ != NO_CHANGE)
            ? overtake_request_.target_lane_virtual_id()
            : virtual_lane_mgr_->current_lane_virtual_id();
    if (request_ != NO_CHANGE && request_source_ == OVERTAKE_REQUEST) {
      int overtake_vehicle_id = overtake_request_.GetOvertakeVehicleId();
      JSON_DEBUG_VALUE("overtake_vehicle_id", overtake_vehicle_id);
    }
  }

  if (request_source_ == OVERTAKE_REQUEST && request_ != NO_CHANGE &&
      overtake_request_.isCancelOverTakingLaneChange(state)) {
    overtake_request_.Finish();
    LOG_DEBUG("Front Vehicle Cutout->isCancelOverTakingLaneChange");
  }

  if (virtual_lane_mgr_->get_lane_with_virtual_id(target_lane_virtual_id_)) {
    int target_lane_order_id =
        virtual_lane_mgr_->get_lane_with_virtual_id(target_lane_virtual_id_)
            ->get_order_id();
    LOG_DEBUG(
        "[LCRequestManager::update] final :target_lane_order_id %d, "
        "target_lane_virtual_id: %d \n",
        target_lane_order_id, target_lane_virtual_id_);
  } else {
    target_lane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
    LOG_DEBUG(
        "[LCRequestManager::update] Target lane lost !!! final "
        "target_lane_virtual_id: %d \n",
        target_lane_virtual_id_);
  }

  GenerateHMIInfo();

  LOG_WARNING(
      "[LCRequestManager::update] ===cur_state: %d=== gen_turn_signal_: %d \n",
      lc_status, gen_turn_signal_);
  // JSON_DEBUG_VALUE("cur_state", lc_status)
  return true;
}

void LaneChangeRequestManager::GenerateHMIInfo() {
  if (request_ == NO_CHANGE) {
    LOG_WARNING("[LCRequestManager::update] request: None \n");
    // MDEBUG_JSON_ADD_ITEM(request_shape, "========", LaneChangeRequestManager)
  } else if (request_ == LEFT_CHANGE) {
    LOG_WARNING(
        "[LCRequestManager::update] request: Left Change <<<<<<<<<<<<<<<<< \n");
    LOG_WARNING("[LCRequestManager::update] source: %d \n", request_source_);
  } else {
    LOG_WARNING(
        "[LCRequestManager::update] request: Right Change >>>>>>>>>>>>>>>> \n");
    LOG_WARNING("[LCRequestManager::update] source: %d \n", request_source_);
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
  } else if (source == ACT_REQUEST) {
    return act_request_.tstart();
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
  } else if (source == ACT_REQUEST) {
    return act_request_.tfinish();
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

}  // namespace planning
#include "lane_change_request_manager.h"

#include "adas_function/mrc_condition.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "ego_planning_config.h"
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
      virtual_lane_mgr_(virtual_lane_mgr),
      session_(session) {
  config_ = config_builder->cast<EgoPlanningConfig>();
}

void LaneChangeRequestManager::FinishRequest() {
  int_request_.finish_and_clear();
  map_request_.Finish();
  overtake_request_.Finish();
  overtake_request_.Reset();

  request_ = NO_CHANGE;
  request_source_ = NO_REQUEST;
  gen_turn_signal_ = NO_CHANGE;
}

bool LaneChangeRequestManager::Update(
    std::shared_ptr<ObjectSelector> object_selector, int lc_status,
    const bool hd_map_valid) {
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
  const double intersection_distance_of_suppression_active_lane_change = 90.0;
  bool EnableGenerateOvertakeQequestByFrontSlowVehicle = true;
  bool use_overtake_lane_change_request =
      config_
          .use_overtake_lane_change_request_instead_of_active_lane_change_request;
  double kMinDistanceNearbyRampToSurpressOvretakeLC =
      config_.minimum_distance_nearby_ramp_to_surpress_overtake_lane_change;

  int state = lane_change_decider_output.curr_state;
  if (int_request_.enable_int_request() || enable_mrc_pull_over) {
    int_request_.Update(lc_status);
    int_request_cancel_reason_ = int_request_.request_cancel_reason();
  } else {
    int_request_.reset_int_cnt();
  }
  if (int_request_.request_type() == NO_CHANGE) {
    if (hd_map_valid) {
      map_request_.update(lc_status, map_request_.tfinish());
    }
    if (location_valid && use_overtake_lane_change_request) {
      // 添加至可运行区域边界的距离小于一定值时overtake_count_ = 0
      const auto& clane = virtual_lane_mgr_->get_current_lane();
      const int left_car_point_size =
          clane->get_left_lane_boundary().car_points_size;
      const int right_car_points_size =
          clane->get_right_lane_boundary().car_points_size;
      if (clane->get_left_lane_boundary()
                  .car_points[left_car_point_size - 1]
                  .x <=
              intersection_distance_of_suppression_active_lane_change &&
          clane->get_right_lane_boundary()
                  .car_points[right_car_points_size - 1]
                  .x <=
              intersection_distance_of_suppression_active_lane_change) {
        overtake_request_.Reset();
        LOG_DEBUG(
            "cann't generate overtake lane change close to the intersection");
        EnableGenerateOvertakeQequestByFrontSlowVehicle = false;
      }

      if (virtual_lane_mgr_->is_on_ramp() || 
          virtual_lane_mgr_->dis_to_ramp() <= kMinDistanceNearbyRampToSurpressOvretakeLC) {
        overtake_request_.Reset();
        LOG_DEBUG("cann't generate overtake lane change in ramp");
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
      if (lc_status != ROAD_NONE && lc_status != ROAD_LC_LWAIT &&
          lc_status != ROAD_LC_RWAIT) {
        LOG_DEBUG("cann't generate overtake lane change when not lane keep!");
        overtake_request_.Reset();
        EnableGenerateOvertakeQequestByFrontSlowVehicle = false;
      }
      if (EnableGenerateOvertakeQequestByFrontSlowVehicle) {
        overtake_request_.Update(lc_status);
      }
    } else {
      // hack：实时planner跑原主动变道生成逻辑
      // WB hack:
      bool accident_ahead = false;
      bool not_accident = true;
      double start_move_distolane = session_->planning_context()
                                        .lane_change_decider_output()
                                        .start_move_dist_lane;
      act_request_.Update(object_selector, lc_status, start_move_distolane,
                          int_request_.tfinish(), map_request_.tfinish(),
                          accident_ahead, not_accident);
    }
  }

  if (location_valid && use_overtake_lane_change_request) {
    LOG_DEBUG(
        "[LaneChangeRequestManager::update] int_request: %d, map_request: %d, "
        "overtake_request: %d,"
        "int_cancel_reason: %d, turn_signal: %d \n",
        int_request_.request_type(), map_request_.request_type(),
        overtake_request_.request_type(), int_request_cancel_reason_,
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
      if (map_request_.request_type() != NO_CHANGE) {
        map_request_.Finish();
      }
      if (overtake_request_.request_type() != NO_CHANGE) {
        overtake_request_.Finish();
        overtake_request_.Reset();
      }
      request_ = int_request_.request_type();
      request_source_ = INT_REQUEST;
      target_lane_virtual_id_ = int_request_.target_lane_virtual_id();
    } else if (map_request_.request_type() != NO_CHANGE) {
      if (overtake_request_.request_type() != NO_CHANGE) {
        overtake_request_.Finish();
        overtake_request_.Reset();
      }
      request_ = map_request_.request_type();
      request_source_ = MAP_REQUEST;
      target_lane_virtual_id_ = map_request_.target_lane_virtual_id();
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

    GenerateHMIInfoForOvertake();
  } else {
    LOG_DEBUG(
        "[LaneChangeRequestManager::update] int_request: %d, map_request: %d, "
        "act_request: %d,"
        "int_cancel_reason: %d, turn_signal: %d \n",
        int_request_.request_type(), map_request_.request_type(),
        act_request_.request_type(), int_request_cancel_reason_,
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
      if (map_request_.request_type() != NO_CHANGE) {
        map_request_.Finish();
      }
      if (act_request_.request_type() != NO_CHANGE) {
        act_request_.Finish();
        act_request_.Reset();
      }
      request_ = int_request_.request_type();
      request_source_ = INT_REQUEST;
      target_lane_virtual_id_ = int_request_.target_lane_virtual_id();
    } else if (map_request_.request_type() != NO_CHANGE) {
      if (act_request_.request_type() != NO_CHANGE) {
        act_request_.Finish();
        act_request_.Reset();
      }
      request_ = map_request_.request_type();
      request_source_ = MAP_REQUEST;
      target_lane_virtual_id_ = map_request_.target_lane_virtual_id();
    } else {
      request_ = act_request_.request_type();
      request_source_ = (request_ != NO_CHANGE) ? ACT_REQUEST : NO_REQUEST;
      target_lane_virtual_id_ =
          (request_ != NO_CHANGE)
              ? act_request_.target_lane_virtual_id()
              : virtual_lane_mgr_->current_lane_virtual_id();
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
  }

  LOG_WARNING(
      "[LCRequestManager::update] ===cur_state: %d=== gen_turn_signal_: %d \n",
      lc_status, gen_turn_signal_);
  // JSON_DEBUG_VALUE("cur_state", lc_status)
  return true;
}

void LaneChangeRequestManager::GenerateHMIInfoForOvertake() {
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

  auto ad_info = &(session_->mutable_planning_context()
                       ->mutable_planning_hmi_info()
                       ->ad_info);
  ad_info->lane_change_intent = iflyauto::NO_INTENT;
  ad_info->lane_change_source = iflyauto::LC_SOURCE_NONE;
  if (request_source_ == MAP_REQUEST) {
    gen_turn_signal_ = map_request_.turn_signal();
    auto current_lane = virtual_lane_mgr_->get_current_lane();
    int lc_map_decision = virtual_lane_mgr_->lc_map_decision(current_lane);
    if (lc_map_decision > 0) {
      ad_info->lane_change_intent = iflyauto::OUT_INTENT;
    } else if (lc_map_decision < 0) {
      ad_info->lane_change_intent = iflyauto::IN_INTENT;
    }
    ad_info->lane_change_source = iflyauto::LC_SOURCE_MAP;
  } else if (request_source_ == OVERTAKE_REQUEST) {
    gen_turn_signal_ = overtake_request_.turn_signal();
    ad_info->lane_change_intent = iflyauto::SLOWING_INTENT;
    ad_info->lane_change_source = iflyauto::LC_SOURCE_ACT;
  } else if (request_source_ == INT_REQUEST) {
    gen_turn_signal_ = NO_CHANGE;
    ad_info->lane_change_intent = iflyauto::BLINKSWITCH_INTENT;
    ad_info->lane_change_source = iflyauto::LC_SOURCE_INT;
  } else {
    gen_turn_signal_ = NO_CHANGE;
  }
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

  auto ad_info = &(session_->mutable_planning_context()
                       ->mutable_planning_hmi_info()
                       ->ad_info);
  ad_info->lane_change_intent = iflyauto::NO_INTENT;
  ad_info->lane_change_source = iflyauto::LC_SOURCE_NONE;
  if (request_source_ == MAP_REQUEST) {
    gen_turn_signal_ = map_request_.turn_signal();
    auto current_lane = virtual_lane_mgr_->get_current_lane();
    int lc_map_decision = virtual_lane_mgr_->lc_map_decision(current_lane);
    if (lc_map_decision > 0) {
      ad_info->lane_change_intent = iflyauto::OUT_INTENT;
    } else if (lc_map_decision < 0) {
      ad_info->lane_change_intent = iflyauto::IN_INTENT;
    }
    ad_info->lane_change_source = iflyauto::LC_SOURCE_MAP;
  } else if (request_source_ == ACT_REQUEST) {
    gen_turn_signal_ = act_request_.turn_signal();
    ad_info->lane_change_intent = iflyauto::SLOWING_INTENT;
    ad_info->lane_change_source = iflyauto::LC_SOURCE_ACT;
  } else if (request_source_ == INT_REQUEST) {
    gen_turn_signal_ = NO_CHANGE;
    ad_info->lane_change_intent = iflyauto::BLINKSWITCH_INTENT;
    ad_info->lane_change_source = iflyauto::LC_SOURCE_INT;
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
  }
  return DBL_MAX;
}

}  // namespace planning
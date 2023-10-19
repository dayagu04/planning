#include "lane_change_request_manager.h"

#include "config/basic_type.h"
#include "mrc_condition.h"
#include "planning_output_context.h"
#include "scenario_state_machine.h"

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
      virtual_lane_mgr_(virtual_lane_mgr),
      session_(session) {}

void LaneChangeRequestManager::FinishRequest() {
  int_request_.finish_and_clear();
  // act_request_.finish_and_clear();
  map_request_.Finish();
  std::cout << "????????????????? " << std::endl;

  request_ = NO_CHANGE;
  request_source_ = NO_REQUEST;
  gen_turn_signal_ = NO_CHANGE;
}

void LaneChangeRequestManager::Update(int lc_status, const bool hd_map_valid) {
  LOG_DEBUG("LaneChangeRequestManager.Update() \n");
  // MDEBUG_JSON_BEGIN_DICT(LaneChangeRequestManager)
  // TBD： 后续考虑json形式进行数据存储
  auto mrc_condition = session_->mutable_planning_context()->mrc_condition();
  bool const enable_mrc_pull_over = mrc_condition->enable_mrc_pull_over();
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
    // WB hack:
    bool accident_ahead = false;
    bool not_accident = true;
    double start_move_distolane = session_->planning_context()
                                      .scenario_state_machine()
                                      ->get_start_move_dist_lane();
    act_request_.Update(lc_status, start_move_distolane, int_request_.tfinish(),
                        map_request_.tfinish(), accident_ahead, not_accident);
  }

  LOG_DEBUG(
      "[LaneChangeRequestManager::update] int_request: %d, map_request: %d, "
      "act_request: %d,"
      "int_cancel_reason: %d, turn_signal: %d \n",
      int_request_.request_type(), map_request_.request_type(),
      act_request_.request_type(), int_request_cancel_reason_,
      gen_turn_signal_);

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
    std::cout << "DDDDDDDDDDDDDDDDDDDDDDD " << std::endl;
    LOG_DEBUG(
        "[LaneChangeRequestManager::update] manual cancel finish dd or map "
        "request! \n");
  }
  if (int_request_.request_type() != NO_CHANGE) {
    if (map_request_.request_type() != NO_CHANGE) {
      map_request_.Finish();
      std::cout << "BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB " << std::endl;
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
        (request_ != NO_CHANGE) ? act_request_.target_lane_virtual_id()
                                : virtual_lane_mgr_->current_lane_virtual_id();
  }

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

  auto ad_info = session_->mutable_planning_output_context()
                     ->mutable_planning_hmi_info()
                     ->mutable_ad_info();
  ad_info->set_lane_change_intent(::PlanningHMI::LaneChangeIntent::NO_INTENT);
  ad_info->set_lane_change_source(::PlanningHMI::LaneChangeSource::NONE);
  if (request_source_ == MAP_REQUEST) {
    gen_turn_signal_ = map_request_.turn_signal();
    auto current_lane = virtual_lane_mgr_->get_current_lane();
    int lc_map_decision = virtual_lane_mgr_->lc_map_decision(current_lane);
    if (lc_map_decision > 0) {
      ad_info->set_lane_change_intent(
          ::PlanningHMI::LaneChangeIntent::OUT_INTENT);
    } else if (lc_map_decision < 0) {
      ad_info->set_lane_change_intent(
          ::PlanningHMI::LaneChangeIntent::IN_INTENT);
    }
    ad_info->set_lane_change_source(::PlanningHMI::LaneChangeSource::MAP);
  } else if (request_source_ == ACT_REQUEST) {
    gen_turn_signal_ = act_request_.turn_signal();
    ad_info->set_lane_change_intent(
        ::PlanningHMI::LaneChangeIntent::SLOWING_INTENT);
    ad_info->set_lane_change_source(::PlanningHMI::LaneChangeSource::ACT);
  } else if (request_source_ == INT_REQUEST) {
    gen_turn_signal_ = NO_CHANGE;
    ad_info->set_lane_change_intent(
        ::PlanningHMI::LaneChangeIntent::BLINKSWITCH_INTENT);
    ad_info->set_lane_change_source(::PlanningHMI::LaneChangeSource::INT);
  } else {
    gen_turn_signal_ = NO_CHANGE;
  }

  LOG_WARNING(
      "[LCRequestManager::update] ===cur_state: %d=== gen_turn_signal_: %d \n",
      lc_status, gen_turn_signal_);
}

double LaneChangeRequestManager::GetReqStartTime(int source) const {
  if (source == INT_REQUEST) {
    return int_request_.tstart();
  } else if (source == MAP_REQUEST) {
    return map_request_.tstart();
  } else if (source == ACT_REQUEST) {
    return act_request_.tstart();
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
  }
  return DBL_MAX;
}

}  // namespace planning
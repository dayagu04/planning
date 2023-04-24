#include "src/modules/scenario/lane_change_request_manager.h"

#include "src/modules/scc_function/mrc_condition.h"

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
      session_(session) {}

void LaneChangeRequestManager::FinishRequest() {
  int_request_.finish_and_clear();
  // act_request_.finish_and_clear();
  // map_request_.finish();

  request_ = NO_CHANGE;
  request_source_ = NO_REQUEST;
  turn_signal_ = NO_CHANGE;
}

void LaneChangeRequestManager::Update(
    int lc_status, std::shared_ptr<ObjectSelector>& object_selector,
    const bool hd_map_valid) {
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
      map_request_.update(lc_status, int_request_.get_left_cancel_freeze_cnt(),
                          int_request_.get_right_cancel_freeze_cnt());
    }
    act_request_.Update(lc_status, int_request_.tfinish(),
                        map_request_.tfinish(), object_selector);
  }

  LOG_DEBUG(
      "[LaneChangeRequestManager::update] int_request: %d, map_request: %d, "
      "act_request: %d,"
      "int_cancel_reason: %d, turn_signal: %d \n",
      int_request_.request_type(), map_request_.request_type(),
      act_request_.request_type(), int_request_cancel_reason_, turn_signal_);

  if (int_request_cancel_reason_ == MANUAL_CANCEL &&
      turn_signal_ != NO_CHANGE &&
      target_lane_virtual_id_ != virtual_lane_mgr_->current_lane_virtual_id() &&
      request_source_ == MAP_REQUEST) {
    if (turn_signal_ == LEFT_CHANGE) {
      int_request_.set_left_cancel_freeze_cnt(
          DisplayStateConfig::DefaultCancelFreezeCnt);
    } else if (turn_signal_ == RIGHT_CHANGE) {
      int_request_.set_right_cancel_freeze_cnt(
          DisplayStateConfig::DefaultCancelFreezeCnt);
    }
    map_request_.Finish();
    LOG_DEBUG(
        "[LaneChangeRequestManager::update] manual cancel finish dd or map "
        "request! \n");
  }
  if (int_request_.request_type() != NO_CHANGE) {
    if (map_request_.request_type() != NO_CHANGE) {
      map_request_.Finish();
    }
    request_ = int_request_.request_type();
    request_source_ = INT_REQUEST;
    target_lane_virtual_id_ = int_request_.target_lane_virtual_id();
  } else if (map_request_.request_type() != NO_CHANGE) {
    request_ = map_request_.request_type();
    request_source_ = MAP_REQUEST;
    target_lane_virtual_id_ = map_request_.target_lane_virtual_id();
  } else {
    request_ = NO_CHANGE;
    request_source_ = NO_REQUEST;
    // WB: 这里去掉模型的换道，需要补充默认的目标车道id
    // target_lane_virtual_id_ = model_request_.target_lane_virtual_id();
  }

  if (request_ == NO_CHANGE) {
    LOG_WARNING("[LCRequestManager::update] request: None \n");
    // MDEBUG_JSON_ADD_ITEM(request_shape, "========", LaneChangeRequestManager)
  } else if (request_ == LEFT_CHANGE) {
    LOG_WARNING(
        "[LCRequestManager::update] request: Left Change <<<<<<<<<<<<<<<<< \n");
    LOG_WARNING("[LCRequestManager::update] source: %d \n", request_source_);
    // MDEBUG_JSON_ADD_ITEM(request_shape, "<<<<<<<<", LaneChangeRequestManager)
  } else {
    LOG_WARNING(
        "[LCRequestManager::update] request: Right Change >>>>>>>>>>>>>>>> \n");
    LOG_WARNING("[LCRequestManager::update] source: %d \n", request_source_);
    // MDEBUG_JSON_ADD_ITEM(request_shape, ">>>>>>>>", LaneChangeRequestManager)
  }
  if (virtual_lane_mgr_->get_lane_with_virtual_id(target_lane_virtual_id_) !=
      nullptr) {
    int target_lane_order_id =
        virtual_lane_mgr_->get_lane_with_virtual_id(target_lane_virtual_id_)
            ->get_order_id();
    LOG_DEBUG("[LCRequestManager::update] final target_lane_order_id: %d \n",
              target_lane_order_id);
    // MDEBUG_JSON_ADD_ITEM(tlane_oid, target_lane_order_id,
    //                      LaneChangeRequestManager)
  }
  // MDEBUG_JSON_ADD_ITEM(request, static_cast<int>(request_),
  //                      LaneChangeRequestManager)
  // MDEBUG_JSON_ADD_ITEM(request_source, static_cast<int>(request_source_),
  //                      LaneChangeRequestManager)
  // MDEBUG_JSON_ADD_ITEM(current_state, lc_status, LaneChangeRequestManager)
  if (request_source_ == MAP_REQUEST) {
    turn_signal_ = map_request_.turn_signal();
  } else if (request_source_ == INT_REQUEST &&
             (lc_status == ROAD_LC_LCHANGE || lc_status == ROAD_LC_RCHANGE)) {
    turn_signal_ = int_request_.turn_signal();
  } else {
    turn_signal_ = NO_CHANGE;
  }
  LOG_WARNING("[LCRequestManager::update] ===cur_state: %d=== \n", lc_status);
  // MDEBUG_JSON_END_DICT(LaneChangeRequestManager)
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
#include "src/modules/scenario/lane_change_request_manager.h"

#include "lane_change_request_manager.h"
#include "src/modules/common/utils/lateral_utils.h"
#include "src/modules/scc_function/mrc_condition.h"

namespace planning {

LaneChangeRequest::LaneChangeRequest(
    framework::Session* session,
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
    std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr)
    : session_(session),
      virtual_lane_mgr_(virtual_lane_mgr),
      lane_change_lane_mgr_(lane_change_lane_mgr) {}

void LaneChangeRequest::GenerateRequest(RequestType direction) {
  if (direction != LEFT_CHANGE && direction != RIGHT_CHANGE) {
    LOG_DEBUG("[LaneChangeRequest::GenerateRequest] Illgeal direction[%d] \n",
              direction);
  }

  if (request_ == direction) {
    LOG_DEBUG(
        "[LaneChangeRequest::GenerateRequest] duplicated request, "
        "direction[%d] \n",
        direction);
    return;
  }

  request_ = direction;
  turn_signal_ = direction;
  tstart_ = IflyTime::Now_s();
}

void LaneChangeRequest::Finish() {
  if (request_ == NO_CHANGE) {
    LOG_DEBUG("[LaneChangeRequest::Finish] No request to finish \n");
    turn_signal_ = NO_CHANGE;
    return;
  }

  request_ = NO_CHANGE;
  turn_signal_ = NO_CHANGE;
  tfinish_ = IflyTime::Now_s();
}

bool LaneChangeRequest::AggressiveChange() const {
  auto origin_lane = lane_change_lane_mgr_->has_origin_lane()
                         ? lane_change_lane_mgr_->olane()
                         : virtual_lane_mgr_->get_current_lane();
  auto lc_map_decision = 0; // hack
      // origin_lane != nullptr ? origin_lane->lc_map_decision() : 0;
  auto aggressive_change_distance = 200.0;  // WB: hack
  // virtual_lane_mgr_->is_on_highway()
  //     ? aggressive_lane_change_distance_highway_
  //     : aggressive_lane_change_distance_urban_;

  auto aggressive_change =
      origin_lane != nullptr
          ? origin_lane->must_change_lane(virtual_lane_mgr_->get_lane_num(), aggressive_change_distance *
                                          std::fabs(lc_map_decision))
          : false;
  return aggressive_change && (request_ != NO_CHANGE);
}

bool LaneChangeRequest::IsDashedLineEnough(
    RequestType direction, const double ego_vel,
    std::shared_ptr<VirtualLaneManager> map_info_mgr) {
  LOG_DEBUG("dashed_enough: direction: %d \n", static_cast<int>(direction));
  LOG_DEBUG("dashed_enough: vel: %.2f \n", ego_vel);
  double dash_length = 80;
  double right_dash_line_len = map_info_mgr->get_distance_to_dash_line(
      RIGHT_CHANGE, origin_lane_order_id_);
  double left_dash_line_len = map_info_mgr->get_distance_to_dash_line(
      LEFT_CHANGE, origin_lane_order_id_);
  LOG_DEBUG("dashed_enough: right_dash_line_len: %.2f \n", right_dash_line_len);
  LOG_DEBUG("dashed_enough: left_dash_line_len: %.2f \n", left_dash_line_len);
  if (direction == LEFT_CHANGE && left_dash_line_len > 0.) {
    if (left_dash_line_len > ego_vel * 6.0) {
      return true;
    } else {
      dash_length = left_dash_line_len;
    }
  } else if (direction == RIGHT_CHANGE && right_dash_line_len > 0.) {
    if (right_dash_line_len > ego_vel * 6.0) {
      LOG_DEBUG("dashed_enough: right_dash_line_len > ego_vel * 6 \n");
      return true;
    } else {
      LOG_DEBUG("dashed_enough: right_dash_line_len <= ego_vel * 6 \n");
      dash_length = right_dash_line_len;
    }
  } else {
    LOG_ERROR("!dashed_enough \n");
    return false;
  }
  double error_buffer = std::fmin(ego_vel * 0.5, 5);
  dash_length -= error_buffer;

  double distance_thld = 500.0 ; // hack for distance
      // std::max(map_info_mgr->map_velocity_limit(), ego_vel) * 4.0;
  bool must_change_lane = map_info_mgr->get_current_lane()->must_change_lane(virtual_lane_mgr_->get_lane_num(),distance_thld);

  if (!must_change_lane && cal_lat_offset(ego_vel, dash_length) < 3.6) {
    LOG_ERROR("!dashed_enough \n");
    return false;
  }

  return true;
}

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

void LaneChangeRequestManager::finish_request() {}

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
  if (int_request_.request() == NO_CHANGE && hd_map_valid) {
    map_request_.update(lc_status, int_request_.get_left_cancel_freeze_cnt(),
                        int_request_.get_right_cancel_freeze_cnt());
  }

  LOG_DEBUG(
      "[LaneChangeRequestManager::update] int_request: %d, map_request: %d, "
      "int_cancel_reason: %d, turn_signal: %d \n",
      int_request_.request(), map_request_.request(),
      int_request_cancel_reason_, turn_signal_);

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
  if (int_request_.request() != NO_CHANGE) {
    if (map_request_.request() != NO_CHANGE) {
      map_request_.Finish();
    }
    request_ = int_request_.request();
    request_source_ = INT_REQUEST;
    target_lane_virtual_id_ = int_request_.target_lane_virtual_id();
  } else if (map_request_.request() != NO_CHANGE) {
    request_ = map_request_.request();
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

// class: IntRequest
IntRequest::IntRequest(
    planning::framework::Session* session,
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
    std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr)
    : LaneChangeRequest(session, virtual_lane_mgr, lane_change_lane_mgr) {
  auto config_builder = session_->mutable_environmental_model()->config_builder(
      planning::common::SceneType::HIGHWAY);
  auto int_request_config = config_builder->cast<ScenarioDisplayStateConfig>();
  enable_int_request_ = int_request_config.enable_int_request_function;
  count_trsh_ = int_request_config.int_rqt_cnt_trsh;
}

void IntRequest::Update(int lc_status) {}

// class: MapRequest
MapRequest::MapRequest(
    framework::Session* session, const EgoPlanningConfigBuilder* config_builder,
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
    std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr)
    : LaneChangeRequest(session, virtual_lane_mgr, lane_change_lane_mgr) {}

void MapRequest::update(int lc_status, int left_int_freeze_cnt,
                        int right_int_freeze_cnt) {
  ;
}

// class: ActRequest
ActRequest::ActRequest(
    planning::framework::Session* session,
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
    std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr)
    : LaneChangeRequest(session, virtual_lane_mgr, lane_change_lane_mgr) {}

void ActRequest::update(int lc_status) {}

}  // namespace planning
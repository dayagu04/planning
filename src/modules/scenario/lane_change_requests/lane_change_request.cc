#include "lane_change_requests/lane_change_request.h"

#include "utils/lateral_utils.h"

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
  if (request_type_ == direction) {
    LOG_DEBUG(
        "[LaneChangeRequest::GenerateRequest] duplicated request, "
        "direction[%d] \n",
        direction);
    return;
  }

  request_type_ = direction;
  turn_signal_ = direction;
  tstart_ = IflyTime::Now_s();
}

void LaneChangeRequest::Finish() {
  if (request_type_ == NO_CHANGE) {
    LOG_DEBUG("[LaneChangeRequest::Finish] No request to finish \n");
    turn_signal_ = NO_CHANGE;
    return;
  }

  request_type_ = NO_CHANGE;
  turn_signal_ = NO_CHANGE;
  tfinish_ = IflyTime::Now_s();
}

bool LaneChangeRequest::AggressiveChange() const {
  auto origin_lane = lane_change_lane_mgr_->has_origin_lane()
                         ? lane_change_lane_mgr_->olane()
                         : virtual_lane_mgr_->get_current_lane();
  auto lc_map_decision =
      0;  // hack
          // origin_lane != nullptr ? origin_lane->lc_map_decision() : 0;
  auto aggressive_change_distance = 200.0;  // WB: hack
  // virtual_lane_mgr_->is_on_highway()
  //     ? aggressive_lane_change_distance_highway_
  //     : aggressive_lane_change_distance_urban_;

  // auto aggressive_change =
  //     origin_lane != nullptr
  //         ? origin_lane->must_change_lane(aggressive_change_distance *
  //                                         std::fabs(lc_map_decision))
  //         : false;
  auto aggressive_change =
      origin_lane != nullptr ? virtual_lane_mgr_->must_change_lane(
                                   origin_lane, aggressive_change_distance *
                                                    std::fabs(lc_map_decision))
                             : false;
  return aggressive_change && (request_type_ != NO_CHANGE);
}

bool LaneChangeRequest::IsDashedLineEnough(
    RequestType direction, const double ego_vel,
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr) {
  LOG_DEBUG("dashed_enough: direction: %d \n", static_cast<int>(direction));
  LOG_DEBUG("dashed_enough: vel: %.2f \n", ego_vel);
  const double kInputBoundaryLenLimit = 145.;
  const double kDefaultBoundaryLen = 5000.;
  double dash_length = 80;
  double right_dash_line_len = virtual_lane_mgr->get_distance_to_dash_line(
      RIGHT_CHANGE, origin_lane_virtual_id_);
  double left_dash_line_len = virtual_lane_mgr->get_distance_to_dash_line(
      LEFT_CHANGE, origin_lane_virtual_id_);
  LOG_DEBUG("dashed_enough: right_dash_line_len: %.2f \n", right_dash_line_len);
  LOG_DEBUG("dashed_enough: left_dash_line_len: %.2f \n", left_dash_line_len);
  std::cout << "origin_lane_virtual_id_: " << origin_lane_virtual_id_ << "origin_lane_order_id_: " << origin_lane_virtual_id_ << std::endl;
  // HACK RUI
  if (virtual_lane_mgr->dis_to_ramp() < 1000.) return true;
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
  dash_length = (dash_length > kInputBoundaryLenLimit) ? kDefaultBoundaryLen
                                                       : dash_length;
  double error_buffer = std::fmin(ego_vel * 0.5, 5);
  dash_length -= error_buffer;

  double v_target =
      session_->environmental_model().get_ego_state_manager()->ego_v_cruise();

  double distance_thld = std::max(v_target, ego_vel) * 4.0;
  // bool must_change_lane =
  //     virtual_lane_mgr->get_current_lane()->must_change_lane(distance_thld);
  auto current_lane = virtual_lane_mgr_->get_current_lane();
  auto must_change_lane =
      current_lane != nullptr
          ? virtual_lane_mgr_->must_change_lane(current_lane, distance_thld)
          : false;
  if (!must_change_lane && cal_lat_offset(ego_vel, dash_length) < 3.6) {
    LOG_ERROR("!dashed_enough \n");
    return false;
  }

  return true;
}

}  // namespace planning
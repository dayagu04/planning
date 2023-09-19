#include "lane_change_requests/map_lane_change_request.h"

namespace planning {

// class: MapRequest
MapRequest::MapRequest(
    framework::Session* session, const EgoPlanningConfigBuilder* config_builder,
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
    std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr)
    : LaneChangeRequest(session, virtual_lane_mgr, lane_change_lane_mgr) {
  auto map_request_config = config_builder->cast<ScenarioDisplayStateConfig>();
}

bool MapRequest::check_mlc_enable(double lc_map_tfinish) {
  auto current_lane = virtual_lane_mgr_->get_current_lane();
  auto& ego_state = session_->environmental_model().get_ego_state_manager();
  int lc_map_decision = current_lane != nullptr
                            ? virtual_lane_mgr_->lc_map_decision(current_lane)
                            : 0;
  const double kTmpRampLength = 100.;
  const double kResponseOffset = 300.;
  const double kDefaultMapDelay = 2.;

  double lc_end_dis = virtual_lane_mgr_->dis_to_ramp() - kTmpRampLength;
  if (lc_map_decision < 0) {
    lc_end_dis =
        virtual_lane_mgr_->distance_to_first_road_split() - kTmpRampLength;
  }

  double delay_map = 0;
  double v_limit =
      session_->environmental_model().get_ego_state_manager()->ego_v_cruise();

  std::array<double, 3> xp{40.0 / 3.6, 80.0 / 3.6, 120.0 / 3.6};
  std::array<double, 3> fp{300.0, 500.0, 800.0};
  double adaptor_interval = interp(v_limit, xp, fp);
  double map_response_dist =
      kResponseOffset + adaptor_interval * std::fabs(lc_map_decision);
  double v_ego = ego_state->ego_v();
  if ((lc_map_decision == 1 && lc_end_dis < 150.0) ||
      (lc_map_decision <= -1 && lc_end_dis < 150.0)) {
    delay_map = 0;
  } else if ((lc_map_decision == 2 || lc_map_decision == 3) &&
             (lc_end_dis <
              7 * v_ego + 200 * (std::fabs(lc_map_decision) - 1))) {
    delay_map = kDefaultMapDelay - 1;
  } else {
    delay_map = kDefaultMapDelay;
  }

  double map_duration = IflyTime::Now_s() - lc_map_tfinish;
  LOG_DEBUG(
      "MapRequest:: lc_map_tfinish: %f, map_duration: %f, lc_end_dis: %f,  "
      "map_response_dist: %f, delay_map: %f",
      lc_map_tfinish, map_duration, lc_end_dis, map_response_dist, delay_map);
  std::cout << "[MapRequest::update] : lc_end_dis: " << lc_end_dis << " lc_map_decision: " << lc_map_decision << " map_response_dist: " << map_response_dist << " map_duration: " << map_duration << " delay_map: " << delay_map << std::endl;

  return (lc_end_dis < map_response_dist && map_duration > delay_map);
}

void MapRequest::update(int lc_status, double lc_map_tfinish) {
  LOG_DEBUG("MapRequest::update");
  std::cout << "MAP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " << std::endl;

  auto current_lane_virtual_id = virtual_lane_mgr_->current_lane_virtual_id();
  if (lane_change_lane_mgr_->has_origin_lane()) {
    auto origin_lane = lane_change_lane_mgr_->olane();
    origin_lane_virtual_id_ = origin_lane->get_virtual_id();
    origin_lane_order_id_ = origin_lane->get_order_id();
    LOG_DEBUG("has_origin_lane: %d, origin_lane_virtual_id_: %d",
              origin_lane_order_id_, origin_lane_virtual_id_);
  } else {
    // if olane disappeared, replace it with current_lane
    origin_lane_virtual_id_ = current_lane_virtual_id;
    origin_lane_order_id_ =
        virtual_lane_mgr_->get_current_lane()->get_order_id();
    LOG_DEBUG(
        "no origin lane, origin_lane_order_id_ : %d, origin_lane_virtual_id_ : "
        "%d",
        origin_lane_order_id_, origin_lane_virtual_id_);
  }
  int target_lane_virtual_id_tmp{current_lane_virtual_id};
  auto current_lane = virtual_lane_mgr_->get_current_lane();
  int lc_map_decision = current_lane != nullptr
                            ? virtual_lane_mgr_->lc_map_decision(current_lane)
                            : 0;

  bool allow_cancel = (lc_status == ROAD_NONE || lc_status == ROAD_LC_LWAIT ||
                       lc_status == ROAD_LC_RWAIT);

  bool allow_generate =
      ((lc_status == ROAD_LC_LBACK || lc_status == ROAD_LC_RBACK) ||
       lc_status == ROAD_NONE || lc_status == ROAD_LC_LWAIT ||
       lc_status == ROAD_LC_RWAIT);

  auto& ego_state = session_->environmental_model().get_ego_state_manager();
  double v_ego = ego_state->ego_v();

  auto olane =
      virtual_lane_mgr_->get_lane_with_virtual_id(origin_lane_virtual_id_);

  if (current_lane != nullptr &&
      ((lc_map_decision > 0 && (!current_lane->is_solid_line(1) || virtual_lane_mgr_->dis_to_ramp() < 1000.)) ||
       (lc_map_decision < 0 && (!current_lane->is_solid_line(0) || virtual_lane_mgr_->dis_to_ramp() < 1000.)))) {
    LOG_DEBUG("!!!!!!!!!!! lc_map_decision is %d", lc_map_decision);
    if (check_mlc_enable(lc_map_tfinish) == true && allow_generate == true) {
      if (lc_map_decision < 0) {
        if (request_type_ != LEFT_CHANGE) {
          target_lane_virtual_id_tmp = origin_lane_virtual_id_ - 1;
          auto tlane = virtual_lane_mgr_->get_lane_with_virtual_id(
              target_lane_virtual_id_tmp);
          if (tlane != nullptr) {
            GenerateRequest(LEFT_CHANGE);
            set_target_lane_virtual_id(target_lane_virtual_id_tmp);
            LOG_DEBUG(
                "[MapRequest::update] Ask for map changing lane to left\n");
          } else {
            LOG_WARNING(
                "[MapRequest::update] Ask for map changing lane to left "
                "but left lane is null \n");
          }
        }

        if (!IsDashedLineEnough(LEFT_CHANGE, v_ego, virtual_lane_mgr_) &&
            request_type_ != NO_CHANGE &&
            (lc_status == ROAD_NONE || lc_status == ROAD_LC_LWAIT ||
             (lc_status == ROAD_LC_LBACK &&
              (lane_change_lane_mgr_->has_origin_lane() &&
               lane_change_lane_mgr_->is_ego_on(olane)))) &&
            virtual_lane_mgr_->distance_to_first_road_split() >
                300 + 300 * std::fabs(lc_map_decision)) {
          Finish();
          set_target_lane_virtual_id(current_lane_virtual_id);
          LOG_DEBUG("[MapRequest::update] : finish request, dashed not enough");
        }
      } else {
        if (request_type_ != RIGHT_CHANGE) {
          target_lane_virtual_id_tmp = origin_lane_virtual_id_ + 1;
          auto tlane = virtual_lane_mgr_->get_lane_with_virtual_id(
              target_lane_virtual_id_tmp);
          if (tlane != nullptr) {
            GenerateRequest(RIGHT_CHANGE);
            set_target_lane_virtual_id(target_lane_virtual_id_tmp);
            LOG_DEBUG(
                "[MapRequest::update] Ask for map changing lane to right\n");
          } else {
            LOG_WARNING(
                "[MapRequest::update] Ask for map changing lane to right "
                "but left lane is null \n");
          }
        }

        if (!IsDashedLineEnough(RIGHT_CHANGE, v_ego, virtual_lane_mgr_) &&
            request_type_ != NO_CHANGE &&
            (lc_status == ROAD_NONE || lc_status == ROAD_LC_RWAIT ||
             (lc_status == ROAD_LC_RBACK &&
              (lane_change_lane_mgr_->has_origin_lane() &&
               lane_change_lane_mgr_->is_ego_on(olane)))) &&
            virtual_lane_mgr_->dis_to_ramp() >
                300 + 300 * std::fabs(lc_map_decision)) {
          Finish();
          set_target_lane_virtual_id(current_lane_virtual_id);
          LOG_DEBUG("[MapRequest::update] : finish request, dashed not enough");
        }
      }
    }
  } else if (allow_cancel) {
    if (request_type_ != NO_CHANGE) {
      Finish();
      set_target_lane_virtual_id(current_lane_virtual_id);
      LOG_DEBUG("[MapRequest::update] cancel map request as allow cancel");
    }
  }

  LOG_DEBUG("MapRequest::update: finished");
}

}  // namespace planning
#include "map_lane_change_request.h"
#include "debug_info_log.h"

namespace planning {
const double kNeedLaneChangeTime = 4.0;
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
  //TODO(fengwang31):目前自车在最左侧车道上时，无法确认到最右边有几条车道，因此hack一下，判断自车在左侧，提前产生变道任务
  bool is_current_lane_on_leftmost = false;
  auto right_lane = virtual_lane_mgr_->get_right_lane();
  if (right_lane) {
    bool is_right_lane_boundary_both_dash = !right_lane->is_solid_line(0) && !right_lane->is_solid_line(1);
    bool is_current_lane_is_leftmost =  current_lane->is_solid_line(0) && !current_lane->is_solid_line(1);
    is_current_lane_on_leftmost = is_current_lane_is_leftmost && is_right_lane_boundary_both_dash;
  }
  if (lc_map_decision == 1) {
    lc_map_decision = is_current_lane_on_leftmost ? 2 : 1;
  }
  const double kTmpRampLength = 100.;
  const double kResponseOffset = 300.;
  const double kDefaultMapDelay = 2.;

  double lc_end_dis = virtual_lane_mgr_->dis_to_ramp() - kTmpRampLength;

  double delay_map = 0;
  double v_limit =
      session_->environmental_model().get_ego_state_manager()->ego_v_cruise();

  std::array<double, 3> xp{40.0 / 3.6, 80.0 / 3.6, 120.0 / 3.6};
  std::array<double, 3> fp{500.0, 800.0, 1200.0};
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
  if (lc_map_decision < 0) {
    lc_end_dis = 0.;
    map_duration = 10.;  // hack
  }
  LOG_DEBUG(
      "MapRequest:: lc_map_tfinish: %f, map_duration: %f, lc_end_dis: %f,  "
      "map_response_dist: %f, delay_map: %f",
      lc_map_tfinish, map_duration, lc_end_dis, map_response_dist, delay_map);
  std::cout << "[MapRequest::update] : lc_end_dis: " << lc_end_dis
            << " lc_map_decision: " << lc_map_decision
            << " map_response_dist: " << map_response_dist
            << " map_duration: " << map_duration << " delay_map: " << delay_map
            << "lc_map_tfinish: " << lc_map_tfinish
            << " IflyTime::Now_s(): " << IflyTime::Now_s() << std::endl;

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
  JSON_DEBUG_VALUE("lc_map_decision", lc_map_decision);

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
  std::cout << "lc_map_decision: " << lc_map_decision
            << " current_lane->is_solid_line(1): "
            << current_lane->is_solid_line(1)
            << " current_lane->is_solid_line(0): "
            << current_lane->is_solid_line(0) << std::endl;

  if (current_lane != nullptr &&
      ((lc_map_decision > 0 && (!current_lane->is_solid_line(1))) ||
       (lc_map_decision < 0 && (!current_lane->is_solid_line(0))))) {
    LOG_DEBUG("!!!!!!!!!!! lc_map_decision is %d", lc_map_decision);
    if (check_mlc_enable(lc_map_tfinish) == true && allow_generate == true) {
      if (lc_map_decision < 0) {
        std::cout << "request_type_:" << request_type_ << std::endl;
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

        if (!IsDashEnoughForRepeatSegments(lc_map_decision, current_lane)) {
          Finish();
          set_target_lane_virtual_id(current_lane_virtual_id);
          LOG_DEBUG(
              "[MapRequest::update] : mlc finish request, dashed not enough");
        }

        // if (!IsDashedLineEnough(LEFT_CHANGE, v_ego, virtual_lane_mgr_) &&
        //     request_type_ != NO_CHANGE &&
        //     (lc_status == ROAD_NONE || lc_status == ROAD_LC_LWAIT ||
        //      (lc_status == ROAD_LC_LBACK &&
        //       (lane_change_lane_mgr_->has_origin_lane() &&
        //        lane_change_lane_mgr_->is_ego_on(olane)))) &&
        //     virtual_lane_mgr_->distance_to_first_road_split() >
        //         300 + 300 * std::fabs(lc_map_decision)) {
        //   Finish();
        //   set_target_lane_virtual_id(current_lane_virtual_id);
        //   LOG_DEBUG("[MapRequest::update] : finish request, dashed not
        //   enough");
        // }
      } else {
        if (request_type_ != RIGHT_CHANGE) {
          target_lane_virtual_id_tmp = origin_lane_virtual_id_ + 1;
          auto tlane = virtual_lane_mgr_->get_lane_with_virtual_id(
              target_lane_virtual_id_tmp);
          if (tlane != nullptr) {
            auto tlane_c_poly = tlane->c_poly();
            if (virtual_lane_mgr_->dis_to_ramp() > 500. &&
                tlane_c_poly[0] > -2.8) {
              LOG_WARNING(
                  "[MapRequest::update] Ask for map changing lane to right "
                  "but split canceled \n");
            } else {
              GenerateRequest(RIGHT_CHANGE);
              set_target_lane_virtual_id(target_lane_virtual_id_tmp);
              LOG_DEBUG(
                  "[MapRequest::update] Ask for map changing lane to right\n");
            }
          } else {
            LOG_WARNING(
                "[MapRequest::update] Ask for map changing lane to right "
                "but left lane is null \n");
          }
        }

        if (!IsDashEnoughForRepeatSegments(lc_map_decision, current_lane) &&
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
bool MapRequest::IsDashEnoughForRepeatSegments(
    const int lc_map_decision,
    std::shared_ptr<VirtualLane> current_lane) const {
  const double ego_v =
      session_->environmental_model().get_ego_state_manager()->ego_v();
  double dash_length = 0;
  bool all_lane_boundary_types_are_dashed = true;
  double default_lc_boundary_length = 100.0;
  if (lc_map_decision < 0) {
    const auto& left_lane_boundarys = current_lane->get_left_lane_boundary();
    for (int i = 0; i < left_lane_boundarys.type_segments_size; i++) {
      if (left_lane_boundarys.type_segments[i].type ==
          iflyauto::LaneBoundaryType_MARKING_DASHED ||
          left_lane_boundarys.type_segments[i].type ==
          iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
        dash_length += left_lane_boundarys.type_segments[i].length;
      } else {
        all_lane_boundary_types_are_dashed = false;
        break;
      }
    }
  } else if (lc_map_decision > 0) {
    const auto& right_lane_boundarys = current_lane->get_right_lane_boundary();
    for (int i = 0; i < right_lane_boundarys.type_segments_size; i++) {
      if (right_lane_boundarys.type_segments[i].type ==
          iflyauto::LaneBoundaryType_MARKING_DASHED ||
          right_lane_boundarys.type_segments[i].type ==
          iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
        dash_length += right_lane_boundarys.type_segments[i].length;
      } else {
        all_lane_boundary_types_are_dashed = false;
        break;
      }
    }
  }
  const double map_response_dist = ego_v * kNeedLaneChangeTime;  // hack
  std::cout << "dash_length:" << dash_length
            << ",map_response_dist:" << map_response_dist << std::endl;
  if (dash_length > default_lc_boundary_length || all_lane_boundary_types_are_dashed || dash_length > map_response_dist) {
    return true;
  }
  // if (dash_length < default_lc_boundary_length && !all_lane_boundary_types_are_dashed_) {
  //   std::cout << "dash lengh less than map response dist!!!!" << std::endl;
  //   return false;
  // }
  std::cout << "dash lengh less than map response dist!!!!" << std::endl;
  return false;
}

}  // namespace planning
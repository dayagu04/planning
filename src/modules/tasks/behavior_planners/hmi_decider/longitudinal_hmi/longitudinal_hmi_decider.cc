#include "modules/tasks/behavior_planners/hmi_decider/longitudinal_hmi/longitudinal_hmi_decider.h"
#include "modules/context/planning_context.h"
#include "modules/context/environmental_model_manager.h"
#include "modules/context/traffic_light_decision_manager.h"

namespace planning {

LongitudinalHmiDecider::LongitudinalHmiDecider(framework::Session* session,
                                               const HmiDeciderConfig& config)
    : session_(session), config_(config) {}

void LongitudinalHmiDecider::IntersectionLeftRightLaneTakeOverProc() {
  const auto &virtual_lane_manager =
    session_->environmental_model().get_virtual_lane_manager();
  const auto current_ego_lane_mark =
    virtual_lane_manager->lane_mark_at_ego_front_edge_pos_current();

  const auto distance_to_stop_line =
    virtual_lane_manager->GetEgoDistanceToStopline();
  const auto distance_to_crosswalk =
    virtual_lane_manager->GetEgoDistanceToCrosswalk();

  if (distance_to_stop_line > config_.left_right_lane_mild_dis &&
      distance_to_crosswalk > config_.left_right_lane_mild_dis) {
    return;
  }

  auto& planning_output = session_->mutable_planning_context()
                      ->mutable_planning_output();
  planning_output.planning_request.take_over_req_level = iflyauto::REQUEST_LEVEL_MILD;
  if (distance_to_stop_line < config_.left_right_lane_middle_dis ||
      distance_to_crosswalk < config_.left_right_lane_middle_dis) {
    planning_output.planning_request.take_over_req_level = iflyauto::REQUEST_LEVEL_MIDDLE;
  }

  static const std::unordered_set<iflyauto::LaneDrivableDirection>
    left_turning_direction_set = {
        iflyauto::LaneDrivableDirection::LaneDrivableDirection_DIRECTION_LEFT,
        iflyauto::LaneDrivableDirection::
            LaneDrivableDirection_DIRECTION_UTURN_LEFT,
        iflyauto::LaneDrivableDirection::
            LaneDrivableDirection_DIRECTION_LEFT_UTURN,
        iflyauto::LaneDrivableDirection::
            LaneDrivableDirection_DIRECTION_LEFT_RIGHT,
        iflyauto::LaneDrivableDirection::
            LaneDrivableDirection_DIRECTION_UTURNLEFT_RIGHT};

  static const std::unordered_set<iflyauto::LaneDrivableDirection>
    right_turning_direction_set = {
        iflyauto::LaneDrivableDirection::
            LaneDrivableDirection_DIRECTION_RIGHT,
        iflyauto::LaneDrivableDirection::
            LaneDrivableDirection_DIRECTION_UTURN_RIGHT,
        iflyauto::LaneDrivableDirection::
            LaneDrivableDirection_DIRECTION_RIGHT_UTURN};

  if (left_turning_direction_set.find(current_ego_lane_mark) !=
            left_turning_direction_set.end()) {
    planning_output.planning_request.request_reason =
              iflyauto::RequestReason::REQUEST_REASON_ON_INTERSECTION_LEFT_LANE;
  } else if (right_turning_direction_set.find(current_ego_lane_mark) !=
                   right_turning_direction_set.end()) {
    planning_output.planning_request.request_reason =
              iflyauto::RequestReason::REQUEST_REASON_ON_INTERSECTION_RIGHT_LANE;
  }

}

bool LongitudinalHmiDecider::Execute() {
  if (!session_) {
    return false;
  }
  auto& ad_info = session_->mutable_planning_context()
                      ->mutable_planning_hmi_info()
                      ->ad_info;
  const auto tfl_manager =
      session_->environmental_model().get_traffic_light_decision_manager();
  const auto traffic_status = tfl_manager->GetTrafficStatus();
  bool is_red_tfl = traffic_status.go_straight == 1 || traffic_status.go_straight == 41 ||
                    traffic_status.go_straight == 11 || traffic_status.go_straight == 10;
  const auto& tfl_decider = session_->mutable_planning_context()
                                ->mutable_traffic_light_decider_output();

  double dis_to_stopline = session_->environmental_model()
                               .get_virtual_lane_manager()
                               ->GetEgoDistanceToStopline();
  double dis_to_crosswalk = session_->environmental_model()
                               .get_virtual_lane_manager()
                               ->GetEgoDistanceToCrosswalk();
  const auto& ego_vehi_param =
               VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double ego_rear_axle_to_front_edge =
               ego_vehi_param.length - ego_vehi_param.rear_edge_to_rear_axle;

  planning::common::IntersectionState intersection_state =
      session_->environmental_model()
          .get_virtual_lane_manager()
          ->GetIntersectionState();

  ad_info.intersection_state = iflyauto::IntersectionState(intersection_state);

  const auto lateral_obstacles = session_->environmental_model().get_lateral_obstacle();
  auto start_stop_info =
      session_->planning_context().start_stop_result().state();

  // update intersection traffic lights reminder hmi info
  constexpr uint8 kIntersectionStatusNone = 8;
  ad_info.intersection_pass_sts =
      iflyauto::IntersectionPassSts(kIntersectionStatusNone);
  if (tfl_decider.is_in_straight_lane && is_red_tfl && !tfl_decider.can_pass && start_stop_info != common::StartStopInfo::STOP &&
      intersection_state == planning::common::APPROACH_INTERSECTION && (!tfl_decider.is_small_front_intersection ||
      tfl_decider.is_tfl_match_intersection) && (lateral_obstacles->leadone() == nullptr ||
      (lateral_obstacles->leadone()->d_s_rel() + ego_rear_axle_to_front_edge > dis_to_stopline + config_.tfl_reminder_cipv_dis ||
      lateral_obstacles->leadone()->d_s_rel() + ego_rear_axle_to_front_edge > dis_to_crosswalk + config_.tfl_reminder_cipv_dis))) {
    ad_info.intersection_pass_sts =
        iflyauto::IntersectionPassSts::INTERSECTION_RED_LIGHT_STOP;
  }
  JSON_DEBUG_VALUE("intersection_pass_sts", int(ad_info.intersection_pass_sts));

  //intersection left/right takeover request
  if (!tfl_decider.is_in_straight_lane) {
    IntersectionLeftRightLaneTakeOverProc();
  }
  return true;
}
}  // namespace planning

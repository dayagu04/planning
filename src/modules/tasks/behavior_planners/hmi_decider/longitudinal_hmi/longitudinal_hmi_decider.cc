#include "modules/tasks/behavior_planners/hmi_decider/longitudinal_hmi/longitudinal_hmi_decider.h"
#include "modules/context/planning_context.h"
#include "modules/context/environmental_model_manager.h"
#include "modules/context/traffic_light_decision_manager.h"

namespace planning {

LongitudinalHmiDecider::LongitudinalHmiDecider(framework::Session* session,
                                               const HmiDeciderConfig& config)
    : session_(session), config_(config) {}

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

  // update intersection traffic lights reminder hmi info
  constexpr uint8 kIntersectionStatusNone = 8;
  ad_info.intersection_pass_sts =
      iflyauto::IntersectionPassSts(kIntersectionStatusNone);
  if (is_red_tfl && !tfl_decider.can_pass && intersection_state ==
      planning::common::APPROACH_INTERSECTION && (!tfl_decider.is_small_front_intersection ||
      tfl_decider.is_tfl_match_intersection) && (lateral_obstacles->leadone() == nullptr ||
      (lateral_obstacles->leadone()->d_s_rel() + ego_rear_axle_to_front_edge > dis_to_stopline + config_.tfl_reminder_cipv_dis ||
      lateral_obstacles->leadone()->d_s_rel() + ego_rear_axle_to_front_edge > dis_to_crosswalk + config_.tfl_reminder_cipv_dis))) {
    ad_info.intersection_pass_sts =
        iflyauto::IntersectionPassSts::INTERSECTION_RED_LIGHT_STOP;
  }
  JSON_DEBUG_VALUE("intersection_pass_sts", int(ad_info.intersection_pass_sts));
  return true;
}
}  // namespace planning

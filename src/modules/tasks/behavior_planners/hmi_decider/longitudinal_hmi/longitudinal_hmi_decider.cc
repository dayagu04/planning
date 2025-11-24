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
  const auto& tfl_decider = session_->mutable_planning_context()
                                ->mutable_traffic_light_decider_output();
  planning::common::IntersectionState intersection_state =
      session_->environmental_model()
          .get_virtual_lane_manager()
          ->GetIntersectionState();
  const auto cipv_info = session_->planning_context().cipv_decider_output();

  // update intersection traffic lights reminder hmi info
  constexpr uint8 kIntersectionStatusNone = 8;
  ad_info.intersection_pass_sts =
      iflyauto::IntersectionPassSts(kIntersectionStatusNone);
  if ((traffic_status.go_straight != 3 && traffic_status.go_straight != 43) &&
      intersection_state == planning::common::APPROACH_INTERSECTION &&
      (!tfl_decider.is_small_front_intersection ||
       tfl_decider.is_tfl_match_intersection) &&
      (cipv_info.cipv_id() == -1 ||
       cipv_info.relative_s() > config_.tfl_reminder_cipv_dis)) {
    ad_info.intersection_pass_sts =
        iflyauto::IntersectionPassSts::INTERSECTION_RED_LIGHT_STOP;
  }
  JSON_DEBUG_VALUE("intersection_pass_sts", int(ad_info.intersection_pass_sts));
  return true;
}
}  // namespace planning

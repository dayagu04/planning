#include "modules/tasks/behavior_planners/hmi_decider/longitudinal_hmi/longitudinal_hmi_decider.h"
#include "modules/context/planning_context.h"
#include "modules/context/environmental_model_manager.h"
#include "modules/context/traffic_light_decision_manager.h"

namespace planning {
namespace {
constexpr int32_t kLonCollisionCountThred = 3;
}
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
  auto& planning_output = session_->mutable_planning_context()
                      ->mutable_planning_output();

  const auto tfl_manager =
      session_->environmental_model().get_traffic_light_decision_manager();
  const auto traffic_status = tfl_manager->GetTrafficStatus();
  const auto& tfl_decider = session_->mutable_planning_context()
                                ->mutable_traffic_light_decider_output();
  planning::common::IntersectionState intersection_state =
      session_->environmental_model()
          .get_virtual_lane_manager()
          ->GetIntersectionState();

  ad_info.intersection_state = iflyauto::IntersectionState(intersection_state);

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

  //lon collision check by traj deceleration less than thred by multi-frame
  const auto& raw_traj_points =
      session_->planning_context().planning_result().raw_traj_points;
  if (std::any_of(raw_traj_points.begin(), raw_traj_points.end(),
                    [&](const TrajectoryPoint& traj_point) {
                        return traj_point.a < config_.lon_collision_dec_thred;
                    })) {
    lon_collision_count_.first =
        std::min(++(lon_collision_count_.first), kLonCollisionCountThred);
  } else {
    lon_collision_count_.first =
        std::max(--(lon_collision_count_.first), 0);
  }

  // update flag
  if (lon_collision_count_.first == kLonCollisionCountThred) {
    lon_collision_count_.second = 1;
  }
  if (lon_collision_count_.first == 0) {
    lon_collision_count_.second = 0;
  }

  if (lon_collision_count_.second > 0) {
    planning_output.planning_request.take_over_req_level = iflyauto::REQUEST_LEVEL_WARRING;
    planning_output.planning_request.request_reason = iflyauto::REQUEST_REASON_LON_COLLISION_RISK;
  }
  return true;
}

}  // namespace planning

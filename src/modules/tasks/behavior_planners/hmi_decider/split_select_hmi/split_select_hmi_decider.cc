#include "modules/tasks/behavior_planners/hmi_decider/split_select_hmi/split_select_hmi_decider.h"
#include "common/ifly_time.h"
#include "modules/context/environmental_model_manager.h"
#include "modules/context/planning_context.h"
#include "modules/context/virtual_lane_manager.h"
#include "planning_hmi_c.h"

namespace planning {

SplitSelectHmiDecider::SplitSelectHmiDecider(framework::Session* session)
    : session_(session) {}

bool SplitSelectHmiDecider::Execute() {
  if (!session_) {
    return false;
  }

  const auto& function_info = session_->environmental_model().function_info();
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto& route_info_output =
      session_->environmental_model().get_route_info()->get_route_info_output();
  auto& ad_info = session_->mutable_planning_context()
                      ->mutable_planning_hmi_info()
                      ->ad_info;
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const bool enable_output_split_select_classical_chinese =
      virtual_lane_manager->get_enable_output_split_select_classical_chinese();
  const SplitSelectDirection split_select_direction =
      virtual_lane_manager->get_split_select_direction();
  const auto curr_state = lane_change_decider_output.curr_state;
  ad_info.split_select_direction =
      iflyauto::SplitSelectDirection::NO_SPLIT_SPLIT;
  if (function_info.function_mode() != common::DrivingFunctionInfo::SCC) {
    return true;
  }
  if (!virtual_lane_manager
           ->get_enable_output_split_select_classical_chinese()) {
    return true;
  }
  // 过滤自车处于路口中的状态
  UpdateIntersection();
  if (ego_in_intersection_state_) {
    ILOG_DEBUG << "SplitSelectHmiDecider::ego not in intersection!";
    return true;
  }

  if (curr_state == kLaneKeeping || curr_state == kLaneChangePropose) {
    if (split_select_direction == SPLIT_SELECT_LEFT_LANE) {
      ad_info.split_select_direction =
          iflyauto::SplitSelectDirection::SPLIT_SELECT_TO_LEFT;
    } else if (split_select_direction == SPLIT_SELECT_RIGHT_LANE) {
      ad_info.split_select_direction =
          iflyauto::SplitSelectDirection::SPLIT_SELECT_TO_RIGHT;
    } else {
      ad_info.split_select_direction =
          iflyauto::SplitSelectDirection::NO_SPLIT_SPLIT;
    }
  }

  return true;
}

void SplitSelectHmiDecider::UpdateIntersection() {
  const auto& tfl_decider = session_->mutable_planning_context()
                                ->mutable_traffic_light_decider_output();
  const auto intersection_state = session_->environmental_model()
                                      .get_virtual_lane_manager()
                                      ->GetIntersectionState();
  const double distance_to_stopline = session_->environmental_model()
                                          .get_virtual_lane_manager()
                                          ->GetEgoDistanceToStopline();
  const double distance_to_crosswalk = session_->environmental_model()
                                           .get_virtual_lane_manager()
                                           ->GetEgoDistanceToCrosswalk();
  bool current_intersection_state =
      intersection_state == common::IntersectionState::IN_INTERSECTION ||
      distance_to_stopline <= 15.0;
  bool is_small_intersection = false;
  // bool is_small_intersection = tfl_decider.is_small_front_intersection;
  // distance_to_crosswalk <= kDistanceThresholdApproachToCrosswalk;
  if (current_intersection_state) {
    intersection_count_ = 3;
  } else {
    intersection_count_ = std::max(intersection_count_ - 1, 0);
  }

  ego_in_intersection_state_ = intersection_count_ > 0;
  return;
}

}  // namespace planning
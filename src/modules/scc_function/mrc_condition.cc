#include "src/modules/scc_function/mrc_condition.h"

#include "mrc_condition.h"

namespace planning {
void MrcCondition::init() { set_mrc_config(); }

void MrcCondition::reset_before_process() {
  request_type_change_ = false;
  right_lane_has_emergency_ = false;
}

void MrcCondition::update() {
  reset_before_process();

  update_external_variables();

  process();
}

void MrcCondition::update_external_variables() {
  // auto virtual_lane_mgr =
  //     session_->mutable_environmental_model()->virtual_lane_manager();
  // // auto display_state =
  // //     session_->mutable_environmental_model()->scenario_state_display();
  // auto clane = virtual_lane_mgr->current_lane();
  // auto ramp = virtual_lane_mgr->curr_ramp();
  // dbw_status_ = session_->environmental_model().GetVehicleDbwStatus();
  // // display_state_ = display_state->display_state();
  // merge_split_point_distance_ = virtual_lane_mgr->cal_merge_split_dis();
  // clane_order_id_from_right_ =
  //     clane == nullptr ? -1 :
  //     clane->get_car_cursor().laneId().getLaneCount();
  // if (ramp.isValid()) {
  //   distance_to_ramp_ = (ramp.hasLanePathOffset() ? ramp.getLanePathOffset()
  //                                                 : ramp.getRoadOffset());
  // } else {
  //   distance_to_ramp_ = std::numeric_limits<double>::max();
  // }

  // /*check whether right has emergency type */
  // check_right_lane_type();

  // // NLOGI(
  // //     "[MrcCondition] clane_order_id_from_right_: %d, mrc_request_type_: "
  // //     "%d, distance_to_ramp_ : %f",
  // //     clane_order_id_from_right_, mrc_request_type_, distance_to_ramp_);
  // // MDEBUG_JSON_ADD_ITEM(dis_to_ramp, distance_to_ramp_, MrcCondition)
  // // MDEBUG_JSON_ADD_ITEM(mrc_request_type,
  // static_cast<int>(mrc_request_type_),
  // //                      MrcCondition)
  // // MDEBUG_JSON_ADD_ITEM(pre_mrc_request_type,
  // //                      static_cast<int>(pre_mrc_request_type_),
  // MrcCondition)
  // // MDEBUG_JSON_ADD_ITEM(display_state, static_cast<int>(display_state_),
  // //                      MrcCondition)
  // // MDEBUG_JSON_ADD_ITEM(pre_display_state,
  // static_cast<int>(pre_display_state_),
  // //                      MrcCondition)
  // // MDEBUG_JSON_ADD_ITEM(clane_order_id_from_right,
  // clane_order_id_from_right_,
  // //                      MrcCondition)
  // // MDEBUG_JSON_ADD_ITEM(right_lane_has_emergency,
  // right_lane_has_emergency_,
  // //                      MrcCondition)
  // // MDEBUG_JSON_ADD_ITEM(merge_split_point_distance,
  // merge_split_point_distance_,
  // //                      MrcCondition)
}

void MrcCondition::check_inline_brake_condition() { ; }

void MrcCondition::execute_pull_over() { ; }

void MrcCondition::process() {
  if (!dbw_status_ || !enable_mrc_condition_) {
    clear_all();
  }
  if (mrc_request_type_ == MrcRequestType::NO_RQT) return;

  decide_mrc_type();
  switch (mrc_execute_type_) {
    case MrcExecuteType::NOT_EXECUTE:
      execute_type_output_ = MrcExecuteType::NOT_EXECUTE;
      break;
    case MrcExecuteType::PULL_OVER:
      execute_type_output_ = MrcExecuteType::PULL_OVER;
      execute_pull_over();
      break;
    case MrcExecuteType::INLANE_BRAKE:
      execute_type_output_ = pull_over_to_brake_ ? MrcExecuteType::PULL_OVER
                                                 : MrcExecuteType::INLANE_BRAKE;
      check_inline_brake_condition();
      break;
    case MrcExecuteType::BLIND_BRAKE:
      mrc_brake_type_ = MrcBrakeType::EMERGENCY_BRAKE;
      execute_type_output_ = MrcExecuteType::BLIND_BRAKE;
      break;
    default:
      LOG_DEBUG("%s", "[MrcCondition] INVALID MRC EXECUTION \n");
      break;
  }
  if (mrc_execute_type_ > MrcExecuteType::PULL_OVER) {
    has_decided_pull_over_acc_ = false;
  }
  // MDEBUG_JSON_ADD_ITEM(mrc_execute_type, static_cast<int>(mrc_execute_type_),
  //                      MrcCondition)
  // MDEBUG_JSON_ADD_ITEM(mrc_brake_type, static_cast<int>(mrc_brake_type_),
  //                      MrcCondition)
  // MDEBUG_JSON_ADD_ITEM(has_decided_pull_over_acc, has_decided_pull_over_acc_,
  //                      MrcCondition)
}

void MrcCondition::decide_mrc_type() { ; }

void MrcCondition::mrc_engage_p_gear(double ego_v) { ; }

void MrcCondition::mrc_brake_execute(LonRefPath& lon_ref_path,
                                     PlanningResult& ego_prediction_result,
                                     PlanningInitPoint& planning_init_point) {
  ;
}

void MrcCondition::update_inline_brake_by_obstacle(
    LonDecisionInfo& lon_decision_information,
    PlanningInitPoint& planning_init_point) {
  ;
}

void MrcCondition::clear_all() {
  mrc_request_type_ = MrcRequestType::NO_RQT;
  pre_mrc_request_type_ = MrcRequestType::NO_RQT;
  mrc_execute_type_ = MrcExecuteType::NOT_EXECUTE;
  mrc_brake_type_ = MrcBrakeType::NOT_BRAKE;
  mrc_engage_p_gear_ = false;
}

void MrcCondition::set_mrc_config() {
  enable_mrc_condition_ = config_.enable_mrc_condition;
  fcw_threshold_ = config_.fcw_threshold;
  a_brake_slow_ = config_.a_brake_slow;
  a_brake_hard_ = config_.a_brake_hard;
  a_brake_emergency_ = config_.a_brake_emergency;
  jerk_brake_slow_ = config_.jerk_brake_slow;
  jerk_brake_hard_ = config_.jerk_brake_hard;
  jerk_brake_emergency_ = config_.jerk_brake_emergency;
  v_pull_over_threshold_ = config_.v_pull_over_threshold;
  time_out_threshold_ = config_.time_out_threshold;
  construction_zone_dist_threshold_ = config_.construction_zone_dist_threshold;
  ramp_dist_threshold_ = config_.ramp_dist_threshold;
  split_merge_dist_threshold_ = config_.split_merge_dist_threshold;
  tunnel_dist_threshold_ = config_.tunnel_dist_threshold;
}

}  // namespace planning
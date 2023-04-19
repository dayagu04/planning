#include "src/modules/scc_function/adaptive_cruise_control.h"

#include "adaptive_cruise_control.h"

namespace planning {

AdaptiveCruiseControl::AdaptiveCruiseControl(
    const EgoPlanningConfigBuilder* config_builder,
    framework::Session* session) {}

void AdaptiveCruiseControl::adaptive_cruise_control(
    LonDecisionInfo& lon_decision_information,
    AdaptiveCruiseControlInfo& acc_info,
    PlanningResult& ego_prediction_result) {
  auto ego_state = session_->environmental_model().get_ego_state_manager();

  // const double ego_v = ego_state->velocity();
  // const double ego_a = ego_state->acc();
}

std::pair<double, double> AdaptiveCruiseControl::calculate_max_acc(
    double ego_v) {
  return std::pair<double, double>();
}

void AdaptiveCruiseControl::acc_update_ds_refs(
    AdaptiveCruiseControlInfo& acc_info, LonRefPath& lon_ref_path,
    PlanningResult& ego_prediction_result,
    PlanningInitPoint& planning_init_point) {}

void AdaptiveCruiseControl::acc_update_jerk_bound(
    AdaptiveCruiseControlInfo& acc_info, LonRefPath& lon_ref_path,
    PlanningResult& ego_prediction_result,
    PlanningInitPoint& planning_init_point) {}

bool AdaptiveCruiseControl::recognize_model_speed_up(
    PlanningResult& ego_prediction_result) {
  return false;
}

}  // namespace planning
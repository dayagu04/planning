#include "adaptive_cruise_control.h"

#include "ego_state_manager.h"
#include "environmental_model.h"

namespace planning {

AdaptiveCruiseControl::AdaptiveCruiseControl(
    const EgoPlanningConfigBuilder* config_builder,
    framework::Session* session) {
  session_ = session;
  config_ = config_builder->cast<AdaptiveCruiseControlConfig>();
  common::SceneType scene_type = session_->get_scene_type();
  auto config_all = session_->environmental_model().config_builder(scene_type);
  config_lon_ = config_all->cast<LongitudinalDeciderV3Config>();
}

void AdaptiveCruiseControl::adaptive_cruise_control(
    LonDecisionInfo& lon_decision_information,
    AdaptiveCruiseControlInfo& acc_info,
    PlanningResult& ego_prediction_result) {
  // auto ego_state = session_->environmental_model().get_ego_state_manager();
  auto& ego_state = session_->environmental_model().get_ego_state_manager();

  const double ego_v = ego_state->ego_v();
  const double ego_a = ego_state->ego_acc();
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
    PlanningInitPoint& planning_init_point) {
  const double ego_v = planning_init_point.v;
  const double ego_a = planning_init_point.a;
  double a_threshold = 0.0;
  double jerk_upper_bound = -0.2;
  double jerk_lower_bound = 0.2;
  bool model_speed_up = recognize_model_speed_up(ego_prediction_result);
  size_t acc_jerk_control_num = config_.acc_jerk_control_num;
  for (size_t i = 0; i < ego_prediction_result.traj_points.size(); i++) {
    if (acc_info.navi_time_distance_flag &&
        acc_info.navi_time_distance_info.enable_jerk_down &&
        i < acc_jerk_control_num) {
      if (ego_a < -0.5) {
        jerk_upper_bound = 7;
      } else if (ego_a > 0) {
        jerk_upper_bound = -0.2;
      } else {
        jerk_upper_bound = -0.2 * ego_a - 0.2;
      }
      lon_ref_path.lon_bound_jerk.emplace_back(Bound{-7.0, jerk_upper_bound});
    } else if (acc_info.navi_time_distance_flag &&
               acc_info.navi_time_distance_info.enable_jerk_up &&
               i < acc_jerk_control_num && !model_speed_up) {
      if (ego_v < 10.0) {
        a_threshold = 0.5;
      } else if (ego_v > 30.0) {
        a_threshold = 0;
      } else {
        a_threshold = -0.025 * ego_v + 0.75;  // x/(100/3.6) + y/0.4 = 1
      }
      if (ego_a > a_threshold) {
        jerk_lower_bound = -7;
      } else {
        jerk_lower_bound = 0.2;
      }
      lon_ref_path.lon_bound_jerk.emplace_back(Bound{jerk_lower_bound, 7.0});
    } else {
      lon_ref_path.lon_bound_jerk.emplace_back(Bound{-7.0, 7.0});
    }
  }
}

bool AdaptiveCruiseControl::recognize_model_speed_up(
    PlanningResult& ego_prediction_result) {
  return false;
}

}  // namespace planning
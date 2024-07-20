#include "lateral_motion_planning_weight.h"

#include "math_lib.h"
#include "refline.h"

namespace pnc {
namespace lateral_planning {

LateralMotionPlanningWeight::LateralMotionPlanningWeight(
    const planning::LateralMotionPlannerConfig &config)
    : config_(config) {}

void LateralMotionPlanningWeight::SetLateralMotionWeight(
    const LateralMotionSceneEnum scene,
    planning::common::LateralPlanningInput &planning_input) {
  lateral_motion_scene_ = scene;
  // CalculateInitInfo(planning_input);
  // set cost weight by scene
  switch (scene) {
    case LANE_KEEP: {
      planning_input.set_q_ref_x(config_.q_ref_x);
      planning_input.set_q_ref_y(config_.q_ref_y);
      planning_input.set_q_ref_theta(config_.q_ref_theta);
      planning_input.set_q_continuity(config_.q_continuity);
      planning_input.set_q_acc(config_.q_acc);
      planning_input.set_q_jerk(config_.q_jerk);
      SetAccJerkBoundByVelocity(planning_input);
      MakeDynamicWeight(planning_input);
      break;
    }
    case AVOID: {
      planning_input.set_q_ref_x(config_.q_ref_x_avoid);
      planning_input.set_q_ref_y(config_.q_ref_y_avoid);
      planning_input.set_q_ref_theta(config_.q_ref_theta_avoid);
      planning_input.set_q_continuity(config_.q_continuity);
      planning_input.set_q_acc(config_.q_acc_avoid);
      planning_input.set_q_jerk(config_.q_jerk_avoid);
      SetAccJerkBoundByVelocity(planning_input);
      concerned_start_q_jerk_ = config_.q_jerk_avoid;
      break;
    }
    case LANE_CHANGE: {
      planning_input.set_acc_bound(config_.acc_bound_lane_change);
      planning_input.set_jerk_bound(config_.jerk_bound_lane_change);
      planning_input.set_q_ref_x(config_.q_ref_x_lane_change);
      planning_input.set_q_ref_y(config_.q_ref_y_lane_change);
      planning_input.set_q_ref_theta(config_.q_ref_theta_lane_change);
      planning_input.set_q_continuity(config_.q_continuity);
      planning_input.set_q_acc(config_.q_acc_lane_change);
      if (std::fabs(ego_l_) > config_.lane_change_ego_l_thr) {
        planning_input.set_q_jerk(config_.q_jerk_lane_change);
        concerned_start_q_jerk_ = config_.q_jerk_lane_change;
      } else {
        planning_input.set_q_jerk(config_.q_jerk_lane_change2);
        concerned_start_q_jerk_ = config_.q_jerk_lane_change2;
      }
      if ((is_lane_change_back_)) {
        planning_input.set_q_jerk(config_.q_jerk_lane_change_back);
        concerned_start_q_jerk_ = config_.q_jerk_lane_change_back;
      }
      break;
    }
    default: { break; }
  }
  planning_input.set_q_acc_bound(config_.q_acc_bound);
  planning_input.set_q_jerk_bound(config_.q_jerk_bound);
}

void LateralMotionPlanningWeight::CalculateInitInfo(
    const planning::common::LateralPlanningInput &planning_input) {
  const double a = planning_input.ref_y_vec(1) - planning_input.ref_y_vec(0);
  const double b = planning_input.ref_x_vec(0) - planning_input.ref_x_vec(1);
  const double c = planning_input.ref_y_vec(0) * planning_input.ref_x_vec(1) -
                   planning_input.ref_x_vec(0) * planning_input.ref_y_vec(1);
  const double d = Square(a) + Square(b);
  if (d > 1e-8) {
    init_dis_to_ref_ = -(a * planning_input.init_state().x() +
                         b * planning_input.init_state().y() + c) /
                       d;
  } else {
    double x_error =
        planning_input.init_state().x() - planning_input.ref_x_vec(0);
    double y_error =
        planning_input.init_state().y() - planning_input.ref_y_vec(0);
    double direction = (a * planning_input.init_state().x() +
                        b * planning_input.init_state().y() + c) > 0.0
                           ? -1.0
                           : 1.0;
    init_dis_to_ref_ =
        direction * std::sqrt((x_error * x_error) + (y_error * y_error));
  }
  init_ref_theta_error_ =
      (planning_input.init_state().theta() - planning_input.ref_theta_vec(0)) *
      57.3;
}

void LateralMotionPlanningWeight::SetAccJerkBoundByVelocity(
    planning::common::LateralPlanningInput &planning_input) {
  const double velocity = ego_vel_ * 3.6;
  const double jerk_bound1 = config_.jerk_bound1;
  const double jerk_bound2 = config_.jerk_bound2;
  const double jerk_bound3 = config_.jerk_bound3;
  const double jerk_bound4 = config_.jerk_bound4;
  std::array<double, 4> xp_vel{10.0, 50.0, 80.0, 100.0};
  std::array<double, 4> fp_jerk_bound{jerk_bound1, jerk_bound2, jerk_bound3,
                                      jerk_bound4};
  double jerk_bound = interp(velocity, xp_vel, fp_jerk_bound);
  planning_input.set_acc_bound(config_.acc_bound);
  planning_input.set_jerk_bound(jerk_bound);
}

void LateralMotionPlanningWeight::MakeDynamicWeight(
    planning::common::LateralPlanningInput &planning_input) {
  std::array<double, 3> xp_v{5.0, 10.0, 20.0};
  std::array<double, 3> fp_qxy{config_.map_qxy0, config_.map_qxy1,
                               config_.map_qxy2};
  double q_xy = planning::interp(ego_vel_, xp_v, fp_qxy);
  planning_input.set_q_ref_x(q_xy);
  planning_input.set_q_ref_y(q_xy);

  std::array<double, 4> xp_xy{0.1, 0.2, 0.4, 0.8};
  std::array<double, 4> fp_qjerk0{config_.map1_qjerk0, config_.map1_qjerk1,
                                  config_.map1_qjerk2, config_.map1_qjerk3};
  std::array<double, 4> fp_qjerk1{config_.map2_qjerk0, config_.map2_qjerk1,
                                  config_.map2_qjerk2, config_.map2_qjerk3};
  double q_jerk0 =
      planning::interp(std::fabs(init_dis_to_ref_), xp_xy, fp_qjerk0);
  double q_jerk1 =
      planning::interp(std::fabs(init_dis_to_ref_), xp_xy, fp_qjerk1);

  planning_input.set_q_jerk(q_jerk1);
  concerned_start_q_jerk_ = q_jerk0;
}

}  // namespace lateral_planning
}  // namespace pnc
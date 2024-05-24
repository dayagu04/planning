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
  CalculateConcernedStartRatio();
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
      if (config_.close_jerk_enable) {
        SetCloseRangeJerkWeight(planning_input);
      }
      if (config_.far_jerk_enable) {
        SetJerkWeightByBigBias(planning_input);
      }
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
      planning_input.set_q_jerk(config_.q_jerk_lane_change);
      break;
    }
    case BEND: {
      planning_input.set_acc_bound(config_.acc_bound_bend);
      planning_input.set_jerk_bound(config_.jerk_bound_bend);
      planning_input.set_q_ref_x(config_.q_ref_x);
      planning_input.set_q_ref_y(config_.q_ref_y);
      planning_input.set_q_ref_theta(config_.q_ref_theta);
      planning_input.set_q_continuity(config_.q_continuity);
      planning_input.set_q_acc(config_.q_acc_bend);
      planning_input.set_q_jerk(config_.q_jerk_bend);
      if (config_.close_jerk_enable) {
        SetCloseRangeJerkWeight(planning_input);
      }
      if (config_.far_jerk_enable) {
        SetJerkWeightByBigBias(planning_input);
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
  const double velocity = real_vel_ * 3.6;
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

void LateralMotionPlanningWeight::SetCloseRangeJerkWeight(
    planning::common::LateralPlanningInput &planning_input) {
  double q_jerk_close = config_.q_jerk_close;
  double motion_plan_concerned_start_ratio0 = config_.motion_plan_concerned_start_ratio0;
  double motion_plan_concerned_start_ratio1 = config_.motion_plan_concerned_start_ratio1;
  if  (lateral_motion_scene_ == BEND) {
    q_jerk_close = config_.q_jerk_bend_close;
    motion_plan_concerned_start_ratio0 = config_.motion_plan_concerned_start_ratio_bend0;
    motion_plan_concerned_start_ratio1 = config_.motion_plan_concerned_start_ratio_bend1;
  }
  if (std::fabs(init_dis_to_ref_) < config_.init_ref_dist_close_thr) {
    // planning_input.set_q_acc(config_.q_acc_close);
    // planning_input.set_q_jerk(q_jerk_close);
    concerned_start_ratio_ = motion_plan_concerned_start_ratio0;
  } else {
    planning_input.set_q_ref_theta(config_.q_ref_theta_close);
    planning_input.set_q_jerk(q_jerk_close);
    concerned_start_ratio_ = motion_plan_concerned_start_ratio1;
  }
}

void LateralMotionPlanningWeight::SetJerkWeightByBigBias(
    planning::common::LateralPlanningInput &planning_input) {
  double q_jerk_far = config_.q_jerk_far;
  double motion_plan_concerned_start_ratio2 = config_.motion_plan_concerned_start_ratio2;
  if  (lateral_motion_scene_ == BEND) {
    q_jerk_far = config_.q_jerk_bend_far;
    motion_plan_concerned_start_ratio2 = config_.motion_plan_concerned_start_ratio_bend2;
  }
  if ((init_dis_to_ref_ > config_.init_ref_dist_far_thr) &&
      (init_ref_theta_error_ > config_.init_ref_theta_far_thr)) {
    planning_input.set_q_jerk(q_jerk_far);
    concerned_start_ratio_ = motion_plan_concerned_start_ratio2;
  } else if ((init_dis_to_ref_ < (-config_.init_ref_dist_far_thr)) &&
             (init_ref_theta_error_ < (-config_.init_ref_theta_far_thr))) {
    planning_input.set_q_jerk(q_jerk_far);
    concerned_start_ratio_ = motion_plan_concerned_start_ratio2;
  }
}

void LateralMotionPlanningWeight::SetWeightByEnterAutoTime(const double auto_time, planning::common::LateralPlanningInput &planning_input) {
  const double q_jerk = planning_input.q_jerk();
  std::array<double, 3> xp_time{0.1, 1.5, config_.auto_start_time_thr};
  std::array<double, 3> fp_qjerk{config_.q_jerk_auto, 0.9 * config_.q_jerk_auto, q_jerk};
  double q_jerk_quto = interp(auto_time, xp_time, fp_qjerk);
  planning_input.set_q_jerk(q_jerk_quto);
}

void LateralMotionPlanningWeight::SetPosBoundWeightByLane(bool direction,
    planning::common::LateralPlanningInput &planning_input) {
  if (!direction) {
    if ((init_dis_to_ref_ > 0.0) &&
        (init_ref_theta_error_ > 0.0)) {
      planning_input.set_q_soft_corridor(config_.q_soft_corridor);
      planning_input.set_q_hard_corridor(config_.q_hard_corridor);
    }
  } else {
    if ((init_dis_to_ref_ < 0.0) &&
              (init_ref_theta_error_ < 0.0)) {
      planning_input.set_q_soft_corridor(config_.q_soft_corridor);
      planning_input.set_q_hard_corridor(config_.q_hard_corridor);
    }
  }
}

void LateralMotionPlanningWeight::CalculateConcernedStartRatio() {
  concerned_start_ratio_ = config_.motion_plan_concerned_start_ratio0;
  if (lateral_motion_scene_ == BEND) {
    concerned_start_ratio_ = config_.motion_plan_concerned_start_ratio_bend0;
  } else if (lateral_motion_scene_ == AVOID) {
    concerned_start_ratio_ = config_.motion_plan_concerned_start_ratio_avoid;
  }
}

}  // namespace lateral_planning
}  // namespace pnc
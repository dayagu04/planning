#include "lateral_motion_planning_weight.h"

#include "math_lib.h"
#include "refline.h"

namespace pnc {
namespace lateral_planning {

LateralMotionPlanningWeight::LateralMotionPlanningWeight(
    const planning::LateralMotionPlannerConfig &config)
    : config_(config) {
  Init();
}

void LateralMotionPlanningWeight::Init() {
  lateral_motion_scene_ = pnc::lateral_planning::LANE_KEEP;
  init_dis_to_ref_ = 0.0;
  init_ref_theta_error_ = 0.0;
  concerned_start_q_jerk_ = 0.0;
  ego_vel_ = 0.0;
  ego_l_ = 0.0;
  end_ratio_for_qrefxy_ = 0.0;
  end_ratio_for_qreftheta_ = 0.0;
  is_lane_change_back_ = false;
  is_in_intersection_ = false;
  is_emergence_ = false;
}

void LateralMotionPlanningWeight::SetLateralMotionWeight(
    const LateralMotionSceneEnum scene,
    planning::common::LateralPlanningInput &planning_input) {
  lateral_motion_scene_ = scene;
  // CalculateInitInfo(planning_input);
  planning_input.set_q_acc_bound(config_.q_acc_bound);
  planning_input.set_q_jerk_bound(config_.q_jerk_bound);
  // if (is_in_intersection_) {
  //   planning_input.set_q_soft_corridor(config_.q_soft_corridor_intersection);
  // } else {
  //   planning_input.set_q_soft_corridor(config_.q_soft_corridor);
  // }

  // planning_input.set_q_hard_corridor(config_.q_hard_corridor);
  MakeDynamicPosBoundWeight(planning_input);
  end_ratio_for_qrefxy_ = config_.end_ratio_for_qrefxy;
  end_ratio_for_qreftheta_ = config_.end_ratio_for_qreftheta;
  // set cost weight by scene
  switch (scene) {
    case LANE_KEEP: {
      if (is_in_intersection_) {
        planning_input.set_q_ref_x(config_.q_ref_xy_intersection);
        planning_input.set_q_ref_y(config_.q_ref_xy_intersection);
        planning_input.set_q_ref_theta(config_.q_ref_theta_intersection);
        planning_input.set_q_continuity(config_.q_continuity_intersection);
        planning_input.set_q_acc(config_.q_acc_intersection);
        planning_input.set_q_jerk(config_.q_jerk_intersection_mid);
        concerned_start_q_jerk_ = config_.q_jerk_intersection_close;
        planning_input.set_acc_bound(config_.acc_bound);
        planning_input.set_jerk_bound(config_.jerk_bound_intersection);
        planning_input.set_q_jerk_bound(config_.q_jerk_bound_intersection);
      } else {
        planning_input.set_q_ref_x(config_.q_ref_x);
        planning_input.set_q_ref_y(config_.q_ref_y);
        planning_input.set_q_ref_theta(config_.q_ref_theta);
        planning_input.set_q_continuity(config_.q_continuity);
        planning_input.set_q_acc(config_.q_acc);
        planning_input.set_q_jerk(config_.q_jerk);
        concerned_start_q_jerk_ = config_.q_jerk;
        SetAccJerkBoundByVelocity(planning_input);
        MakeDynamicWeight(planning_input);
        // [hack]
        if ((std::fabs(init_dis_to_ref_) < 0.3) &&
            (std::fabs(init_ref_theta_error_) > config_.big_theta_thr)) {
          planning_input.set_q_jerk(config_.q_jerk_for_big_theta);
          concerned_start_q_jerk_ = config_.q_jerk_for_big_theta;
        }
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

      concerned_start_q_jerk_ = config_.q_jerk_avoid;
      if (ego_vel_ > config_.avoid_high_vel) {
        planning_input.set_jerk_bound(config_.jerk_bound_avoid);
        planning_input.set_q_ref_theta(config_.q_ref_theta_avoid_high_vel);
        planning_input.set_q_jerk(config_.q_jerk_avoid_high_vel_middle);
        concerned_start_q_jerk_ = config_.q_jerk_avoid_high_vel_close;
        planning_input.set_q_jerk_bound(config_.q_jerk_bound_avoid_high_vel);
      }
      break;
    }
    case LANE_CHANGE: {
      end_ratio_for_qreftheta_ = config_.lc_end_ratio_for_qreftheta;
      planning_input.set_acc_bound(config_.acc_bound_lane_change);
      planning_input.set_jerk_bound(config_.jerk_bound_lane_change);
      planning_input.set_q_ref_x(config_.q_ref_x_lane_change);
      planning_input.set_q_ref_y(config_.q_ref_y_lane_change);
      planning_input.set_q_ref_theta(config_.q_ref_theta_lane_change);
      planning_input.set_q_continuity(config_.q_continuity);
      planning_input.set_q_acc(config_.q_acc_lane_change);
      planning_input.set_q_jerk(config_.q_jerk_lane_change);
      concerned_start_q_jerk_ = config_.q_jerk_lane_change;
      planning_input.set_q_jerk_bound(config_.q_jerk_bound_lane_change);
      // if (std::fabs(ego_l_) > config_.lane_change_ego_l_thr) {
      //   planning_input.set_q_jerk(config_.q_jerk_lane_change);
      //   concerned_start_q_jerk_ = config_.q_jerk_lane_change;
      // } else {
      //   planning_input.set_q_jerk(config_.q_jerk_lane_change2);
      //   concerned_start_q_jerk_ = config_.q_jerk_lane_change2;
      // }
      if (ego_vel_ <= config_.lane_change_high_vel) {
        end_ratio_for_qrefxy_ = config_.lc_end_ratio_for_first_qrefxy;
      } else {
        if (config_.use_new_lc_param) {
          end_ratio_for_qrefxy_ = config_.lc_end_ratio_for_first_qrefxy;
          planning_input.set_q_ref_theta(
              config_.q_ref_theta_lane_change_high_vel);
        } else {
          planning_input.set_q_ref_theta(
              config_.q_ref_theta_lane_change_high_vel1);
        }
      }

      // planning_input.set_q_ref_x(config_.q_ref_xy_lane_change_high_vel);
      // planning_input.set_q_ref_y(config_.q_ref_xy_lane_change_high_vel);
      // planning_input.set_q_ref_theta(config_.q_ref_theta_lane_change_high_vel);
      // planning_input.set_q_jerk(config_.q_jerk_lane_change);
      // concerned_start_q_jerk_ = config_.q_jerk_lane_change;
      // planning_input.set_jerk_bound(config_.jerk_bound_lane_change_high_vel);
      // planning_input.set_q_jerk_bound(config_.q_jerk_bound_lane_change);

      if (is_lane_change_back_) {
        planning_input.set_q_ref_x(config_.q_ref_xy_lane_change_back);
        planning_input.set_q_ref_y(config_.q_ref_xy_lane_change_back);
        planning_input.set_q_ref_theta(config_.q_ref_theta_lane_change_back);
        planning_input.set_q_jerk(config_.q_jerk_lane_change_back);
        concerned_start_q_jerk_ = config_.q_jerk_lane_change_back;
        planning_input.set_jerk_bound(config_.jerk_bound_lane_change_back);
        planning_input.set_q_jerk_bound(config_.q_jerk_bound_lane_change_back);
      }
      break;
    }
    case STATIC_AVOID: {
      planning_input.set_q_ref_x(config_.q_ref_x_static_avoid);
      planning_input.set_q_ref_y(config_.q_ref_y_static_avoid);
      planning_input.set_q_ref_theta(config_.q_ref_theta_static_avoid);
      planning_input.set_q_continuity(config_.q_continuity);
      planning_input.set_q_acc(config_.q_acc_static_avoid);
      planning_input.set_q_jerk(config_.q_jerk_static_avoid_middle);
      SetAccJerkBoundByVelocity(planning_input);
      concerned_start_q_jerk_ = config_.q_jerk_static_avoid_close;
      break;
    }
    case SPLIT: {
      planning_input.set_acc_bound(config_.acc_bound_lane_change);
      planning_input.set_jerk_bound(config_.jerk_bound_split);
      planning_input.set_q_ref_x(config_.q_ref_xy_split);
      planning_input.set_q_ref_y(config_.q_ref_xy_split);
      planning_input.set_q_ref_theta(config_.q_ref_theta_split);
      planning_input.set_q_continuity(config_.q_continuity);
      planning_input.set_q_acc(config_.q_acc_lane_change);
      planning_input.set_q_jerk(config_.q_jerk_split);
      concerned_start_q_jerk_ = config_.q_jerk_lane_change;
      planning_input.set_q_jerk_bound(config_.q_jerk_bound_split);
      break;
    }
    case RAMP: {
      planning_input.set_acc_bound(config_.acc_bound_ramp);
      planning_input.set_jerk_bound(config_.jerk_bound_ramp);
      planning_input.set_q_ref_x(config_.q_ref_x_ramp);
      planning_input.set_q_ref_y(config_.q_ref_y_ramp);
      planning_input.set_q_ref_theta(config_.q_ref_theta_ramp);
      planning_input.set_q_continuity(config_.q_continuity);
      planning_input.set_q_acc(config_.q_acc_ramp);
      planning_input.set_q_jerk(config_.q_jerk_ramp_mid);
      concerned_start_q_jerk_ = config_.q_jerk_ramp_close;
      // SetAccJerkBoundByVelocity(planning_input);
      // MakeDynamicWeight(planning_input);
      break;
    }
    default: { break; }
  }
}

void LateralMotionPlanningWeight::CalculateInitInfo(
    const planning::common::LateralPlanningInput &planning_input) {
  const double a = planning_input.ref_y_vec(1) - planning_input.ref_y_vec(0);
  const double b = planning_input.ref_x_vec(0) - planning_input.ref_x_vec(1);
  const double c = planning_input.ref_y_vec(0) * planning_input.ref_x_vec(1) -
                   planning_input.ref_x_vec(0) * planning_input.ref_y_vec(1);
  const double d = pnc::mathlib::Square(a) + pnc::mathlib::Square(b);
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
  std::vector<double> xp_vel{10.0, 50.0, 80.0, 100.0};
  double jerk_bound = planning::interp(velocity, xp_vel, config_.map_jerk_bound);
  planning_input.set_acc_bound(config_.acc_bound);
  planning_input.set_jerk_bound(jerk_bound);
}

void LateralMotionPlanningWeight::MakeDynamicWeight(
    planning::common::LateralPlanningInput &planning_input) {
  std::vector<double> xp_v{5.0, 10.0, 20.0, 30.0};
  double q_xy = planning::interp(ego_vel_, xp_v, config_.map_qxy);
  planning_input.set_q_ref_x(q_xy);
  planning_input.set_q_ref_y(q_xy);

  std::vector<double> xp_xy{0.1, 0.2, 0.4, 0.8};
  double q_jerk1 =
      planning::interp(std::fabs(init_dis_to_ref_), xp_xy, config_.map_qjerk1);
  double q_jerk2 =
      planning::interp(std::fabs(init_dis_to_ref_), xp_xy, config_.map_qjerk2);

  concerned_start_q_jerk_ = q_jerk1;
  planning_input.set_q_jerk(q_jerk2);
}

void LateralMotionPlanningWeight::MakeLaneChangeDynamicWeight(
    planning::common::LateralPlanningInput &planning_input) {
  end_ratio_for_qrefxy_ = config_.lc_end_ratio_for_second_qrefxy;
  std::vector<double> xp_xy{0.25, 0.5, 1.0, 1.5};
  std::vector<double> xp_xy2{0.15, 0.5, 1.0, 1.5};
  std::vector<double> xp_xy3{0.15, 0.5, 1.5};
  if (ego_vel_ <= config_.lane_change_high_vel || config_.use_new_lc_param) {
    double q_jerk = planning::interp(std::fabs(init_dis_to_ref_), xp_xy,
                                     config_.map_qjerk_lc_high_vel);
    concerned_start_q_jerk_ = q_jerk;
    planning_input.set_q_jerk(config_.q_jerk_lane_change_high_vel);

    double q_ref_xy = planning::interp(std::fabs(init_dis_to_ref_), xp_xy2,
                                       config_.map_qrefxy_lc_high_vel);
    planning_input.set_q_ref_x(q_ref_xy);
    planning_input.set_q_ref_y(q_ref_xy);

    std::vector<double> fp_qtheta{config_.q_ref_theta_lane_change_high_vel3,
                                  config_.q_ref_theta_lane_change_high_vel2,
                                  config_.q_ref_theta_lane_change_high_vel};
    double q_ref_theta =
        planning::interp(std::fabs(init_dis_to_ref_), xp_xy3, fp_qtheta);
    if (ego_vel_ <= config_.lane_change_high_vel) {
      q_ref_theta *= 0.1;
    } else {
      planning_input.set_jerk_bound(config_.jerk_bound_lane_change_high_vel);
      planning_input.set_q_jerk_bound(
          config_.q_jerk_bound_lane_change_high_vel);
    }
    planning_input.set_q_ref_theta(q_ref_theta);
  } else {
    double q_jerk = planning::interp(std::fabs(init_dis_to_ref_), xp_xy,
                                     config_.map_qjerk_lc_high_vel_old);
    concerned_start_q_jerk_ = q_jerk;
    planning_input.set_q_jerk(config_.q_jerk_lane_change_high_vel);

    // std::vector<double> xp_xy2{0.15, 0.5, 1.0, 1.5};
    double q_ref_xy = planning::interp(std::fabs(init_dis_to_ref_), xp_xy2,
                                       config_.map_qrefxy_lc_high_vel);
    planning_input.set_q_ref_x(q_ref_xy);
    planning_input.set_q_ref_y(q_ref_xy);

    std::vector<double> xp_xy3{0.15, 0.5, 1.5};
    std::vector<double> fp_qtheta{config_.q_ref_theta_lane_change_high_vel3,
                                  config_.q_ref_theta_lane_change_high_vel2,
                                  config_.q_ref_theta_lane_change_high_vel};
    double q_ref_theta =
        planning::interp(std::fabs(init_dis_to_ref_), xp_xy3, fp_qtheta);
    planning_input.set_q_ref_theta(q_ref_theta);
    planning_input.set_jerk_bound(config_.jerk_bound_lane_change_high_vel);
    planning_input.set_q_jerk_bound(config_.q_jerk_bound_lane_change_high_vel);
  }
}

void LateralMotionPlanningWeight::MakeSplitDynamicWeight(
    planning::common::LateralPlanningInput &planning_input) {
  end_ratio_for_qrefxy_ = config_.lc_end_ratio_for_second_qrefxy;
  std::vector<double> xp_xy{0.25, 0.5, 1.0, 1.5};
  double q_jerk = planning::interp(std::fabs(init_dis_to_ref_), xp_xy,
                                   config_.map_qjerk_lc_high_vel_old);
  concerned_start_q_jerk_ = q_jerk;
  planning_input.set_q_jerk(config_.q_jerk_lane_change_high_vel);
  std::vector<double> xp_xy2{0.15, 0.5, 1.0, 1.5};
  double q_ref_xy = planning::interp(std::fabs(init_dis_to_ref_), xp_xy2,
                                     config_.map_qrefxy_lc_high_vel);
  planning_input.set_q_ref_x(q_ref_xy);
  planning_input.set_q_ref_y(q_ref_xy);

  std::vector<double> xp_xy3{0.15, 0.5, 1.5};
  std::vector<double> fp_qtheta{config_.q_ref_theta_lane_change_high_vel3,
                                config_.q_ref_theta_lane_change_high_vel2,
                                config_.q_ref_theta_lane_change_high_vel};
  double q_ref_theta =
      planning::interp(std::fabs(init_dis_to_ref_), xp_xy3, fp_qtheta);
  planning_input.set_q_ref_theta(q_ref_theta);
  planning_input.set_jerk_bound(config_.jerk_bound_lane_change_high_vel);
  planning_input.set_q_jerk_bound(config_.q_jerk_bound_lane_change_high_vel);
}

void LateralMotionPlanningWeight::MakeDynamicPosBoundWeight(
    planning::common::LateralPlanningInput &planning_input) {
  double emergence_factor = 1.0;
  double intersection_factor = 1.0;
  if (is_emergence_) {
    emergence_factor = config_.emergence_avoid_factor;
  }
  if (is_in_intersection_) {
    intersection_factor = config_.intersection_avoid_factor;
  }
  // Set according to different vel
  std::vector<double> xp_v{5.0, 10.0, 20.0, 30.0};
  double q_soft_bound =
      planning::interp(ego_vel_, xp_v, config_.map_qsoft_bound) *
      emergence_factor * intersection_factor;
  double q_hard_bound =
      planning::interp(ego_vel_, xp_v, config_.map_qhard_bound);

  planning_input.set_q_soft_corridor(q_soft_bound);
  planning_input.set_q_hard_corridor(q_hard_bound);
}

}  // namespace lateral_planning
}  // namespace pnc
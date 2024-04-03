#include "lateral_motion_planning_weight.h"

#include "math_lib.h"
#include "refline.h"

namespace pnc {
namespace lateral_planning {

LateralMotionPlanningWeight::LateralMotionPlanningWeight(
    const planning::LateralMotionPlannerConfig &config)
    : config_(config) {}

void LateralMotionPlanningWeight::SetWeightByScene(
    LateralMotionSceneEnum scene,
    planning::common::LateralPlanningInput &planning_input) {
  // set cost weight by scene
  switch (scene) {
    case LANE_KEEP: {
      planning_input.set_acc_bound(config_.acc_bound);
      planning_input.set_jerk_bound(config_.jerk_bound);
      planning_input.set_q_ref_x(config_.q_ref_x);
      planning_input.set_q_ref_y(config_.q_ref_y);
      planning_input.set_q_ref_theta(config_.q_ref_theta);
      planning_input.set_q_continuity(config_.q_continuity);
      planning_input.set_q_acc(config_.q_acc);
      planning_input.set_q_jerk(config_.q_jerk);
      break;
    }
    case AVOID: {
      planning_input.set_acc_bound(config_.acc_bound);
      planning_input.set_jerk_bound(config_.jerk_bound);
      planning_input.set_q_ref_x(config_.q_ref_x_avoid);
      planning_input.set_q_ref_y(config_.q_ref_y_avoid);
      planning_input.set_q_ref_theta(config_.q_ref_theta_avoid);
      planning_input.set_q_continuity(config_.q_continuity);
      planning_input.set_q_acc(config_.q_acc_avoid);
      planning_input.set_q_jerk(config_.q_jerk_avoid);
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
    case BIG_CURVATURE: {
      planning_input.set_acc_bound(config_.acc_bound_lane_change);
      planning_input.set_jerk_bound(config_.jerk_bound_lane_change);
      planning_input.set_q_ref_x(config_.q_ref_x);
      planning_input.set_q_ref_y(config_.q_ref_y);
      planning_input.set_q_ref_theta(config_.q_ref_theta);
      planning_input.set_q_continuity(config_.q_continuity);
      planning_input.set_q_acc(config_.q_acc_big_curvature);
      planning_input.set_q_jerk(config_.q_jerk_big_curvature);
      break;
    }
    default: { break; }
  }
  planning_input.set_q_acc_bound(config_.q_acc_bound);
  planning_input.set_q_jerk_bound(config_.q_jerk_bound);
}

void LateralMotionPlanningWeight::SetWeightByPosBoundAndLaneType(
    std::shared_ptr<VirtualLane> current_lane,
    planning::common::LateralPlanningInput &planning_input) {
  // upper
  const double a1 = planning_input.soft_upper_bound_y1_vec(0) -
                    planning_input.soft_upper_bound_y0_vec(0);
  const double b1 = planning_input.soft_upper_bound_x0_vec(0) -
                    planning_input.soft_upper_bound_x1_vec(0);
  const double c1 = planning_input.soft_upper_bound_y0_vec(0) *
                        planning_input.soft_upper_bound_x1_vec(0) -
                    planning_input.soft_upper_bound_x0_vec(0) *
                        planning_input.soft_upper_bound_y1_vec(0);
  const double init_dis_to_upper = (a1 * planning_input.init_state().x() +
                                    b1 * planning_input.init_state().y() + c1);
  // lower
  const double a2 = planning_input.soft_lower_bound_y1_vec(0) -
                    planning_input.soft_lower_bound_y0_vec(0);
  const double b2 = planning_input.soft_lower_bound_x0_vec(0) -
                    planning_input.soft_lower_bound_x1_vec(0);
  const double c2 = planning_input.soft_lower_bound_y0_vec(0) *
                        planning_input.soft_lower_bound_x1_vec(0) -
                    planning_input.soft_lower_bound_x0_vec(0) *
                        planning_input.soft_lower_bound_y1_vec(0);
  const double init_dis_to_lower = (a2 * planning_input.init_state().x() +
                                    b2 * planning_input.init_state().y() + c2);

  const double ref_s = planning_input.ref_vel() * config_.delta_t *
                       (planning_input.ref_x_vec().size() - 1);
  const double theta_error =
      (planning_input.init_state().theta() - planning_input.ref_theta_vec(0)) *
      57.3;
  double q_acc = planning_input.q_acc();
  double q_jerk = planning_input.q_jerk();
  double q_soft_corridor = config_.q_soft_corridor;
  double q_hard_corridor = config_.q_hard_corridor;
  double distance_to_dash_line = 0.0;
  // set soft bound weight
  if (init_dis_to_upper > 0.0 && init_dis_to_lower < 0.0) {
    q_soft_corridor *= 4.0;
  } else if (init_dis_to_upper < 0.0) {
    for (int i = 0;
         i < current_lane->get_left_lane_boundary().type_segments_size; ++i) {
      const auto &type_segment =
          current_lane->get_left_lane_boundary().type_segments[i];
      if (type_segment.type == iflyauto::LaneBoundaryType_MARKING_DASHED ||
          type_segment.type ==
              iflyauto::LaneBoundaryType_MARKING_SHORT_DASHED ||
          type_segment.type ==
              iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED ||
          type_segment.type ==
              iflyauto::LaneBoundaryType_MARKING_LEFT_SOLID_RIGHT_DASHED) {
        distance_to_dash_line += type_segment.length;
      } else {
        break;
      }
    }
    if (distance_to_dash_line >= std::min(ref_s, 100.0)) {
      q_jerk *= 2.0;
    } else if (theta_error > 0.0) {
      q_soft_corridor *= 4.0;
    }
  } else if (init_dis_to_lower > 0) {
    for (int i = 0;
         i < current_lane->get_left_lane_boundary().type_segments_size; ++i) {
      const auto &type_segment =
          current_lane->get_left_lane_boundary().type_segments[i];
      if (type_segment.type == iflyauto::LaneBoundaryType_MARKING_DASHED ||
          type_segment.type ==
              iflyauto::LaneBoundaryType_MARKING_SHORT_DASHED ||
          type_segment.type ==
              iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED ||
          type_segment.type ==
              iflyauto::LaneBoundaryType_MARKING_LEFT_SOLID_RIGHT_DASHED) {
        distance_to_dash_line += type_segment.length;
      } else {
        break;
      }
    }
    if (distance_to_dash_line >= std::min(ref_s, 100.0)) {
      q_jerk *= 2.0;
    } else if (theta_error < 0.0) {
      q_soft_corridor *= 4.0;
    }
  }

  planning_input.set_q_acc(q_acc);
  planning_input.set_q_jerk(q_jerk);
  planning_input.set_q_soft_corridor(q_soft_corridor);
  planning_input.set_q_hard_corridor(q_hard_corridor);
}

}  // namespace lateral_planning
}  // namespace pnc
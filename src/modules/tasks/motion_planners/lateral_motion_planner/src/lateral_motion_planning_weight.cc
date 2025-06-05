#include "lateral_motion_planning_weight.h"
#include <cmath>
#include <cstddef>

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
  lat_offset_ = 0.0;
  avoid_dist_ = 0.0;
  init_dis_to_ref_ = 0.0;
  init_ref_theta_error_ = 0.0;
  concerned_start_q_jerk_ = 0.0;
  ego_vel_ = 0.0;
  ego_l_ = 0.0;
  ref_vel_ = 0.0;
  init_l_ = 0.0;
  end_ratio_for_qrefxy_ = 0.0;
  end_ratio_for_qreftheta_ = 0.0;
  end_ratio_for_qjerk_ = 0.0;
  max_acc_ = 3.0;
  max_jerk_ = 1.5;
  last_expected_average_acc_ = 0.0;
  last_jerk_bound_limit_ = 0.2;
  expected_average_acc_ = 0.0;
  expected_max_acc_ = 0.0;
  expected_min_acc_ = 0.0;
  min_road_radius_ = 10000.0;
  min_q_jerk_ = 2.0;
  last_path_max_dist2ref_ = 0.0;
  is_lane_change_back_ = false;
  is_in_intersection_ = false;
  is_emergency_ = false;
  is_search_success_ = false;
  is_s_bend_ = false;
  soft_bound_qratio_vec_.resize(26, 1.0);
  hard_bound_qratio_vec_.resize(26, 1.0);
  curvature_radius_vec_.resize(6, 10000.0);
  weight_.Init();
  weight_.dt = config_.delta_t;
}

void LateralMotionPlanningWeight::SetLateralMotionWeight(
    const LateralMotionSceneEnum scene,
    planning::common::LateralPlanningInput &planning_input) {
  lateral_motion_scene_ = scene;
  weight_.proximal_index = config_.motion_plan_concerned_start_index;
  weight_.remotely_index = config_.motion_plan_concerned_end_index;
  weight_.complete_follow = false;
  end_ratio_for_qrefxy_ = config_.end_ratio_for_qrefxy;
  end_ratio_for_qreftheta_ = config_.end_ratio_for_qreftheta;
  end_ratio_for_qjerk_ = config_.end_ratio_for_qjerk;
  SetAccJerkBoundAndWeight(planning_input);
  MakeDynamicPosBoundWeight(planning_input);
  if (lateral_motion_scene_ != RAMP) {
    weight_.expected_acc.clear();  // temp
    weight_.expected_acc.resize(26, 0.0);
  }
  // set cost weight by scene
  switch (scene) {
    case LANE_KEEP: {
      planning_input.set_q_ref_x(config_.q_ref_x);
      planning_input.set_q_ref_y(config_.q_ref_y);
      planning_input.set_q_ref_theta(config_.q_ref_theta);
      planning_input.set_q_continuity(config_.q_continuity);
      planning_input.set_q_acc(config_.q_acc);
      planning_input.set_q_jerk(config_.q_jerk);
      concerned_start_q_jerk_ = config_.q_jerk;
      MakeDynamicWeight(planning_input);

      if (is_in_intersection_) {
        planning_input.set_q_continuity(config_.q_continuity_intersection);
        planning_input.set_q_acc(config_.q_acc_intersection);
      }
      if (is_search_success_) {
        planning_input.set_q_continuity(config_.q_continuity_search);
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
      concerned_start_q_jerk_ = config_.q_jerk_avoid;
      if (ref_vel_ > config_.avoid_high_vel) {
        planning_input.set_q_ref_theta(config_.q_ref_theta_avoid_high_vel);
        planning_input.set_q_jerk(config_.q_jerk_avoid_high_vel_middle);
        concerned_start_q_jerk_ = config_.q_jerk_avoid_high_vel_close;
      }
      break;
    }
    case LANE_CHANGE: {
      end_ratio_for_qreftheta_ = config_.lc_end_ratio_for_qreftheta;
      planning_input.set_q_ref_x(config_.q_ref_x_lane_change);
      planning_input.set_q_ref_y(config_.q_ref_y_lane_change);
      planning_input.set_q_ref_theta(config_.q_ref_theta_lane_change);
      planning_input.set_q_continuity(config_.q_continuity);
      planning_input.set_q_acc(config_.q_acc_lane_change);
      planning_input.set_q_jerk(config_.q_jerk_lane_change);
      concerned_start_q_jerk_ = config_.q_jerk_lane_change;

      if (ref_vel_ <= config_.lane_change_high_vel) {
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

      if (is_lane_change_back_) {
        MakeLaneChangeBackDynamicWeight(planning_input);
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
      concerned_start_q_jerk_ = config_.q_jerk_static_avoid_close;
      break;
    }
    case SPLIT: {
      planning_input.set_q_ref_x(config_.q_ref_xy_split);
      planning_input.set_q_ref_y(config_.q_ref_xy_split);
      planning_input.set_q_ref_theta(config_.q_ref_theta_split);
      planning_input.set_q_continuity(config_.q_continuity);
      planning_input.set_q_acc(config_.q_acc_lane_change);
      planning_input.set_q_jerk(config_.q_jerk_split);
      concerned_start_q_jerk_ = config_.q_jerk_lane_change;
      break;
    }
    case RAMP: {
      // end_ratio_for_qrefxy_ = 0.2;
      // end_ratio_for_qreftheta_ = 0.2;
      planning_input.set_q_ref_x(config_.q_ref_x_ramp);
      planning_input.set_q_ref_y(config_.q_ref_y_ramp);
      planning_input.set_q_ref_theta(config_.q_ref_theta_ramp);
      planning_input.set_q_continuity(config_.q_continuity);
      planning_input.set_q_acc(config_.q_acc_ramp);
      planning_input.set_q_jerk(config_.q_jerk_ramp_mid);
      concerned_start_q_jerk_ = config_.q_jerk_ramp_close;
      MakeRampDynamicWeight(planning_input);
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

void LateralMotionPlanningWeight::CalculateLastPathDistToRef(
    const std::shared_ptr<planning::ReferencePath> &reference_path,
    planning::common::LateralPlanningInput &planning_input) {
  last_path_max_dist2ref_ = 0.0;
  double min_x_size =
      std::min(planning_input.last_x_vec_size(), planning_input.ref_x_vec_size());
  double min_y_size =
      std::min(planning_input.last_y_vec_size(), planning_input.ref_y_vec_size());
  double min_size =
      std::min(min_x_size, min_y_size);;
  if (reference_path != nullptr &&
      min_size > 0) {
    const auto &frenet_coord = reference_path->get_frenet_coord();
    for (size_t i = 0; i < min_size; ++i) {
      planning::Point2D cart_last_xy(planning_input.last_x_vec(i),
                          planning_input.last_y_vec(i));
      planning::Point2D frenet_last_xy;
      planning::Point2D cart_ref_xy(planning_input.ref_x_vec(i),
                          planning_input.ref_y_vec(i));
      planning::Point2D frenet_ref_xy;
      if (frenet_coord->XYToSL(cart_last_xy, frenet_last_xy) &&
          frenet_coord->XYToSL(cart_ref_xy, frenet_ref_xy)) {
        last_path_max_dist2ref_ =
            std::max(std::fabs(frenet_last_xy.y - frenet_ref_xy.y), last_path_max_dist2ref_);
      }
    }
  }
}

void LateralMotionPlanningWeight::CalculateExpectedLatAccAndSteerAngle(
    double init_s, double ref_vel, double wheel_base, double steer_ratio,
    const std::shared_ptr<planning::ReferencePath> &reference_path,
    std::vector<double>& expected_steer_vec) {
  expected_average_acc_ = 0.0;
  expected_max_acc_ = -10.0;
  expected_min_acc_ = 10.0;
  min_road_radius_ = 10000.0;
  is_s_bend_ = false;
  bool is_left_bend = false;
  bool is_right_bend = false;
  size_t time = 0;
  size_t kappa_gap = 0;
  double sum_kappa = 0;
  double ds = ref_vel * config_.delta_t;
  double kv2 = config_.curv_factor * ref_vel_ * ref_vel_;
  weight_.expected_acc.clear();
  weight_.expected_acc.resize(26, 0.0);
  curvature_radius_vec_.clear();
  curvature_radius_vec_.resize(6, 10000.0);
  for (size_t i = 0; i < weight_.point_num; ++i) {
    planning::ReferencePathPoint ref_point;
    if (reference_path->get_reference_point_by_lon(init_s, ref_point)) {
      kappa_gap += 1;
      sum_kappa += ref_point.path_point.kappa();
      double expected_delta = std::atan(wheel_base * ref_point.path_point.kappa());  // rad
      double expected_steer = steer_ratio * expected_delta * 57.3;  // deg
      double expected_lat_acc = kv2 * expected_delta;
      expected_steer_vec[i] = expected_steer;
      weight_.expected_acc[i] = expected_lat_acc;
      if (i < 16) {
        expected_average_acc_ += expected_lat_acc;
      }
      if (i < 21) {
        expected_max_acc_ = std::max(expected_lat_acc, expected_max_acc_);
        expected_min_acc_ = std::min(expected_lat_acc, expected_min_acc_);
      }
      if (i % 5 == 0) {  // 0 1 2 3 4 5
        double average_kappa = sum_kappa / kappa_gap;
        if (std::fabs(average_kappa) > 1e-6) {
          double average_radius = 1 / average_kappa;
          if (average_radius > -750.0 && average_radius < -1e-6) {
            is_left_bend = true;
          } else if (average_radius < 750.0 && average_radius > 1e-6) {
            is_right_bend = true;
          }
          curvature_radius_vec_[time] = average_radius;
          min_road_radius_ = std::min(std::fabs(curvature_radius_vec_[time]), min_road_radius_);
        }
        time += 1;
        sum_kappa = 0;
        kappa_gap = 0;
      }
    } else {
      if (i > 0) {
        expected_steer_vec[i] = expected_steer_vec[i - 1];
        weight_.expected_acc[i] = weight_.expected_acc[i - 1];
      }
    }
    init_s += ds;
  }
  expected_average_acc_ = expected_average_acc_ / 16;
  if (std::fabs(expected_max_acc_) > std::fabs(expected_min_acc_)) {
    expected_average_acc_ +=
        std::max(std::min(0.5 * (expected_max_acc_ - expected_average_acc_), 0.5), -0.5);
  } else {
    expected_average_acc_ +=
        std::max(std::min(0.5 * (expected_min_acc_ - expected_average_acc_), 0.5), -0.5);
  }
  expected_average_acc_ +=
      std::max(std::min(0.8 * (last_expected_average_acc_ - expected_average_acc_), 0.2), -0.2);
  expected_average_acc_ =
      std::max(std::min(expected_average_acc_, expected_max_acc_), expected_min_acc_);
  last_expected_average_acc_ = expected_average_acc_;
  // S bend
  if (is_left_bend && is_right_bend) {
    is_s_bend_ = true;
  }
  // anchor: min road radius
  double min_radius =
      std::min(std::fabs(curvature_radius_vec_[3]), std::fabs(curvature_radius_vec_[4]));
  if (min_radius < 400.0) {
    if (std::fabs(curvature_radius_vec_[0]) < 100.0) {
      min_radius = min_road_radius_;
    } else if (std::fabs(curvature_radius_vec_[0]) < 400.0 ||
               std::fabs(curvature_radius_vec_[1]) < 400.0) {
      min_radius = std::fabs(curvature_radius_vec_[1]);
    } else {
      min_radius = std::fabs(curvature_radius_vec_[0]);
    }
  }
  min_road_radius_ = min_radius;
}

void LateralMotionPlanningWeight::CalculateLatAvoidDistance(
    const std::vector<std::pair<double, double>> &bounds) {
  avoid_dist_ = 0;
  for (size_t i = 0; i < bounds.size() - 5; ++i) {
    if (bounds[i].first > init_l_) {
      avoid_dist_ = std::max(bounds[i].first - init_l_, avoid_dist_);
    } else if (bounds[i].second < init_l_) {
      avoid_dist_ = std::max(init_l_ - bounds[i].second, avoid_dist_);
    }
  }
}

void LateralMotionPlanningWeight::CalculateLatAvoidBoundPriority(
    const std::vector<std::pair<double, double>> &soft_bounds,
    const std::vector<std::pair<double, double>> &hard_bounds,
    const std::vector<planning::WeightedBounds> hard_bounds_vec,
    const std::vector<std::pair<planning::BoundInfo, planning::BoundInfo>>& soft_bounds_info,
    const std::vector<std::pair<planning::BoundInfo, planning::BoundInfo>>& hard_bounds_info) {
  std::vector<double> hard_lbound_s_vector;
  std::vector<double> hard_ubound_s_vector;
  std::vector<double> hard_lbound_l_vector;
  std::vector<double> hard_ubound_l_vector;
  size_t bounds_size =
      std::min(soft_bounds_info.size(), hard_bounds_info.size());
  if (bounds_size < weight_.point_num) {
    soft_bound_qratio_vec_.clear();
    hard_bound_qratio_vec_.clear();
    soft_bound_qratio_vec_.resize(26, 1.0);
    hard_bound_qratio_vec_.resize(26, 1.0);
    hard_lbound_l_s_spline_.set_points(hard_lbound_s_vector, hard_lbound_l_vector);
    hard_ubound_l_s_spline_.set_points(hard_ubound_s_vector, hard_ubound_l_vector);
    return;
  }
  bool is_update_soft_lbound = false;
  bool is_update_soft_ubound = false;
  bool is_update_hard_lbound = false;
  bool is_update_hard_ubound = false;
  double time_factor = 0.98;
  double q_soft_bound_ratio = 1.4;
  double q_hard_bound_ratio = 1.6;
  soft_bound_qratio_vec_[0] = 1.0;
  hard_bound_qratio_vec_[0] = 1.0;
  int last_soft_lbound_id = soft_bounds_info[1].first.id;
  int last_soft_ubound_id = soft_bounds_info[1].second.id;
  int last_hard_lbound_id = hard_bounds_info[1].first.id;
  int last_hard_ubound_id = hard_bounds_info[1].second.id;
  double init_s = 0.0;
  double ds = ref_vel_ * weight_.dt;
  hard_lbound_s_vector.emplace_back(init_s);
  hard_ubound_s_vector.emplace_back(init_s);
  hard_lbound_l_vector.emplace_back(hard_bounds[0].first);
  hard_ubound_l_vector.emplace_back(hard_bounds[0].second);
  for (size_t i = 1; i < soft_bounds_info.size(); ++i) {
    int soft_lbound_id = soft_bounds_info[i].first.id;
    int soft_ubound_id = soft_bounds_info[i].second.id;
    int hard_lbound_id = hard_bounds_info[i].first.id;
    int hard_ubound_id = hard_bounds_info[i].second.id;
    auto soft_lbound_type = soft_bounds_info[i].first.type;
    auto soft_ubound_type = soft_bounds_info[i].second.type;
    auto hard_lbound_type = hard_bounds_info[i].first.type;
    auto hard_ubound_type = hard_bounds_info[i].second.type;
    if (soft_lbound_id != last_soft_lbound_id) {
      is_update_soft_lbound = true;
    }
    if (soft_ubound_id != last_soft_ubound_id) {
      is_update_soft_ubound = true;
    }
    if (hard_lbound_id != last_hard_lbound_id) {
      is_update_hard_lbound = true;
    }
    if (hard_ubound_id != last_hard_ubound_id) {
      is_update_hard_ubound = true;
    }
    if (is_update_soft_lbound || is_update_soft_ubound) {
      if ((is_update_soft_lbound &&
           soft_lbound_type == planning::BoundType::ROAD_BORDER) ||
          (is_update_soft_ubound &&
           soft_ubound_type == planning::BoundType::ROAD_BORDER)) {
          q_soft_bound_ratio += 0.2;
      } else {
        auto base_ratio = weight_.time2soft_ratio.find((i - 1) / 5);
        if (base_ratio != weight_.time2soft_ratio.end()) {
          q_soft_bound_ratio =
            std::max(base_ratio->second, q_soft_bound_ratio - 0.1);
        } else {
          q_soft_bound_ratio -= 0.1;
        }
      }
      is_update_soft_lbound = false;
      is_update_soft_ubound = false;
    }
    if (is_update_hard_lbound || is_update_hard_ubound) {
      if ((is_update_hard_lbound &&
           hard_lbound_type == planning::BoundType::ROAD_BORDER) ||
          (is_update_hard_ubound &&
           hard_ubound_type == planning::BoundType::ROAD_BORDER)) {
          q_hard_bound_ratio += 0.4;
      } else {
        auto base_ratio = weight_.time2hard_ratio.find((i - 1) / 5);
        if (base_ratio != weight_.time2hard_ratio.end()) {
          q_hard_bound_ratio =
            std::max(base_ratio->second, q_hard_bound_ratio - 0.1);
        } else {
          q_hard_bound_ratio -= 0.1;
        }
      }
      is_update_hard_lbound = false;
      is_update_hard_ubound = false;
    }
    last_soft_lbound_id = soft_lbound_id;
    last_soft_ubound_id = soft_ubound_id;
    last_hard_lbound_id = hard_lbound_id;
    last_hard_ubound_id = hard_ubound_id;
    q_soft_bound_ratio *= time_factor;
    q_hard_bound_ratio *= time_factor;
    if (q_soft_bound_ratio >= q_hard_bound_ratio - 0.1) {
      q_soft_bound_ratio = q_hard_bound_ratio - 0.1;
    }
    soft_bound_qratio_vec_[i] = std::max(q_soft_bound_ratio, 1.0);
    hard_bound_qratio_vec_[i] = std::max(q_hard_bound_ratio, 1.0);
    //
    init_s += ds;
    // only ROAD_BORDER
    for (auto &hard_bound : hard_bounds_vec[i]) {
      if (hard_bound.bound_info.type == planning::BoundType::ROAD_BORDER) {
        hard_lbound_s_vector.emplace_back(init_s);
        hard_lbound_l_vector.emplace_back(hard_bound.lower);
        hard_ubound_s_vector.emplace_back(init_s);
        hard_ubound_l_vector.emplace_back(hard_bound.upper);
      }
    }
  }
  hard_lbound_l_s_spline_.set_points(
    hard_lbound_s_vector, hard_lbound_l_vector, pnc::mathlib::spline::linear);
  hard_ubound_l_s_spline_.set_points(
    hard_ubound_s_vector, hard_ubound_l_vector, pnc::mathlib::spline::linear);
}

void LateralMotionPlanningWeight::SetAccJerkBoundAndWeight(
    planning::common::LateralPlanningInput &planning_input) {
  double acc_bound = std::min(config_.acc_bound, max_acc_);
  double jerk_bound = config_.jerk_bound;  // 0.2
  if (lateral_motion_scene_ == LANE_CHANGE ||
      is_lane_change_back_) {
    jerk_bound = config_.jerk_bound_lane_change;  // 0.55,
  } else if (lateral_motion_scene_ == SPLIT) {
    jerk_bound = config_.jerk_bound_split;  // 0.6
  } else if (lateral_motion_scene_ == RAMP) {
    jerk_bound = config_.jerk_bound_ramp;  // 1.0
  }
  jerk_bound = std::min(jerk_bound, max_jerk_);
  weight_.q_acc.clear();
  weight_.q_acc.resize(weight_.point_num, 0.1);
  for (size_t i = 0; i < weight_.point_num; ++i) {
    weight_.acc_upper_bound[i] = acc_bound;
    weight_.acc_lower_bound[i] = -acc_bound;
    weight_.jerk_upper_bound[i] = jerk_bound;
    weight_.jerk_lower_bound[i] = -jerk_bound;
    weight_.q_acc_bound[i] = config_.q_acc_bound;
    weight_.q_jerk_bound[i] = config_.q_jerk_bound;
    if (lateral_motion_scene_ == LANE_KEEP &&
        is_in_intersection_) {
      weight_.q_acc[i] = config_.q_acc_intersection;
    } else if (lateral_motion_scene_ == RAMP) {
      if (i > weight_.proximal_index) {
        weight_.q_acc[i] = config_.q_acc_ramp;
      }
    }
    if (i < 6) {
      weight_.expected_acc[i] = expected_average_acc_ * 1.2 - init_dis_to_ref_;
    } else {
      weight_.expected_acc[i] = expected_average_acc_ * 1.2;
    }
  }

  planning_input.set_acc_bound(acc_bound);
  planning_input.set_jerk_bound(jerk_bound);
  planning_input.set_q_acc_bound(config_.q_acc_bound);
  planning_input.set_q_jerk_bound(config_.q_jerk_bound);
}

void LateralMotionPlanningWeight::SetMinJerkWeightByVel(
    planning::common::LateralPlanningInput &planning_input) {
  std::vector<double> xp_v{0.2, 2.0, 4.167, 8.333, 25.0};
  std::vector<double> fp_min_qjerk{500.0, 40.0, 20.0, 5.0, 2.0};
  min_q_jerk_ = planning::interp(ref_vel_, xp_v, fp_min_qjerk);
  double origin_q_jerk = planning_input.q_jerk();
  concerned_start_q_jerk_ = std::max(concerned_start_q_jerk_, min_q_jerk_);
  planning_input.set_q_jerk(std::max(origin_q_jerk, min_q_jerk_));

  if (ref_vel_ < 1.389) {  // 5 kph
    planning_input.set_q_continuity(config_.q_continuity_low_speed);
  }
}

void LateralMotionPlanningWeight::CalculateJerkBoundByLastJerk(
    const std::shared_ptr<planning::ReferencePath> &reference_path,
    const planning::common::LateralPlanningOutput &last_planning_output,
    planning::common::LateralPlanningInput &planning_input) {
  // set upper limit
  std::vector<double> xp_v{4.167, 8.333, 15.0, 25.0};
  std::vector<double> fp_emergency_jerk{1.0, 1.0, 0.8, 0.6};
  double emergency_jerk_bound = planning::interp(ref_vel_, xp_v, fp_emergency_jerk);
  double jerk_bound =  // 0.4 0.4 0.4 0.3
      planning::interp(ref_vel_, xp_v, config_.map_jerk_bound);
  if (lateral_motion_scene_ == AVOID) {
    jerk_bound = config_.jerk_bound_avoid;  // 0.4,
  } else if (lateral_motion_scene_ == LANE_CHANGE ||
      is_lane_change_back_) {
    jerk_bound = config_.jerk_bound_lane_change;  // 0.55,
  } else if (lateral_motion_scene_ == SPLIT) {
    jerk_bound = config_.jerk_bound_split;  // 0.6
  } else if (lateral_motion_scene_ == RAMP) {
    jerk_bound = config_.jerk_bound_ramp;  // 1.0
  }
  // emergency
  if (reference_path != nullptr &&
      !hard_lbound_l_s_spline_.get_x().empty() &&
      !hard_lbound_l_s_spline_.get_y().empty() &&
      !hard_ubound_l_s_spline_.get_x().empty() &&
      !hard_ubound_l_s_spline_.get_y().empty()) {
    double init_s =
        reference_path->get_frenet_ego_state()
                      .planning_init_point()
                      .frenet_state.s;
    const auto& frenet_coord = reference_path->get_frenet_coord();
    if (frenet_coord != nullptr) {
      size_t last_points_size =
          std::min(last_planning_output.x_vec_size(), last_planning_output.y_vec_size());
      for (size_t i = 0; i < last_points_size; ++i) {
        planning::Point2D cart_last_xy(last_planning_output.x_vec(i),
                             last_planning_output.y_vec(i));
        planning::Point2D frenet_last_sl;
        if (frenet_coord->XYToSL(cart_last_xy, frenet_last_sl)) {
          double last_point_rel_s = frenet_last_sl.x - init_s;
          if (last_point_rel_s < 1e-6) {
            continue;
          }
          double hard_lbound_l = hard_lbound_l_s_spline_(last_point_rel_s);
          double hard_ubound_l = hard_ubound_l_s_spline_(last_point_rel_s);
          if (frenet_last_sl.y < hard_lbound_l ||
              frenet_last_sl.y > hard_ubound_l) {
              is_emergency_ = true;
              break;
          }
        }
      }
    } else {
      is_emergency_ = false;
    }
  } else {
    is_emergency_ = false;
  }
  if (is_emergency_) {
    jerk_bound = std::max(emergency_jerk_bound, jerk_bound);
  }
  jerk_bound = std::max(last_jerk_bound_limit_, jerk_bound);
  jerk_bound = std::min(jerk_bound, max_jerk_);
  // use last jerk
  // when last big jerk exceed jerk bound , loosening jerk bound
  bool is_need_loosening_upper_jerk_bound = false;
  bool is_need_loosening_lower_jerk_bound = false;
  double new_jerk_ubound = 0;
  double new_jerk_lbound = 0;
  for (size_t i = 0; i < last_planning_output.jerk_vec_size(); ++i) {
    double last_jerk_i = last_planning_output.jerk_vec(i);
    if (i < 20) {  // only use 4s
      double extra_upper_jerk = last_jerk_i - weight_.jerk_upper_bound[i];
      double extra_lower_jerk = last_jerk_i - weight_.jerk_lower_bound[i];
      if (extra_upper_jerk > -1e-3) {
        is_need_loosening_upper_jerk_bound = true;
        new_jerk_ubound = std::min(std::max(weight_.jerk_upper_bound[i] + extra_upper_jerk + 0.1, new_jerk_ubound), jerk_bound);
      } else if (extra_lower_jerk < 1e-3) {
        is_need_loosening_lower_jerk_bound = true;
        new_jerk_lbound = std::max(std::min(weight_.jerk_lower_bound[i] + extra_lower_jerk - 0.1, new_jerk_lbound), -jerk_bound);
      }
    }
  }
  if (is_need_loosening_upper_jerk_bound ||
      is_need_loosening_lower_jerk_bound) {
    double new_jerk_bound =
        std::max(std::fabs(new_jerk_ubound), std::fabs(new_jerk_lbound));
    new_jerk_bound = std::min(new_jerk_bound, max_jerk_);
    weight_.jerk_upper_bound.clear();
    weight_.jerk_upper_bound.resize(weight_.point_num, std::fabs(new_jerk_bound));
    weight_.jerk_lower_bound.clear();
    weight_.jerk_lower_bound.resize(weight_.point_num, -std::fabs(new_jerk_bound));
    planning_input.set_jerk_bound(std::fabs(new_jerk_bound));
  } else {
    is_emergency_ = false;
  }
  last_jerk_bound_limit_ = planning_input.jerk_bound();
  // when last positive or negative jerk is big, adding zero jerk bound protection for this time
  if (lateral_motion_scene_ == LANE_KEEP) {  // not big curvature
    if (last_planning_output.jerk_vec_size() >= 2) {
      double jerk_thr = 0.1;
      double init_jerk =
          0.5 * (last_planning_output.jerk_vec(0) + last_planning_output.jerk_vec(1));
      if (init_jerk > jerk_thr) {
        weight_.jerk_lower_bound[0] = 0.;
        weight_.jerk_lower_bound[1] = -0.1;
      } else if (init_jerk < -jerk_thr) {
        weight_.jerk_upper_bound[0] = 0.;
        weight_.jerk_upper_bound[1] = 0.1;
      }
    }
  }
}

void LateralMotionPlanningWeight::MakeDynamicWeight(
    planning::common::LateralPlanningInput &planning_input) {
  std::vector<double> xp_v{1.0, 5.0, 10.0, 20.0};
  double q_xy = planning::interp(ref_vel_, xp_v, config_.map_qxy);
  std::vector<double> xp_xy{0.2, 0.4, 0.8, 1.5, 3.0};
  std::vector<double> fp_ratio_to_xy1{1.0, 0.9,0.8, 0.6, 0.5};
  std::vector<double> xp_theta{0.5, 2.0, 5.0};
  std::vector<double> fp_ratio_to_xy2{1.0, 0.5, 0.3};
  double lateral_dist = std::max(std::fabs(avoid_dist_), std::fabs(init_l_ - lat_offset_));
  double q_xy_ratio1 =
    planning::interp(lateral_dist, xp_xy, fp_ratio_to_xy1);
  double q_xy_ratio2 =
    planning::interp(std::fabs(init_ref_theta_error_), xp_theta, fp_ratio_to_xy2);
  planning_input.set_q_ref_x(q_xy_ratio1 * q_xy_ratio2 * q_xy);
  planning_input.set_q_ref_y(q_xy_ratio1 * q_xy_ratio2 * q_xy);


  std::vector<double> fp_ratio_to_theta1{1.0, 1.1, 1.3, 1.5, 1.8};
  std::vector<double> fp_ratio_to_theta2{1.0, 1.3, 1.8, 2.5, 4.0};
  std::vector<double> fp_ratio_to_theta3{1.0, 1.5, 3.0};
  double q_theta = planning::interp(ref_vel_, xp_v, config_.map_qtheta);
  double q_theta_ratio1 =
    planning::interp(std::fabs(init_l_ - lat_offset_), xp_xy, fp_ratio_to_theta1);
  double q_theta_ratio2 =
    planning::interp(std::fabs(avoid_dist_), xp_xy, fp_ratio_to_theta2);
  double q_theta_ratio3 =
    planning::interp(std::fabs(init_ref_theta_error_), xp_theta, fp_ratio_to_theta3);
  planning_input.set_q_ref_theta(q_theta_ratio1 * q_theta_ratio2 * q_theta_ratio3 * q_theta);

  double q_jerk1 =
      planning::interp(lateral_dist, xp_xy, config_.map_qjerk1);
  double q_jerk2 =
      planning::interp(lateral_dist, xp_xy, config_.map_qjerk2);
  std::vector<double> fp_ratio_to_jerk{1.0, 4.0, 10.0};
  double q_jerk_ratio1 =
    planning::interp(std::fabs(init_ref_theta_error_), xp_theta, fp_ratio_to_jerk);

  concerned_start_q_jerk_ = q_jerk_ratio1 * q_jerk1;
  planning_input.set_q_jerk(q_jerk_ratio1 * q_jerk2);

  if ((std::fabs(init_ref_theta_error_) >= config_.big_theta_thr) &&
      // (std::fabs(init_ref_theta_error_) <= 2.0) &&
      // (std::fabs(init_dis_to_ref_) < 0.4) &&
      (std::fabs(avoid_dist_) < 0.2)) {
    weight_.complete_follow = true;
  }
}

void LateralMotionPlanningWeight::MakeRampDynamicWeight(
    planning::common::LateralPlanningInput &planning_input) {
  double q_xy = config_.q_ref_x_ramp;
  std::vector<double> xp_xy{0.2, 0.4, 0.8, 1.5};
  std::vector<double> fp_ratio_to_xy1{1.0, 1.0, 0.5, 0.25};
  double lateral_dist = std::fabs(init_l_ - lat_offset_);
  double q_xy_ratio1 =
    planning::interp(lateral_dist, xp_xy, fp_ratio_to_xy1);
  planning_input.set_q_ref_x(q_xy_ratio1 * q_xy);
  planning_input.set_q_ref_y(q_xy_ratio1 * q_xy);

  double q_theta = config_.q_ref_theta_ramp;
  std::vector<double> xp_theta{1.0, 2.0};
  std::vector<double> fp_ratio_to_theta1{0.25, 1.0};
  double q_theta_ratio1 =
    planning::interp(std::fabs(init_ref_theta_error_), xp_theta, fp_ratio_to_theta1);
  if (lateral_dist < 0.4) {
    q_theta_ratio1 = 0.25;
  }
  planning_input.set_q_ref_theta(q_theta_ratio1 * q_theta);
}

void LateralMotionPlanningWeight::MakeLaneChangeDynamicWeight(
    planning::common::LateralPlanningInput &planning_input) {
  end_ratio_for_qrefxy_ = config_.lc_end_ratio_for_second_qrefxy;
  if (std::fabs(avoid_dist_) > 1e-3) {
    end_ratio_for_qrefxy_ = config_.end_ratio_for_qrefxy;
    end_ratio_for_qreftheta_ = config_.end_ratio_for_qreftheta;
  }
  std::vector<double> xp_xy{0.25, 0.5, 1.0, 1.5};
  std::vector<double> xp_xy2{0.15, 0.5, 1.0, 1.5};
  std::vector<double> xp_xy3{0.15, 0.5, 1.5};
  if (ref_vel_ <= config_.lane_change_high_vel || config_.use_new_lc_param) {
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
    if (ref_vel_ <= config_.lane_change_high_vel) {
      q_ref_theta *= 0.1;
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
  }
}

void LateralMotionPlanningWeight::MakeLaneChangeBackDynamicWeight(
    planning::common::LateralPlanningInput &planning_input) {
  std::vector<double> xp_xy{0.2, 0.4, 0.6};
  std::vector<double> fp_factor1{10.0, 5.0, 1.0};
  std::vector<double> fp_factor2{0.2, 0.6, 1.0};
  double init_lat_dist = std::fabs(init_l_ - lat_offset_);
  double q_ref_xy_factor =
      planning::interp(init_lat_dist, xp_xy, fp_factor1);
  planning_input.set_q_ref_x(config_.q_ref_xy_lane_change_back * q_ref_xy_factor);
  planning_input.set_q_ref_y(config_.q_ref_xy_lane_change_back * q_ref_xy_factor);
  double q_ref_theta_lc_back = std::max(config_.q_ref_theta_lane_change_back +
      (ref_vel_ - config_.lane_change_high_vel) * 1000.0,
      config_.q_ref_theta_lane_change_back);
  double q_ref_theta_factor =
      planning::interp(init_lat_dist, xp_xy, fp_factor2);
  planning_input.set_q_ref_theta(q_ref_theta_lc_back * q_ref_theta_factor);
  std::vector<double> xp_v{1.5, 3.0, 4.167, 8.333};
  std::vector<double> fp_qjerk{500.0, 200.0, 100.0, config_.q_jerk_lane_change_back};
  double q_jerk = planning::interp(ref_vel_, xp_v, fp_qjerk);
  planning_input.set_q_jerk(q_jerk);
  concerned_start_q_jerk_ = q_jerk;
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
  std::vector<double> xp_vel{5, 10, 15, 20};
  std::vector<double> fp_dr{0.125, 0.25, 0.5, 1.0};
  double decay_ratio =
      planning::interp(ref_vel_, xp_vel, fp_dr);
  double q_ref_theta =
      planning::interp(std::fabs(init_dis_to_ref_), xp_xy3, fp_qtheta) * decay_ratio;
  planning_input.set_q_ref_theta(q_ref_theta);
}

void LateralMotionPlanningWeight::MakeDynamicPosBoundWeight(
    planning::common::LateralPlanningInput &planning_input) {
  double emergence_factor = 1.0;
  double intersection_factor = 1.0;
  if (is_emergency_) {
    emergence_factor = config_.emergence_avoid_factor;
  }
  if (is_in_intersection_) {
    intersection_factor = config_.intersection_avoid_factor;
  }
  if (std::fabs(avoid_dist_) <= 0.1 ||
      lateral_motion_scene_ == RAMP) {
    soft_bound_qratio_vec_.clear();
    hard_bound_qratio_vec_.clear();
    soft_bound_qratio_vec_.resize(26, 1.0);
    hard_bound_qratio_vec_.resize(26, 1.0);
  }
  // std::vector<double> xp_road_radius{50.0, 200.0, 400.0, 800.0};
  // std::vector<double> fp_kappa_coefficient{0.2, 0.4, 0.6, 1.0};
  // double kappa_coeff =
  //     planning::interp(std::fabs(curvature_radius_vec_[3]), xp_road_radius, fp_kappa_coefficient);
  // Set according to different vel
  std::vector<double> xp_v{1.0, 4.167, 8.333, 16.667, 25.0, 30.0};
  double q_soft_bound =
      planning::interp(ref_vel_, xp_v, config_.map_qsoft_bound) *
      emergence_factor * intersection_factor;
  double q_hard_bound =
      planning::interp(ref_vel_, xp_v, config_.map_qhard_bound) *
      emergence_factor * intersection_factor;
  if (std::fabs(avoid_dist_) > 0.4 &&
      (lateral_motion_scene_ != LANE_CHANGE &&
       !is_lane_change_back_)) {
    std::vector<double> xp_lat_dist{0.4, 1.5, 3.0};
    std::vector<double> fp_ratio_to_bound{1.0, 0.4, 0.1};
    double decay_ratio = planning::interp(std::fabs(avoid_dist_), xp_lat_dist, fp_ratio_to_bound);
    q_soft_bound *= decay_ratio;
    q_hard_bound *= decay_ratio;
  }
  planning_input.set_q_soft_corridor(q_soft_bound);
  planning_input.set_q_hard_corridor(q_hard_bound);
  for (size_t i = 0; i < weight_.point_num; ++i) {
    weight_.q_pos_soft_bound[i] = q_soft_bound * soft_bound_qratio_vec_[i];
    weight_.q_pos_hard_bound[i] = q_hard_bound * hard_bound_qratio_vec_[i];
  }

  if (std::fabs(avoid_dist_) > 0.1) {
    end_ratio_for_qreftheta_ = 0.7;
    end_ratio_for_qjerk_ = 0.7;
  }
}

void LateralMotionPlanningWeight::SetMotionPlanConcernedEndIndex(
    const bool origin_complete_follow, const bool is_divide_lane_into_two,
    const std::shared_ptr<planning::ReferencePath> &reference_path,
    planning::common::LateralPlanningInput &planning_input) {
  double ego_s = reference_path->get_frenet_ego_state().s();
  const auto& frenet_coord = reference_path->get_frenet_coord();
  if (origin_complete_follow) {
    weight_.complete_follow = origin_complete_follow;
  }

  std::vector<double> xp_road_radius{50.0, 150.0, 500.0, 1000.0, 2000.0};
  double valid_perception_range =
      planning::interp(
        min_road_radius_, xp_road_radius, config_.valid_perception_range);
  for (size_t i = weight_.proximal_index + 1; i < weight_.remotely_index; ++i) {
    planning::Point2D cart_ref_xy(planning_input.ref_x_vec(i),
                        planning_input.ref_y_vec(i));
    planning::Point2D frenet_ref_sl;
    if (frenet_coord != nullptr &&
        frenet_coord->XYToSL(cart_ref_xy,
                             frenet_ref_sl)) {
      if (frenet_ref_sl.x > (ego_s + valid_perception_range)) {
        weight_.remotely_index = i;
        break;
      }
    }
  }
  double lateral_dist = std::max(std::fabs(avoid_dist_), std::fabs(init_l_ - lat_offset_));
  if (lateral_dist > 0.6) {
    size_t min_remotely_index = 15;
    weight_.remotely_index = std::max(min_remotely_index, weight_.remotely_index);
  }

  if ((lateral_motion_scene_ == RAMP &&
       is_in_intersection_) ||
      is_s_bend_) {
    weight_.remotely_index = 20;
    planning_input.set_q_acc(0.0);
  }

  // const double lateral_offset = lateral_offset_decider_output.lateral_offset;
  if ((lateral_motion_scene_ == LANE_CHANGE) && (!is_lane_change_back_)) {
    if (ref_vel_ <= config_.lane_change_high_vel || config_.use_new_lc_param) {
      weight_.complete_follow = false;
      weight_.remotely_index = 20;
    }
    // complete_follow = false;
    // weight_.remotely_index = 17;
    // for (size_t i = 1; i < 17; ++i) {
    //   planning::Point2D cart_refi(planning_input_.ref_x_vec(i),
    //                     planning_input_.ref_y_vec(i));
    //   planning::Point2D frenet_refi;
    //   if (reference_path_ptr->get_frenet_coord() != nullptr &&
    //       reference_path_ptr->get_frenet_coord()->XYToSL(cart_refi,
    //                                                      frenet_refi)) {
    //     if (std::fabs(frenet_refi.y - lateral_offset) < 0.05) {
    //       weight_.remotely_index = i - 1;
    //       break;
    //     }
    //   }
    // }
    // if (weight_.remotely_index < 1 || std::fabs(frenet_init.y -
    // frenet_ref0.y) > 1e-6) {
    if (std::fabs(init_dis_to_ref_) > 0.01) {
      weight_.complete_follow = false;
      if (ref_vel_ <= config_.lane_change_high_vel) {
        weight_.remotely_index = 20;
      } else {
        weight_.remotely_index = 17;
      }
      MakeLaneChangeDynamicWeight(planning_input);
    } else {
      if (last_path_max_dist2ref_ > 1.0) {
        end_ratio_for_qrefxy_ = config_.end_ratio_for_qrefxy;
        end_ratio_for_qreftheta_ = config_.end_ratio_for_qreftheta;
        planning_input.set_q_continuity(config_.q_continuity_lane_change);
      }
    }
  } else if (lateral_motion_scene_ == SPLIT) {
    if (ref_vel_ <= config_.lane_change_high_vel) {
      weight_.remotely_index = 20;
    } else {
      weight_.remotely_index = 17;
    }
    if (!is_divide_lane_into_two) {
      MakeSplitDynamicWeight(planning_input);
    } else {
      weight_.complete_follow = true;
    }
  }

  // set complete hold flag, concerned index
  planning_input.set_complete_follow(weight_.complete_follow);
  planning_input.set_motion_plan_concerned_index(
      weight_.remotely_index);
  // set low speed protection
  SetMinJerkWeightByVel(planning_input);
}

}  // namespace lateral_planning
}  // namespace pnc

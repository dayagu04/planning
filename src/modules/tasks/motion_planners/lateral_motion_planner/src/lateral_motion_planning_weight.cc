#include "lateral_motion_planning_weight.h"
#include <cmath>
#include <cstddef>

#include "math_lib.h"
#include "refline.h"

static const double kCurvatureThreshold =
    1 / 750;  // 750m raidus for big curvature
static const double kRad2Deg = 57.3;

namespace pnc {
namespace lateral_planning {

LateralMotionPlanningWeight::LateralMotionPlanningWeight(
    const planning::LateralMotionPlannerConfig &config)
    : config_(config) {
  Init();
}

void LateralMotionPlanningWeight::Init() {
  lateral_motion_scene_ = LANE_KEEP;
  emergency_level_ = NONE;
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
  max_jerk_low_speed_ = 0.2;
  last_expected_average_acc_ = 0.0;
  last_jerk_bound_limit_ = 0.2;
  expected_average_acc_ = 0.0;
  expected_max_acc_ = 0.0;
  expected_min_acc_ = 0.0;
  target_road_radius_ = 10000.0;
  min_q_jerk_ = 2.0;
  last_path_max_dist2ref_ = 0.0;
  last_max_omega_ = 0.0;
  last_remotely_index_ = 20;
  is_lane_change_hold_ = false;
  is_lane_change_back_ = false;
  is_in_intersection_ = false;
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
      planning_input.set_q_continuity(config_.q_continuity);
      planning_input.set_q_acc(config_.q_acc_avoid);
      MakeLateralOffsetAvoidDynamicWeight(planning_input);
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

      if (is_lane_change_back_ ||
          is_lane_change_hold_) {
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
    case LANE_BORROW: {
      planning_input.set_q_ref_x(config_.q_ref_x_lane_change);
      planning_input.set_q_ref_y(config_.q_ref_y_lane_change);
      planning_input.set_q_ref_theta(config_.q_ref_theta_lane_change);
      planning_input.set_q_continuity(config_.q_continuity);
      planning_input.set_q_acc(config_.q_acc_lane_change);
      planning_input.set_q_jerk(config_.q_jerk_lane_change);
      concerned_start_q_jerk_ = config_.q_jerk_lane_change;
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
      kRad2Deg;
}

void LateralMotionPlanningWeight::CalculateLastPathDistToRef(
    const std::shared_ptr<planning::ReferencePath> &reference_path,
    planning::common::LateralPlanningInput &planning_input) {
  last_path_max_dist2ref_ = 0.0;
  double min_ref_size =
      std::min(planning_input.ref_x_vec_size(),
               planning_input.ref_y_vec_size());
  double min_last_path_size =
      std::min(planning_input.last_x_vec_size(),
               planning_input.last_y_vec_size());
  if (reference_path == nullptr ||
      min_ref_size == 0 ||
      min_last_path_size == 0) {
    return;
  }
  const auto &frenet_coord = reference_path->get_frenet_coord();
  std::vector<double> ref_s_vec;
  std::vector<double> ref_l_vec;
  ref_s_vec.reserve(min_ref_size);
  ref_l_vec.reserve(min_ref_size);
  double ref_last_s = 0;
  for (size_t i = 0; i < min_ref_size; ++i) {
    planning::Point2D cart_ref_xy(planning_input.ref_x_vec(i),
                                  planning_input.ref_y_vec(i));
    planning::Point2D frenet_ref_xy;
    if (frenet_coord->XYToSL(cart_ref_xy, frenet_ref_xy)) {
      if (i > 0 &&
          frenet_ref_xy.x <= ref_last_s) {
        continue;
      }
      ref_last_s = frenet_ref_xy.x;
      ref_s_vec.emplace_back(frenet_ref_xy.x);
      ref_l_vec.emplace_back(frenet_ref_xy.y);
    }
  }
  if (ref_s_vec.size() < 3) {
    return;
  }
  pnc::mathlib::spline ref_l_s_spline;
  ref_l_s_spline.set_points(ref_s_vec, ref_l_vec);
  for (size_t j = 0; j < min_last_path_size; ++j) {
    planning::Point2D cart_last_xy(planning_input.last_x_vec(j),
                                   planning_input.last_y_vec(j));
    planning::Point2D frenet_last_xy;
    if (frenet_coord->XYToSL(cart_last_xy, frenet_last_xy)) {
      if (frenet_last_xy.x < ref_s_vec.front() ||
          frenet_last_xy.x > ref_s_vec.back()) {
        continue;
      }
      double ref_rel_l = ref_l_s_spline(frenet_last_xy.x);
      last_path_max_dist2ref_ =
          std::max(std::fabs(frenet_last_xy.y - ref_rel_l), last_path_max_dist2ref_);
    }
  }
}

void LateralMotionPlanningWeight::CalculateExpectedLatAccAndSteerAngle(
    double init_s, double ref_vel, double wheel_base, double steer_ratio,
    const planning::CoarsePlanningInfo &coarse_planning_info,
    const std::shared_ptr<planning::ReferencePath> &reference_path,
    std::vector<double>& expected_steer_vec) {
  const auto &k_s_spline =
      coarse_planning_info.cart_ref_info.k_s_spline;
  expected_average_acc_ = 0.0;
  expected_max_acc_ = -10.0;
  expected_min_acc_ = 10.0;
  target_road_radius_ = 10000.0;
  is_s_bend_ = false;
  bool is_k_s_spline_valid = false;
  if (!k_s_spline.get_x().empty() &&
      !k_s_spline.get_y().empty()) {
    is_k_s_spline_valid = true;
  }
  bool is_left_bend = false;
  bool is_right_bend = false;
  size_t time = 0;
  size_t kappa_gap = 0;
  double min_road_radius = 10000.0;
  double max_road_radius = 1.0;
  double sum_kappa = 0;
  double ds = ref_vel * config_.delta_t;
  double kv2 = config_.curv_factor * ref_vel_ * ref_vel_;
  max_jerk_low_speed_ =
      (config_.max_steer_angle_dot_low_speed / steer_ratio / kRad2Deg) * kv2;
  weight_.expected_acc.clear();
  weight_.expected_acc.resize(26, 0.0);
  curvature_radius_vec_.clear();
  curvature_radius_vec_.resize(6, 10000.0);
  double pt_kappa = 1e-6;
  planning::ReferencePathPoint ref_point;
  for (size_t i = 0; i < weight_.point_num; ++i) {
    if (is_k_s_spline_valid &&
        reference_path->get_reference_point_by_lon(init_s, ref_point)) {
      pt_kappa = k_s_spline(init_s);
      if (ref_point.path_point.kappa() < 0) {
        pt_kappa *= -1;
      }
    } else {
      if (i > 0) {
        expected_steer_vec[i] = expected_steer_vec[i - 1];
        weight_.expected_acc[i] = weight_.expected_acc[i - 1];
        if (i < 16) {
          expected_average_acc_ += weight_.expected_acc[i - 1];
        }
      }
      init_s += ds;
      continue;
    }
    if (pt_kappa < -kCurvatureThreshold) {
      is_left_bend = true;
    } else if (pt_kappa > kCurvatureThreshold) {
      is_right_bend = true;
    }
    kappa_gap += 1;
    sum_kappa += pt_kappa;
    double expected_delta = std::atan(wheel_base * pt_kappa);  // rad
    double expected_steer = steer_ratio * expected_delta * kRad2Deg;  // deg
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
        curvature_radius_vec_[time] = average_radius;
        if (time < 5) {
          min_road_radius = std::min(std::fabs(curvature_radius_vec_[time]), min_road_radius);
          max_road_radius = std::max(std::fabs(curvature_radius_vec_[time]), max_road_radius);
        }
      }
      time += 1;
      sum_kappa = 0;
      kappa_gap = 0;
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
  // anchor: target road radius
  double end_radius =
      std::fabs(curvature_radius_vec_[4]);
  double far_radius =
      std::fabs(curvature_radius_vec_[3]);
  double mid_radius =
      std::fabs(curvature_radius_vec_[2]);
  double max_near_radius =
      std::max(std::fabs(curvature_radius_vec_[1]),
               std::fabs(curvature_radius_vec_[0]));
  if (max_road_radius < 150.0 ||   // in large bend
      max_near_radius >= 400.0) {  // ego not-in large bend
    target_road_radius_ = max_near_radius;
  } else {
    if (end_radius - far_radius >= 100.0) {
      target_road_radius_ =
          std::max(0.5 * (end_radius + far_radius),
                   max_near_radius);
    } else {
      if (far_radius - mid_radius >= 100.0) {
        target_road_radius_ =
            std::max(0.5 * (far_radius + mid_radius),
                     max_near_radius);
      } else {
        if (mid_radius - max_near_radius >= 50.0) {
          target_road_radius_ =
              0.5 * (mid_radius + max_near_radius);
        } else {
          if (far_radius - max_near_radius >= 100.0) {
            target_road_radius_ =
                0.5 * (far_radius + max_near_radius);
          } else {
            target_road_radius_ = max_near_radius;
          }
        }
      }
    }
  }
  target_road_radius_ = std::min(target_road_radius_, 10000.0);
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
    const std::vector<planning::WeightedBounds> soft_bounds_vec,
    const std::vector<planning::WeightedBounds> hard_bounds_vec,
    const std::vector<std::pair<planning::BoundInfo, planning::BoundInfo>>& soft_bounds_info,
    const std::vector<std::pair<planning::BoundInfo, planning::BoundInfo>>& hard_bounds_info) {
  std::vector<double> bound_s_vector;
  std::vector<double> soft_lbound_l_vector;
  std::vector<double> soft_ubound_l_vector;
  std::vector<double> hard_lbound_l_vector;
  std::vector<double> hard_ubound_l_vector;
  size_t bounds_size =
      std::min(soft_bounds_info.size(), hard_bounds_info.size());
  if (bounds_size < weight_.point_num ||
      ref_vel_ <= 1e-2) {
    soft_bound_qratio_vec_.clear();
    hard_bound_qratio_vec_.clear();
    soft_bound_qratio_vec_.resize(26, 1.0);
    hard_bound_qratio_vec_.resize(26, 1.0);
    soft_lbound_l_s_spline_.get_x().clear();
    soft_lbound_l_s_spline_.get_y().clear();
    soft_ubound_l_s_spline_.get_x().clear();
    soft_ubound_l_s_spline_.get_y().clear();
    hard_lbound_l_s_spline_.get_x().clear();
    hard_lbound_l_s_spline_.get_y().clear();
    hard_ubound_l_s_spline_.get_x().clear();
    hard_ubound_l_s_spline_.get_y().clear();
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
  bound_s_vector.emplace_back(init_s);
  soft_lbound_l_vector.emplace_back(soft_bounds[0].first);
  soft_ubound_l_vector.emplace_back(soft_bounds[0].second);
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
    bound_s_vector.emplace_back(init_s);
    // only ROAD_BORDER
    for (auto &soft_bound : soft_bounds_vec[i]) {
      if (soft_bound.bound_info.type == planning::BoundType::ROAD_BORDER) {
        soft_lbound_l_vector.emplace_back(soft_bound.lower);
        soft_ubound_l_vector.emplace_back(soft_bound.upper);
      }
    }
    for (auto &hard_bound : hard_bounds_vec[i]) {
      if (hard_bound.bound_info.type == planning::BoundType::ROAD_BORDER) {
        hard_lbound_l_vector.emplace_back(hard_bound.lower);
        hard_ubound_l_vector.emplace_back(hard_bound.upper);
      }
    }
  }
  soft_lbound_l_s_spline_.set_points(
    bound_s_vector, soft_lbound_l_vector, pnc::mathlib::spline::linear);
  soft_ubound_l_s_spline_.set_points(
    bound_s_vector, soft_ubound_l_vector, pnc::mathlib::spline::linear);
  hard_lbound_l_s_spline_.set_points(
    bound_s_vector, hard_lbound_l_vector, pnc::mathlib::spline::linear);
  hard_ubound_l_s_spline_.set_points(
    bound_s_vector, hard_ubound_l_vector, pnc::mathlib::spline::linear);
}

void LateralMotionPlanningWeight::SetAccJerkBoundAndWeight(
    planning::common::LateralPlanningInput &planning_input) {
  double acc_bound = std::min(config_.acc_bound, max_acc_);
  double jerk_bound = config_.jerk_bound;  // 0.2
  if (lateral_motion_scene_ == AVOID) {
    jerk_bound = config_.jerk_bound_avoid;  // 0.4,
  } else if (lateral_motion_scene_ == LANE_CHANGE ||
             lateral_motion_scene_ == LANE_BORROW) {
    jerk_bound = config_.jerk_bound_lane_change;  // 0.55,
  } else if (lateral_motion_scene_ == SPLIT) {
    jerk_bound = config_.jerk_bound_split;  // 0.6
  } else if (lateral_motion_scene_ == RAMP) {
    std::vector<double> xp_road_radius{250.0, 400.0, 600.0, 750.0};
    jerk_bound = // 1.0 0.8 0.6 0.5
      planning::interp(target_road_radius_, xp_road_radius, config_.map_jerk_bound_ramp);
  }
  // tiny speed
  if (ego_vel_ < 0.2 &&
      lateral_motion_scene_ == LANE_KEEP &&
      std::fabs(avoid_dist_) < 0.1) {
    jerk_bound = max_jerk_low_speed_;
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
  // tiny speed
  if (ego_vel_ < 0.2 &&
      lateral_motion_scene_ == LANE_KEEP &&
      std::fabs(avoid_dist_) < 0.1) {
    planning_input.set_q_continuity(config_.q_continuity_low_speed);
  }
}

void LateralMotionPlanningWeight::CalculateJerkBoundByLastJerk(
    const bool is_high_priority_back,  const bool is_in_function,
    const double enter_lccnoa_time,
    const std::shared_ptr<planning::ReferencePath> &reference_path,
    const planning::common::LateralPlanningOutput &last_planning_output,
    planning::common::LateralPlanningInput &planning_input) {
  const auto &last_omega_vec = last_planning_output.omega_vec();
  last_max_omega_ = 0;
  for (size_t omega_i = 0; omega_i < last_omega_vec.size(); ++omega_i) {
    last_max_omega_ = std::max(std::fabs(last_omega_vec[omega_i]), last_max_omega_);
  }
  double last_omega_to_jerk =
      std::min(last_max_omega_, last_jerk_bound_limit_) *
      config_.curv_factor * ref_vel_ * ref_vel_;
  // set upper limit
  double extra_jerk_buffer = 0.1;
  std::vector<double> xp_v{4.167, 8.333, 16.667, 25.0};
  std::vector<double> fp_P0_emergency_jerk{1.2, 1.5, 1.4, 1.0};
  std::vector<double> fp_P1_emergency_jerk{1.0, 1.3, 1.2, 0.8};
  std::vector<double> fp_P2_emergency_jerk{0.8, 1.1, 1.0, 0.6};
  double P0_emergency_jerk_bound =
      planning::interp(ref_vel_, xp_v, fp_P0_emergency_jerk);
  double P1_emergency_jerk_bound =
      planning::interp(ref_vel_, xp_v, fp_P1_emergency_jerk);
  double P2_emergency_jerk_bound =
      planning::interp(ref_vel_, xp_v, fp_P2_emergency_jerk);
  double jerk_bound =  // 0.4 0.4 0.4 0.3
      planning::interp(ref_vel_, xp_v, config_.map_jerk_bound);
  if (lateral_motion_scene_ == AVOID) {
    jerk_bound = config_.jerk_bound_avoid;  // 0.4,
  } else if (lateral_motion_scene_ == LANE_CHANGE ||
             lateral_motion_scene_ == LANE_BORROW) {
    jerk_bound = config_.jerk_bound_lane_change;  // 0.55,
  } else if (lateral_motion_scene_ == SPLIT) {
    jerk_bound = config_.jerk_bound_split;  // 0.6
  } else if (lateral_motion_scene_ == RAMP) {
    std::vector<double> xp_road_radius{250.0, 400.0, 600.0, 750.0};
    jerk_bound = // 1.0 0.8 0.6 0.5
      planning::interp(target_road_radius_, xp_road_radius, config_.map_jerk_bound_ramp);
  }
  // emergency
  bool is_soft_bound_spline_valid = false;
  if (!soft_lbound_l_s_spline_.get_x().empty() &&
      !soft_lbound_l_s_spline_.get_y().empty() &&
      !soft_ubound_l_s_spline_.get_x().empty() &&
      !soft_ubound_l_s_spline_.get_y().empty()) {
    is_soft_bound_spline_valid = true;
  }
  bool is_hard_bound_spline_valid = false;
  if (!hard_lbound_l_s_spline_.get_x().empty() &&
      !hard_lbound_l_s_spline_.get_y().empty() &&
      !hard_ubound_l_s_spline_.get_x().empty() &&
      !hard_ubound_l_s_spline_.get_y().empty()) {
    is_hard_bound_spline_valid = true;
  }
  double violate_bound_max_dist = 0;
  if (reference_path != nullptr) {
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
          if (is_hard_bound_spline_valid) {
            double hard_lbound_l = hard_lbound_l_s_spline_(last_point_rel_s);
            double hard_ubound_l = hard_ubound_l_s_spline_(last_point_rel_s);
            if (frenet_last_sl.y < hard_lbound_l ||
                frenet_last_sl.y > hard_ubound_l) {
              emergency_level_ = P0;
              break;
            }
          }
          if (is_soft_bound_spline_valid) {
            double soft_lbound_l = soft_lbound_l_s_spline_(last_point_rel_s);
            double soft_ubound_l = soft_ubound_l_s_spline_(last_point_rel_s);
            if (frenet_last_sl.y < soft_lbound_l ||
                frenet_last_sl.y > soft_ubound_l) {
              violate_bound_max_dist =
                  std::max(std::max(soft_lbound_l - frenet_last_sl.y,
                    frenet_last_sl.y - soft_ubound_l), violate_bound_max_dist);
              emergency_level_ = P1;
            }
          }
        }
      }
    }
  }
  if (is_high_priority_back &&
      (is_lane_change_hold_ ||
       is_lane_change_back_)) {
    emergency_level_ = P0;
  }
  if (!is_in_function) {
    emergency_level_ = NONE;
  }
  double emergency_jerk_bound = P2_emergency_jerk_bound;
  if (emergency_level_ == P0) {
    jerk_bound = P0_emergency_jerk_bound;
    emergency_jerk_bound = P0_emergency_jerk_bound;
    extra_jerk_buffer = 1.0;
  } else if (emergency_level_ == P1) {
    std::vector<double> xp_violate_dist{0.05, 0.15, 0.3, 0.5};
    // std::vector<double> fp_extra_p1_jerk_bound{0.05, 0.1, 0.5, 0.8};
    std::vector<double> fp_p1_jerk_buffer{0.1, 0.2, 0.5, 0.7};
    extra_jerk_buffer =
        planning::interp(violate_bound_max_dist, xp_violate_dist, fp_p1_jerk_buffer);
    jerk_bound = P1_emergency_jerk_bound;
    emergency_jerk_bound = P0_emergency_jerk_bound;
  } else if (emergency_level_ == P2) {
    std::vector<double> xp_violate_dist{0.05, 0.15, 0.3, 0.5};
    // std::vector<double> fp_extra_p1_jerk_bound{0.05, 0.1, 0.4, 0.6};
    std::vector<double> fp_p2_jerk_buffer{0.1, 0.1, 0.3, 0.5};
    extra_jerk_buffer =
        planning::interp(violate_bound_max_dist, xp_violate_dist, fp_p2_jerk_buffer);
    jerk_bound = P2_emergency_jerk_bound;
    emergency_jerk_bound = P1_emergency_jerk_bound;
  }
  if (!is_in_function) {
    emergency_jerk_bound = config_.jerk_bound_inactivated_limit;
  }
  // consider 20 frame
  if (enter_lccnoa_time > 1e-6 &&
      enter_lccnoa_time < 2.0) {
    emergency_jerk_bound = P2_emergency_jerk_bound;
    extra_jerk_buffer =
        std::min(0.025, extra_jerk_buffer);
  }
  // jerk_bound = std::max(last_jerk_bound_limit_, jerk_bound);
  jerk_bound = std::max(last_omega_to_jerk, jerk_bound);
  // tiny speed
  if (ego_vel_ < 0.2 &&
      lateral_motion_scene_ == LANE_KEEP &&
      std::fabs(avoid_dist_) < 0.1) {
    jerk_bound = max_jerk_low_speed_;
  }
  jerk_bound =
      std::min(std::min(emergency_jerk_bound, jerk_bound), max_jerk_);
  // use last jerk
  // when last big jerk exceed jerk bound , loosening jerk bound
  bool is_need_loosening_upper_jerk_bound = false;
  bool is_need_loosening_lower_jerk_bound = false;
  double new_jerk_ubound = 0;
  double new_jerk_lbound = 0;
  if (is_in_function) {
    for (size_t i = 0; i < last_planning_output.jerk_vec_size(); ++i) {
      double last_jerk_i = last_planning_output.jerk_vec(i);
      if (i < 20) {  // only use 4s
        double extra_upper_jerk = last_jerk_i - weight_.jerk_upper_bound[i];
        double extra_lower_jerk = last_jerk_i - weight_.jerk_lower_bound[i];
        if (extra_upper_jerk > -1e-3) {
          is_need_loosening_upper_jerk_bound = true;
          new_jerk_ubound =
            std::min(std::max(weight_.jerk_upper_bound[i] + extra_upper_jerk + extra_jerk_buffer,
                              new_jerk_ubound), jerk_bound);
        } else if (extra_lower_jerk < 1e-3) {
          is_need_loosening_lower_jerk_bound = true;
          new_jerk_lbound =
            std::max(std::min(weight_.jerk_lower_bound[i] + extra_lower_jerk - extra_jerk_buffer,
                              new_jerk_lbound), -jerk_bound);
        }
      }
    }
  }
  if (std::fabs(planning_input.jerk_bound()) > emergency_jerk_bound) {
    is_need_loosening_upper_jerk_bound = true;
    is_need_loosening_lower_jerk_bound = true;
    new_jerk_ubound = emergency_jerk_bound;
    new_jerk_lbound = -emergency_jerk_bound;
  }
  if (is_need_loosening_upper_jerk_bound ||
      is_need_loosening_lower_jerk_bound) {
    double new_jerk_bound =
        std::max(std::fabs(new_jerk_ubound), std::fabs(new_jerk_lbound));
    new_jerk_bound =
        std::min(std::min(emergency_jerk_bound, new_jerk_bound), max_jerk_);
    weight_.jerk_upper_bound.clear();
    weight_.jerk_upper_bound.resize(weight_.point_num, std::fabs(new_jerk_bound));
    weight_.jerk_lower_bound.clear();
    weight_.jerk_lower_bound.resize(weight_.point_num, -std::fabs(new_jerk_bound));
    planning_input.set_jerk_bound(std::fabs(new_jerk_bound));
  } else {
    emergency_level_ = NONE;
  }
  // last_jerk_bound_limit_ = planning_input.jerk_bound();
  last_jerk_bound_limit_ =
      planning_input.jerk_bound() / (config_.curv_factor * ref_vel_ * ref_vel_);
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
  std::vector<double> xp_xy{0.2, 0.3, 0.6, 1.2, 2.5};
  std::vector<double> fp_ratio_to_xy1{1.0, 0.9, 0.8, 0.6, 0.5};
  std::vector<double> xp_theta{0.5, 2.0, 5.0};
  std::vector<double> fp_ratio_to_xy2{1.0, 0.5, 0.3};
  double lateral_dist = std::max(std::fabs(avoid_dist_), std::fabs(init_l_ - lat_offset_));
  double q_xy_ratio1 =
    planning::interp(lateral_dist, xp_xy, fp_ratio_to_xy1);
  double q_xy_ratio2 =
    planning::interp(std::fabs(init_ref_theta_error_), xp_theta, fp_ratio_to_xy2);
  planning_input.set_q_ref_x(q_xy_ratio1 * q_xy_ratio2 * q_xy);
  planning_input.set_q_ref_y(q_xy_ratio1 * q_xy_ratio2 * q_xy);

  // std::vector<double> fp_ratio_to_theta1{1.0, 1.1, 1.3, 1.5, 1.8};
  std::vector<double> fp_ratio_to_theta1{1.0, 1.5, 2.0, 2.5, 4.0};
  std::vector<double> fp_ratio_to_theta2{1.0, 1.5, 3.0};
  double q_theta = planning::interp(ref_vel_, xp_v, config_.map_qtheta);
  // double q_theta_ratio1 =
  //   planning::interp(std::fabs(init_l_ - lat_offset_), xp_xy, fp_ratio_to_theta1);
  // double q_theta_ratio2 =
  //   planning::interp(std::fabs(avoid_dist_), xp_xy, fp_ratio_to_theta2);
  double q_theta_ratio1 =
    planning::interp(lateral_dist, xp_xy, fp_ratio_to_theta1);
  double q_theta_ratio2 =
    planning::interp(std::fabs(init_ref_theta_error_), xp_theta, fp_ratio_to_theta2);
  planning_input.set_q_ref_theta(q_theta_ratio1 * q_theta_ratio2 * q_theta);

  double q_jerk1 =
      planning::interp(lateral_dist, xp_xy, config_.map_qjerk1);
  double q_jerk2 =
      planning::interp(lateral_dist, xp_xy, config_.map_qjerk2);
  std::vector<double> fp_ratio_to_jerk{1.0, 2.0, 4.0};
  double q_jerk_ratio1 =
    planning::interp(std::fabs(init_ref_theta_error_), xp_theta, fp_ratio_to_jerk);
  std::vector<double> fp_decay_jerk_ratio{0.3, 0.8, 1.0, 1.0, 1.0};
  double decay_jerk_ratio =
    planning::interp(lateral_dist, xp_xy, fp_decay_jerk_ratio);
  q_jerk_ratio1 = std::max(q_jerk_ratio1 * decay_jerk_ratio, 1.0);
  if (lateral_dist < 0.1) {
    q_jerk2 = q_jerk1;
  }
  concerned_start_q_jerk_ = q_jerk_ratio1 * q_jerk1;
  planning_input.set_q_jerk(q_jerk_ratio1 * q_jerk2);

  if ((std::fabs(init_ref_theta_error_) >= config_.big_theta_thr) &&
      // (std::fabs(init_ref_theta_error_) <= 2.0) &&
      // (std::fabs(init_dis_to_ref_) < 0.4) &&
      (std::fabs(avoid_dist_) < 0.2)) {
    weight_.complete_follow = true;
  }
}

void LateralMotionPlanningWeight::MakeLateralOffsetAvoidDynamicWeight(
    planning::common::LateralPlanningInput &planning_input) {
  std::vector<double> xp_v{2.0, 4.167, 8.333, 20.0};
  std::vector<double> fp_qtheta{3000.0, 5000.0, config_.q_ref_theta_avoid, config_.q_ref_theta_avoid_high_vel};
  double q_ref_theta = planning::interp(ref_vel_, xp_v, fp_qtheta);
  planning_input.set_q_ref_theta(q_ref_theta);

  std::vector<double> fp_qjerk{50.0, 30.0, config_.q_jerk_avoid, config_.q_jerk_avoid_high_vel};
  double q_jerk = planning::interp(ref_vel_, xp_v, fp_qjerk);
  planning_input.set_q_jerk(q_jerk);
  concerned_start_q_jerk_ = q_jerk;
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
  std::vector<double> xp_v{1.5, 3.0, 4.167, 8.333, 16.667};
  std::vector<double> fp_qjerk{500.0, 200.0, 100.0, 20.0, config_.q_jerk_lane_change_back};
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
  if (emergency_level_ == P0) {
    emergence_factor = 2.0 * config_.emergence_avoid_factor;
  } else if (emergency_level_ == P1) {
    emergence_factor = 1.5 * config_.emergence_avoid_factor;
  } else if (emergency_level_ == P2) {
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
      lateral_motion_scene_ != LANE_CHANGE) {
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

  std::vector<double> xp_road_radius{50.0, 150.0, 400.0, 1000.0, 2000.0};
  double valid_perception_range =
      planning::interp(
        target_road_radius_, xp_road_radius, config_.valid_perception_range);
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
  // limit large diff
  if (last_remotely_index_ > weight_.remotely_index + 2) {
    weight_.remotely_index = std::max(last_remotely_index_ - 2, weight_.proximal_index);
  }
  // consider avoid
  double lateral_dist = std::max(std::fabs(avoid_dist_), std::fabs(init_l_ - lat_offset_));
  if (lateral_dist > 0.6) {
    size_t min_remotely_index = 15;
    weight_.remotely_index = std::max(min_remotely_index, weight_.remotely_index);
  }
  // consider intersection and S bend
  if ((lateral_motion_scene_ == RAMP &&
       is_in_intersection_) ||
      is_s_bend_) {
    weight_.remotely_index = 20;
    planning_input.set_q_acc(0.0);
  }

  // const double lateral_offset = lateral_offset_decider_output.lateral_offset;
  if ((lateral_motion_scene_ == LANE_CHANGE) &&
      (!is_lane_change_back_) &&
      (!is_lane_change_hold_)) {
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
  last_remotely_index_ = weight_.remotely_index;
  // set low speed protection
  SetMinJerkWeightByVel(planning_input);
  // set large pos diff protection
  if (last_path_max_dist2ref_ > 10.0) {
    planning_input.set_q_continuity(config_.q_continuity_lane_change);
  }
}

}  // namespace lateral_planning
}  // namespace pnc

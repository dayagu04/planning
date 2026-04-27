#include "sample_speed_adjust_cost.h"

#include <boost/function/function_base.hpp>
#include <cmath>

#include "behavior_planners/lateral_offset_decider/lateral_offset_decider_utils.h"
#include "behavior_planners/sample_poly_speed_adjust_decider/sample_poly_const.h"
#include "task_interface/lane_change_utils.h"

namespace planning {
using planning::speed::STPointWithLateral;

void MatchGapCost::GetCost(
    const STPointWithLateral& upper_st_point, const STPointWithLateral& lower_st_point,
    const double poly_end_s, const double poly_end_t, const double poly_end_v,
    const double poly_end_a,
    const double reliable_safe_distance_to_gap_front_obj,
    const double reliable_safe_distance_to_gap_back_obj,
    const double ego_current_vel, const bool is_merge_change,
    const LanChangeSafetyCheckConfig& lc_safety_distance_config,
    const bool is_emergency_scene, const double extreme_time_back) {
  // Helper function to calculate the cost for distance and velocity
  const double ttc_safe_limit = is_merge_change ? 0.0 : 0.0;
  const double gap_vel_gain = is_merge_change ? 1.0 : 1.0;
  is_gap_changeable_ = true;
  auto& xp = lc_safety_distance_config.rear_vehicle_speed_min_space_map
                 .rear_speed_kph_table;  // 后车速度kph
  auto& fp = lc_safety_distance_config.rear_vehicle_speed_min_space_map
                 .min_space_table;  //触发变道需要预留最小空间 下方
                                    //大车额外增加5m基础距离
  auto& xpv = lc_safety_distance_config.diff_speed_init_ttc_map
                  .diff_kph_table;  // 后车 - 自车速度 kph
  auto& fpv = is_emergency_scene?
      lc_safety_distance_config.diff_speed_init_ttc_map.aggressive_ttc_table:
      lc_safety_distance_config.diff_speed_init_ttc_map.ttc_table;  //起始ttc
  auto calculate_gap_distance_match_cost =
      [](double dist_to_obj, double safe_border_distance,
         double clip_border_distance, double safe_dis_penalty_factor_coef,
         double clip_dis_penalty_factor_coef, double weight) {
        return (dist_to_obj < safe_border_distance)
                   ? (dist_to_obj > clip_border_distance)
                         ? weight *
                               std::exp(safe_dis_penalty_factor_coef *
                                        (safe_border_distance - dist_to_obj) /
                                        safe_border_distance)
                         : weight * std::exp(clip_dis_penalty_factor_coef *
                                             (safe_border_distance +
                                              clip_border_distance) /
                                             safe_border_distance)
                   : 0.0;
      };

  auto calculate_gap_vel_match_cost =
      [ttc_safe_limit, gap_vel_gain](
          double gap_front_car_vel, double gap_rear_car_vel,
          double distance_to_gap_front_car, double distance_to_gap_rear_car,
          double sample_poly_end_v, double gap_vel_penalty_threshold,
          double rel_vel_penalty_factor_coef, double weight) {
        double vel_diff_to_gap_front_car =
            std::fmax(sample_poly_end_v - gap_front_car_vel, kZeroEpsilon);
        double vel_diff_to_gap_rear_car =
            std::fmax(gap_rear_car_vel - sample_poly_end_v, kZeroEpsilon);
        const double ttc_to_front_car =
            distance_to_gap_front_car / vel_diff_to_gap_front_car;
        const double ttc_to_rear_car =
            distance_to_gap_rear_car / vel_diff_to_gap_rear_car;

        double cost = 0.0;
        if (ttc_to_front_car > 0 && ttc_to_front_car < ttc_safe_limit) {
          cost = vel_diff_to_gap_front_car > 0
                     ? weight * std::exp(rel_vel_penalty_factor_coef *
                                         std::fmin(vel_diff_to_gap_front_car,
                                                   gap_vel_penalty_threshold) /
                                         gap_vel_gain)
                     : 0.0;
        }

        if (ttc_to_rear_car > 0 && ttc_to_rear_car < ttc_safe_limit) {
          cost = std::fmax(
              cost,
              vel_diff_to_gap_rear_car > 0
                  ? weight * std::exp(rel_vel_penalty_factor_coef *
                                      std::fmin(vel_diff_to_gap_rear_car,
                                                gap_vel_penalty_threshold) /
                                      gap_vel_gain)
                  : 0.0);
        }
        return cost;
      };

  auto calculate_narrow_gap_center_attract_cost =
      [](double dist_to_gap_center, double attract_border_distance,
         double clip_border_distance, double narrow_gap_attract_factor_coef,
         double weight) {
        return (dist_to_gap_center > attract_border_distance)
                   ? weight * std::exp(narrow_gap_attract_factor_coef *
                                       (attract_border_distance -
                                        clip_border_distance) /
                                       attract_border_distance)
                   : dist_to_gap_center > clip_border_distance
                         ? weight * std::exp(narrow_gap_attract_factor_coef *
                                             (dist_to_gap_center -
                                              clip_border_distance) /
                                             attract_border_distance)
                         : 0.0;
      };

  auto linear_expand_extra_gap_distance_by_ego_vel =
      [](double ego_v, double ego_v_max, double ego_v_min, double extra_dis_min,
         double extra_dis_max) {
        return extra_dis_min + (extra_dis_max - extra_dis_min) *
                                   (ego_v - ego_v_min) /
                                   (ego_v_max - ego_v_min + kZeroEpsilon);
      };
  match_s_cost_ = 0.0;
  match_v_cost_ = 0.0;
  match_gap_center_cost_ = 0.0;
  cost_ = 0.0;

  double safe_border_distance_to_gap_front_obj = 0.0;
  double min_safe_distance_front = 3.8;
  double safe_border_distance_to_gap_back_obj = 0.0;
  double min_safe_distance_rear = 0.0;
  if (upper_st_point.agent_id() != kNoAgentId) {
    if (upper_st_point.velocity() < poly_end_v) {
      double rel_v = poly_end_v - upper_st_point.velocity();
      double front_ttc_buffer = (poly_end_v * rel_v) / (2.0 * 2.5);
      min_safe_distance_front = std::max(front_ttc_buffer, 3.5);
    }
    min_safe_distance_front = std::max(min_safe_distance_front, poly_end_v * 0.3);
    bool is_large_car = agent::AgentType::BUS == upper_st_point.type() ||
                        agent::AgentType::TRUCK == upper_st_point.type() ||
                        agent::AgentType::TRAILER == upper_st_point.type() ||
                        upper_st_point.vehicle_length() > kLargeAgentLengthM;
    double large_car_buffer = is_large_car ? 3.0 : 0.0;
    double front_buffer_redundancy =
        std::fmax(0.5 - reliable_safe_distance_to_gap_front_obj, 0.0);
    safe_border_distance_to_gap_front_obj =
        min_safe_distance_front + reliable_safe_distance_to_gap_front_obj +
        large_car_buffer + front_buffer_redundancy;
  }

  if (lower_st_point.agent_id() != kNoAgentId) {
    double rel_vel = lower_st_point.velocity() - poly_end_v;
    double abs_buffer = interp(lower_st_point.velocity() * 3.6, xp, fp);
    double dist_rel_vel =
        (rel_vel > 0) ? rel_vel * interp(rel_vel * 3.6, xpv, fpv) : 0.0;
    min_safe_distance_rear = std::fmax(abs_buffer, dist_rel_vel);
    min_safe_distance_rear = std::fmax(min_safe_distance_rear, 0.1) +
                             linear_expand_extra_gap_distance_by_ego_vel(
                                 poly_end_v, kEgoVelMax, kEgoVelMin, 0.0, 2.0);
    bool is_large_car = agent::AgentType::BUS == lower_st_point.type() ||
                        agent::AgentType::TRUCK == lower_st_point.type() ||
                        agent::AgentType::TRAILER == lower_st_point.type() ||
                        lower_st_point.vehicle_length() > kLargeAgentLengthM;
    double large_car_buffer = is_large_car ? 5.0 : 0.0;
    double coffi_index = extreme_time_back / 0.2;
    safe_border_distance_to_gap_back_obj =
        lower_st_point.acceleration() > poly_end_a
            ? reliable_safe_distance_to_gap_back_obj + min_safe_distance_rear +
                  large_car_buffer
            : std::max(reliable_safe_distance_to_gap_back_obj +
                           min_safe_distance_rear * std::pow(0.9, coffi_index),
                       min_safe_distance_rear) +
                  large_car_buffer;
  }
  safe_border_distance_to_gap_front_obj_ =
      safe_border_distance_to_gap_front_obj;
  safe_border_distance_to_gap_back_obj_ = safe_border_distance_to_gap_back_obj;
  // Case 1: Both upper and lower points are not present
  if (lower_st_point.agent_id() == kNoAgentId &&
      upper_st_point.agent_id() == kNoAgentId) {
    cost_ = 0.0;
    return;
  }

  // Case 2: Only the upper point is present (lower is absent)
  if (lower_st_point.agent_id() == kNoAgentId &&
      upper_st_point.agent_id() != kNoAgentId) {
    double dist_to_upper_border =
        upper_st_point.s() - (poly_end_s + front_edge_to_rear_axle_);
    match_s_cost_ = calculate_gap_distance_match_cost(
        dist_to_upper_border, safe_border_distance_to_gap_front_obj,
        reliable_safe_distance_to_gap_front_obj, safe_dis_penalty_factor_coef_,
        clip_dis_penalty_factor_coef_, weight_match_s_);

    cost_ += match_s_cost_;

    match_v_cost_ = calculate_gap_vel_match_cost(
        upper_st_point.velocity(), -kAgentNoValidVel, dist_to_upper_border,
        kAgentNoValidDis, poly_end_v, kMatchGapVelPenaltyThreshold,
        rel_vel_penalty_factor_coef_, weight_match_v_);
    cost_ += match_v_cost_;

    match_gap_center_cost_ = calculate_narrow_gap_center_attract_cost(
        (safe_border_distance_to_gap_front_obj - dist_to_upper_border),
        safe_border_distance_to_gap_front_obj, 0.0,
        narrow_gap_penalty_factor_coef_, weight_match_s_);
    cost_ += match_gap_center_cost_;
    return;
  }

  // Case 3: Only the lower point is present (upper is absent)
  if (lower_st_point.agent_id() != kNoAgentId &&
      upper_st_point.agent_id() == kNoAgentId) {
    double dist_to_lower_border =
        poly_end_s - rear_edge_to_rear_axle_ - lower_st_point.s();
    match_s_cost_ = calculate_gap_distance_match_cost(
        dist_to_lower_border, safe_border_distance_to_gap_back_obj,
        reliable_safe_distance_to_gap_back_obj, safe_dis_penalty_factor_coef_,
        clip_dis_penalty_factor_coef_, weight_match_s_);
    cost_ += match_s_cost_;

    match_v_cost_ = calculate_gap_vel_match_cost(
        kAgentNoValidVel, lower_st_point.velocity(), kAgentNoValidDis,
        dist_to_lower_border, poly_end_v, kMatchGapVelPenaltyThreshold,
        rel_vel_penalty_factor_coef_, weight_match_v_);
    cost_ += match_v_cost_;

    match_gap_center_cost_ = calculate_narrow_gap_center_attract_cost(
        (safe_border_distance_to_gap_back_obj - dist_to_lower_border),
        safe_border_distance_to_gap_back_obj, 0.0,
        narrow_gap_penalty_factor_coef_, weight_match_s_);
    cost_ += match_gap_center_cost_;
    return;
  }

  //  Case 4: Both upper and lower points are present
  if (lower_st_point.agent_id() != kNoAgentId &&
      upper_st_point.agent_id() != kNoAgentId &&
      lower_st_point.agent_id() != upper_st_point.agent_id()) {
    double dist_to_lower_border =
        poly_end_s - rear_edge_to_rear_axle_ - lower_st_point.s();
    double dist_to_upper_border =
        upper_st_point.s() - (poly_end_s + front_edge_to_rear_axle_);
    double dist_to_lower_cost = calculate_gap_distance_match_cost(
        dist_to_lower_border, safe_border_distance_to_gap_back_obj,
        reliable_safe_distance_to_gap_back_obj, safe_dis_penalty_factor_coef_,
        clip_dis_penalty_factor_coef_, weight_match_s_);
    double dist_to_upper_cost = calculate_gap_distance_match_cost(
        dist_to_upper_border, safe_border_distance_to_gap_front_obj,
        reliable_safe_distance_to_gap_front_obj, safe_dis_penalty_factor_coef_,
        clip_dis_penalty_factor_coef_, weight_match_s_);
    double safe_gap_center =
        (upper_st_point.s() - safe_border_distance_to_gap_front_obj -
         front_edge_to_rear_axle_ + rear_edge_to_rear_axle_ +
         lower_st_point.s() + safe_border_distance_to_gap_back_obj) /
        2.0;
    double left_changeable_gap = upper_st_point.s() - lower_st_point.s() -
                                 safe_border_distance_to_gap_back_obj -
                                 safe_border_distance_to_gap_front_obj -
                                 front_edge_to_rear_axle_ -
                                 rear_edge_to_rear_axle_;
    double min_gap_center_distance = left_changeable_gap / 2.0;
    if (left_changeable_gap >= 0) {
      // extra match s center cost
      match_s_cost_ = std::fmax(dist_to_lower_cost, dist_to_upper_cost);
      match_gap_center_cost_ = calculate_narrow_gap_center_attract_cost(
          std::fabs(safe_gap_center - poly_end_s),
          (upper_st_point.s() + lower_st_point.s()) * 0.5,
          min_gap_center_distance, narrow_gap_penalty_factor_coef_,
          weight_match_s_);
    } else {
      double proportion_gap_center_distance =
          std::fabs(0.5 * upper_st_point.s() + 0.5 * lower_st_point.s() -
                    poly_end_s) /
          (upper_st_point.s() - lower_st_point.s());
      proportion_gap_center_distance =
          std::fmin(proportion_gap_center_distance, 0.5);
      match_gap_center_cost_ = calculate_narrow_gap_center_attract_cost(
          kBasicSafeDistance + 1, kBasicSafeDistance,
          -(0.5 - proportion_gap_center_distance) * kBasicSafeDistance,
          narrow_gap_penalty_factor_coef_, weight_match_s_);
      match_s_cost_ = std::fmax(dist_to_lower_cost, dist_to_upper_cost);
      match_s_cost_ =
          std::fmax(match_s_cost_,
                    weight_match_s_ * std::exp(clip_dis_penalty_factor_coef_));
      is_gap_changeable_ = false;
      // match_s_cost_ = weight_match_s_ * std::exp()
    }
    match_v_cost_ = calculate_gap_vel_match_cost(
        upper_st_point.velocity(), lower_st_point.velocity(),
        dist_to_upper_border, dist_to_lower_border, poly_end_v,
        kMatchGapVelPenaltyThreshold, rel_vel_penalty_factor_coef_,
        weight_match_v_);
    // Final cost is the max between distances and velocities
    cost_ = match_v_cost_ + match_s_cost_ + match_gap_center_cost_;
    return;
  }

  // Default penalty when no valid agent data
  // match_s_cost_ = kMaxPenalty;
  if (is_merge_change) {
    auto half_object_vehicle_length =
        (upper_st_point.s() - lower_st_point.s()) * 0.5;
    auto compare_point = (upper_st_point.s() + lower_st_point.s()) * 0.5;
    auto ego_center_point =
        (front_edge_to_rear_axle_ - rear_edge_to_rear_axle_) * 0.5 + poly_end_s;
    auto half_ego_length =
        (front_edge_to_rear_axle_ + rear_edge_to_rear_axle_) * 0.5;
    auto deviation_ratio = std::fabs(ego_center_point - compare_point) /
                           (half_object_vehicle_length + half_ego_length);
    if (ego_center_point >= compare_point) {
      cost_ = std::exp((1.0 - deviation_ratio) * acc_speed_weight_ + 1.0);
    } else {
      cost_ = std::exp((1.0 - deviation_ratio) * dec_speed_weight_ + 1.0);
    }
  } else {
    match_s_cost_ = calculate_gap_distance_match_cost(
        kMinSafeDistanceFront, safe_border_distance_to_gap_front_obj,
        kMinSafeDistanceFront, safe_dis_penalty_factor_coef_,
        clip_dis_penalty_factor_coef_, weight_match_s_);
    match_v_cost_ = 0.0;
    match_gap_center_cost_ = 0.0;
    cost_ = match_s_cost_ + match_v_cost_ + match_gap_center_cost_;
  }
}

void FollowVelCost::GetCost(const double poly_end_v, const double cruise_v,
                            const double follow_vel_penalty_benchmark) {
  cost_ = weight_ * std::exp(std::fabs(poly_end_v - cruise_v) / cruise_v);
}

void StopLineCost::GetCost(const double stop_line_dis_to_ego,
                           const double poly_end_s_dis_to_ego,
                           const double v_curve_final,
                           const bool is_merge_request) {
  const double half_lane_chnage_duration = 3.0;
  const double half_lane_change_length =
      is_merge_request ? 0.0 : half_lane_chnage_duration * v_curve_final;
  double poly_end_dis_to_virtual_stop_line =
      stop_line_dis_to_ego - half_lane_change_length - poly_end_s_dis_to_ego;

  // 以舒适减速度从v_final减速到0所需距离
  // merge场景容许稍大减速（紧迫性更高），non-merge更保守
  const double a_dec = 0.5;  // m/s²
  const double d_stop_needed =
      is_merge_request ? (v_curve_final * v_curve_final) / (2.0 * a_dec) : 0.0;

  // 安全裕量：>0 来得及减速，<0 减速不及
  // 额外预留安全缓冲（地图误差补偿）
  const double safety_buffer = is_merge_request ? 5.0 : 10.0;  // m
  const double safety_margin =
      poly_end_dis_to_virtual_stop_line - d_stop_needed - safety_buffer;

  double adaptive_penalty = std::log(kStopLineBasicPenaltyDis);
  double distance_penalty_factor =
      poly_end_dis_to_virtual_stop_line / kStopLineBasicPenaltyDis;
  constexpr double eps_floor = 1e-15;
  distance_penalty_factor = std::max(distance_penalty_factor, eps_floor);
  //   constexpr double eps_floor = 1e-15;
  //   distance_penalty_factor = std::max(distance_penalty_factor, eps_floor);
  if (!is_merge_request) {
    if (poly_end_dis_to_virtual_stop_line > kStopLineBasicPenaltyDis_merge) {
      cost_ = 0.0;
    } else if (poly_end_dis_to_virtual_stop_line >
               kStopLineNormalPenaltyDis_merge) {
      cost_ = weight_ * std::exp(mid_stop_dis_penalty_coef_ *
                                 (kStopLineBasicPenaltyDis_merge -
                                  poly_end_dis_to_virtual_stop_line) /
                                 600);
    } else if (poly_end_dis_to_virtual_stop_line > 0) {
      cost_ = weight_ * std::exp(near_stop_dis_penalty_coef_ *
                                 (kStopLineBasicPenaltyDis_merge -
                                  poly_end_dis_to_virtual_stop_line) /
                                 600);
    } else {
      cost_ = weight_ * std::exp(near_stop_dis_penalty_coef_ *
                                 (kStopLineBasicPenaltyDis_merge -
                                  poly_end_dis_to_virtual_stop_line) /
                                 500);
    }
  } else {
    if (safety_margin > 0.0) {
      cost_ = behind_stop_dis_penalty_coef_ * pow(safety_margin / 10.0, 2.0);
    } else if(safety_margin < -safety_buffer){
      cost_ = weight_ * over_stop_dis_penalty_coef_ *
              pow(safety_margin / 10.0, 2.0);
    }
  }
}

void LeadingVehSafeCost::GetCost(const double poly_end_s,
                                 const double poly_end_v,
                                 const double leading_veh_pred_s,
                                 const double leading_veh_v) {
  cost_ = 0.0;
  auto calculate_poly_dis_to_lead_cost = [this](double dist, double safe_distance,
                                            double weight) {
    return (dist < leading_safe_distance_gain_ * safe_distance)
               ? (dist < safe_distance)
                     ? weight * std::exp(leading_safe_overstep_gain_ * (safe_distance - dist) /
                                         safe_distance + leading_safe_overstep_buffer_)
                     : std::exp((safe_distance - dist) /
                                         safe_distance)
               : 0.0;
  };
  double delta_v = poly_end_v - leading_veh_v;
  double follow_distance = poly_end_v * delta_v / (2.0 * leading_safe_max_dec_);
  double thw = leading_safe_delay_time_ * poly_end_v;
  double min_follow_distance = 3.0;
  double safe_distance = std::fmax(min_follow_distance + thw + follow_distance, min_follow_distance);
  cost_ = calculate_poly_dis_to_lead_cost(leading_veh_pred_s - poly_end_s,
                                          safe_distance, weight_);
}

void SpeedVariableCost::GetCost(const double vel_integral) {
  cost_ =
      weight_ * std::exp(std::fabs(vel_integral * kMaxVelVariableValueInverse));
}

void GapAvaliableCost::GetCost(const STPointWithLateral& upper_st_point,
                               const STPointWithLateral& lower_st_point,
                               const double safe_gap_front_obj,
                               const double safe_gap_back_obj,
                               const double ego_s) {
  double gap_length = upper_st_point.s() - lower_st_point.s();
  double total_safe_distance = safe_gap_front_obj + safe_gap_back_obj;
  double left_distance = gap_length - total_safe_distance;
  if(left_distance < 0.0) {
    cost_ = weight_ * std::exp(1.0);
  }else{
    double gap_center = ((upper_st_point.s() - safe_gap_front_obj) +
                         (lower_st_point.s() + safe_gap_back_obj)) /
                        2.0;
    cost_ = weight_ * std::exp(std::min(std::abs(ego_s - gap_center) /
                                        std::max(left_distance, kZeroEpsilon),
                               1.0));
  }
}

void StopPenaltyCost::GetCost(const double end_v) {
  cost_ = end_v > kPenaltyVel ? 0.0 : weight_ * kMaxPenalty;
}

void AccLimitCost::GetCost(const double acc_extrema) {
  cost_ = acc_extrema > kAccPenaltyLimit
              ? weight_ * std::exp(kAccPenaltyScaleFactor *
                                   (acc_extrema - kAccPenaltyLimit))
              : 0.0;
}

void SpeedChangeCost::GetCost(const double end_v, const double ego_v,
                              const double end_t) {
  double average_vel_differ = (end_v - ego_v) / end_t;
  cost_ = average_vel_differ > 0
              ? 0.0
              : weight_ * average_vel_differ * average_vel_differ;
}

void StopPointCost::GetCost(const double distance_to_stop_point) {
  cost_ = distance_to_stop_point > 0.0
              ? 0.0
              : weight_ * std::pow((-distance_to_stop_point + 10.0), 2.0);
}

void LeadingVehFollowCost::GetCost(const double leading_veh_pred_s,
                                   const double ego_v,
                                   const double ego_pred_s) {
  double follow_dis = leading_veh_pred_s - (ego_pred_s);
  double thw = follow_dis / std::fmax(ego_v, kZeroEpsilon);
  cost_ = thw > 1.5 ? thw > 11.5 ? weight_ * std::exp(1.0)
                                 : weight_ * std::exp((thw - 1.5) / 10.0)
                    : 0.0;
}

void JerkLimitCost::GetCost(const double jerk_extrema) {
  cost_ =
      jerk_extrema > 1.5 ? weight_ * std::exp((jerk_extrema - 1.5) / 2.0) : 0.0;
}

void SafeDistanceCost::GetCost(const double distance_to_gap_front_obj,
                               const double distance_to_gap_back_obj,
                               const double limi_safe_distance_to_gap_front_obj,
                               const double limi_safe_distance_to_gap_back_obj,
                               const double max_safe_distance_to_gap_front_obj,
                               const double max_safe_distance_to_gap_back_obj) {
  auto front_obj_cost =
      distance_to_gap_front_obj > max_safe_distance_to_gap_front_obj
          ? 0.0
          : std::exp((max_safe_distance_to_gap_front_obj -
                      distance_to_gap_front_obj) /
                     (max_safe_distance_to_gap_front_obj -
                      limi_safe_distance_to_gap_front_obj));
  auto back_obj_cost =
      distance_to_gap_back_obj > max_safe_distance_to_gap_back_obj
          ? 0.0
          : std::exp(
                (max_safe_distance_to_gap_back_obj - distance_to_gap_back_obj) /
                (max_safe_distance_to_gap_back_obj -
                 limi_safe_distance_to_gap_back_obj));
  cost_ = weight_ * std::max(front_obj_cost, back_obj_cost);
}
}  // namespace planning
#include "sample_speed_adjust_cost.h"

#include <boost/function/function_base.hpp>
#include <cmath>

#include "behavior_planners/lateral_offset_decider/lateral_offset_decider_utils.h"
#include "behavior_planners/sample_poly_speed_adjust_decider/sample_poly_const.h"
#include "task_interface/lane_change_utils.h"

namespace planning {

void MatchGapCost::GetCost(
    const STPoint& upper_st_point, const STPoint& lower_st_point,
    const double poly_end_s, const double poly_end_t, const double poly_end_v,
    const double reliable_safe_distance_to_gap_front_obj,
    const double reliable_safe_distance_to_gap_back_obj) {
  // Helper function to calculate the cost for distance and velocity
  auto calculate_gap_distance_match_cost = [](double dist_to_obj,
                                              double safe_border_distance,
                                              double clip_border_distance,
                                              double weight) {
    return (dist_to_obj < safe_border_distance)
               ? (dist_to_obj > clip_border_distance)
                     ? weight * std::exp((safe_border_distance - dist_to_obj) /
                                         safe_border_distance)
                     : weight * std::exp((safe_border_distance -
                                          clip_border_distance) /
                                         safe_border_distance)
               : 0.0;
  };

  auto calculate_gap_vel_match_cost =
      [](double gap_front_car_vel, double gap_rear_car_vel,
         double distance_to_gap_front_car, double distance_to_gap_rear_car,
         double sample_poly_end_v, double gap_vel_penalty_threshold,
         double weight) {
        double vel_diff_to_gap_front_car =
            std::fmax(sample_poly_end_v - gap_front_car_vel, kZeroEpsilon);
        double vel_diff_to_gap_rear_car =
            std::fmax(gap_rear_car_vel - sample_poly_end_v, kZeroEpsilon);
        const double ttc_to_front_car =
            distance_to_gap_front_car / vel_diff_to_gap_front_car;
        const double ttc_to_rear_car =
            distance_to_gap_rear_car / vel_diff_to_gap_rear_car;

        double cost = 0.0;
        if (ttc_to_front_car > 0 && ttc_to_front_car < 2.0) {
          cost = vel_diff_to_gap_front_car > 0
                     ? weight * std::exp(std::fmin(vel_diff_to_gap_front_car,
                                                   gap_vel_penalty_threshold))
                     : 0.0;
        }

        if (ttc_to_rear_car > 0 && ttc_to_rear_car < 2.0) {
          cost = std::fmax(cost, vel_diff_to_gap_rear_car > 0
                                     ? weight * std::exp(std::fmin(
                                                    vel_diff_to_gap_rear_car,
                                                    gap_vel_penalty_threshold))
                                     : 0.0);
        }
        return cost;
      };

  auto calculate_narrow_gap_center_attract_cost =
      [](double dist_to_gap_center, double attract_border_distance,
         double clip_border_distance, double weight) {
        return (dist_to_gap_center > attract_border_distance)
                   ? weight * std::exp((attract_border_distance -
                                        clip_border_distance) /
                                       attract_border_distance)
               : dist_to_gap_center > clip_border_distance
                   ? weight *
                         std::exp((dist_to_gap_center - clip_border_distance) /
                                  attract_border_distance)
                   : 0.0;
      };

  match_s_cost_ = 0.0;
  match_v_cost_ = 0.0;
  match_gap_center_cost_ = 0.0;
  cost_ = 0.0;

  const double safe_border_distance_to_gap_front_obj =
      std::fmax(kBasicSafeDistance, reliable_safe_distance_to_gap_front_obj);
  const double safe_border_distance_to_gap_back_obj =
      std::fmax(kBasicSafeDistance, reliable_safe_distance_to_gap_back_obj);

  // Case 1: Both upper and lower points are not present
  if (lower_st_point.agent_id() == kNoAgentId &&
      upper_st_point.agent_id() == kNoAgentId) {
    cost_ = 0.0;
    return;
  }

  // Case 2: Only the upper point is present (lower is absent)
  if (lower_st_point.agent_id() == kNoAgentId &&
      upper_st_point.agent_id() != kNoAgentId) {
    double dist_to_upper_border = upper_st_point.s() - poly_end_s;
    match_s_cost_ = calculate_gap_distance_match_cost(
        dist_to_upper_border, safe_border_distance_to_gap_front_obj,
        kMinSafeDistance, weight_match_s_);

    cost_ += match_s_cost_;

    match_v_cost_ = calculate_gap_vel_match_cost(
        upper_st_point.velocity(), -kAgentNoValidVel, dist_to_upper_border,
        kAgentNoValidDis, poly_end_v, kMatchGapVelPenaltyThreshold,
        weight_match_v_);
    cost_ += match_v_cost_;

    return;
  }

  // Case 3: Only the lower point is present (upper is absent)
  if (lower_st_point.agent_id() != kNoAgentId &&
      upper_st_point.agent_id() == kNoAgentId) {
    double dist_to_lower_border = poly_end_s - lower_st_point.s();
    match_s_cost_ = calculate_gap_distance_match_cost(
        dist_to_lower_border, safe_border_distance_to_gap_back_obj,
        kMinSafeDistance, weight_match_s_);

    cost_ += match_s_cost_;

    match_v_cost_ = calculate_gap_vel_match_cost(
        kAgentNoValidVel, lower_st_point.velocity(), kAgentNoValidDis,
        dist_to_lower_border, poly_end_v, kMatchGapVelPenaltyThreshold,
        weight_match_v_);
    cost_ += match_v_cost_;
    return;
  }

  // Case 4: Both upper and lower points are present

  if (lower_st_point.agent_id() != kNoAgentId &&
      upper_st_point.agent_id() != kNoAgentId) {
    double dist_to_lower_border = poly_end_s - lower_st_point.s();
    double dist_to_upper_border = upper_st_point.s() - poly_end_s;

    double dist_cost = 0.0;
    double dist_to_lower_cost = calculate_gap_distance_match_cost(
        dist_to_lower_border, safe_border_distance_to_gap_back_obj,
        kMinSafeDistance, weight_match_s_);
    double dist_to_upper_cost = calculate_gap_distance_match_cost(
        dist_to_upper_border, safe_border_distance_to_gap_front_obj,
        kMinSafeDistance, weight_match_s_);
    match_s_cost_ = std::fmax(dist_to_lower_cost, dist_to_upper_cost);

    if (dist_to_lower_border < kBasicSafeDistance &&
        dist_to_upper_border < kBasicSafeDistance) {
      // extra match s center cost
      match_gap_center_cost_ = calculate_narrow_gap_center_attract_cost(
          std::fabs((upper_st_point.s() + lower_st_point.s()) * 0.5 -
                    poly_end_s),
          kBasicSafeDistance, kMinSafeDistance, weight_match_s_);
    }
    match_v_cost_ = calculate_gap_vel_match_cost(
        upper_st_point.velocity(), lower_st_point.velocity(),
        dist_to_upper_border, dist_to_lower_border, poly_end_v,
        kMatchGapVelPenaltyThreshold, weight_match_v_);

    // Final cost is the max between distances and velocities
    cost_ = match_v_cost_ + match_s_cost_ + match_gap_center_cost_;
    return;
  }

  // Default penalty when no valid agent data
  cost_ = kMaxPenalty;
}

void FollowVelCost::GetCost(const double poly_end_v, const double cruise_v,
                            const double follow_vel_penalty_benchmark) {
  cost_ = weight_ *
          std::exp(std::fabs(poly_end_v - cruise_v) / kFollowSpeedBenchmark);
}

void StopLineCost::GetCost(const double stop_line_dis_to_ego,
                           const double poly_end_s_dis_to_ego) {
  double poly_end_dis_to_stop_line =
      stop_line_dis_to_ego - poly_end_s_dis_to_ego;

  if (poly_end_dis_to_stop_line > kStopLineBasicPenaltyDis) {
    cost_ = 0.0;
  } else if (poly_end_dis_to_stop_line > kStopLineNormalPenaltyDis) {
    cost_ = weight_ *
            std::exp(-(poly_end_dis_to_stop_line + kStopLineNormalPenaltyDis) /
                     kStopLineBasicPenaltyDis);

  } else if (poly_end_dis_to_stop_line > 0) {
    cost_ = weight_ *
            std::exp(-(poly_end_dis_to_stop_line) / kStopLineNormalPenaltyDis);

  } else {
    cost_ = weight_ * std::exp(0);
  }
}

void LeadingVehSafeCost::GetCost(const double poly_end_s,
                                 const double poly_end_v,
                                 const double leading_veh_pred_s,
                                 const double leading_veh_v) {
  cost_ = 0.0;
  auto calculate_poly_dis_to_lead_cost = [](double dist, double safe_distance,
                                            double weight) {
    return (dist < safe_distance)
               ? weight * std::exp((safe_distance - dist) / safe_distance)
               : 0.0;
  };

  if (poly_end_v > leading_veh_v) {
    cost_ += calculate_poly_dis_to_lead_cost(leading_veh_pred_s - poly_end_s,
                                             kBasicSafeDistance, weight_);
  }
}

void SpeedVariableCost::GetCost(const double vel_integral) {
  cost_ =
      weight_ * std::exp(std::fabs(vel_integral * kMaxVelVariableValueInverse));
}

void GapAvaliableCost::GetCost(const double future_gap_length,
                               const double gap_length) {
  double future_gap_cost =
      weight_ * (future_gap_length > 2 * kBasicSafeDistance ? 0.0
                 : future_gap_length > kBasicSafeDistance
                     ? std::exp(1 - (future_gap_length - kBasicSafeDistance) /
                                        kBasicSafeDistance)
                     : std::exp(1));
  double gap_cost =
      weight_ * (gap_length > 2 * kBasicSafeDistance ? 0.0
                 : gap_length > kBasicSafeDistance
                     ? std::exp(1 - (gap_length - kBasicSafeDistance) /
                                        kBasicSafeDistance)
                     : std::exp(1));
  cost_ = std::fmax(future_gap_cost, gap_cost);
}

void StopPenaltyCost::GetCost(const double end_v) {
  cost_ = end_v > kPenaltyVel ? 0.0 : kMaxPenalty;
}
}  // namespace planning
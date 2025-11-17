#include "sample_poly_curve.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "behavior_planners/sample_poly_speed_adjust_decider/sample_poly_const.h"
#include "behavior_planners/sample_poly_speed_adjust_decider/sample_speed_adjust_cost.h"
#include "st_graph/st_point.h"
#include "task_interface/lane_change_utils.h"
namespace planning {

SampleQuarticPolynomialCurve::SampleQuarticPolynomialCurve(
    QuarticPolynomial& poly, double arrived_t, double mid_t,
    const double weight_match_gap_vel, const double weight_match_gap_s,
    const double weight_follow_vel, const double weight_stop_line,
    const double weight_leading_veh_safe_s, const double weight_speed_variable,
    const double weight_gap_avaliable, const double weight_acc_limit,
    const double weight_stop_penalty, const double weight_speed_change,
    const double weight_leading_veh_follow_s,
    const double front_edge_to_rear_axle, const double back_edge_to_rear_axle) {
  poly_ = poly;
  arrived_t_ = arrived_t;
  arrived_v_ = poly_.CalculateFirstDerivative(poly_.T());
  arrived_s_ = arrived_t_ - poly_.T() > 0
                   ? poly_.CalculatePoint(poly_.T()) +
                         arrived_v_ * (arrived_t_ - poly_.T())
                   : poly_.CalculatePoint(arrived_t_);

  mid_v_ = poly_.CalculateFirstDerivative(mid_t);
  mid_s_ = mid_t - poly_.T() > 0 ? poly_.CalculateSecondDerivative(poly_.T()) +
                                       arrived_v_ * (mid_t - poly_.T())
                                 : poly_.CalculatePoint(mid_t);
  mid_t_ = mid_t;
  cost_sum_ = std::numeric_limits<double>::max();
  front_edge_to_rear_axle_ = front_edge_to_rear_axle;
  back_edge_to_rear_axle_ = back_edge_to_rear_axle;

  follow_vel_cost_.SetWeight(weight_follow_vel);
  stop_line_cost_.SetWeight(weight_stop_line);
  leading_veh_safe_cost_.SetWeight(weight_leading_veh_safe_s);
  leading_veh_safe_cost_.SetRearAxleToBumpDis(front_edge_to_rear_axle);
  speed_variable_cost_.SetWeight(weight_speed_variable);
  gap_avaliable_cost_.SetWeight(weight_gap_avaliable);
  acc_limit_cost_.SetWeight(weight_acc_limit);
  stop_penalty_cost_.SetWeight(weight_stop_penalty);
  speed_change_cost_.SetWeight(weight_speed_change);
  anchor_points_match_gap_cost_.SetWeightMatchVel(weight_match_gap_vel);
  anchor_points_match_gap_cost_.SetWeightMatchS(weight_match_gap_s);
  anchor_points_match_gap_cost_.SetRearAxleToBumpDis(front_edge_to_rear_axle,
                                                     back_edge_to_rear_axle);
  leading_veh_follow_s_cost_.SetWeight(weight_leading_veh_follow_s);
  leading_veh_follow_s_cost_.SetRearAxleToBumpDis(front_edge_to_rear_axle);
};

void SampleQuarticPolynomialCurve::CostInit() {
  follow_vel_cost_.Init();
  stop_line_cost_.Init();
  leading_veh_safe_cost_.Init();
  speed_variable_cost_.Init();
  gap_avaliable_cost_.Init();
  acc_limit_cost_.Init();
  stop_penalty_cost_.Init();
  speed_change_cost_.Init();
  anchor_points_match_gap_cost_.Init();
  cost_sum_ = 0.0;
}
double SampleQuarticPolynomialCurve::CalcS(const double t) const {
  return t - poly_.T() >= 0
             ? poly_.CalculatePoint(poly_.T()) + arrived_v_ * (t - poly_.T())
             : poly_.CalculatePoint(t);
}
double SampleQuarticPolynomialCurve::CalcV(const double t) const {
  return t - poly_.T() >= 0 ? arrived_v_ : poly_.CalculateFirstDerivative(t);
}
double SampleQuarticPolynomialCurve::CalcAcc(const double t) const {
  return t - poly_.T() >= 0 ? 0.0 : poly_.CalculateSecondDerivative(t);
}

double SampleQuarticPolynomialCurve::CalcJerk(const double t) const {
  return t - poly_.T() >= 0 ? 0.0 : poly_.CalculateThirdDerivative(t);
}

double SampleQuarticPolynomialCurve::CalcVelIntegral(
    const double evaluate_t) const {
  const double t = evaluate_t > poly_.T() ? poly_.T() : evaluate_t;
  const double t_2 = t * t;
  const double t_3 = t_2 * t;
  const double t_4 = t_3 * t;
  const auto& coeff = poly_.coefficients();
  return coeff[4] * t_4 + coeff[3] * t_3 + coeff[2] * t_2;
}

double SampleQuarticPolynomialCurve::CalcGapVelSafeDistance(const double ego_v,
                                                            const double obj_v,
                                                            const double ego_a,
                                                            const double obj_a,
                                                            bool is_front_car) {
  double differ_acc =
      std::fabs(ego_a - obj_a) < kZeroEpsilon ? 0.001 : (ego_a - obj_a);
  differ_acc =
      std::fabs(differ_acc) * std::max(0.5, std::fabs(differ_acc)) / differ_acc;
  const double calculate_collision_time = (obj_v - ego_v) / differ_acc;
  if (calculate_collision_time < 0.0 ||
      calculate_collision_time > prediction_time_) {
    double limit_distance =
        (ego_v - obj_v) * prediction_time_ +
        0.5 * differ_acc * prediction_time_ * prediction_time_;
    if (ego_v > obj_v) {
      return is_front_car ? limit_distance : 0.0;
    } else {
      return is_front_car ? 0.0 : -limit_distance;
    }
  } else {
    double limit_distance =
        (ego_v - obj_v) * calculate_collision_time +
        0.5 * differ_acc * calculate_collision_time * calculate_collision_time;
    if (ego_v > obj_v) {
      return is_front_car ? limit_distance
                          : std::max((obj_v - ego_v) * prediction_time_ -
                                         0.5 * differ_acc * prediction_time_ *
                                             prediction_time_,
                                     0.0);
    } else {
      return is_front_car ? std::max((ego_v - obj_v) * prediction_time_ +
                                         0.5 * differ_acc * prediction_time_ *
                                             prediction_time_,
                                     0.0)
                          : -limit_distance;
    }
  }
}

void SampleQuarticPolynomialCurve::CalcCost(
    STSampleSpaceBase& sample_space_base, const double ego_v,
    const double ego_a, const double suggested_v, const double stop_line_s,
    const double leading_veh_s, const double leading_veh_v,
    int32_t leading_veh_id, bool enable_merge_decelaration,
    double speed_differ_gain, double distance_to_stop_point,
    const LanChangeSafetyCheckConfig& lc_safety_distance_config,
    const double cur_time) {
  // anchor points cost
  double last_cost = cost_sum_;
  double last_arrived_s = arrived_s_;
  double last_arrived_v = arrived_v_;
  double last_arrived_a = arrived_a_;
  double last_arrived_t = arrived_t_;
  STPoint anchor_matched_upper_st_point;
  STPoint anchor_matched_lower_st_point;
  const double& anchor_arrived_t = cur_time;
  const double& anchor_arrived_v =
      anchor_arrived_t - poly_.T() > 0
          ? poly_.CalculateFirstDerivative(poly_.T())
          : poly_.CalculateFirstDerivative(anchor_arrived_t);
  double anchor_arrived_s =
      anchor_arrived_t - poly_.T() > 0
          ? poly_.CalculatePoint(poly_.T()) +
                anchor_arrived_v * (anchor_arrived_t - poly_.T())
          : poly_.CalculatePoint(anchor_arrived_t);
  double anchor_arrived_a =
      anchor_arrived_t - poly_.T() > 0.0
          ? 0.0
          : poly_.CalculateSecondDerivative(anchor_arrived_t);
  sample_space_base.GetBorderByAvailable(anchor_arrived_s, anchor_arrived_t,
                                         &anchor_matched_lower_st_point,
                                         &anchor_matched_upper_st_point);
  const double safe_distance_to_gap_front_obj = CalcGapVelSafeDistance(
      anchor_arrived_v, anchor_matched_upper_st_point.velocity(),
      anchor_arrived_a, anchor_matched_upper_st_point.acceleration(), true);
  const double safe_distance_to_gap_back_obj = CalcGapVelSafeDistance(
      anchor_arrived_v, anchor_matched_lower_st_point.velocity(),
      anchor_arrived_a, anchor_matched_lower_st_point.acceleration(), false);
  double rest_changeable_distance =
      anchor_matched_upper_st_point.s() - anchor_matched_lower_st_point.s() -
      safe_distance_to_gap_front_obj - safe_distance_to_gap_back_obj -
      front_edge_to_rear_axle_ - back_edge_to_rear_axle_;
  if (rest_changeable_distance < 2.0 && !enable_merge_decelaration) {
    return;
  }
  cost_sum_ = 0.0;
  auto anchor_points_match_gap_cost = anchor_points_match_gap_cost_;
  auto speed_change_cost = speed_change_cost_;
  auto stop_line_cost = stop_line_cost_;
  auto leading_veh_safe_cost = leading_veh_safe_cost_;
  auto stop_point_cost = stop_point_cost_;
  auto follow_vel_cost = follow_vel_cost_;
  auto leading_veh_follow_s_cost = leading_veh_follow_s_cost_;
  auto speed_variable_cost = speed_variable_cost_;
  auto stop_penalty_cost = stop_penalty_cost_;
  auto gap_avaliable_cost = gap_avaliable_cost_;
  auto acc_limit_cost = acc_limit_cost_;
  CostInit();
  anchor_points_match_gap_cost_.GetCost(
      anchor_matched_upper_st_point, anchor_matched_lower_st_point,
      anchor_arrived_s, anchor_arrived_t, anchor_arrived_v,
      safe_distance_to_gap_front_obj, safe_distance_to_gap_back_obj, ego_v,
      enable_merge_decelaration, lc_safety_distance_config);
  arrived_s_ = anchor_arrived_s;
  arrived_v_ = anchor_arrived_v;
  arrived_a_ = anchor_arrived_a;
  arrived_t_ = anchor_arrived_t;
  arrived_v_ = std::max(arrived_v_, kZeroEpsilon);
  if (anchor_points_match_gap_cost_.cost() < kZeroEpsilon) {
    if ((stop_line_s - (arrived_s_ - CalcS(0))) / arrived_v_ > 2.5) {
      speed_differ_gain = 0.0;
      distance_to_stop_point = kMaxDistanceToStopPoint;
    }
  }

  if (leading_veh_id != kNoAgentId && leading_veh_id != -1 &&
      anchor_points_match_gap_cost_.cost() > kZeroEpsilon) {
    follow_vel_cost_.GetCost(arrived_v_, suggested_v, kFollowSpeedBenchmark);
    leading_veh_follow_s_cost_.GetCost(
        leading_veh_s + leading_veh_v * arrived_t_, arrived_v_, arrived_s_);
  }

  gap_valid_ = anchor_points_match_gap_cost_.is_gap_changeable() or
               (leading_veh_follow_s_cost_.cost() < kZeroEpsilon);

  // // poly curve cost
  speed_change_cost_.GetCost(arrived_v_, ego_v, arrived_t_);

  // follow_vel_cost_.GetCost(arrived_v_, suggested_v, kFollowSpeedBenchmark);

  stop_line_cost_.GetCost(stop_line_s, arrived_s_ - CalcS(0), arrived_v_, true);

  if (leading_veh_id != kNoAgentId && leading_veh_id != -1) {
    leading_veh_safe_cost_.GetCost(arrived_s_, arrived_v_,
                                   leading_veh_s + leading_veh_v * arrived_t_,
                                   leading_veh_v);
  }

  const double vel_integral = CalcVelIntegral(arrived_t_);
  speed_variable_cost_.GetCost(vel_integral);

  const auto& agent_map = sample_space_base.agent_id_veh_info();
  const auto& front_agent =
      agent_map.find(anchor_matched_upper_st_point.agent_id());
  const auto& rear_agent =
      agent_map.find(anchor_matched_lower_st_point.agent_id());
  if (anchor_matched_upper_st_point.agent_id() != kNoAgentId &&
      anchor_matched_lower_st_point.agent_id() != kNoAgentId) {
    if (front_agent != agent_map.end() && rear_agent != agent_map.end()) {
      const double future_gap_length =
          anchor_matched_upper_st_point.s() - anchor_matched_lower_st_point.s();
      const double gap_length =
          front_agent->second->center_s - front_agent->second->half_length -
          (rear_agent->second->center_s + rear_agent->second->half_length);
      gap_avaliable_cost_.GetCost(future_gap_length, gap_length);
    }
  }

  stop_penalty_cost_.GetCost(arrived_v_);

  stop_point_cost_.GetCost(distance_to_stop_point + CalcS(0) - arrived_s_);

  const double acc_extrema = std::fmax(std::fabs(poly_.acc_extrema().first),
                                       std::fabs(poly_.acc_extrema().second));
  acc_limit_cost_.GetCost(acc_extrema);
  cost_sum_ = anchor_points_match_gap_cost_.cost() + follow_vel_cost_.cost() +
              stop_line_cost_.cost() * speed_differ_gain +
              leading_veh_safe_cost_.cost() + speed_variable_cost_.cost() +
              gap_avaliable_cost_.cost() + stop_penalty_cost_.cost() +
              acc_limit_cost_.cost() + speed_change_cost_.cost() +
              stop_point_cost_.cost() + leading_veh_follow_s_cost_.cost() +
              std::exp(arrived_t_ / 5.0);

  if (cost_sum_ > last_cost) {
    cost_sum_ = last_cost;
    arrived_s_ = last_arrived_s;
    arrived_v_ = last_arrived_v;
    arrived_a_ = last_arrived_a;
    arrived_t_ = last_arrived_t;
    anchor_points_match_gap_cost_ = std::move(anchor_points_match_gap_cost);
    speed_change_cost_ = std::move(speed_change_cost);
    stop_line_cost_ = std::move(stop_line_cost);
    leading_veh_safe_cost_ = std::move(leading_veh_safe_cost);
    stop_point_cost_ = std::move(stop_point_cost);
    follow_vel_cost_ = std::move(follow_vel_cost);
    leading_veh_follow_s_cost_ = std::move(leading_veh_follow_s_cost);
    speed_variable_cost_ = std::move(speed_variable_cost);
    stop_penalty_cost_ = std::move(stop_penalty_cost);
    gap_avaliable_cost_ = std::move(gap_avaliable_cost);
    acc_limit_cost_ = std::move(acc_limit_cost);
  } else {
    safe_border_distance_to_gap_front_obj_ = safe_distance_to_gap_front_obj;
    safe_border_distance_to_gap_back_obj_ = safe_distance_to_gap_back_obj;
    rest_changeable_distance_ = rest_changeable_distance;
    end_point_matched_gap_back_id_ = anchor_matched_lower_st_point.agent_id();
    end_point_matched_gap_front_id_ = anchor_matched_upper_st_point.agent_id();
  }
}
}  // namespace planning
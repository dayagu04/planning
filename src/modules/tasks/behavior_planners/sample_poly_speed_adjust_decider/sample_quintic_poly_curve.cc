#include "sample_quintic_poly_curve.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "behavior_planners/sample_poly_speed_adjust_decider/sample_poly_const.h"
#include "behavior_planners/sample_poly_speed_adjust_decider/sample_speed_adjust_cost.h"
#include "st_graph/st_point.h"
#include "task_interface/lane_change_utils.h"
namespace planning {

SampleQuinticPolynomialCurve::SampleQuinticPolynomialCurve(
    QuinticPolynomial& poly, double arrived_t, double mid_t,
    const double weight_match_gap_vel, const double weight_match_gap_s,
    const double weight_follow_vel, const double weight_stop_line,
    const double weight_leading_veh_safe_s, const double weight_speed_variable,
    const double weight_gap_avaliable, const double weight_acc_limit,
    const double weight_stop_penalty, const double weight_speed_change,
    const double weight_leading_veh_follow_s, const double weight_jerk_limit,
    const double front_edge_to_rear_axle, const double back_edge_to_rear_axle) {
  poly_ = poly;
  arrived_t_ = arrived_t;
  arrived_v_ = arrived_t_ - poly_.T() > 0
                   ? poly_.CalculateFirstDerivative(poly_.T())
                   : poly_.CalculateFirstDerivative(arrived_t_);
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
  safe_distance_cost_.SetWeight(weight_match_gap_s);
  leading_veh_follow_s_cost_.SetWeight(weight_leading_veh_follow_s);
  leading_veh_follow_s_cost_.SetRearAxleToBumpDis(front_edge_to_rear_axle);
  jerk_limit_cost_.SetWeight(weight_jerk_limit);
};

void SampleQuinticPolynomialCurve::CostInit() {
  follow_vel_cost_.Init();
  stop_line_cost_.Init();
  leading_veh_safe_cost_.Init();
  speed_variable_cost_.Init();
  gap_avaliable_cost_.Init();
  acc_limit_cost_.Init();
  stop_penalty_cost_.Init();
  speed_change_cost_.Init();
  safe_distance_cost_.Init();
  jerk_limit_cost_.Init();
  cost_sum_ = 0.0;
}
double SampleQuinticPolynomialCurve::CalcS(const double t) const {
  return t - poly_.T() >= 0
             ? poly_.CalculatePoint(poly_.T()) + arrived_v_ * (t - poly_.T())
             : poly_.CalculatePoint(t);
}
double SampleQuinticPolynomialCurve::CalcV(const double t) const {
  return t - poly_.T() >= 0 ? arrived_v_ : poly_.CalculateFirstDerivative(t);
}
double SampleQuinticPolynomialCurve::CalcAcc(const double t) const {
  return t - poly_.T() >= 0 ? 0.0 : poly_.CalculateSecondDerivative(t);
}

double SampleQuinticPolynomialCurve::CalcJerk(const double t) const {
  return t - poly_.T() >= 0 ? 0.0 : poly_.CalculateThirdDerivative(t);
}

double SampleQuinticPolynomialCurve::CalcRef(const double t,
                                             const double decay_coffi) const {
  double s = 0.0;
  if (arrived_t_ < poly_.T()) {
    double acc =
        std::max(arrived_a_, -arrived_v_ / std::fmax(5.0 - arrived_t_, 0.1));
    double left_t = t - arrived_t_;
    s = left_t > 0.1 ? poly_.CalculatePoint(arrived_t_) +
                           (arrived_v_ - acc / decay_coffi) * left_t +
                           acc * std::exp(decay_coffi * left_t) /
                               (decay_coffi * decay_coffi) -
                           acc / (decay_coffi * decay_coffi)
                     : poly_.CalculatePoint(t);
  } else {
    s = t - poly_.T() > 0
            ? poly_.CalculatePoint(poly_.T()) + arrived_v_ * (t - poly_.T())
            : poly_.CalculatePoint(t);
  }
  return s;
}

double SampleQuinticPolynomialCurve::CalcVelIntegral(
    const double evaluate_t) const {
  const double t = evaluate_t > poly_.T() ? poly_.T() : evaluate_t;
  const double t_2 = t * t;
  const double t_3 = t_2 * t;
  const double t_4 = t_3 * t;
  const auto& coeff = poly_.coefficients();
  return coeff[4] * t_4 + coeff[3] * t_3 + coeff[2] * t_2;
}

double SampleQuinticPolynomialCurve::CalcGapVelSafeDistance(const double ego_v,
                                                            const double obj_v,
                                                            const double ego_a,
                                                            const double obj_a,
                                                            bool is_front_car) {
  double differ_acc =
      std::fabs(ego_a - obj_a) < kZeroEpsilon ? 0.001 : (ego_a - obj_a);
  differ_acc =
      std::fabs(differ_acc) * std::min(0.8, std::fabs(differ_acc)) / differ_acc;
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

void SampleQuinticPolynomialCurve::CalcCost(
    STSampleSpaceBase& sample_space_base, const double ego_v,
    const double ego_a, const double suggested_v,
    const LeadingAgentInfo& leading_veh) {
  // anchor points cost
  STPoint anchor_matched_upper_st_point;
  STPoint anchor_matched_lower_st_point;
  const double& anchor_arrived_t = poly_.T();
  const double& anchor_arrived_v =
      poly_.CalculateFirstDerivative(anchor_arrived_t);
  double anchor_arrived_s = poly_.CalculatePoint(anchor_arrived_t);
  double anchor_arrived_a = poly_.CalculateSecondDerivative(anchor_arrived_t);
  arrived_s_ = anchor_arrived_s;
  arrived_v_ = anchor_arrived_v;
  arrived_a_ = anchor_arrived_a;
  arrived_t_ = anchor_arrived_t;
  arrived_v_ = std::max(arrived_v_, kZeroEpsilon);
  sample_space_base.GetBorderByAvailable(anchor_arrived_s, anchor_arrived_t,
                                         &anchor_matched_lower_st_point,
                                         &anchor_matched_upper_st_point);
  const double safe_distance_to_gap_front_obj = CalcGapVelSafeDistance(
      anchor_arrived_v, anchor_matched_upper_st_point.velocity(),
      anchor_arrived_a, anchor_matched_upper_st_point.acceleration(), true);
  const double safe_distance_to_gap_back_obj = CalcGapVelSafeDistance(
      anchor_arrived_v, anchor_matched_lower_st_point.velocity(),
      anchor_arrived_a, anchor_matched_lower_st_point.acceleration(), false);
  double ditsance_to_gap_front_obj = anchor_matched_upper_st_point.s() -
                                     anchor_arrived_s -
                                     front_edge_to_rear_axle_;
  double distance_to_gap_back_obj = anchor_arrived_s -
                                    anchor_matched_lower_st_point.s() -
                                    back_edge_to_rear_axle_;
  double limi_safe_distance_to_gap_front_obj =
      std::fmax(3.5 + 0.3 * anchor_arrived_v, safe_distance_to_gap_front_obj);
  double limi_safe_distance_to_gap_back_obj =
      std::fmax(3.5 + 0.3 * anchor_matched_upper_st_point.velocity(),
                safe_distance_to_gap_back_obj);
  if (ditsance_to_gap_front_obj < limi_safe_distance_to_gap_front_obj ||
      distance_to_gap_back_obj < limi_safe_distance_to_gap_back_obj) {
    return;
  }
  CostInit();
  double max_safe_distance_to_gap_front_obj;
  double max_safe_distance_to_gap_back_obj;
  safe_distance_cost_.GetCost(
      ditsance_to_gap_front_obj, distance_to_gap_back_obj,
      limi_safe_distance_to_gap_front_obj, limi_safe_distance_to_gap_back_obj,
      max_safe_distance_to_gap_front_obj, max_safe_distance_to_gap_back_obj);

  int temp_index = static_cast<int>(arrived_t_ / kTimeResolution + 0.51);
  double traveled_distance = 0.0;
  double leading_end_v = 0.0;
  if (temp_index < leading_veh.prediction_path.size()) {
    traveled_distance = leading_veh.prediction_path[temp_index].first;
    leading_end_v = leading_veh.prediction_path[temp_index].second;
  } else {
    traveled_distance = leading_veh.v * arrived_t_;
    leading_end_v = leading_veh.v;
  }

  if (leading_veh.id != kNoAgentId && leading_veh.id != -1) {
    follow_vel_cost_.GetCost(arrived_v_, suggested_v, kFollowSpeedBenchmark);
    leading_veh_follow_s_cost_.GetCost(
        leading_veh.center_s + traveled_distance + CalcS(0), arrived_v_,
        arrived_s_);
  }
  // // poly curve cost
  speed_change_cost_.GetCost(arrived_v_, ego_v, arrived_t_);

  // follow_vel_cost_.GetCost(arrived_v_, suggested_v, kFollowSpeedBenchmark);

  if (leading_veh.id != kNoAgentId && leading_veh.id != -1) {
    leading_veh_safe_cost_.GetCost(arrived_s_ - CalcS(0), arrived_v_,
                                   leading_veh.center_s + traveled_distance,
                                   leading_end_v);
  }

  // const double vel_integral = CalcVelIntegral(arrived_t_);
  // speed_variable_cost_.GetCost(vel_integral);
  auto jerk_limit = std::fmax(std::fabs(poly_.jerk_extrema().first),
                              std::fabs(poly_.jerk_extrema().second));
  jerk_limit_cost_.GetCost(jerk_limit);

  const double acc_extrema = std::fmax(std::fabs(poly_.acc_extrema().first),
                                       std::fabs(poly_.acc_extrema().second));
  acc_limit_cost_.GetCost(acc_extrema);
  cost_sum_ = safe_distance_cost_.cost() + follow_vel_cost_.cost() +
              leading_veh_safe_cost_.cost() + speed_variable_cost_.cost() +
              acc_limit_cost_.cost() + speed_change_cost_.cost() +
              leading_veh_follow_s_cost_.cost() + jerk_limit_cost_.cost();
}
}  // namespace planning
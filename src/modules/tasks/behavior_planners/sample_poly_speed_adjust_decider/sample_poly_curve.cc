#include "sample_poly_curve.h"

#include <cstdint>

#include "behavior_planners/sample_poly_speed_adjust_decider/sample_poly_const.h"
#include "st_graph/st_point.h"
#include "task_interface/lane_change_utils.h"
namespace planning {

SampleQuarticPolynomialCurve::SampleQuarticPolynomialCurve(
    QuarticPolynomial& poly, double arrived_t, double mid_t,
    const double weight_match_gap_vel, const double weight_match_gap_s,
    const double weight_follow_vel, const double weight_stop_line,
    const double weight_leading_veh_safe_s, const double weight_speed_variable,
    const double weight_gap_avaliable) {
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

  end_point_match_gap_cost_.SetWeightMatchVel(weight_match_gap_vel);
  end_point_match_gap_cost_.SetWeightMatchS(weight_match_gap_s);

  mid_point_match_gap_cost_.SetWeightMatchVel(weight_match_gap_vel);
  mid_point_match_gap_cost_.SetWeightMatchS(weight_match_gap_s);

  follow_vel_cost_.SetWeight(weight_follow_vel);
  stop_line_cost_.SetWeight(weight_stop_line);
  leading_veh_safe_cost_.SetWeight(weight_leading_veh_safe_s);
  speed_variable_cost_.SetWeight(weight_speed_variable);
  gap_avaliable_cost_.SetWeight(weight_gap_avaliable);
};

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

void SampleQuarticPolynomialCurve::CalcCost(
    STSampleSpaceBase& sample_space_base, const double ego_v,
    const double ego_a, const double suggested_v, const double stop_line_s,
    const double leading_veh_s, const double leading_veh_v,
    int32_t leading_veh_id) {
  STPoint end_point_lower_st_point;
  STPoint end_point_upper_st_point;

  sample_space_base.GetBorderByAvailable(arrived_s_, arrived_t_,
                                         &end_point_lower_st_point,
                                         &end_point_upper_st_point);
  end_point_matched_gap_front_id_ = end_point_upper_st_point.agent_id();
  end_point_matched_gap_back_id_ = end_point_lower_st_point.agent_id();

  const double mid_point_reliable_safe_distance_to_gap_front_obj =
      planning::CalcGapObjSafeDistance(
          ego_v, end_point_upper_st_point.velocity(), 0.0, false, true);
  const double mid_point_reliable_safe_distance_to_gap_back_obj =
      planning::CalcGapObjSafeDistance(
          ego_v, end_point_lower_st_point.velocity(), 0.0, false, false);
  end_point_match_gap_cost_.GetCost(
      end_point_upper_st_point, end_point_lower_st_point, arrived_s_,
      arrived_t_, arrived_v_, mid_point_reliable_safe_distance_to_gap_front_obj,
      mid_point_reliable_safe_distance_to_gap_back_obj);

  STPoint mid_point_lower_st_point;
  STPoint mid_point_upper_st_point;
  sample_space_base.GetBorderByAvailable(
      mid_s_, mid_t_, &mid_point_lower_st_point, &mid_point_upper_st_point);
  mid_point_match_gap_front_id_ = mid_point_upper_st_point.agent_id();
  mid_point_match_gap_back_id_ = mid_point_lower_st_point.agent_id();
  const double end_point_reliable_safe_distance_to_gap_front_obj =
      planning::CalcGapObjSafeDistance(
          ego_v, mid_point_upper_st_point.velocity(), 0.0, false, true);
  const double end_point_reliable_safe_distance_to_gap_back_obj =
      planning::CalcGapObjSafeDistance(
          ego_v, mid_point_lower_st_point.velocity(), 0.0, false, false);
  mid_point_match_gap_cost_.GetCost(
      mid_point_upper_st_point, mid_point_lower_st_point, mid_s_, mid_t_,
      mid_v_, end_point_reliable_safe_distance_to_gap_front_obj,
      end_point_reliable_safe_distance_to_gap_back_obj);

  follow_vel_cost_.GetCost(arrived_v_, suggested_v, kFollowSpeedBenchmark);

  stop_line_cost_.GetCost(stop_line_s, arrived_s_ - CalcS(0));

  if (leading_veh_id != kNoAgentId && leading_veh_id != -1) {
    leading_veh_safe_cost_.GetCost(arrived_s_, arrived_v_, leading_veh_s,
                                   leading_veh_v);
  }

  const double vel_integral = CalcVelIntegral(arrived_t_);
  speed_variable_cost_.GetCost(vel_integral);

  const auto& agent_map = sample_space_base.agent_id_veh_info();
  const auto& front_agent = agent_map.find(end_point_upper_st_point.agent_id());
  const auto& rear_agent = agent_map.find(end_point_lower_st_point.agent_id());
  if (end_point_upper_st_point.agent_id() != kNoAgentId &&
      end_point_lower_st_point.agent_id() != kNoAgentId) {
    if (front_agent != agent_map.end() && rear_agent != agent_map.end()) {
      const double future_gap_length =
          end_point_upper_st_point.s() - end_point_lower_st_point.s();
      const double gap_length =
          front_agent->second->center_s - front_agent->second->half_length -
          (rear_agent->second->center_s + rear_agent->second->half_length);
      gap_avaliable_cost_.GetCost(future_gap_length, gap_length);
    }
  }

  stop_penalty_cost_.GetCost(arrived_v_);

  cost_sum_ = mid_point_match_gap_cost_.cost() +
              end_point_match_gap_cost_.cost() + follow_vel_cost_.cost() +
              stop_line_cost_.cost() + leading_veh_safe_cost_.cost() +
              speed_variable_cost_.cost() + gap_avaliable_cost_.cost() +
              stop_penalty_cost_.cost();
}
}  // namespace planning
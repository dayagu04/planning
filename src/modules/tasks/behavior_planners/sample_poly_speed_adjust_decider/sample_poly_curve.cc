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
    const double weight_stop_penalty, const double front_edge_to_rear_axle,
    const double back_edge_to_rear_axle) {
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

  follow_vel_cost_.SetWeight(weight_follow_vel);
  stop_line_cost_.SetWeight(weight_stop_line);
  leading_veh_safe_cost_.SetWeight(weight_leading_veh_safe_s);
  leading_veh_safe_cost_.SetRearAxleToBumpDis(front_edge_to_rear_axle);
  speed_variable_cost_.SetWeight(weight_speed_variable);
  gap_avaliable_cost_.SetWeight(weight_gap_avaliable);
  acc_limit_cost_.SetWeight(weight_acc_limit);
  stop_penalty_cost_.SetWeight(weight_stop_penalty);

  std::vector<double> anchor_point_t_vec = {1.0, 2.0, 3.0, 4.0, 5.0};
  anchor_points_match_gap_cost_vec_.reserve(anchor_point_t_vec.size());

  for (size_t i = 0; i < anchor_point_t_vec.size(); i++) {
    MatchGapCost anchor_point_match_gap_cost;
    anchor_point_match_gap_cost.SetWeightMatchVel(weight_match_gap_vel);
    anchor_point_match_gap_cost.SetWeightMatchS(weight_match_gap_s);
    anchor_point_match_gap_cost.SetRearAxleToBumpDis(front_edge_to_rear_axle,
                                                     back_edge_to_rear_axle);
    anchor_point_match_gap_cost.SetAnchorT(anchor_point_t_vec[i]);
    anchor_points_match_gap_cost_vec_.emplace_back(
        std::move(anchor_point_match_gap_cost));
  }
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
  // anchor points cost
  STPoint end_point_lower_st_point;
  STPoint end_point_upper_st_point;
  for (size_t i = 0; i < anchor_points_match_gap_cost_vec_.size(); i++) {
    STPoint anchor_matched_upper_st_point;
    STPoint anchor_matched_lower_st_point;
    const double& anchor_arrived_t =
        anchor_points_match_gap_cost_vec_[i].anchor_t();
    const double& anchor_arrived_v =
        anchor_arrived_t - poly_.T() > 0
            ? poly_.CalculateFirstDerivative(poly_.T())
            : poly_.CalculateFirstDerivative(anchor_arrived_t);
    double anchor_arrived_s =
        anchor_arrived_t - poly_.T() > 0
            ? poly_.CalculatePoint(poly_.T()) +
                  anchor_arrived_v * (anchor_arrived_t - poly_.T())
            : poly_.CalculatePoint(anchor_arrived_t);

    sample_space_base.GetBorderByAvailable(anchor_arrived_s, anchor_arrived_t,
                                           &anchor_matched_lower_st_point,
                                           &anchor_matched_upper_st_point);
    const double safe_distance_to_gap_front_obj =
        planning::CalcGapObjSafeDistance(
            ego_v, anchor_matched_upper_st_point.velocity(), 0.0, false, true);
    const double safe_distance_to_gap_back_obj =
        planning::CalcGapObjSafeDistance(
            ego_v, anchor_matched_upper_st_point.velocity(), 0.0, false, true);

    anchor_points_match_gap_cost_vec_[i].GetCost(
        anchor_matched_upper_st_point, anchor_matched_lower_st_point,
        anchor_arrived_s, anchor_arrived_t, anchor_arrived_v,
        safe_distance_to_gap_front_obj, safe_distance_to_gap_back_obj, ego_v);

    if (i == anchor_points_match_gap_cost_vec_.size() - 1) {
      end_point_lower_st_point = anchor_matched_lower_st_point;
      end_point_upper_st_point = anchor_matched_upper_st_point;
    }
    cost_sum_ += anchor_points_match_gap_cost_vec_[i].cost();
  }
  // // poly curve cost

  follow_vel_cost_.GetCost(arrived_v_, suggested_v, kFollowSpeedBenchmark);

  stop_line_cost_.GetCost(stop_line_s, arrived_s_ - CalcS(0), arrived_v_);

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

  const double acc_extrema = std::fmax(std::fabs(poly_.acc_extrema().first),
                                       std::fabs(poly_.acc_extrema().second));
  acc_limit_cost_.GetCost(acc_extrema);
  cost_sum_ += follow_vel_cost_.cost() + stop_line_cost_.cost() +
               leading_veh_safe_cost_.cost() + speed_variable_cost_.cost() +
               gap_avaliable_cost_.cost() + stop_penalty_cost_.cost() +
               acc_limit_cost_.cost();
}
}  // namespace planning
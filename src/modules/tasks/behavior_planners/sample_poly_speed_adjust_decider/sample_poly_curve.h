#pragma once
#include <vector>

#include "behavior_planners/sample_poly_speed_adjust_decider/sample_poly_const.h"
#include "behavior_planners/sample_poly_speed_adjust_decider/sample_space_base.h"
#include "sample_speed_adjust_cost.h"
#include "trajectory1d/quartic_poly_trajectory1d.h"
namespace planning {

class SamplePolyCurve {
 public:
  SamplePolyCurve() = default;
  virtual ~SamplePolyCurve() = default;
  virtual double CalcS(const double t) const = 0;
  virtual double CalcV(const double t) const = 0;
  virtual double CalcAcc(const double t) const = 0;
  virtual double CalcJerk(const double t) const = 0;

  double arrived_t() const { return arrived_t_; }
  double arrived_s() const { return arrived_s_; }
  double arrived_v() const { return arrived_v_; }

  double mid_s() const { return mid_s_; }
  double mid_t() const { return mid_t_; }
  double mid_v() const { return mid_v_; }

  void set_safe_valid(const bool safe_valid) { safe_valid_ = safe_valid; };

 public:
  double arrived_s_;
  double arrived_t_;
  double arrived_v_;
  double mid_s_;
  double mid_t_;
  double mid_v_;

  double cost_sum_ = 0.0;
  bool safe_valid_{false};
};

class SampleQuarticPolynomialCurve : public SamplePolyCurve {
 public:
  SampleQuarticPolynomialCurve() = default;
  SampleQuarticPolynomialCurve(
      QuarticPolynomial& poly, double arrived_t, double mid_t,
      const double weight_match_gap_vel, const double weight_match_gap_s,
      const double weight_follow_vel, const double weight_stop_line,
      const double weight_leading_veh_safe_s,
      const double weight_speed_variable, const double weight_gap_avaliable,
      const double weight_acc_limit, const double weight_stop_penalty,
      const double front_edge_to_rear_axle,
      const double back_edge_to_rear_axle);

  double CalcS(const double t) const override;
  double CalcV(const double t) const override;
  double CalcAcc(const double t) const override;
  double CalcJerk(const double t) const override;

  void CalcCost(STSampleSpaceBase& sample_space_base, const double ego_v,
                const double ego_a, const double suggested_v,
                const double stop_line_s, const double leading_veh_s,
                const double leading_veh_v, int32_t leading_veh_id);
  double CalcVelIntegral(const double t) const;

  // interface:
  const QuarticPolynomial& poly() const { return poly_; };
  int32_t end_point_matched_gap_front_id() const {
    return end_point_matched_gap_front_id_;
  };
  int32_t end_point_matched_gap_back_id() const {
    return end_point_matched_gap_back_id_;
  };

  void set_end_point_matched_gap_front_id(
      const int32_t end_point_matched_gap_front_id) {
    end_point_matched_gap_front_id_ = end_point_matched_gap_front_id;
  };

  void set_end_point_matched_gap_back_id(
      const int32_t end_point_matched_gap_back_id) {
    end_point_matched_gap_back_id_ = end_point_matched_gap_back_id;
  };

  const FollowVelCost& follow_vel_cost() const { return follow_vel_cost_; };

  const StopLineCost& stop_line_cost() const { return stop_line_cost_; };

  const LeadingVehSafeCost& leading_veh_safe_cost() const {
    return leading_veh_safe_cost_;
  };

  const SpeedVariableCost& speed_variable_cost() const {
    return speed_variable_cost_;
  };

  const GapAvaliableCost& gap_avaliable_cost() const {
    return gap_avaliable_cost_;
  };

  const StopPenaltyCost& stop_penalty_cost() const {
    return stop_penalty_cost_;
  };

  const std::vector<MatchGapCost>& anchor_points_match_gap_cost_vec() const {
    return anchor_points_match_gap_cost_vec_;
  };

  const AccLimitCost& acc_limit_cost() const { return acc_limit_cost_; }

 private:
  QuarticPolynomial poly_;
  FollowVelCost follow_vel_cost_;
  StopLineCost stop_line_cost_;
  LeadingVehSafeCost leading_veh_safe_cost_;
  SpeedVariableCost speed_variable_cost_;
  GapAvaliableCost gap_avaliable_cost_;
  StopPenaltyCost stop_penalty_cost_;
  AccLimitCost acc_limit_cost_;

  int32_t end_point_matched_gap_front_id_ = kNoAgentId;
  int32_t end_point_matched_gap_back_id_ = kNoAgentId;

  // std::vector<double> anchor_points_checked_t_vec_;
  std::vector<MatchGapCost> anchor_points_match_gap_cost_vec_;
};

}  // namespace planning
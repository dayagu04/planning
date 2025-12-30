#pragma once
#include <vector>

#include <limits>
#include "behavior_planners/sample_poly_speed_adjust_decider/sample_poly_const.h"
#include "behavior_planners/sample_poly_speed_adjust_decider/sample_space_base.h"
#include "sample_poly_curve.h"
#include "sample_speed_adjust_cost.h"
#include "trajectory1d/quartic_poly_trajectory1d.h"
namespace planning {
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
      const double weight_speed_change,
      const double weight_leading_veh_follow_s, const double weight_jerk_limit,
      const double front_edge_to_rear_axle,
      const double back_edge_to_rear_axle);

  double CalcS(const double t) const override;
  double CalcV(const double t) const override;
  double CalcAcc(const double t) const override;
  double CalcJerk(const double t) const override;
  double CalcRef(const double t, const double decay_coffi) const;

  void CalcCost(STSampleSpaceBase& sample_space_base, const double ego_v,
                const double ego_a, const double suggested_v,
                const double stop_line_s, const LeadingAgentInfo& leading_veh,
                bool enable_merge_decelaration, double speed_differ_gain,
                double distance_to_stop_point,
                const LanChangeSafetyCheckConfig& lc_safety_distance_config,
                const double cur_time, bool is_mergr_change);
  double CalcVelIntegral(const double t) const;
  double CalcGapVelSafeDistance(const double ego_v, const double obj_v,
                                const double ego_a, const double obj_a,
                                bool is_front_car);
  void CostInit();
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

  const MatchGapCost& anchor_points_match_gap_cost() const {
    return anchor_points_match_gap_cost_;
  };

  const AccLimitCost& acc_limit_cost() const { return acc_limit_cost_; }

  const SpeedChangeCost& speed_change_cost() const {
    return speed_change_cost_;
  }
  bool is_left_distance_enough() const { return is_left_distance_enough_; }

 private:
  QuarticPolynomial poly_;
  FollowVelCost follow_vel_cost_;
  StopLineCost stop_line_cost_;
  LeadingVehSafeCost leading_veh_safe_cost_;
  SpeedVariableCost speed_variable_cost_;
  GapAvaliableCost gap_avaliable_cost_;
  StopPenaltyCost stop_penalty_cost_;
  AccLimitCost acc_limit_cost_;
  SpeedChangeCost speed_change_cost_;
  StopPointCost stop_point_cost_;
  LeadingVehFollowCost leading_veh_follow_s_cost_;
  JerkLimitCost jerk_limit_cost_;

  int32_t end_point_matched_gap_front_id_ = kNoAgentId;
  int32_t end_point_matched_gap_back_id_ = kNoAgentId;
  double prediction_time_ = 3.0;
  double safe_border_distance_to_gap_back_obj_ = 0.0;
  double safe_border_distance_to_gap_front_obj_ = 0.0;
  double rest_changeable_distance_ = 0.0;
  bool is_left_distance_enough_ = true;

  // std::vector<double> anchor_points_checked_t_vec_;
  MatchGapCost anchor_points_match_gap_cost_;
};

}  // namespace planning
#pragma once
#include "curve_cost.h"
#include "st_graph/st_point.h"
using planning::speed::STPoint;
namespace planning {

class MatchGapCost : public CurveCost {
 public:
  MatchGapCost() = default;
  void GetCost(const STPoint& upper_st_point, const STPoint& lower_st_point,
               const double poly_end_s, const double poly_end_t,
               const double poly_end_v,
               const double reliable_safe_distance_to_gap_front_obj,
               const double reliable_safe_distance_to_gap_back_obj);
  void SetWeightMatchS(const double weight_s) { weight_match_s_ = weight_s; }
  void SetWeightMatchVel(const double weight_vel) {
    weight_match_v_ = weight_vel;
  }

  const double match_s_cost() const { return match_s_cost_; }
  const double match_v_cost() const { return match_v_cost_; }
  const double match_gap_center_cost() const { return match_gap_center_cost_; }

 private:
  double weight_match_s_ = 0.0;
  double weight_match_v_ = 0.0;

  double match_s_cost_ = 0.0;
  double match_v_cost_ = 0.0;
  double match_gap_center_cost_ = 0.0;
};

class FollowVelCost : public CurveCost {
 public:
  FollowVelCost() = default;
  void GetCost(const double poly_end_v, const double cruise_v,
               const double follow_vel_penalty_benchmark);
};

class StopLineCost : public CurveCost {
 public:
  StopLineCost() = default;
  void GetCost(const double stop_line_s, const double poly_end_s);

  private:
  double mid_stop_dis_penalty_coef_ = 3.0;
  double near_stop_dis_penalty_coef_ = 4.0;
};

class LeadingVehSafeCost : public CurveCost {
 public:
  LeadingVehSafeCost() = default;
  void GetCost(const double poly_end_s, const double poly_end_v,
               const double leading_veh_pred_s, const double leading_veh_v);
};

class SpeedVariableCost : public CurveCost {
 public:
  SpeedVariableCost() = default;
  void GetCost(const double vel_integral);
};

class GapAvaliableCost : public CurveCost {
 public:
  GapAvaliableCost() = default;
  void GetCost(const double future_gap_length, const double gap_length);
};

class StopPenaltyCost : public CurveCost {
 public:
  StopPenaltyCost() = default;
  void GetCost(const double end_v);
};

class AccLimitCost : public CurveCost {
public:
  AccLimitCost() =default;
  void GetCost(const double acc_extrema);

};
}  // namespace planning

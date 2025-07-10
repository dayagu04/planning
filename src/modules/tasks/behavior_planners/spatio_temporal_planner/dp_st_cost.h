#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ego_planning_config.h"
#include "session.h"
#include "slt_graph_point.h"
#include "spatio_temporal_union_dp_input.pb.h"

namespace planning {

// struct DpStSpeedOptimizer {
//   double default_speed_cost = 1.0;
//   double exceed_speed_penalty = 1.5;
//   double low_speed_penalty = 1.5;
//   double reference_speed_penalty = 10.0;
//   double keep_clear_low_speed_penalty = 10.0;
//   double accel_penalty = 1.0;
//   double decel_penalty = 1.0;

//   double positive_jerk_coeff = 1.0;
//   double negative_jerk_coeff = 1.0;

//   double max_acceleration = 2.0;
//   double max_deceleration = -4.0;
//   double spatial_potential_penalty = 1.0e2;
// };

class DpStCost {
 public:
  DpStCost();

  void Init(
      const planning::common::LongitudinalWeightParams& long_weight_params) {
    config_ = long_weight_params;
    return;
  }

  // double GetReferenceCost(const SLTPoint& point,
  //                         const SLTPoint& reference_point) const;

  double GetSpatialPotentialCost(const SLTGraphPoint& point);

  double GetSpeedCost(const SLTGraphPoint& first, const SLTGraphPoint& second,
                      const double& speed_limit, const double& inv_speed_limit,
                      const double& cruise_speed) const;

  double GetAccelCostByTwoPoints(const double pre_speed,
                                 const SLTGraphPoint& first,
                                 const SLTGraphPoint& second);
  double GetAccelCostByThreePoints(const SLTGraphPoint& first,
                                   const SLTGraphPoint& second,
                                   const SLTGraphPoint& third);

  double GetJerkCostByTwoPoints(const double pre_speed, const double pre_acc,
                                const SLTGraphPoint& pre_point,
                                const SLTGraphPoint& curr_point);
  double GetJerkCostByThreePoints(const double first_speed,
                                  const SLTGraphPoint& first_point,
                                  const SLTGraphPoint& second_point,
                                  const SLTGraphPoint& third_point);

  double GetJerkCostByFourPoints(const SLTGraphPoint& first,
                                 const SLTGraphPoint& second,
                                 const SLTGraphPoint& third,
                                 const SLTGraphPoint& fourth);

 private:
  double GetAccelCost(const double accel);
  double JerkCost(const double jerk);

  //   void AddToKeepClearRange(const std::vector<const Obstacle*>& obstacles);
  static void SortAndMergeRange(
      std::vector<std::pair<double, double>>* keep_clear_range_);
  bool InKeepClearRange(double s) const;

  planning::common::LongitudinalWeightParams config_;

  PlanningInitPoint init_point_;

  double unit_t_ = 0.0;
  double total_s_ = 0.0;
  double inv_unit_t_ = 0.0;
  double inv_t_squared_ = 0.0;
  double inv_t_cube_ = 0.0;

  std::unordered_map<std::string, int> boundary_map_;
  std::vector<std::vector<std::pair<double, double>>> boundary_cost_;

  std::vector<std::pair<double, double>> keep_clear_range_;

  std::array<double, 200> accel_cost_;
  std::array<double, 400> jerk_cost_;
};

}  // namespace planning

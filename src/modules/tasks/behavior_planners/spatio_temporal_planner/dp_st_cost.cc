#include "dp_st_cost.h"

#include <algorithm>
#include <limits>
#include "behavior_planners/lateral_offset_decider/lateral_offset_decider_utils.h"

namespace planning {
namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kDefaultTotalTime = std::numeric_limits<double>::infinity();

}

DpStCost::DpStCost() {
  unit_t_ = 1.0;
  inv_unit_t_ = 1.0 / unit_t_;
  inv_t_squared_ = 1.0 / (unit_t_ * unit_t_);
  inv_t_cube_ = 1.0 / (unit_t_ * unit_t_ * unit_t_);
  accel_cost_.fill(-1.0);
  jerk_cost_.fill(-1.0);
}

// void DpStCost::AddToKeepClearRange(
//     const std::vector<const Obstacle*>& obstacles) {
//   for (const auto& obstacle : obstacles) {
//     if (obstacle->path_st_boundary().IsEmpty()) {
//       continue;
//     }
//     if (obstacle->path_st_boundary().boundary_type() !=
//         STBoundary::BoundaryType::KEEP_CLEAR) {
//       continue;
//     }

//     double start_s = obstacle->path_st_boundary().min_s();
//     double end_s = obstacle->path_st_boundary().max_s();
//     keep_clear_range_.emplace_back(start_s, end_s);
//   }
//   SortAndMergeRange(&keep_clear_range_);
// }

// void DpStCost::SortAndMergeRange(
//     std::vector<std::pair<double, double>>* keep_clear_range) {
//   if (!keep_clear_range || keep_clear_range->empty()) {
//     return;
//   }
//   std::sort(keep_clear_range->begin(), keep_clear_range->end());
//   size_t i = 0;
//   size_t j = i + 1;
//   while (j < keep_clear_range->size()) {
//     if (keep_clear_range->at(i).second < keep_clear_range->at(j).first) {
//       ++i;
//       ++j;
//     } else {
//       keep_clear_range->at(i).second = std::max(keep_clear_range->at(i).second,
//                                                 keep_clear_range->at(j).second);
//       ++j;
//     }
//   }
//   keep_clear_range->resize(i + 1);
// }

// bool DpStCost::InKeepClearRange(double s) const {
//   for (const auto& p : keep_clear_range_) {
//     if (p.first <= s && p.second >= s) {
//       return true;
//     }
//   }
//   return false;
// }

// double DpStCost::GetReferenceCost(const STPoint& point,
//                                   const STPoint& reference_point) const {
//   return config_.reference_weight() * (point.s() - reference_point.s()) *
//          (point.s() - reference_point.s()) * unit_t_;
// }


double DpStCost::GetSpatialPotentialCost(const SLTGraphPoint& point) {
  return (total_s_ - point.point().s()) * config_.spatial_potential_penalty();
}

double DpStCost::GetSpeedCost(const SLTGraphPoint& first, const SLTGraphPoint& second,
                              const double &speed_limit,
                              const double &inv_speed_limit,
                              const double &cruise_speed) const {
  const bool enable_dp_reference_speed = true;
  double cost = 0.0;
  const double speed = (second.point().s() - first.point().s()) * inv_unit_t_;
  if (speed < 0) {
    return kInf;
  }

  const double max_adc_stop_speed = 0.2;
  if (speed < max_adc_stop_speed) {
    // first.s in range
    cost += config_.keep_clear_low_speed_penalty() * unit_t_ *
            config_.default_speed_cost();
  }

  double det_speed = (speed - speed_limit) * inv_speed_limit;
  if (det_speed > 0) {
    cost += config_.exceed_speed_penalty() * config_.default_speed_cost() *
            (det_speed * det_speed) * unit_t_;
  } else if (det_speed < 0) {
    cost += config_.low_speed_penalty() * config_.default_speed_cost() *
            -det_speed * unit_t_;
  }

  if (enable_dp_reference_speed) {
    double diff_speed = (speed - cruise_speed) / cruise_speed;
    cost += config_.reference_speed_penalty() * config_.default_speed_cost() *
            fabs(diff_speed) * unit_t_;
  }

  return cost;
}

double DpStCost::GetAccelCost(const double accel) {
  double cost = 0.0;
  static constexpr double kEpsilon = 0.1;
  static constexpr size_t kShift = 100;
  const size_t accel_key = static_cast<size_t>(accel / kEpsilon + 0.5 + kShift);
  DCHECK_LT(accel_key, accel_cost_.size());
  if (accel_key >= accel_cost_.size()) {
    return kInf;
  }

  if (accel_cost_.at(accel_key) < 0.0) {
    const double accel_sq = accel * accel;
    double max_acc = 2.0;
    double max_dec = -4.0;
    double accel_penalty = config_.accel_penalty();
    double decel_penalty = config_.decel_penalty();

    if (accel > 0.0) {
      cost = accel_penalty * accel_sq;
    } else {
      cost = decel_penalty * accel_sq;
    }
    cost += accel_sq * decel_penalty * decel_penalty /
                (1 + std::exp(1.0 * (accel - max_dec))) +
            accel_sq * accel_penalty * accel_penalty /
                (1 + std::exp(-1.0 * (accel - max_acc)));

    // cost += accel_sq * decel_penalty * decel_penalty /
    //             (2 + 4 * (accel - max_dec) * (accel - max_dec)) +
    //         accel_sq * accel_penalty * accel_penalty /
    //             (2 + 2 * (accel - max_acc) * (accel - max_acc));

    accel_cost_.at(accel_key) = cost;
  } else {
    cost = accel_cost_.at(accel_key);
  }
  return cost * unit_t_;
}

double DpStCost::GetAccelCostByThreePoints(const SLTGraphPoint& first,
                                           const SLTGraphPoint& second,
                                           const SLTGraphPoint& third) {
  double accel = 
      (first.point().s() + third.point().s() - 2 * second.point().s()) * inv_t_squared_;
  return GetAccelCost(accel);
}

double DpStCost::GetAccelCostByTwoPoints(const double pre_speed,
                                         const SLTGraphPoint& pre_point,
                                         const SLTGraphPoint& curr_point) {
  double current_speed = (curr_point.point().s() - pre_point.point().s()) * inv_unit_t_;
  double accel = (current_speed - pre_speed) * inv_unit_t_;
  return GetAccelCost(accel);
}

double DpStCost::JerkCost(const double jerk) {
  double cost = 0.0;
  static constexpr double kEpsilon = 0.1;
  static constexpr size_t kShift = 200;
  const size_t jerk_key = static_cast<size_t>(jerk / kEpsilon + 0.5 + kShift);
  if (jerk_key >= jerk_cost_.size()) {
    return kInf;
  }

  if (jerk_cost_.at(jerk_key) < 0.0) {
    double jerk_sq = jerk * jerk;
    if (jerk > 0) {
      cost = config_.positive_jerk_coeff() * jerk_sq * unit_t_;
    } else {
      cost = config_.negative_jerk_coeff() * jerk_sq * unit_t_;
    }
    jerk_cost_.at(jerk_key) = cost;
  } else {
    cost = jerk_cost_.at(jerk_key);
  }

  // TODO(All): normalize to unit_t_
  return cost;
}

double DpStCost::GetJerkCostByFourPoints(const SLTGraphPoint& first,
                                         const SLTGraphPoint& second,
                                         const SLTGraphPoint& third,
                                         const SLTGraphPoint& fourth) {
  double jerk = 
      (fourth.point().s() - 3 * third.point().s() + 3 * second.point().s() - first.point().s()) *
      inv_t_cube_;
  return JerkCost(jerk);
}

double DpStCost::GetJerkCostByTwoPoints(const double pre_speed,
                                        const double pre_acc,
                                        const SLTGraphPoint& pre_point,
                                        const SLTGraphPoint& curr_point) {
  const double curr_speed = 
      (curr_point.point().s() - pre_point.point().s()) * inv_unit_t_;
  const double curr_accel = (curr_speed - pre_speed) * inv_unit_t_;
  const double jerk = (curr_accel - pre_acc) * inv_unit_t_;
  return JerkCost(jerk);
}

double DpStCost::GetJerkCostByThreePoints(const double first_speed,
                                          const SLTGraphPoint& first,
                                          const SLTGraphPoint& second,
                                          const SLTGraphPoint& third) {
  const double pre_speed = (second.point().s() - first.point().s()) * inv_unit_t_;
  const double pre_acc = (pre_speed - first_speed) * inv_unit_t_;
  const double curr_speed = (third.point().s() - second.point().s()) * inv_unit_t_;
  const double curr_acc = (curr_speed - pre_speed) * inv_unit_t_;
  const double jerk = (curr_acc - pre_acc) * inv_unit_t_;
  return JerkCost(jerk);
}

}  // namespace planning
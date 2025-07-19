#include "dp_cost_generator.h"

#include <cmath>

namespace planning {
namespace apa_planner {
void DPCostGenerator::CalcStopoverCost(SVGraphNode* point) {
  if (point->IsZeroSpeed() && point->GetSVPoint().s < total_s_) {
    point->MutableCost()->stopover_cost =
        std::fabs(total_s_ - point->GetSVPoint().s) * config_->stopover_penalty;
  } else {
    point->MutableCost()->stopover_cost = 0;
  }

  return;
}

double DPCostGenerator::CalcSpeedUpCost(const double acc, const double node_v,
                                        const double speed_limit) {
  if (acc > config_->acceleration_limit) {
    return 10000.0;
  }

  // get speed limit around value, then the node acc value is always good even
  // if it is 0.1.
  double speed_limit_around =
      speed_limit * speed_limit -
      2 * config_->advised_acceleration * config_->unit_s;
  speed_limit_around = std::max(speed_limit_around, 0.0);
  speed_limit_around = std::sqrt(speed_limit_around);

  double acc_diff;
  if (node_v < speed_limit_around) {
    acc_diff = acc - config_->advised_acceleration;
  } else if (node_v > speed_limit) {
    acc_diff = acc;
  } else {
    // [ speed_limit_around, speed_limit ]
    // no need an acc penalty.
    if (acc <= config_->advised_acceleration) {
      acc_diff = 0.0;
    } else {
      acc_diff = acc - config_->advised_acceleration;
    }
  }

  return config_->acceleration_penalty * acc_diff * acc_diff;
}

void DPCostGenerator::CalcSpeedCost(const double speed_limit,
                                    SVGraphNode* point) {
  double cost = 0.0;
  const double speed = point->GetSVPoint().v;

  // speed limit is 0
  if (speed_limit < 0.001) {
    if (speed > 0) {
      cost += config_->exceed_speed_penalty * speed;
    }
    point->MutableCost()->speed_limit_cost = cost;
    return;
  }

  if (speed < config_->low_speed_limit) {
    cost += config_->low_speed_penalty * (config_->low_speed_limit - speed);
  }

  double speed_diff = speed - speed_limit;
  if (speed_diff > 0) {
    cost += config_->exceed_speed_penalty * speed_diff;
  } else if (speed_diff < 0) {
    cost += config_->ref_speed_gap_penalty * std::fabs(speed_diff);
  }

  point->MutableCost()->speed_limit_cost = cost;

  return;
}

double DPCostGenerator::CalcSpeedDownCost(const double dec, const double node_v,
                                          const double speed_limit) {
  if (dec < config_->deceleration_limit) {
    return 10000.0;
  }

  // get speed limit around value, then the node dec value is always good even
  // if it is 0.1.
  double speed_limit_around =
      speed_limit * speed_limit +
      2 * config_->advised_deceleration * config_->unit_s;
  speed_limit_around = std::max(speed_limit_around, 0.0);
  speed_limit_around = std::sqrt(speed_limit_around);

  double acc_diff;
  if (node_v < speed_limit_around) {
    acc_diff  = dec - config_->advised_deceleration;
  } else if (node_v > speed_limit) {
    acc_diff = dec;
  } else {
    // [ speed_limit_around, speed_limit ]
    // no need an acc penalty.
    if (dec > config_->advised_deceleration) {
      acc_diff = 0.0;
    } else {
      acc_diff = dec - config_->advised_deceleration;
    }
  }

  return config_->deceleration_penalty * acc_diff * acc_diff;
}

void DPCostGenerator::CalcAccCost(SVGraphNode* point) {
  if (std::fabs(point->GetSVPoint().acc) < 1e-2) {
    point->MutableCost()->acc_cost = 0.0;
  } else if (point->GetSVPoint().acc < 0) {
    point->MutableCost()->acc_cost = CalcSpeedDownCost(
        point->GetSVPoint().acc, point->GetSVPoint().v, point->GetSpeedLimit());
  } else {
    point->MutableCost()->acc_cost = CalcSpeedUpCost(
        point->GetSVPoint().acc, point->GetSVPoint().v, point->GetSpeedLimit());
  }

  return;
}

void DPCostGenerator::CalcJerkCost(SVGraphNode* point) {
  double cost = config_->jerk_penalty * point->GetSVPoint().jerk *
                point->GetSVPoint().jerk;

  point->MutableCost()->jerk_cost = cost;

  return;
}

void DPCostGenerator::UpdateTotalCost(const SVGraphNode* parent,
                                      SVGraphNode* child) {
  DpSpeedCost* child_cost = child->MutableCost();
  child_cost->parent_cost = parent->TotalCost();

  child_cost->total_cost = child_cost->acc_cost + child_cost->speed_limit_cost +
                           child_cost->stopover_cost + child_cost->jerk_cost +
                           child_cost->parent_cost;

  return;
}

void DPCostGenerator::Init(const double total_s, const DpSpeedConfig* config) {
  total_s_ = total_s;
  config_ = config;

  return;
}
}  // namespace apa_planner
}  // namespace planning
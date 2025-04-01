#pragma once

namespace planning {
namespace apa_planner {
// todo: physical dimension is different, unify them in a same physical
// dimension.
struct DpSpeedCost {
  // kappa 限速，路网限速，obstacle限速；
  double speed_limit_cost;
  double acc_cost;
  double jerk_cost;
  // 中途停车
  double stopover_cost;

  double parent_cost;
  // 如果没有探索到这个节点，那么cost无穷大
  double total_cost;

  void Clear();
  void DebugCost() const;
};
}  // namespace apa_planner
}  // namespace planning

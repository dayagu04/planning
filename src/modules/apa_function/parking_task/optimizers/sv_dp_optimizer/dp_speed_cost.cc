#include "dp_speed_cost.h"

#include "log_glog.h"

namespace planning {

#define DP_SPEED_INVALID_COST (10000.0)

void DpSpeedCost::DebugCost() const {
  ILOG_INFO << "speed limit cost = " << speed_limit_cost
            << ",acc_cost = " << acc_cost << ",jerk cost = " << jerk_cost
            << ",stopover cost = " << stopover_cost
            << ",parent cost = " << parent_cost
            << ",total cost = " << total_cost;
  return;
}

void DpSpeedCost::Clear() {
  speed_limit_cost = DP_SPEED_INVALID_COST;
  acc_cost = DP_SPEED_INVALID_COST;
  jerk_cost = DP_SPEED_INVALID_COST;
  stopover_cost = DP_SPEED_INVALID_COST;
  parent_cost = DP_SPEED_INVALID_COST;

  total_cost = DP_SPEED_INVALID_COST;
  return;
}

}  // namespace planning

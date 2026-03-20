#pragma once

#include <cstdint>
#include <string>

#include "ego_planning_config.h"
#include "session.h"
#include "trajectory1d/second_order_time_optimal_trajectory.h"
namespace planning {

class YieldFrontVehicleSafeFunction {
 public:
  YieldFrontVehicleSafeFunction(framework::Session* session,
                                const StGraphSearcherConfig config);
  ~YieldFrontVehicleSafeFunction() = default;

  SecondOrderTimeOptimalTrajectory GenerateMaxDecelerationCurve(
      const double front_node_vel);

  bool IsYieldSafe(const SecondOrderTimeOptimalTrajectory& max_decrease_st,
                   const int64_t target_lane_front_node_id);

 private:
  StGraphSearcherConfig config_;
  framework::Session* session_;
};
}  // namespace planning
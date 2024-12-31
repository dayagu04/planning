#pragma once
#include <memory.h>

#include <memory>

#include "lon_target_maker.pb.h"
#include "target.h"
#include "trajectory1d/second_order_time_optimal_trajectory.h"
#include "trajectory1d/trajectory1d.h"
#include "trajectory1d/variable_coordinate_time_optimal_trajectory.h"

namespace planning {
class NeighborTarget : public Target {
 public:
  NeighborTarget(const SpeedPlannerConfig& config, framework::Session* session);
  ~NeighborTarget() = default;

 private:
  void GenerateNeighborTarget();

  void GenerateNeighborTargetCurve();

  void AddNeighborTargetDataToProto();

  //   void DetermineStateLimitParams(const int32_t yield_agent_id,
  //                                  const int32_t overtake_agent_id,
  //                                  StateLimit* const state_limit);

 private:
  TargetType neighbor_target_type_ = TargetType::kNotSet;
  std::unique_ptr<VariableCoordinateTimeOptimalTrajectory>
      neighbor_target_curve_;
  std::unique_ptr<SecondOrderTimeOptimalTrajectory>
      neighbor_target_curve_lower_bound_;
  planning::common::NeighborTarget neighbor_target_pb_;
};
}  // namespace planning
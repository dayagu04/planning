#pragma once

#include "src/modules/common/agent/agent.h"
#include "tasks/task.h"
#include "trajectory1d/second_order_time_optimal_trajectory.h"

namespace planning {

class LongitudinalDecisionDecider : public Task {
 public:
  LongitudinalDecisionDecider(const EgoPlanningConfigBuilder *config_builder,
                              framework::Session *session);
  ~LongitudinalDecisionDecider() override = default;

  bool Execute() override;

  void Reset();  // 后续考虑抽象到Task中

 private:
  void DetermineKinematicBoundForCruiseScenario();

  void UpdateInvadeNeighborResults();

  // only consider lane change execution stage
  void UpdateLaneChangeNeighborResults();

  bool ConstructNeighborLaneStGraph(const agent::Agent *const neighbor_agent);

  void MakeDebugMessage();

  double CalculateAgentsAverageSpeedAroundEgo() const;

  bool IsMaxAccCurvSafeInStGraph() const;

  SecondOrderTimeOptimalTrajectory GenerateMaxDecelerationCurve(
      const PlanningInitPoint &init_point) const;

 private:
  EgoPlanningConfig config_;
  // <counter, flag>
  std::pair<int32_t, int32_t> cruise_accelerate_count_{0, 0};
  int32_t plan_points_num_ = 26;
  double plan_time_ = 5.0;
  double dt_ = 0.2;

  static constexpr double kCruiseSpeedMinThd = 60.0 / 3.6;
  static constexpr double kEgoSpeedWithCruiseSpeedDiffThd = 15.0 / 3.6;
  static constexpr double kEgoPreviewTimeThd = 6.0;
  static constexpr double kPreviewDistanceStep = 2.0;
  static constexpr double kMaxCurvThd = 0.01;
  static constexpr double kAgentsAverageSpeedRatioByCruiseThd = 0.7;
  static constexpr int32_t kIncreaseAccBoundCountThd = 3;
  static constexpr double kCruiseAccelerateThd = 1.0;

  static constexpr double kAroundEgoLateralDistanceThd = 5.4;
  static constexpr double kAroundEgoLongitudinalPreviewTimeThd = 3.0;
  static constexpr double kAroundEgoLongitudinalBackwardTimeThd = 1.0;
};

}  // namespace planning

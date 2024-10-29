#pragma once

#include "tasks/task.h"

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

  void MakeDebugMessage();

  double CalculateAgentsAverageSpeedAroundEgo() const;

 private:
  EgoPlanningConfig config_;
  // <counter, flag>
  std::pair<int32_t, int32_t> cruise_accelerate_count_{0, 0};
  int32_t plan_points_num_ = 0;
  double plan_time_ = 0.0;
  double dt_ = 0.0;

  static constexpr double kCruiseSpeedMinThd = 60.0 / 3.6;
  static constexpr double kEgoSpeedWithCruiseSpeedDiffThd = 15.0 / 3.6;
  static constexpr double kEgoPreviewTimeThd = 6.0;
  static constexpr double kPreviewDistanceStep = 2.0;
  static constexpr double kMaxCurvThd = 0.01;
  static constexpr double kAgentsAverageSpeedRatioByCruiseThd = 0.7;
};

}  // namespace planning

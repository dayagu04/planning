#pragma once
#include "avoid_obstacle_maintainer.h"
#include "avoid_obstacle_maintainer5V.h"
#include "lateral_offset_calculator.h"
#include "lateral_offset_calculatorV2.h"
#include "session.h"
#include "tasks/task.h"
namespace planning {

class LateralOffsetDecider : public Task {
 public:
  explicit LateralOffsetDecider(const EgoPlanningConfigBuilder *config_builder,
                                framework::Session *session);

  virtual ~LateralOffsetDecider() = default;

  bool Execute() override;

  bool ExecuteTest(bool pipeline_test);

 private:
  void SmoothLateralOffset(double in_lat_offset);
  void SaveDebugInfo();
  void Reset();
  void GenerateOutput();
  LateralOffsetDeciderConfig config_;
  AvoidObstacleMaintainer avoid_obstacle_maintainer_;
  LateralOffsetCalculator lateral_offset_calculator_;

  AvoidObstacleMaintainer5V avoid_obstacle_maintainer5v_;
  LateralOffsetCalculatorV2 lateral_offset_calculatorv2_;

  double lateral_offset_ = 0.0;
};

}  // namespace planning

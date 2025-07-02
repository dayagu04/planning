#pragma once
#include "avoid_obstacle_maintainer5V.h"
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
  void CheckAvoidObstaclesDecision();
  bool IsObstacleDecisionSwitch(
      LatObstacleDecisionType last_decision,
      LatObstacleDecisionType current_decision);
  void Reset();
  void GenerateOutput();
  LateralOffsetDeciderConfig config_;

  AvoidObstacleMaintainer5V avoid_obstacle_maintainer5v_;
  LateralOffsetCalculatorV2 lateral_offset_calculatorv2_;

  LatObstacleDecisionType last_first_obstacle_decision_ = LatObstacleDecisionType::IGNORE;
  LatObstacleDecisionType last_second_obstacle_decision_ = LatObstacleDecisionType::IGNORE;
  uint last_first_obstacle_id_ = 0;
  uint last_second_obstacle_id_ = 0;

  double lateral_offset_ = 0.0;
};

}  // namespace planning

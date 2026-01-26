#pragma once
#include "avoid_obstacle_maintainer5V.h"
#include "lateral_offset_calculatorV2.h"
#include "session.h"
#include "tasks/task.h"
#include "lateral_offset_decider_utils.h"

namespace planning {

enum class HMIAvoidState : int { RUNNING, EXITING, COOLDOWN, IDLE };

struct HMIAvoidParam {
  int cooldown_count = 0;
  int exit_count = 0;
  int avoid_id = -1;
  int avoid_direction = 0;
};

class LateralOffsetDecider : public Task {
 public:
  explicit LateralOffsetDecider(const EgoPlanningConfigBuilder *config_builder,
                                framework::Session *session);

  virtual ~LateralOffsetDecider() = default;

  bool Execute() override;

  bool ExecuteTest(bool pipeline_test);

 private:
  void CalLaneInfo();
  void CalculateNormalLateralOffsetThreshold(const std::shared_ptr<VirtualLane> flane);
  double CalLaneWidth(const std::shared_ptr<VirtualLane> flane);
  void SmoothLateralOffset(double in_lat_offset);
  void SaveDebugInfo();
  void CheckAvoidObstaclesDecision();
  bool IsObstacleDecisionSwitch(LatObstacleDecisionType last_decision,
                                LatObstacleDecisionType current_decision);
  void Reset();
  void GenerateOutput();
  bool IsStartRunning();
  bool IsStopRunning();
  LateralOffsetDeciderConfig config_;
  AvoidObstacleMaintainer5V avoid_obstacle_maintainer5v_;
  LateralOffsetCalculatorV2 lateral_offset_calculatorv2_;

  LatObstacleDecisionType last_first_obstacle_decision_ =
      LatObstacleDecisionType::IGNORE;
  LatObstacleDecisionType last_second_obstacle_decision_ =
      LatObstacleDecisionType::IGNORE;
  uint last_first_obstacle_id_ = 0;
  uint last_second_obstacle_id_ = 0;

  double lateral_offset_ = 0.0;
  double lane_width_ = 3.8;
  LaneInfo lane_info_;
  LaneInfo last_lane_info_;
  HMIAvoidState current_state_ = HMIAvoidState::IDLE;
  HMIAvoidParam hmi_avoid_param_;
};

}  // namespace planning

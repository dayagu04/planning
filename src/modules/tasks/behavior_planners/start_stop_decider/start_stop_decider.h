#pragma once
#include "basic_types.pb.h"
#include "ego_planning_config.h"
#include "start_stop_decider_output.h"
#include "start_stop_status_manager.h"
#include "tasks/task.h"

namespace planning {

class StartStopDecider : public Task {
 public:
  StartStopDecider(const EgoPlanningConfigBuilder* config_builder,
                   framework::Session* session);
  ~StartStopDecider() = default;

  bool Execute() override;

  void UpdateInput();

  void StopSpeedDecisionProcess();

  void Reset();

  void SaveToSession();

 private:
  StartStopDeciderConfig config_;
  StartStopStatusManager start_stop_status_manager_;
  StopSpeedDecisonInfo stop_speed_decision_info_;
  bool rads_scene_is_completed_ = false;
};

}  // namespace planning
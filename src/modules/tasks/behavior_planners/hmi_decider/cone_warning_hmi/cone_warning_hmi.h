#pragma once
#include "planning_context.h"
#include "session.h"

namespace planning {
class ConeWarningHMIDecider {
  enum class ConeWarningState : int { RUNNING, EXITING, IDLE };

 public:
  ConeWarningHMIDecider(framework::Session* session);
  ~ConeWarningHMIDecider() = default;

  bool Execute();

 private:
  bool HasCipvCone();
  bool HasSpeedLimitCone();
  bool HasConeALC();
  bool IsStartRunning();
  bool IsStopRunning();
  void SaveHmiOutput();
  void Reset();
  bool has_cipv_cone_ = false;
  bool has_speed_limit_cone_ = false;
  bool has_alc_cone_ = false;

  int start_running_count_ = 0;
  int stop_running_count_ = 0;

  ConeWarningState current_state_ = ConeWarningState::IDLE;
  framework::Session* session_;
};
}  // namespace planning
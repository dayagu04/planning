#pragma once
#include "planning_context.h"
#include "session.h"

namespace planning {
class ConstructionWarningHMIDecider {
  enum class ConstructionWarningState : int { RUNNING, EXITING, IDLE };

 public:
  ConstructionWarningHMIDecider(framework::Session* session);
  ~ConstructionWarningHMIDecider() = default;

  bool Execute();

 private:
  bool HasConstruction();
  bool IsStartRunning();
  bool IsStopRunning();
  void SaveHmiOutput();
  bool has_construction_ = false;

  int start_running_count_ = 0;
  int stop_running_count_ = 0;

  ConstructionWarningState current_state_ = ConstructionWarningState::IDLE;
  framework::Session* session_;
};
}  // namespace planning
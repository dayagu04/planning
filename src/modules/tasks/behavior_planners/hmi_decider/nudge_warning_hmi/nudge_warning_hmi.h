#pragma once
#include "planning_context.h"
#include "session.h"

namespace planning {
class NudgeWarningHMIDecider {
  enum class NudgeWarningState : int { RUNNING, EXITING, COOLDOWN, IDLE };

 public:
  NudgeWarningHMIDecider(framework::Session* session);
  ~NudgeWarningHMIDecider() = default;

  bool Execute();
  int avoid_id() { return avoid_id_; }
  int avoid_direction() { return avoid_direction_; }

 private:
  bool IsStartRunning();
  bool IsStopRunning();
  void GenerateHmiOutput();
  void Reset();

  int start_running_count_ = 0;
  int stop_running_count_ = 0;
  int cooldown_count_ = 0;
  NudgeWarningState current_state_ = NudgeWarningState::IDLE;

  std::shared_ptr<ReferencePath> reference_path_ptr_;
  framework::Session* session_;

  int avoid_id_ = -1;
  int avoid_direction_ = 0;
};
}  // namespace planning
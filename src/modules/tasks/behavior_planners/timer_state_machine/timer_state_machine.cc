#include "timer_state_machine.h"

#include "debug_info_log.h"
#include "ifly_time.h"
#include "log.h"
#include "task.h"

namespace planning {
namespace time_state_machine {

TimerStateMachine::TimerStateMachine(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {}

void TimerStateMachine::ResetInnerParam() {}

bool TimerStateMachine::Execute() {
#ifdef X86
  return SimulateExecute();
#else
  return RealExecute();
#endif
}

bool TimerStateMachine::RealExecute() {
  if (max_running_time_ < 0.0) {
    LOG_DEBUG("[TimerStateMachine]: max_running_time < 0.0\n");
    return false;
  }
  // cooling
  if (start_cool_down_time_ >= 0.0) {
    double cool_down_time = IflyTime::Now_s() - start_cool_down_time_;
    if (cool_down_time > 0.0 && cool_down_time < cool_down_time_) {
      status_ = TimerStateMachineStatus::COOL_DOWN;
      return true;
    }
  }
  start_cool_down_time_ = -1.0;
  status_ = TimerStateMachineStatus::NOT_SET;

  bool is_start_timer = IsStartTimerRunning();
  // not trigger
  if (!is_start_timer && current_running_time_ < 0.0) {
    return true;
  }

  // stop trigger
  if (IsStopTimerRunning()) {
    Reset();
    status_ = TimerStateMachineStatus::COOL_DOWN;
    return true;
  }
  // start process
  start_running_time_ =
      start_running_time_ < 0.0 ? IflyTime::Now_s() : start_running_time_;
  current_running_time_ = IflyTime::Now_s() - start_running_time_;
  // end process
  if (current_running_time_ > max_running_time_) {
    if (is_start_timer) {
      // if current frame start, also process.
      // this can make continous process when cool down time is 0.0;
      TimerRunning();
    }
    status_ = TimerStateMachineStatus::COOL_DOWN;
    Reset();
    return true;
  }
  // processing
  status_ = TimerStateMachineStatus::RUNNING;
  return TimerRunning();
}

bool TimerStateMachine::SimulateExecute() {
  auto &planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto frame_num = planning_debug_data->frame_info().frame_num();
  if (max_running_count_ < 0) {
    LOG_DEBUG("[Sim::TimerStateMachine]: max_running_time < 0.0\n");
    return false;
  }
  // cooling
  if (start_cool_down_count_ >= 0) {
    double cool_down_time_count = frame_num - start_cool_down_count_;
    if (cool_down_time_count > 0 && cool_down_time_count < cool_down_count_) {
      status_ = TimerStateMachineStatus::COOL_DOWN;
      return true;
    }
  }
  start_cool_down_count_ = -1;
  status_ = TimerStateMachineStatus::NOT_SET;

  bool is_start_timer = IsStartTimerRunning();
  // not trigger
  if (!is_start_timer && current_running_count_ < 0) {
    return true;
  }

  // stop trigger
  if (IsStopTimerRunning()) {
    Reset();
    status_ = TimerStateMachineStatus::COOL_DOWN;
    return true;
  }
  // start process
  start_running_count_ =
      start_running_count_ < 0 ? frame_num : start_running_count_;
  current_running_count_ = frame_num - start_running_count_;
  // end process
  if (current_running_count_ > max_running_count_) {
    if (is_start_timer) {
      // if current frame start, also process.
      // this can make continous process when cool down time is 0.0;
      TimerRunning();
    }
    status_ = TimerStateMachineStatus::COOL_DOWN;
    Reset();
    return true;
  }
  // processing
  status_ = TimerStateMachineStatus::RUNNING;
  return TimerRunning();
}

void TimerStateMachine::Reset() {
  current_running_time_ = -1.0;
  start_running_time_ = -1.0;
  start_cool_down_time_ = IflyTime::Now_s();
  status_ = TimerStateMachineStatus::NOT_SET;
  ResetInnerParam();
#ifdef X86
  auto &planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto frame_num = planning_debug_data->frame_info().frame_num();
  current_running_count_ = -1.0;
  start_running_count_ = -1.0;
  start_cool_down_count_ = frame_num;
  status_ = TimerStateMachineStatus::NOT_SET;
  ResetInnerParam();
#endif
}

void TimerStateMachine::set_max_running_time(const double max_running_time) {
  max_running_time_ = max_running_time;
}

void TimerStateMachine::set_cool_down_time(const double cool_down_time) {
  cool_down_time_ = cool_down_time;
}

void TimerStateMachine::set_max_running_count(const double max_running_count) {
  max_running_count_ = max_running_count;
}

void TimerStateMachine::set_cool_down_count(const double cool_down_count) {
  cool_down_count_ = cool_down_count;
}

const TimerStateMachineStatus TimerStateMachine::status() const {
  return status_;
}

std::string TimerStateMachine::StatusToString(
    const TimerStateMachineStatus status) {
  std::string status_str;
  switch (status) {
    case TimerStateMachineStatus::COOL_DOWN:
      status_str = "status cool_down";
      break;
    case TimerStateMachineStatus::RUNNING:
      status_str = "status running";
      break;
    default:
      status_str = "status not set";
  }
  return status_str;
}

}  // namespace time_state_machine
}  // namespace planning
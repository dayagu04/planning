#pragma once

#include <sstream>

#include "tasks/task.h"

namespace planning {
namespace time_state_machine {

enum class TimerStateMachineStatus {
  NOT_SET = 1,
  RUNNING = 2,
  COOL_DOWN = 3,
};

class TimerStateMachine : public Task {
 public:
  TimerStateMachine(const EgoPlanningConfigBuilder *config_builder,
                    framework::Session *session);

  virtual ~TimerStateMachine() = default;

  void set_max_running_time(const double max_process_time);

  void set_cool_down_time(const double cool_down_time);

  void set_max_running_count(const double max_process_count);

  void set_cool_down_count(const double cool_down_count);

  const TimerStateMachineStatus status() const;

  void Reset();

  static std::string StatusToString(const TimerStateMachineStatus status);

 protected:
  bool Execute() override;

  bool SimulateExecute();

  bool RealExecute();

  virtual bool IsStartTimerRunning() = 0;

  virtual bool IsStopTimerRunning() = 0;

  virtual bool TimerRunning() = 0;

  virtual void ResetInnerParam();

 protected:
  double max_running_time_ = -1.0;
  double start_running_time_ = -1.0;
  double current_running_time_ = -1.0;
  double start_cool_down_time_ = -1.0;
  double cool_down_time_ = -1.0;

  // simulate
  int max_running_count_ = -1;
  int start_running_count_ = -1;
  int current_running_count_ = -1;
  int start_cool_down_count_ = -1;
  int cool_down_count_ = -1;

  TimerStateMachineStatus status_ = TimerStateMachineStatus::NOT_SET;
};

}  // namespace time_state_machine
}  // namespace planning
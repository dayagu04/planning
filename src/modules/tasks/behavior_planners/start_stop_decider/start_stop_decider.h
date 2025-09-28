#pragma once
#include <cstdint>

#include "basic_types.pb.h"
#include "ego_planning_config.h"
#include "start_stop_decider_output.h"
#include "tasks/task.h"

namespace planning {

class StartStopDecider : public Task {
 public:
  StartStopDecider(const EgoPlanningConfigBuilder* config_builder,
                   framework::Session* session);
  ~StartStopDecider() = default;

  bool Execute() override;

 private:
  void UpdateInput();
  void UpdateStartStopStatus();
  void SaveToSession();
  double CalculateStopDistance();
  bool CanTransitionFromStopToStart();
  bool CanTransitionFromStartToCruise();
  bool CanTransitionToStop();

  StartStopDeciderConfig config_;

  // ego state info
  common::StartStopInfo ego_start_stop_info_;
  double planning_init_state_vel_ = 33.33;

  // cipv info
  double cipv_vel_frenet_ = 0.0;
  double cipv_relative_s_ = 0.0;
  double cipv_relative_s_prev_ = 0.0;
  int32_t cipv_id_ = -1;
  bool cipv_is_large_ = false;

  // calculated stop distance
  double stop_distance_ = 3.5;

  // rads scene info
  bool rads_scene_is_completed_ = false;
  bool is_ego_reverse_ = false;
};

}  // namespace planning
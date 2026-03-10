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

  // HPP（前进记忆泊车）专用停车逻辑，整合自原HppStopDecider
  void ExecuteHppStopLogic();
  bool IsHppStopConditionMet(double dist_to_dest, double dist_to_slot,
                             double ego_velocity);
  bool UpdateHppTargetInfo(double* dist_to_dest, double* dist_to_slot);

  // 判断当前CIPV是否为指定类型的虚拟终点障碍物
  bool IsCipvVirtualDestination(int expected_agent_id) const;

  // HPP停车逻辑内部辅助方法
  bool ValidateHppPreconditions();
  struct HppStopConditions {
    double dist_to_dest = 0.0;
    double dist_to_slot = 0.0;
    double ego_velocity = 0.0;
    bool is_reached_target_slot = false;
    bool is_reached_target_dest = false;
    bool is_stop_condition_met = false;
  };
  HppStopConditions EvaluateHppStopConditions();
  bool UpdateHppStopFrameCount(bool is_stop_condition_met);
  void SaveHppStopOutput(const HppStopConditions& conditions,
                         bool is_stopped_at_destination);

  StartStopDeciderConfig config_;
  // HPP停车条件配置（与RADS停车条件不同，需单独配置）
  HppStopDeciderConfig hpp_stop_config_;

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

  // RADS（倒车）场景完成标志
  bool rads_scene_is_completed_ = false;
  bool is_ego_reverse_ = false;

  // HPP（前进）场景停车状态（对应原HppStopDecider的内部状态）
  bool last_frame_hpp_stop_condition_met_ = false;
  int hpp_stop_frame_count_ = 0;
  // 本帧是否满足 HPP 停车到位条件（用于 CanTransitionToStop，含控制超调场景）
  bool hpp_stop_condition_met_this_frame_ = false;

  // fsm request info
  bool stand_wait_ = false;
};

}  // namespace planning
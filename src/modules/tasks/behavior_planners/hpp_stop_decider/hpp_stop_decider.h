#pragma once

#include "tasks/task.h"

namespace planning {

class HppStopDecider : public Task {
 public:
  HppStopDecider(const EgoPlanningConfigBuilder *config_builder,
                 framework::Session *session);
  virtual ~HppStopDecider() = default;

  bool Execute() override;

 private:
  // 基于 reference path 更新到终点和目标车位的信息
  bool UpdateTargetInfoBasedOnReferencePath(
      const std::shared_ptr<ReferencePath>& reference_path);
  
  // 判断是否满足停车条件
  bool IsStopConditionMet(double dist_to_dest, double dist_to_slot, 
                          double ego_velocity);

 private:
  HppStopDeciderConfig config_;

  // 是否到达终点并停车
  bool is_stopped_at_destination_ = false;

  // 是否到达目标停车位
  bool is_reached_target_slot_ = false;

  // 是否到达目标目的地
  bool is_reached_target_dest_ = false;

  // 是否满足停车条件（距离终点<2m，距离轨迹终点<0.5m，速度<0.1）
  bool is_stop_condition_met_ = false;
  
  /****************** internal use ***************/
  // 内部状态：上一帧是否满足停车条件
  bool last_frame_stop_condition_met_ = false;
  // 停车帧数计数（跨帧保持）
  int stop_frame_count_ = 0;
};

}  // namespace planning

#pragma once

#include "hpp_stop_info.h"
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
  HppStopInfo hpp_stop_info_;
  
  /****************** internal use ***************/
  // 内部状态：上一帧是否满足停车条件
  bool last_frame_stop_condition_met_ = false;
  // 停车帧数计数（跨帧保持）
  int stop_frame_count_ = 0;
};

}  // namespace planning

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "tasks/task.h"
#include "session.h"
#include "planning_plan_c.h"
#include "config/basic_type.h"

namespace planning {

class MirrorDecider : public Task {
 public:
  MirrorDecider(const EgoPlanningConfigBuilder *config_builder,
                framework::Session *session);

  virtual ~MirrorDecider() = default;

  bool Execute() override;

 private:

  // 状态变量
  iflyauto::RearViewMirrorSignalType current_mirror_state_ = iflyauto::REAR_VIEW_MIRROR_NONE;
  int unfold_frame_counter_ = 0;
  // 记录value=2（展开）的累计帧数（目标：累计5帧后重置）
  int mirror_unfold_frame_count_{0};
  // 记录上一帧的后视镜指令值（用于判断是否持续为2）
  iflyauto::RearViewMirrorSignalType last_mirror_value_{iflyauto::REAR_VIEW_MIRROR_NONE};

};

}  // namespace planning
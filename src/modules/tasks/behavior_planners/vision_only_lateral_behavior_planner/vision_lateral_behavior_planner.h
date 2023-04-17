#pragma once

#include <assert.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "src/modules/tasks/task.h"
#include "src/modules/tasks/task_basic_types.h"
#include "src/modules/context/lateral_obstacle.h" // TODO(Rui):include object_selector.h

namespace planning {

class VisionLateralBehaviorPlanner : public Task {
  public:
  explicit VisionLateralBehaviorPlanner(
      const EgoPlanningConfigBuilder *config_builder,
      const std::shared_ptr<TaskPipelineContext> &pipeline_context);

  virtual ~VisionLateralBehaviorPlanner() = default;

  bool Execute(planning::framework::Frame *frame) override;

 private:
  bool update_lateral_behavior_planner_output() {}

 private:
  VisionLateralBehaviorPlannerConfig config_;
  planning::framework::Frame *frame_;


};

}  // namespace planning
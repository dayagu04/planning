#pragma once

#include "src/modules/tasks/task_pipeline.h"

namespace planning {

class TaskPipelineVisionOnly final : public TaskPipeline {
 public:
  explicit TaskPipelineVisionOnly(const EgoPlanningConfigBuilder *config_builder,
                              framework::Frame *frame);

  bool Run(const EgoPlanningCandidate &candidate) override;

 protected:
  void CreatePlanningTasks(
      const EgoPlanningConfigBuilder *config_builder) override;

 private:
  EgoPlanningTaskPipelineVisionOnlyConfig config_;
  std::map<std::string, PlanningTaskTypes> version_to_tasks_ = {};
};

}  // namespace planning

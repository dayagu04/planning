#pragma once

#include "tasks/task_pipeline.h"

namespace planning {

class TaskPipelineNormal final : public TaskPipeline {
 public:
  explicit TaskPipelineNormal(const EgoPlanningConfigBuilder *config_builder,
                              framework::Frame *frame);

  bool Run(const EgoPlanningCandidate &candidate) override;

 protected:
  void CreatePlanningTasks(
      const EgoPlanningConfigBuilder *config_builder) override;

 private:
  EgoPlanningTaskPipelineNormalConfig config_;
  std::map<std::string, PlanningTaskTypes> version_to_tasks_ = {};
};

}  // namespace planning

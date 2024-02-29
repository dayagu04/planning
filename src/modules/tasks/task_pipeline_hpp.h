#pragma once

#include "task_pipeline.h"

namespace planning {

class TaskPipelineHpp final : public TaskPipeline {
 public:
  explicit TaskPipelineHpp(const EgoPlanningConfigBuilder *config_builder,
                           framework::Frame *frame);

  bool Run(const EgoPlanningCandidate &candidate) override;

 protected:
  void CreatePlanningTasks(
      const EgoPlanningConfigBuilder *config_builder) override;

 private:
  EgoPlanningTaskPipelineHppConfig config_;
  std::map<std::string, PlanningTaskTypes> version_to_tasks_ = {};
};

}  // namespace planning
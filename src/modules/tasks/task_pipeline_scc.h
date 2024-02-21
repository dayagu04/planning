#pragma once

#include "task_pipeline.h"

namespace planning {

class TaskPipelineScc final : public TaskPipeline {
 public:
  explicit TaskPipelineScc(const EgoPlanningConfigBuilder *config_builder,
                           framework::Frame *frame);

  bool Run(const EgoPlanningCandidate &candidate) override;

 protected:
  void CreatePlanningTasks(
      const EgoPlanningConfigBuilder *config_builder) override;

 private:
  EgoPlanningTaskPipelineSccConfig config_;
  std::map<std::string, PlanningTaskTypes> version_to_tasks_ = {};
};

}  // namespace planning
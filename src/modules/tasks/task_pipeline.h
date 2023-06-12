#pragma once

#include "task.h"

namespace planning {

enum class TaskPipelineType {
  NORMAL = 0,
  VISION_ONLY = 1,
};

class EgoPlanningCandidate;
class TaskPipeline {
 public:
  explicit TaskPipeline(const EgoPlanningConfigBuilder *config_builder, framework::Frame *frame);
  virtual ~TaskPipeline() = default;

  inline const std::string &Name() const { return name_; }

  std::shared_ptr<TaskPipelineContext> get_pipeline_context() const { return pipeline_context_; }

  void SetFrame(framework::Frame *frame) { frame_ = frame; }

  virtual bool Run(const EgoPlanningCandidate &candidate) = 0;

  static std::shared_ptr<TaskPipeline> Make(const TaskPipelineType &task_type,
                                            const EgoPlanningConfigBuilder *config_builder, framework::Frame *frame);

 protected:
  virtual void CreatePlanningTasks(const EgoPlanningConfigBuilder *config_builder) = 0;

  std::shared_ptr<TaskPipelineContext> pipeline_context_;
  std::string name_;
  std::vector<std::pair<TaskType, std::shared_ptr<Task>>> planning_tasks_;
  framework::Frame *frame_;

 private:
  EgoPlanningTaskPipelineConfig config_;
};

}  // namespace planning
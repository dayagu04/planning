#include "src/modules/tasks/task_pipeline.h"

#include "src/modules/tasks/task_pipeline_normal.h"

namespace planning {

TaskPipeline::TaskPipeline(const EgoPlanningConfigBuilder *config_builder,
                           framework::Frame *frame)
    : pipeline_context_(std::make_shared<TaskPipelineContext>()) {
  frame_ = frame;
  config_ = config_builder->cast<EgoPlanningTaskPipelineConfig>();
}

std::shared_ptr<TaskPipeline> TaskPipeline::Make(
    const TaskPipelineType &task_pipeline_type,
    const EgoPlanningConfigBuilder *config_builder, framework::Frame *frame) {
  switch (task_pipeline_type) {
    case TaskPipelineType::NORMAL: {
      return std::make_shared<TaskPipelineNormal>(config_builder, frame);
    }
    // case TaskPipelineType::VISION_ONLY: {
    //   return std::make_shared<TaskPipelineNormal>(config_builder, frame);
    // }
    default: { /*LOG_ERROR*/
      return nullptr;
    }
  }
}

// bool TaskPipeline::run(const std::shared_ptr<ReferencePath>&
// reference_path) {}

}  // namespace planning

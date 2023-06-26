#include "vision_only_adas_function_task.h"

namespace planning {

VisionOnlyAdasFunctionTask::VisionOnlyAdasFunctionTask(
    const EgoPlanningConfigBuilder *config_builder,
    const std::shared_ptr<TaskPipelineContext> &pipeline_context)
    : Task(config_builder, pipeline_context) {
  config_ = config_builder->cast<VisionOnlyAdasFunctionTaskConfig>();
  name_ = "VisionOnlyAdasFunctionTask";
}

bool VisionOnlyAdasFunctionTask::Execute(framework::Frame *frame) {
  if (Task::Execute(frame) == false) {
    return false;
  }

  // lkas_function test
  auto lkas_function_ptr =
      frame->session()->mutable_planning_context()->lane_keep_assit_function();
  lkas_function_ptr->RunOnce();
  // ihc_function test
  auto ihc_function_ptr = frame->session()
                              ->mutable_planning_context()
                              ->intelligent_headlight_control_function();
  ihc_function_ptr->RunOnce();
  // tsr_function test
  auto tsr_function_ptr = frame->session()
                              ->mutable_planning_context()
                              ->traffic_sign_recognition_function();
  tsr_function_ptr->RunOnce();

  return TRUE;
}

}  // namespace planning
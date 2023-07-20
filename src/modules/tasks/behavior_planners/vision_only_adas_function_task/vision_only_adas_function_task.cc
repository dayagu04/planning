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
  double start_time = IflyTime::Now_ms();
  if (Task::Execute(frame) == false) {
    return false;
  }
  double time_0 = IflyTime::Now_ms();
  LOG_DEBUG("adas_config cost is [%f]ms:\n", (time_0 - start_time));

  // lkas_function test
  auto lkas_function_ptr =
      frame->session()->mutable_planning_context()->lane_keep_assit_function();
  lkas_function_ptr->RunOnce();
  double time_1 = IflyTime::Now_ms();
  LOG_DEBUG("lka_function cost is [%f]ms:\n", (time_1 - time_0));
  JSON_DEBUG_VALUE("lka_function time cost is:", (time_1 - time_0));
  // ihc_function test
  auto ihc_function_ptr = frame->session()
                              ->mutable_planning_context()
                              ->intelligent_headlight_control_function();
  ihc_function_ptr->RunOnce();
  double time_2 = IflyTime::Now_ms();
  LOG_DEBUG("ihc_function cost is [%f]ms:\n", (time_2 - time_1));
  JSON_DEBUG_VALUE("ihc_function time cost is:", (time_2 - time_1));
  // tsr_function test
  auto tsr_function_ptr = frame->session()
                              ->mutable_planning_context()
                              ->traffic_sign_recognition_function();
  tsr_function_ptr->RunOnce();
  double time_3 = IflyTime::Now_ms();
  LOG_DEBUG("tsr_function cost is [%f]ms:\n", (time_3 - time_2));
  JSON_DEBUG_VALUE("tsr_function time cost is:", (time_3 - time_2));
  return true;
}

}  // namespace planning
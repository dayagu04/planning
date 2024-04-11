#include "base_task_pipeline.h"

#include "planning_context.h"

namespace planning {

BaseTaskPipeline::BaseTaskPipeline(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : session_(session) {}

void BaseTaskPipeline::AddErrorInfo(const std::string &task_name) {
  auto status_info =
      session_->mutable_planning_context()->mutable_status_info();
  status_info.error_info = "ERROR|" + task_name + "|" + status_info.error_info;
  printf("%s| FAIL =============== \n", task_name.c_str());
}

}  // namespace planning
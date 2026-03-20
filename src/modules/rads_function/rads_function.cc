#include "rads_function.h"

#include "src/task_pipeline_rads.h"

namespace planning {

RadsFunction::RadsFunction(framework::Session *session)
    : BaseFunction(session) {
  auto config_builder = session->environmental_model().rads_config_builder();
  const EgoPlanningConfig ego_config =
      config_builder->cast<planning::EgoPlanningConfig>();
  // SCC_PLANNER_V3: Use agents' prediction info
  if (ego_config.planner_type ==
      planning::context::PlannerType::SCC_PLANNER_V3) {
    task_pipeline_ =
        std::make_unique<TaskPipelineRADS>(config_builder, session);
  }
}

bool RadsFunction::Reset() {
  task_pipeline_.reset(nullptr);

  return true;
}

bool RadsFunction::Plan() { return task_pipeline_->Run(); }

}  // namespace planning
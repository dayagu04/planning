#include "noa_function.h"

#include "ego_planning_config.h"
#include "src/longtime_noa_task_pipeline.h"
#include "src/realtime_noa_task_pipeline.h"

namespace planning {
NoaFunction::NoaFunction(framework::Session *session) : BaseFunction(session) {
  auto config_builder = session->environmental_model().highway_config_builder();

  EgoPlanningConfig ego_config =
      config_builder->cast<planning::EgoPlanningConfig>();
  if (ego_config.planner_type == planning::context::PlannerType::SCC_PLANNER) {
    task_pipeline_ =
        std::make_unique<LongtimeNoaTaskPipeline>(config_builder, session);
  } else {
    task_pipeline_ =
        std::make_unique<RealtimeNoaTaskPipeline>(config_builder, session);
  }
}

bool NoaFunction::Reset() {
  task_pipeline_.reset(nullptr);

  return true;
}

bool NoaFunction::Plan() { return task_pipeline_->Run(); }

}  // namespace planning
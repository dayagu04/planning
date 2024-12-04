#include "scc_function.h"

#include "src/longtime_task_pipeline_v3.h"
#include "src/longtime_task_pipeline_v2.h"
#include "src/realtime_task_pipeline_v1.h"

namespace planning {

SccFunction::SccFunction(framework::Session *session) : BaseFunction(session) {
  auto config_builder = session->environmental_model().highway_config_builder();
  const EgoPlanningConfig ego_config =
      config_builder->cast<planning::EgoPlanningConfig>();
  if (ego_config.planner_type ==
      planning::context::PlannerType::SCC_PLANNER_V3) {
    // SCC_PLANNER_V3: Use agents' prediction info
    task_pipeline_ =
        std::make_unique<LongTimeTaskPipelineV3>(config_builder, session);
  } else if (ego_config.planner_type ==
             planning::context::PlannerType::SCC_PLANNER_V2) {
    // SCC_PLANNER_V2: Only use localization info without agents' prediction
    // info
    task_pipeline_ =
        std::make_unique<LongTimeTaskPipelineV2>(config_builder, session);
  } else {
    // Default: No localization info, no agents' prediction info
    task_pipeline_ =
        std::make_unique<RealtimeSccTaskPipeline>(config_builder, session);
  }
}

bool SccFunction::Reset() {
  task_pipeline_.reset(nullptr);

  return true;
}

bool SccFunction::Plan() { return task_pipeline_->Run(); }

}  // namespace planning
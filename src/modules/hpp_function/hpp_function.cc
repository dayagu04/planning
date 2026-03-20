#include "hpp_function.h"

#include "src/hpp_task_pipeline.h"

namespace planning {
HppFunction::HppFunction(framework::Session *session) : BaseFunction(session) {
  auto config_builder = session->environmental_model().hpp_config_builder();
  task_pipeline_ = std::make_unique<HppTaskPipeline>(config_builder, session);
}

bool HppFunction::Reset() {
  task_pipeline_.reset(nullptr);

  return true;
}

bool HppFunction::Plan() { return task_pipeline_->Run(); }

}  // namespace planning
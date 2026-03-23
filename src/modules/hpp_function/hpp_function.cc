#include "hpp_function.h"

#include "src/hpp_task_pipeline.h"

namespace planning {
HppFunction::HppFunction(framework::Session *session) : BaseFunction(session) {
  auto config_builder = session->environmental_model().hpp_config_builder();
  task_pipeline_ = std::make_unique<HppTaskPipeline>(config_builder, session);
}

bool HppFunction::Reset() {
  const auto config_builder =
      session_->environmental_model().hpp_config_builder();
  task_pipeline_ = std::make_unique<HppTaskPipeline>(config_builder, session_);

  // TODO: 待所有 task 的 Reset 函数统一后，通过调用 pipeline::Reset 进行重置
  //if (!task_pipeline_->Reset()) {
  //  return false;
  //}

  return true;
}

bool HppFunction::Plan() { return task_pipeline_->Run(); }

}  // namespace planning
#include "nsa_function.h"

#include "src/nsa_task_pipeline.h"

namespace planning {
NsaFunction::NsaFunction(framework::Session *session) : BaseFunction(session) {
  auto config_builder = session->environmental_model().nsa_config_builder();
  task_pipeline_ = std::make_unique<NsaTaskPipeline>(config_builder, session);
}

bool NsaFunction::Reset() {
  task_pipeline_.reset(nullptr);

  return true;
}

bool NsaFunction::Plan() { return task_pipeline_->Run(); }

}  // namespace planning
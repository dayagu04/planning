#include "modules/apa_module.h"

namespace planning {
namespace modules {

planning::framework::BaseModule* ApaPlanningModule::clone() const {
  return nullptr;
}

int ApaPlanningModule::init(const ::google::protobuf::Message* config,
    planning::framework::Session* session) {
  return 0;
}

int ApaPlanningModule::reset(const ::google::protobuf::Message* config) {
  return 0;
}

void ApaPlanningModule::compute(framework::Frame* frame) {
  planner_dispatcher_->Update(frame);
}

} // namespace modules
} // namespace planning
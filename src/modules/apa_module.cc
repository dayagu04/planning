#include "apa_module.h"

namespace planning {
namespace modules {

ApaPlanningModule::ApaPlanningModule() {
  planner_dispatcher_ = std::make_unique<apa_planner::ApaPlannerDispatcher>();
}

planning::framework::BaseModule* ApaPlanningModule::clone() const {
  return nullptr;
}

bool ApaPlanningModule::init(const ::google::protobuf::Message* config,
    planning::framework::Session* session) {
  return true;
}

bool ApaPlanningModule::reset(const ::google::protobuf::Message* config) {
  return true;
}

bool ApaPlanningModule::compute(framework::Frame* frame) {
  bool success = false;
  success = planner_dispatcher_->Update(frame);
  return success;
}

} // namespace modules
} // namespace planning
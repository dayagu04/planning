#include "apa_module.h"

#include "apa_plan_interface.h"

namespace planning {
namespace modules {

ApaPlanningModule::ApaPlanningModule() {
  apa_interface_ptr_ = std::make_unique<apa_planner::ApaPlanInterface>();
  apa_interface_ptr_->Init();
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
  return apa_interface_ptr_->UpdateFrame(frame);
}

}  // namespace modules
}  // namespace planning
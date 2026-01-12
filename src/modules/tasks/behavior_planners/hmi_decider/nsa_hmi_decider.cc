#include "nsa_hmi_decider.h"

namespace planning {

NSAHMIDecider::NSAHMIDecider(const EgoPlanningConfigBuilder* config_builder,
    framework::Session* session) : HMIDecider(config_builder, session) {
  name_ = "NSAHMIDecider";
  narrow_space_hmi_decider_ = std::make_shared<NarrowSpaceHMIDecider>(session);
}

bool NSAHMIDecider::Execute() {
  auto planning_hmi_info = session_->mutable_planning_context()->mutable_planning_hmi_info();
  if (planning_hmi_info == nullptr) {
    return false;
  }
  narrow_space_hmi_decider_->Execute();
  return true;
}
}  // namespace planning
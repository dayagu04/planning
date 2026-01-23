#include "rads_hmi_decider.h"

namespace planning {

RADSHMIDecider::RADSHMIDecider(const EgoPlanningConfigBuilder* config_builder,
    framework::Session* session) : HMIDecider(config_builder, session) {
  name_ = "RADSHMIDecider";
  lateral_avoid_hmi_decider_ = std::make_shared<LateralAvoidHMIDecider>(session);
  obstacle_brake_hmi_decider_ = std::make_shared<ObstacleBrakeHMIDecider>(session, config_);
}

bool RADSHMIDecider::Execute() {
  auto planning_hmi_info = session_->mutable_planning_context()->mutable_planning_hmi_info();
  if (planning_hmi_info == nullptr) {
    return false;
  }
  lateral_avoid_hmi_decider_->Execute();
  obstacle_brake_hmi_decider_->Execute();
  return true;
}
}  // namespace planning
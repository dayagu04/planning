#include "hmi_decider.h"

namespace planning {
HMIDecider::HMIDecider(const EgoPlanningConfigBuilder* config_builder,
                       framework::Session* session)
    : Task(config_builder, session) {
  name_ = "HMIDecider";

  cone_warning_hmi_decider_ = std::make_shared<ConeWarningHMIDecider>(session);
  construction_warning_hmi_decider_ = std::make_shared<ConstructionWarningHMIDecider>(session);
}

bool HMIDecider::Execute() {
  cone_warning_hmi_decider_->Execute();
  construction_warning_hmi_decider_->Execute();
  return true;
}
}  // namespace planning
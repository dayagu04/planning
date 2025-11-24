#include "hmi_decider.h"

namespace planning {
HMIDecider::HMIDecider(const EgoPlanningConfigBuilder* config_builder,
                       framework::Session* session)
    : Task(config_builder, session) {
  name_ = "HMIDecider";
  config_ = config_builder->cast<HmiDeciderConfig>();
  cone_warning_hmi_decider_ =
      std::make_shared<ConeWarningHMIDecider>(session);
  construction_warning_hmi_decider_ = std::make_shared<ConstructionWarningHMIDecider>(session);
  lane_change_hmi_decider_ = std::make_shared<LaneChangeHmiDecider>(session);
  longitudinal_hmi_decider_ = std::make_shared<LongitudinalHmiDecider>(session, config_);
}

bool HMIDecider::Execute() {
  cone_warning_hmi_decider_->Execute();
  construction_warning_hmi_decider_->Execute();
  lane_change_hmi_decider_->Execute();
  longitudinal_hmi_decider_->Execute();
  return true;
}
}  // namespace planning
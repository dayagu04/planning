#include "scc_hmi_decider.h"

namespace planning {

SCCHMIDecider::SCCHMIDecider(const EgoPlanningConfigBuilder* config_builder,
                             framework::Session* session)
    : HMIDecider(config_builder, session) {
  name_ = "SCCHMIDecider";
  cone_warning_hmi_decider_ = std::make_shared<ConeWarningHMIDecider>(session);
  construction_warning_hmi_decider_ =
      std::make_shared<ConstructionWarningHMIDecider>(session, config_);
  lane_change_hmi_decider_ = std::make_shared<LaneChangeHmiDecider>(session);
  longitudinal_hmi_decider_ =
      std::make_shared<LongitudinalHmiDecider>(session, config_);
  split_select_hmi_decider_ = std::make_shared<SplitSelectHmiDecider>(session);
  nudge_warning_hmi_decider_ =
      std::make_shared<NudgeWarningHMIDecider>(session);
}

bool SCCHMIDecider::Execute() {
  if (cone_warning_hmi_decider_) {
    cone_warning_hmi_decider_->Execute();
  }
  if (construction_warning_hmi_decider_) {
    construction_warning_hmi_decider_->Execute();
  }
  if (lane_change_hmi_decider_) {
    lane_change_hmi_decider_->Execute();
  }
  if (longitudinal_hmi_decider_) {
    longitudinal_hmi_decider_->Execute();
  }
  if (split_select_hmi_decider_) {
    split_select_hmi_decider_->Execute();
  }
  if (nudge_warning_hmi_decider_) {
    nudge_warning_hmi_decider_->Execute();
  }
  return true;
}
}  // namespace planning
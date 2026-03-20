#include "hmi_decider.h"

namespace planning {

HMIDecider::HMIDecider(const EgoPlanningConfigBuilder* config_builder,
                       framework::Session* session)
    : Task(config_builder, session) {
  config_ = config_builder->cast<HmiDeciderConfig>();
}

bool HMIDecider::Execute() { return true; }

}  // namespace planning
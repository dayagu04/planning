#include "cipv_lost_prohibit_start_decider.h"

namespace planning {
CipvLostProhibitStartDecider::CipvLostProhibitStartDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  name_ = "CipvLostProhibitStartDecider";
}
bool CipvLostProhibitStartDecider::Execute() {

  return true;
}
}  // namespace planning

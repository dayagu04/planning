#include "target_maker.h"

#include "follow_target.h"

namespace planning {

TargetMaker::TargetMaker(const EgoPlanningConfigBuilder *config_builder,
                         framework::Session *session) {
  config_ = config_builder->cast<SpeedPlannerConfig>();
  session_ = session;
}

bool TargetMaker::Run() {
  // 1. cruise target
  // CruiseTarget cruise_target(config_, session_);

  // 2. follow target
  FollowTarget follow_target(config_, session_);

  // 3. overtake target
  // OvertakeTarget overtake_target(config_, session_);

  // 4. neighbor target
  // NeighborTarget neighbor_target(config_, session_);

  // 5. caution target
  // CautionTarget caution_target(config_, session_);

  return true;
}

}  // namespace planning
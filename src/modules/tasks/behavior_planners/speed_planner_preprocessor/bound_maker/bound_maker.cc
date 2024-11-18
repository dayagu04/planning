#include "bound_maker.h"

namespace planning {

BoundMaker::BoundMaker(const SpeedPlannerConfig& speed_planning_config,
                       framework::Session *session) {}

common::Status BoundMaker::Run() { return common::Status::OK(); }

}  // namespace planning
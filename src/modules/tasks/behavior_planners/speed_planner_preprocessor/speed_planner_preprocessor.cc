#include "speed_planner_preprocessor.h"

namespace planning {
SpeedPlannerPreProcessor::SpeedPlannerPreProcessor(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  name_ = "SpeedPlannerPreProcessor";
  target_maker_ = std::make_unique<TargetMaker>(config_builder, session);
  bound_maker_ = std::make_unique<BoundMaker>(config_builder, session);
  weight_maker_ =
      std::make_unique<WeightMaker>(config_builder, session, *target_maker_);
}

bool SpeedPlannerPreProcessor::Execute() {
  target_maker_->Run();
  bound_maker_->Run();
  weight_maker_->Run();
  return true;
}
}  // namespace planning
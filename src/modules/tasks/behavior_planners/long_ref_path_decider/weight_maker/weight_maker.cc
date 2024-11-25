#include "weight_maker.h"

namespace planning {

WeightMaker::WeightMaker(const SpeedPlannerConfig &speed_planning_config,
                         framework::Session *session,
                         const TargetMaker &target_maker)
    : speed_planning_config_(speed_planning_config), session_(session) {
  dt_ = speed_planning_config_.dt;
  plan_time_ = speed_planning_config_.planning_time;
  plan_points_num_ = static_cast<int32_t>(plan_time_ / dt_) + 1;
}

common::Status WeightMaker::Run() {
  LOG_DEBUG("=======LongRefPathDecider: WeightMaker======= \n");
  // MakeSWeight(target_maker);
  // MakeVWeight(target_maker);
  // MakeAccWeight();
  // MakeJerkWeight();

  return common::Status::OK();
}
}  // namespace planning
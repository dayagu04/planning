#include "bound_maker.h"

namespace planning {

BoundMaker::BoundMaker(const SpeedPlannerConfig& speed_planning_config,
                       framework::Session* session)
    : speed_planning_config_(speed_planning_config), session_(session) {
  dt_ = speed_planning_config_.dt;
  plan_time_ = speed_planning_config_.planning_time;
  plan_points_num_ = static_cast<int32_t>(plan_time_ / dt_) + 1;
}

common::Status BoundMaker::Run() {
  LOG_DEBUG("=======SpeedPlannerPreProcessor: BoundMaker======= \n");
  // 1. s bound
  // MakeAccBound();

  // 2. v bound

  // 3. jerk bound

  return common::Status::OK();
}

double BoundMaker::s_lower_bound(const double t) const {
  int32_t index = static_cast<int32_t>(std::round(t / dt_));
  return s_lower_bound_[index];
}

}  // namespace planning
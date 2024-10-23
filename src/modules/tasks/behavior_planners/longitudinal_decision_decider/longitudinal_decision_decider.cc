#include "longitudinal_decision_decider.h"

namespace planning {

LongitudinalDecisionDecider::LongitudinalDecisionDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  name_ = "LongitudinalDecisionDecider";

  // 读取配置文件
  config_ = config_builder->cast<SccLonBehaviorPlannerConfig>();

  speed_planning_conf_ =
      config_manager.GetTaskConfig(cp_planning::proto::TaskName::SPEED_PLANNER)
          ->speed_planning_config();
  plan_time_ = speed_planning_conf_.plan_time();
  dt_ = speed_planning_conf_.dt();
  plan_points_num_ = static_cast<size_t>(plan_time_ / dt_) + 1;
}

bool LongitudinalDecisionDecider::Execute() {




  return true; }

}  // namespace planning

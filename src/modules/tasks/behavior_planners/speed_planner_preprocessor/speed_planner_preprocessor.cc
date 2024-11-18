#include "speed_planner_preprocessor.h"

#include <cstdint>

namespace planning {
SpeedPlannerPreProcessor::SpeedPlannerPreProcessor(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  name_ = "SpeedPlannerPreProcessor";
  speed_planning_config_ = config_builder->cast<SpeedPlannerConfig>();
  target_maker_ =
      std::make_unique<TargetMaker>(speed_planning_config_, session);
  bound_maker_ = std::make_unique<BoundMaker>(speed_planning_config_, session);
  weight_maker_ = std::make_unique<WeightMaker>(speed_planning_config_, session,
                                                *target_maker_);

  dt_ = speed_planning_config_.dt;
  plan_time_ = speed_planning_config_.planning_time;
  plan_points_num_ = static_cast<int32_t>(plan_time_ / dt_) + 1;
}

bool SpeedPlannerPreProcessor::Execute() {
  LOG_DEBUG("=======SpeedPlannerPreProcessor======= \n");

  if (!PreCheck()) {
    LOG_DEBUG("PreCheck failed\n");
    return false;
  }
  // 1. calculate s_ref
  target_maker_->Run();
  // 2. calculate bound
  bound_maker_->Run();
  // 3. calculate weight
  weight_maker_->Run();
  // 4. update lon ref path
  UpdateLonRefPath();

  return true;
}

void SpeedPlannerPreProcessor::UpdateLonRefPath() {
  lon_behav_output_.t_list.resize(plan_points_num_);
  lon_behav_output_.s_refs.resize(plan_points_num_);
  lon_behav_output_.ds_refs.resize(plan_points_num_);
  lon_behav_output_.hard_bounds.resize(plan_points_num_);
  lon_behav_output_.soft_bounds.resize(plan_points_num_);
  lon_behav_output_.lead_bounds.resize(plan_points_num_);
  lon_behav_output_.lon_bound_v.resize(plan_points_num_);
  lon_behav_output_.lon_bound_a.resize(plan_points_num_);
  lon_behav_output_.lon_bound_jerk.resize(plan_points_num_);
}
}  // namespace planning
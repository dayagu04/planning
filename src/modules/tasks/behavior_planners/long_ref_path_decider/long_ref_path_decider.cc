#include "long_ref_path_decider.h"

#include <cstdint>

#include "src/common/ifly_time.h"
#include "src/modules/context/planning_context.h"

namespace planning {
LongRefPathDecider::LongRefPathDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  name_ = "LongRefPathDecider";
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

bool LongRefPathDecider::Execute() {
  LOG_DEBUG("=======LongRefPathDecider======= \n");
  const auto start_timestamp = IflyTime::Now_ms();

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

  const auto end_timestamp = IflyTime::Now_ms();
  LOG_DEBUG("LongRefPathDecider time cost: [%f]ms \n",
            end_timestamp - start_timestamp);

  return true;
}

void LongRefPathDecider::UpdateLonRefPath() {
  WeightedBounds s_hard_bounds;
  WeightedBounds s_soft_bounds;
  // LonLeadBounds s_lead_bounds;
  // Bound lon_a_bound{a_bounds.first, a_bounds.second};
  // Bound lon_j_bound{j_bounds.first, j_bounds.second};

  lon_behav_output_.t_list.resize(plan_points_num_);
  lon_behav_output_.s_refs.resize(plan_points_num_);
  lon_behav_output_.ds_refs.resize(plan_points_num_);
  lon_behav_output_.hard_bounds.resize(plan_points_num_);
  lon_behav_output_.soft_bounds.resize(plan_points_num_);
  lon_behav_output_.lead_bounds.resize(plan_points_num_);
  lon_behav_output_.lon_bound_v.resize(plan_points_num_);
  lon_behav_output_.lon_bound_a.resize(plan_points_num_);
  lon_behav_output_.lon_bound_jerk.resize(plan_points_num_);

  for (unsigned int i = 0; i < plan_points_num_; i++) {
    double t = i * dt_;
    // 1.update t_list
    lon_behav_output_.t_list[i] = t;
    // 2.update s_refs
    lon_behav_output_.s_refs[i] = {target_maker_->s_target(t), 1.0};  // hack
    // lon_behav_output_.s_refs[i] = {target_maker_->s_target(t),
    //                                weight_maker_->s_weight(t)};

    // 3.update ds_refs
    lon_behav_output_.ds_refs[i] = {target_maker_->v_target(t), 1.0};  // hack
    // lon_behav_output_.ds_refs[i] = {target_maker_->v_target(t),
    //                                 weight_maker_->v_weight(t)};

    // 4.update s bounds & weights
    // binwang33: 需要关注后续soft bound形式，采用target
    // marker后，sref更合理的话，只留hard bound也是没问题的
    WeightedBound s_hard_bound;
    s_hard_bound.lower = 0.0;    // hack
    s_hard_bound.upper = 150.0;  // hack
    // s_hard_bound.lower = bound_maker_->s_lower_bound(t);
    // s_hard_bound.upper = bound_maker_->s_upper_bound(t);
    s_hard_bound.weight = 10;
    s_hard_bound.bound_info.id = -1;  // hack: 后续在bound_maker_中查询
    s_hard_bound.bound_info.type = BoundType::AGENT;  // hack: 后续需要区分属性
    // binwang33: hard_bounds不需要再采用vector形式
    lon_behav_output_.hard_bounds[i].emplace_back(s_hard_bound);
    // lon_behav_output_.soft_bounds[i].emplace_back(s_soft_bound);
    // lon_behav_output_.lead_bounds[i].emplace_back(s_lead_bound);

    // 5.update v bounds & weights
    Bound lon_v_bound{0.0, 40.0};  // hack
    // Bound lon_v_bound{bound_maker_->v_lower_bound(t),
    //                   bound_maker_->v_upper_bound(t)};
    lon_behav_output_.lon_bound_v[i] = lon_v_bound;

    // TBD: 缺少一个s-v bound，用于实现精准限速

    // 6.update a bounds & weights
    Bound lon_a_bound{-2.0, 2.0};  // hack
    // Bound lon_a_bound{bound_maker_.a_lower_bound(t),
    //                   bound_maker_.a_upper_bound(t)};
    lon_behav_output_.lon_bound_a[i] = lon_a_bound;
    // 8.update jerk bounds & weights
    Bound lon_j_bound{-6.0, 6.0};  // hack
    // Bound lon_j_bound{bound_maker_.jerk_lower_bound(t),
    //                   bound_maker_.jerk_upper_bound(t)};
    lon_behav_output_.lon_bound_jerk[i] = lon_j_bound;
  }

  // update longitudinal_decider_output
  auto &lon_ref_path = session_->mutable_planning_context()
                           ->mutable_longitudinal_decider_output();

  lon_ref_path = lon_behav_output_;
}
}  // namespace planning
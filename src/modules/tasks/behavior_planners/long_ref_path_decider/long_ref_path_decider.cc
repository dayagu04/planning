#include "long_ref_path_decider.h"

#include <cstdint>

#include "log.h"
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
  weight_maker_ = std::make_unique<WeightMaker>(speed_planning_config_, session);

  dt_ = speed_planning_config_.dt;
  plan_time_ = speed_planning_config_.planning_time;
  plan_points_num_ = static_cast<int32_t>(plan_time_ / dt_) + 1;
}

bool LongRefPathDecider::Execute() {
  LOG_DEBUG("=======LongRefPathDecider======= \n");
  const auto start_timestamp = IflyTime::Now_ms();

  if (!PreCheck()) {
    LOG_ERROR("PreCheck failed\n");
    return false;
  }
  // 0. reset
  Reset();
  // 1. calculate s_ref
  target_maker_->Run();
  // 2. calculate bound
  bound_maker_->Run();
  // 3. calculate weight
  weight_maker_->Run(*target_maker_);
  // 4. update lon ref path
  UpdateLonRefPath();
  // 5. save long ref path
  SaveToSession();
  SaveToDebugInfo();

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

  lon_behavior_output_.t_list.resize(plan_points_num_);
  lon_behavior_output_.s_refs.resize(plan_points_num_);
  lon_behavior_output_.ds_refs.resize(plan_points_num_);
  lon_behavior_output_.hard_bounds.resize(plan_points_num_);
  lon_behavior_output_.soft_bounds.resize(plan_points_num_);
  lon_behavior_output_.lead_bounds.resize(plan_points_num_);
  lon_behavior_output_.lon_bound_v.resize(plan_points_num_);
  lon_behavior_output_.lon_bound_a.resize(plan_points_num_);
  lon_behavior_output_.lon_bound_jerk.resize(plan_points_num_);

  for (unsigned int i = 0; i < plan_points_num_; i++) {
    double t = i * dt_;
    // 1.update t_list
    lon_behavior_output_.t_list[i] = t;
    // 2.update s_refs
    // lon_behavior_output_.s_refs[i] = {target_maker_->s_target(t), 1.0};
    lon_behavior_output_.s_refs[i] = {target_maker_->s_target(t),
                                   weight_maker_->s_weight(t)};

    // 3.update ds_refs
    // lon_behavior_output_.ds_refs[i] = {target_maker_->v_target(t),
    //                                    1.0};
    lon_behavior_output_.ds_refs[i] = {target_maker_->v_target(t),
                                    weight_maker_->v_weight(t)};

    // 4.update s bounds & weights
    // binwang33: 需要关注后续soft bound形式，采用target
    // marker后，sref更合理的话，只留hard bound也是没问题的
    WeightedBound s_hard_bound;
    s_hard_bound.lower = bound_maker_->s_lower_bound(t);
    s_hard_bound.upper = bound_maker_->s_upper_bound(t);
    // s_hard_bound.lower = bound_maker_->s_lower_bound(t);
    // s_hard_bound.upper = bound_maker_->s_upper_bound(t);
    s_hard_bound.weight = 10;
    s_hard_bound.bound_info.id = -1;
    s_hard_bound.bound_info.type = BoundType::AGENT;  // hack: 后续需要区分属性
    // binwang33: hard_bounds不需要再采用vector形式
    lon_behavior_output_.hard_bounds[i].emplace_back(s_hard_bound);
    // lon_behavior_output_.soft_bounds[i].emplace_back(s_soft_bound);
    // lon_behavior_output_.lead_bounds[i].emplace_back(s_lead_bound);

    // 5.update v bounds & weights
    Bound lon_v_bound;
    lon_v_bound.lower = bound_maker_->v_lower_bound(t);
    lon_v_bound.upper = bound_maker_->v_upper_bound(t);
    // Bound lon_v_bound{bound_maker_->v_lower_bound(t),
    //                   bound_maker_->v_upper_bound(t)};
    lon_behavior_output_.lon_bound_v[i] = lon_v_bound;

    // TBD: 缺少一个s-v bound，用于实现精准限速

    // 6.update a bounds & weights
    Bound lon_a_bound;
    lon_a_bound.lower = bound_maker_->a_lower_bound(t);
    lon_a_bound.upper = bound_maker_->a_upper_bound(t);
    // Bound lon_a_bound{bound_maker_.a_lower_bound(t),
    //                   bound_maker_.a_upper_bound(t)};
    lon_behavior_output_.lon_bound_a[i] = lon_a_bound;
    // 8.update jerk bounds & weights
    Bound lon_j_bound;
    lon_j_bound.lower = bound_maker_->jerk_lower_bound(t);
    lon_j_bound.upper = bound_maker_->jerk_upper_bound(t);
    // Bound lon_j_bound{bound_maker_.jerk_lower_bound(t),
    //                   bound_maker_.jerk_upper_bound(t)};
    lon_behavior_output_.lon_bound_jerk[i] = lon_j_bound;
  }
}

void LongRefPathDecider::Reset() {
  target_maker_->Reset();
  // bound_maker_->Reset();
  weight_maker_->Reset();
  ClearOutput();
}

void LongRefPathDecider::SaveToSession() {
  // update longitudinal_decider_output
  auto &lon_ref_path = session_->mutable_planning_context()
                           ->mutable_longitudinal_decider_output();
  lon_ref_path = lon_behavior_output_;
}

void LongRefPathDecider::SaveToDebugInfo() {
  // 转存纵向决策信息至debuginfo
  // 1.update t_list
  lon_behavior_output_pb_.mutable_t_list()->Reserve(
      lon_behavior_output_.t_list.size());
  for (const auto &t : lon_behavior_output_.t_list) {
    lon_behavior_output_pb_.add_t_list(t);
  }
  // 2.update s_refs
  lon_behavior_output_pb_.mutable_s_refs()->Reserve(
      lon_behavior_output_.s_refs.size());
  for (const auto &s_ref : lon_behavior_output_.s_refs) {
    auto add_s_ref = lon_behavior_output_pb_.add_s_refs();
    add_s_ref->set_first(s_ref.first);   // offset
    add_s_ref->set_second(s_ref.first);  // weight
  }
  // 3.update ds_refs
  lon_behavior_output_pb_.mutable_ds_refs()->Reserve(
      lon_behavior_output_.ds_refs.size());
  for (const auto &ds_ref : lon_behavior_output_.ds_refs) {
    auto add_ds_ref = lon_behavior_output_pb_.add_ds_refs();
    add_ds_ref->set_first(ds_ref.first);   // offset
    add_ds_ref->set_second(ds_ref.first);  // weight
  }

  // 4.update hard bounds
  lon_behavior_output_pb_.mutable_bounds()->Reserve(
      lon_behavior_output_.hard_bounds.size());
  for (const auto &bounds : lon_behavior_output_.hard_bounds) {
    auto bounds_pb = lon_behavior_output_pb_.add_bounds();
    for (const auto &bound : bounds) {
      auto bound_pb = bounds_pb->add_bound();
      bound_pb->set_lower(bound.lower);
      bound_pb->set_upper(bound.upper);
      bound_pb->set_weight(bound.weight);
      bound_pb->mutable_bound_info()->set_id(bound.bound_info.id);
      bound_pb->mutable_bound_info()->set_type(
          BoundType2String(bound.bound_info.type));
    }
  }
  // 5.update lon_soft_bounds
  lon_behavior_output_pb_.mutable_soft_bounds()->Reserve(
      lon_behavior_output_.soft_bounds.size());
  for (const auto &lon_soft_bounds : lon_behavior_output_.soft_bounds) {
    auto lon_soft_bounds_pb = lon_behavior_output_pb_.add_soft_bounds();
    for (const auto &lon_soft_bound : lon_soft_bounds) {
      auto lon_soft_bound_pb = lon_soft_bounds_pb->add_bound();
      lon_soft_bound_pb->set_lower(lon_soft_bound.lower);
      lon_soft_bound_pb->set_upper(lon_soft_bound.upper);
      lon_soft_bound_pb->set_weight(lon_soft_bound.weight);
      lon_soft_bound_pb->mutable_bound_info()->set_id(
          lon_soft_bound.bound_info.id);
      lon_soft_bound_pb->mutable_bound_info()->set_type(
          BoundType2String(lon_soft_bound.bound_info.type));
    }
  }
  // 6.update lon_sv_boundary
  // lon_behavior_output_pb_.mutable_lon_sv_boundary()->mutable_sv_bounds()->Reserve(
  //     lon_behavior_output_.lon_sv_boundary.sv_bounds.size());
  // lon_behavior_output_pb_.mutable_lon_sv_boundary()->set_boundary_type(
  //     (common::SVBoundary::SVBoundaryType)
  //         lon_behavior_output_.lon_sv_boundary.boundary_type);

  // for (const auto &sv_bound : lon_behavior_output_.lon_sv_boundary.sv_bounds) {
  //   auto sv_bound_pb =
  //       lon_behavior_output_pb_.mutable_lon_sv_boundary()->add_sv_bounds();
  //   sv_bound_pb->set_s(sv_bound.s);
  //   sv_bound_pb->mutable_v_bound()->set_lower(sv_bound.v_bound.lower);
  //   sv_bound_pb->mutable_v_bound()->set_upper(sv_bound.v_bound.upper);
  // }

  // 7.update lon_bound_v
  auto lon_bounds_v_pb = lon_behavior_output_pb_.mutable_lon_bound_v();
  lon_bounds_v_pb->mutable_bound()->Reserve(
      lon_behavior_output_.lon_bound_v.size());
  for (const auto &lon_bound_v : lon_behavior_output_.lon_bound_v) {
    auto lon_bound_v_pb = lon_bounds_v_pb->add_bound();
    lon_bound_v_pb->set_lower(lon_bound_v.lower);
    lon_bound_v_pb->set_upper(lon_bound_v.upper);
  }
  // 8.update lon_bound_a
  auto lon_bounds_a_pb = lon_behavior_output_pb_.mutable_lon_bound_a();
  lon_bounds_a_pb->mutable_bound()->Reserve(
      lon_behavior_output_.lon_bound_a.size());
  for (const auto &lon_bound_a : lon_behavior_output_.lon_bound_a) {
    auto lon_bound_a_pb = lon_bounds_a_pb->add_bound();
    lon_bound_a_pb->set_lower(lon_bound_a.lower);
    lon_bound_a_pb->set_upper(lon_bound_a.upper);
  }
  // 9.update lon_bound_jerk
  auto lon_bounds_jerk_pb = lon_behavior_output_pb_.mutable_lon_bound_jerk();
  lon_bounds_jerk_pb->mutable_bound()->Reserve(
      lon_behavior_output_.lon_bound_jerk.size());
  for (const auto &lon_bound_jerk : lon_behavior_output_.lon_bound_jerk) {
    auto lon_bound_jerk_pb = lon_bounds_jerk_pb->add_bound();
    lon_bound_jerk_pb->set_lower(lon_bound_jerk.lower);
    lon_bound_jerk_pb->set_upper(lon_bound_jerk.upper);
  }

  auto &debug_info_pb = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto long_ref_path_pb = debug_info_pb->mutable_long_ref_path();
  long_ref_path_pb->CopyFrom(lon_behavior_output_pb_);
}

void LongRefPathDecider::ClearOutput() {
  // clear proto for debug
  lon_behavior_output_pb_.Clear();
  // clear output for context
  lon_behavior_output_.t_list.clear();
  lon_behavior_output_.s_refs.clear();
  lon_behavior_output_.ds_refs.clear();
  lon_behavior_output_.hard_bounds.clear();
  lon_behavior_output_.soft_bounds.clear();
  lon_behavior_output_.lon_lead_bounds.clear();
  lon_behavior_output_.lon_obstacle_overlap_info.clear();
  lon_behavior_output_.lon_obstacle_yield_info.clear();
  lon_behavior_output_.lon_sv_boundary.sv_bounds.clear();
  lon_behavior_output_.lon_bound_v.clear();
  lon_behavior_output_.lon_bound_a.clear();
  lon_behavior_output_.lon_bound_jerk.clear();
}

}  // namespace planning
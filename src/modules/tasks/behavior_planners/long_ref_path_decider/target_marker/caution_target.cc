#include "caution_target.h"
#include <cstddef>
#include <memory>
#include "environmental_model.h"
#include "planning_context.h"

namespace planning {

namespace {
// TODO: read it from confi file later
constexpr double user_time_gap = 1.0;
constexpr double min_follow_distance_m = 4.0;
}  // namespace

CautionTarget::CautionTarget(const SpeedPlannerConfig& config,
                             framework::Session* session)
    : Target(config, session) {
  upper_bound_infos_ =
      std::vector<UpperBoundInfo>(plan_points_num_, UpperBoundInfo());
  GenerateUpperBoundInfo();
  GenerateCautionTarget();
  // AddDebugToProto(TargetType::kCautionYield, planning_debug_msg);
}

void CautionTarget::GenerateUpperBoundInfo() {
  const auto* st_graph = session_->planning_context().st_graph_helper();
  if (st_graph == nullptr) {
    return;
  }
  for (size_t i = 0; i < plan_points_num_; i++) {
    const double t = i * dt_;
    const auto& upper_bound = st_graph->GetSoftPassCorridorUpperBound(t);
    if (upper_bound.agent_id() != speed::kNoAgentId) {
      upper_bound_infos_[i].s = upper_bound.s();
      upper_bound_infos_[i].t = t;
      upper_bound_infos_[i].v = upper_bound.velocity();
      upper_bound_infos_[i].target_type = TargetType::kCautionYield;
      upper_bound_infos_[i].agent_id = upper_bound.agent_id();
    }
  }
}

void CautionTarget::GenerateCautionTarget() {
  double matched_desired_headway = user_time_gap;
  const double default_t = 0.0;
  const bool default_has_target = false;
  const double default_s_target = 0.0;
  const double default_v_target = 0.0;
  const TargetType default_target_type = TargetType::kNotSet;
  auto default_target_value =
      TargetValue(default_t, default_has_target, default_s_target,
                  default_v_target, default_target_type);
  target_values_ =
      std::vector<TargetValue>(plan_points_num_, default_target_value);

  for (int32_t i = 0; i < plan_points_num_; i++) {
    const double t = i * dt_;
    auto& target_value = target_values_[i];
    target_value.set_relative_t(t);
    if (upper_bound_infos_[i].target_type == TargetType::kNotSet) {
      continue;
    }
    target_value.set_has_target(true);
    const double vel = virtual_zero_acc_curve_->Evaluate(1, t);
    double follow_time_gap = user_time_gap;
    double target_s_disatnce = std::max(
        vel * follow_time_gap + min_follow_distance_m, min_follow_distance_m);

    const double s_target_value = upper_bound_infos_[i].s - target_s_disatnce;
    target_value.set_s_target_val(s_target_value);
    target_value.set_target_type(upper_bound_infos_[i].target_type);
  }
}

}  // namespace planning

#include "overtake_target.h"
#include "environmental_model.h"
#include "planning_context.h"

namespace planning {

namespace {
constexpr double kMaxOvertakeTimeGap = 1.7;
constexpr double kMinOvertakeTimeGap = 0.8;
constexpr double kMaxOvertakeSpeedDiff = 1.0;
constexpr double kMinOvertakeSpeedDiff = -1.5;

constexpr double kMinOvertakeSpeed = 8.0;
constexpr double kMaxOvertakeSpeed = 13.5;

constexpr double kLowerFollowTimeGap = 0.4;
} // namespace

OvertakeTarget::OvertakeTarget(const SpeedPlannerConfig& config, framework::Session* session,
                               const FollowTarget& follow_target)
    : Target(config, session) {

  target_values_ = std::vector<TargetValue>(plan_points_num_,
                                            TargetValue(0.0, false, 0.0, 0.0, TargetType::kNotSet));

  // overtake target
  // first part must

  // 1. make s obertake bound
  overtake_bounds_ = std::vector<OvertakeBound>(plan_points_num_, OvertakeBound());
  MakeOvertakeBoundsWithStCorridor();
  // 2. make overtake target with follow target
  MakeOvertakeTarget(follow_target);
  // for (const auto& target_value : target_values_) {
  //   std::cout << "t:" << target_value.relative_t() << std::endl;
  //   std::cout << "has target:" << target_value.has_target() << std::endl;
  //   std::cout << "s target value" << target_value.s_target_val() << std::endl;
  // }
  //AddDebugToProto(TargetType::kOvertake, planning_debug_msg);
}

void OvertakeTarget::MakeOvertakeBoundsWithStCorridor() {
  for (size_t i = 0; i < plan_points_num_; ++i) {
    double relative_t = i * dt_;
    auto lower_bound_point =
        session_->planning_context().st_graph_helper()->GetPassCorridorLowerBound(relative_t);
    if (lower_bound_point.agent_id() != speed::kNoAgentId) {
      overtake_bounds_[i].s = lower_bound_point.s();
      overtake_bounds_[i].v = lower_bound_point.velocity();
      overtake_bounds_[i].t = relative_t;
      overtake_bounds_[i].type = TargetType::kOvertake;
      overtake_bounds_[i].overtake_agent_id = lower_bound_point.agent_id();
      continue;
    }
  }
}

void OvertakeTarget::MakeOvertakeTarget(const FollowTarget& follow_target) {
  for (size_t i = 0; i < plan_points_num_; ++i) {
    double relative_t = i * dt_;
    target_values_[i].set_relative_t(relative_t);
    if (overtake_bounds_[i].type == TargetType::kNotSet) {
      continue;
    }

    // decider overtake time gap
    double speed = virtual_zero_acc_curve_->Evaluate(1, relative_t);
    double speed_diff_with_overtake_agent = speed - overtake_bounds_[i].v;
    double time_gap_with_overtake = planning_math::LerpWithLimit(
        kMaxOvertakeTimeGap, kMinOvertakeSpeedDiff, kMinOvertakeTimeGap, kMaxOvertakeSpeedDiff,
        speed_diff_with_overtake_agent); // x0 + r * (x1 - x0);
    double time_gap_with_zero_acc_speed = planning_math::LerpWithLimit(
        kMaxOvertakeTimeGap, kMinOvertakeSpeed, kMinOvertakeTimeGap, kMaxOvertakeSpeed, speed);

    double proper_time_gap = std::fmin(time_gap_with_overtake, time_gap_with_zero_acc_speed);
    double s_target_for_overtake = overtake_bounds_[i].s + proper_time_gap * overtake_bounds_[i].v;
    // std::cout << "relative t:" << relative_t << std::endl;
    // std::cout << "proper time gap:" << proper_time_gap << std::endl;
    // std::cout << "speed_diff_with_overtake_agent: " << speed_diff_with_overtake_agent <<
    // std::endl; std::cout << "time_gap_with_overtake:" << time_gap_with_overtake << std::endl;
    // std::cout << "time_gap_with_zero_acc_speed:" << time_gap_with_zero_acc_speed << std::endl;
    // std::cout << "overtake_bounds_[i].s :" << overtake_bounds_[i].s << std::endl;
    // std::cout << "overtake_bounds_[i].v :" << overtake_bounds_[i].v << std::endl;
    // std::cout << "overtake t:" << overtake_bounds_[i].t << std::endl;
    // std::cout << "overtake s target 1:" << s_target_for_overtake << std::endl;
    // std::cout << "overtake agent id:" << overtake_bounds_[i].overtake_agent_id << std::endl;

    // if s_target_for_overtake > follow_target s value
    if (follow_target.has_target(relative_t) &&
        follow_target.target_value(relative_t).s_target_val() < s_target_for_overtake) {

      auto upper_bound_point =
          session_->planning_context().st_graph_helper()->GetPassCorridorUpperBound(relative_t);
      double upper_bound_s = upper_bound_point.s();
      // make small follow time gap follow target
      double follow_s_target_distance =
          follow_target.MakeSlowerFollowSTarget(speed, upper_bound_s, kLowerFollowTimeGap);
      // std::cout << "follow_s_target_distance:" << follow_s_target_distance << std::endl;

      // make 60% gap from gap
      // std::cout << "upper_bound_s:" << upper_bound_s << std::endl;
      double s_target_with_corridor = 0.75 * upper_bound_s + 0.25 * overtake_bounds_[i].s;
      // std::cout << "s_target_with_corridor:" << s_target_with_corridor << std::endl;
      double s_target_with_follow = std::fmin(follow_s_target_distance, s_target_with_corridor);
      // std::cout << "s_target_with_follow:" << s_target_with_follow << std::endl;
      s_target_for_overtake = std::fmin(s_target_with_follow, s_target_for_overtake);
      // std::cout << "s_target_for_overtake 2:" << s_target_for_overtake << std::endl;
    }
    double max_s_target = max_speed_limit_curve_->Evaluate(0, relative_t);
    // std::cout << "max s target: " << max_s_target << std::endl;
    s_target_for_overtake = std::fmin(max_s_target, s_target_for_overtake);
    // std::cout << "s_target_for_overtake 3:" << s_target_for_overtake << std::endl;
    target_values_[i].set_has_target(true);
    target_values_[i].set_s_target_val(s_target_for_overtake);
    target_values_[i].set_target_type(TargetType::kOvertake);
  }
}

} // namespace planning

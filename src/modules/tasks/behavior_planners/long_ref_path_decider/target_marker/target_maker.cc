#include "target_maker.h"

#include "cruise_target.h"
#include "follow_target.h"

namespace planning {

TargetMaker::TargetMaker(const SpeedPlannerConfig& speed_planning_config,
                         framework::Session* session)
    : session_(session), speed_planning_config_(speed_planning_config) {}

common::Status TargetMaker::Run() {
  LOG_DEBUG("=======LongRefPathDecider: TargetMaker======= \n");
  dt_ = speed_planning_config_.dt;
  plan_time_ = speed_planning_config_.planning_time;
  plan_points_num_ = static_cast<int32_t>(plan_time_ / dt_) + 1;

  // 1. cruise target @建伟
  CruiseTarget cruise_target(speed_planning_config_, session_);

  // 2. follow target @翼闻
  FollowTarget follow_target(speed_planning_config_, session_);

  // 3. overtake target @国朋
  // OvertakeTarget overtake_target(config_, session_);

  // 4. neighbor target @建伟
  // NeighborTarget neighbor_target(config_, session_);

  // 5. caution target @国朋
  // CautionTarget caution_target(config_, session_);

  // 6. decider final target values
  for (size_t i = 0; i < plan_points_num_; ++i) {
    double relative_t = i * dt_;
    TargetValue cruise_target_value = cruise_target.target_value(relative_t);
    // TargetValue follow_target_value = follow_target.target_value(relative_t);
    // TargetValue overtake_target_value =
    // overtake_target.target_value(relative_t); TargetValue
    // neighbor_target_value = neighbor_target.target_value(relative_t);
    // TargetValue caution_target_value =
    // caution_target.target_value(relative_t);

    TargetValue upper_target_value(0.0, false,
                                   std::numeric_limits<double>::max(), 0.0,
                                   TargetType::kNotSet);
    TargetValue lower_target_value(0.0, false,
                                   -std::numeric_limits<double>::max(), 0.0,
                                   TargetType::kNotSet);
    // 1. update lower and upper value by follow target and overtake target
    // if (follow_target_value.has_target() &&
    //     follow_target_value.s_target_val() <
    //         upper_target_value.s_target_val()) {
    // TBD: 国朋合入overtake
    //   if (overtake_target_value.has_target()) {
    //     if (overtake_target_value.s_target_val() >
    //         follow_target_value.s_target_val()) {
    //       // final_target_value = overtake_target_value;
    //       target_values_.push_back(std::move(overtake_target_value));
    //       continue;
    //     } else {
    //       upper_target_value = follow_target_value;
    //       lower_target_value = overtake_target_value;
    //     }
    //   } else {
    // upper_target_value = follow_target_value;
    //   }
    // } else if (overtake_target_value.has_target() &&
    //            overtake_target_value.s_target_val() >
    //                lower_target_value.s_target_val()) {
    //   lower_target_value = overtake_target_value;
    // }

    // TBD: 国朋合入caution
    // 2.update upper value by caution yield target
    // if (caution_target_value.has_target()) {
    //   upper_target_value =
    //       Target::TargetMin(caution_target_value, upper_target_value);
    // }

    // TBD: 建伟合入neighbor
    // 3.update lower and upper value by neighbor target
    // if (neighbor_target_value.has_target()) {
    //   // neighbor
    //   if (neighbor_target_value.target_type() == TargetType::kNeighbor) {
    //     if (cp_common::WithinBound(lower_target_value.s_target_val(),
    //                                upper_target_value.s_target_val(),
    //                                neighbor_target_value.s_target_val())) {
    //       target_values_.push_back(std::move(neighbor_target_value));
    //       continue;
    //     }
    //   }
    //   if (neighbor_target_value.target_type() == TargetType::kNeighborYeild)
    //   {
    //     if (cp_common::WithinBound(lower_target_value.s_target_val(),
    //                                upper_target_value.s_target_val(),
    //                                neighbor_target_value.s_target_val())) {
    //       upper_target_value = neighbor_target_value;
    //     }
    //   }
    //   if (neighbor_target_value.target_type() ==
    //       TargetType::kNeighborOvertake) {
    //     if (cp_common::WithinBound(lower_target_value.s_target_val(),
    //                                upper_target_value.s_target_val(),
    //                                neighbor_target_value.s_target_val())) {
    //       lower_target_value = neighbor_target_value;
    //     }
    //   }
    // }

    // hack: 先用默认值
    auto final_lower_bound_value = lower_target_value;
    final_lower_bound_value = cruise_target_value;
    // auto final_lower_bound_value =
    //     Target::TargetMax(lower_target_value, cruise_target_value);
    auto final_target_value =
        Target::TargetMin(final_lower_bound_value, upper_target_value);
    target_values_.push_back(std::move(final_target_value));
  }

  return common::Status::OK();
}

double TargetMaker::s_target(const double t) const {
  size_t index = static_cast<size_t>(std::round(t / dt_));

  return target_values_.at(index).s_target_val();
}

double TargetMaker::v_target(const double t) const {
  size_t index = static_cast<size_t>(std::round(t / dt_));
  return target_values_.at(index).v_target_val();
}

const TargetValue& TargetMaker::target_value(const double t) const {
  size_t index = static_cast<size_t>(std::round(t / dt_));
  return target_values_.at(index);
}

}  // namespace planning
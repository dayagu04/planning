#include "neighbor_target.h"

#include <cstdint>
#include <cstdlib>

#include "behavior_planners/long_ref_path_decider/target_marker/target.h"
#include "log.h"
#include "math/linear_interpolation.h"
#include "planning_context.h"
#include "src/modules/context/environmental_model.h"
#include "st_graph/st_point.h"
#include "trajectory1d/trajectory1d.h"

namespace planning {

NeighborTarget::NeighborTarget(const SpeedPlannerConfig& config,
                               framework::Session* session)
    : Target(config, session) {
  GenerateNeighborTarget();
  AddNeighborTargetDataToProto();
}

void NeighborTarget::GenerateNeighborTarget() {
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

  GenerateNeighborTargetCurve();

  if (neighbor_target_curve_ == nullptr) {
    return;
  }

  for (int32_t i = 0; i < plan_points_num_; i++) {
    const double t = i * dt_;
    auto& target_value = target_values_[i];
    target_value.set_relative_t(t);
    target_value.set_has_target(true);
    double s_target_value =
        std::max(neighbor_target_curve_->Evaluate(0, t),
                 neighbor_target_curve_lower_bound_->Evaluate(0, t));
    double v_target_value =
        std::min(neighbor_target_curve_->Evaluate(1, t),
                 neighbor_target_curve_lower_bound_->Evaluate(1, t));
    s_target_value =
        std::min(s_target_value, max_speed_limit_curve_->Evaluate(0, t));
    v_target_value =
        std::min(v_target_value, max_speed_limit_curve_->Evaluate(0, t));
    target_value.set_s_target_val(s_target_value);
    target_value.set_v_target_val(v_target_value);
    target_value.set_target_type(neighbor_target_type_);
  }
}

void NeighborTarget::GenerateNeighborTargetCurve() {
  auto* st_graph_helper = session_->planning_context().st_graph_helper();
  if (st_graph_helper == nullptr) {
    return;
  }
  speed::STPoint first_neighbor_upper_bound = speed::STPoint::HighestSTPoint();
  bool is_yield_valid =
      st_graph_helper->GetFirstNeighborUpperBound(&first_neighbor_upper_bound);
  speed::STPoint first_neighbor_lower_bound = speed::STPoint::LowestSTPoint();
  bool is_overtake_valid =
      st_graph_helper->GetFirstNeighborLowerBound(&first_neighbor_lower_bound);
  if (is_yield_valid == false && is_overtake_valid == false) {
    return;
  }

  double yield_s =
      first_neighbor_upper_bound.s() -
      first_neighbor_upper_bound.velocity() * first_neighbor_upper_bound.t();
  double overtake_s =
      first_neighbor_lower_bound.s() +
      first_neighbor_lower_bound.velocity() * first_neighbor_lower_bound.t();

  double yield_v = first_neighbor_upper_bound.velocity();
  double overtake_v = first_neighbor_lower_bound.velocity();

  if (yield_s < overtake_s) {
    return;
  }

  const double max_acc = planning_math::LerpWithLimit(
      config_.neighbor_target_max_acc_upper,
      config_.neighbor_target_vel_upper_bound,
      config_.neighbor_target_max_acc_lower,
      config_.neighbor_target_vel_lower_bound, init_lon_state_[1]);

  double v_upper_end = init_lon_state_[1] + config_.neighbor_target_vel_buffer;
  const auto& speed_limit_output =
      session_->planning_context().speed_limit_decider_output();
  v_upper_end =
      std::min(v_upper_end, speed_limit_output.map_speed_limit_value() / 3.6);

  const auto& parallel_longitudinal_avoid_output =
      session_->planning_context().parallel_longitudinal_avoid_decider_output();

  if (parallel_longitudinal_avoid_output
          .is_need_parallel_longitudinal_avoid() &&
      (parallel_longitudinal_avoid_output.is_parallel_yield() ||
       parallel_longitudinal_avoid_output.is_lead_and_target_is_truck())) {
    yield_s = std::min(yield_s,
                       parallel_longitudinal_avoid_output.trajectory_start_s());
    yield_v = std::min(yield_v,
                       parallel_longitudinal_avoid_output.trajectory_start_v());
  } else if (parallel_longitudinal_avoid_output
                 .is_need_parallel_longitudinal_avoid() &&
             parallel_longitudinal_avoid_output.is_parallel_overtake()) {
    overtake_s = std::max(
        overtake_s, parallel_longitudinal_avoid_output.trajectory_start_s());
    overtake_v = std::max(
        overtake_v, parallel_longitudinal_avoid_output.trajectory_start_v());
  }

  LonState init_state;
  init_state.p = init_lon_state_[0];
  init_state.v = init_lon_state_[1];
  init_state.a = init_lon_state_[2];

  StateLimit state_limit;
  state_limit.p_end = 0.0;
  state_limit.v_end = 0.0;
  state_limit.v_max = v_upper_end;
  state_limit.v_min = config_.neighbor_target_kEpsilon;
  state_limit.a_max = max_acc;
  state_limit.a_min = config_.neighbor_target_min_acc;
  state_limit.j_max = config_.neighbor_target_max_jerk;
  state_limit.j_min = config_.neighbor_target_min_jerk;

  CoordinateParam variable_coordinate_param;
  if (is_yield_valid && is_overtake_valid) {
    variable_coordinate_param.s_start = (yield_s + overtake_s) * 0.5;
    variable_coordinate_param.v = (yield_v + overtake_v) * 0.5;
    neighbor_target_type_ = TargetType::kNeighbor;
  } else if (is_yield_valid && !is_overtake_valid) {
    variable_coordinate_param.s_start =
        yield_s - config_.neighbor_target_neighbor_yield_distance;
    variable_coordinate_param.v = yield_v;
    neighbor_target_type_ = TargetType::kNeighborYield;
  } else if (!is_yield_valid && is_overtake_valid) {
    variable_coordinate_param.s_start =
        overtake_s + config_.neighbor_target_neighbor_overtake_distance;
    variable_coordinate_param.v = overtake_v;
    neighbor_target_type_ = TargetType::kNeighborOvertake;
  }

  neighbor_target_curve_ =
      std::make_unique<VariableCoordinateTimeOptimalTrajectory>(
          VariableCoordinateTimeOptimalTrajectory::ConstructInstance(
              init_state, state_limit, variable_coordinate_param,
              config_.neighbor_target_p_precision));

  auto state_limit_lower_bound = state_limit;
  state_limit_lower_bound.v_end =
      config_.neighbor_target_neighbor_limit_velocity;
  neighbor_target_curve_lower_bound_ =
      std::make_unique<SecondOrderTimeOptimalTrajectory>(
          SecondOrderTimeOptimalTrajectory(init_state,
                                           state_limit_lower_bound));
}

void NeighborTarget::AddNeighborTargetDataToProto() {
#ifdef ENABLE_PROTO_LOG
  auto& debug_info_pb = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto mutable_neighbor_target_data =
      debug_info_pb->mutable_lon_target_s_ref()->mutable_neighbor_target();
  if (neighbor_target_curve_) {
    for (const auto& value : target_values_) {
      auto* ptr = neighbor_target_pb_.add_neighbor_target_s_ref();
      ptr->set_s(value.s_target_val());
      ptr->set_v(value.v_target_val());
      ptr->set_t(value.relative_t());
      ptr->set_target_type(static_cast<int32_t>(value.target_type()));
    }
    mutable_neighbor_target_data->CopyFrom(neighbor_target_pb_);
  } else {
    mutable_neighbor_target_data->Clear();
  }
#endif
}

}  // namespace planning
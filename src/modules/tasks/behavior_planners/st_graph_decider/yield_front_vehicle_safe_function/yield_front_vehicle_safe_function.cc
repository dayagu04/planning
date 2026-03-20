#include "yield_front_vehicle_safe_function.h"

#include <cstddef>
#include <cstdint>
#include <iostream>

#include "environmental_model.h"
#include "log.h"
#include "planning_context.h"
#include "st_graph/st_boundary.h"

namespace planning {

namespace {
constexpr double IsoAccLimitUpper = -3.5;
constexpr double IsoAccLimitLower = -4.0;
constexpr double IsoAccLimitSpeedUpper = 20.0;
constexpr double IsoAccLimitSpeedLower = 5.0;

constexpr double IsoJerkLimitUpper = -3.5;
constexpr double IsoJerkLimitLower = -5.0;
constexpr double IsoJerkLimitSpeedUpper = 20.0;
constexpr double IsoJerkLimitSpeedLower = 5.0;

// constexpr double kSlowJerkUpperBound = 6.0;

// constexpr double kFollowBuffer = 0.2;
// constexpr double kOvertakeBuffer = 2.0;

// constexpr double kSpeedBoundFactor = 1.1;
// constexpr double kPerSecondPlanLenth = 50.0;
constexpr double kTimeResolution = 0.2;
constexpr double kLowerSpeedAccUpperBound = 1.8;
constexpr double kHighSpeedAccUpperBound = 1.2;
constexpr double kLowSpeedThresholdWithAccUpperBound = 5.5;
constexpr double kHighSpeedThresholdWithAccUpperBound = 16.67;
constexpr double kJerkUpperBound = 10.0;
}  // namespace

YieldFrontVehicleSafeFunction::YieldFrontVehicleSafeFunction(
    framework::Session* session, const StGraphSearcherConfig config)
    : config_(config), session_(session) {}

SecondOrderTimeOptimalTrajectory
YieldFrontVehicleSafeFunction::GenerateMaxDecelerationCurve(
    const double front_node_vel) {
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto& planning_init_point = ego_state_manager->planning_init_point();
  LonState init_state = {0.0, planning_init_point.v, planning_init_point.a};

  // constexpr double kRefSpeedBuffer = 0.1;
  // constexpr double kSpeedBoundFactor = 1.1;
  // const double cruise_speed = ego_state_manager->ego_v_cruise();
  // const double cruise_speed_uppper_bound = cruise_speed * kSpeedBoundFactor;
  // const double ego_speed_upper_bound = init_state.v * kSpeedBoundFactor;

  StateLimit state_limit;
  state_limit.v_end = front_node_vel;

  // 根据速度计算acc bound
  const double acc_lower_bound = planning_math::LerpWithLimit(
      IsoAccLimitLower, IsoAccLimitSpeedLower, IsoAccLimitUpper,
      IsoAccLimitSpeedUpper, init_state.v);

  const double jerk_lower_bound = planning_math::LerpWithLimit(
      IsoJerkLimitLower, IsoJerkLimitSpeedLower, IsoJerkLimitUpper,
      IsoJerkLimitSpeedUpper, init_state.v);

  state_limit.a_max = planning_math::LerpWithLimit(
      kLowerSpeedAccUpperBound, kLowSpeedThresholdWithAccUpperBound,
      kHighSpeedAccUpperBound, kHighSpeedThresholdWithAccUpperBound,
      init_state.v);
  state_limit.a_min = acc_lower_bound;
  state_limit.j_max = kJerkUpperBound;
  state_limit.j_min = jerk_lower_bound;

  return SecondOrderTimeOptimalTrajectory(init_state, state_limit);
}

bool YieldFrontVehicleSafeFunction::IsYieldSafe(
    const SecondOrderTimeOptimalTrajectory& max_decrease_st,
    const int64_t target_lane_front_node_id) {
  const double yield_front_vehicle_min_decrease_max_check_time_s =
      config_.yield_front_vehicle_min_decrease_max_check_time_s;
  if (target_lane_front_node_id == planning_data::kInvalidId) {
    return true;
  }
  const auto* st_graph_helper = session_->planning_context().st_graph_helper();
  if (st_graph_helper == nullptr) {
    return true;
  }
  std::vector<int64_t> st_boundaries;
  st_graph_helper->GetAgentStBoundaries(target_lane_front_node_id,
                                        &st_boundaries);
  for (const auto boundary_id : st_boundaries) {
    speed::STBoundary front_st_boundary;
    st_graph_helper->GetStBoundary(boundary_id, &front_st_boundary);

    for (double time = front_st_boundary.min_t();
         time < front_st_boundary.max_t(); time += kTimeResolution) {
      if (time > yield_front_vehicle_min_decrease_max_check_time_s) {
        break;
      }
      const double curve_s = max_decrease_st.Evaluate(0, time);
      double s_lower = 0.0;
      double s_upper = 0.0;
      if (!front_st_boundary.GetBoundarySRange(time, &s_lower, &s_upper)) {
        break;
      }
      const double yield_front_vehicle_collision_s_buffer =
          config_.yield_front_vehicle_collision_s_buffer;
      if (curve_s > s_lower - yield_front_vehicle_collision_s_buffer) {
        ILOG_DEBUG << "yield not safe with max decrease curve!";
        return false;
      }
    }
  }
  return true;
}

}  // namespace planning
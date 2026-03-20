#include "variable_coordinate_time_optimal_trajectory.h"

namespace planning {

VariableCoordinateTimeOptimalTrajectory::
    VariableCoordinateTimeOptimalTrajectory(
        const CoordinateParam& relative_coordinate_param,
        const LonState& relative_init_state,
        const StateLimit& relative_state_limit, const double p_precision)
    : ThirdOrderTimeOptimalTrajectory(relative_init_state, relative_state_limit,
                                      p_precision),
      relative_coordinate_param_(relative_coordinate_param) {}

VariableCoordinateTimeOptimalTrajectory
VariableCoordinateTimeOptimalTrajectory::ConstructInstance(
    const LonState& init_state, const StateLimit& state_limit,
    const CoordinateParam& base_coordinate, const double p_precision) {
  constexpr double kEpsilon = 1e-10;
  LonState relative_init_state = init_state;
  relative_init_state.p -= base_coordinate.s_start;
  relative_init_state.v -= base_coordinate.v;

  StateLimit relative_state_limit = state_limit;
  relative_state_limit.v_min -= base_coordinate.v;
  relative_state_limit.v_min = std::min(relative_state_limit.v_min, -kEpsilon);
  relative_state_limit.v_max -= base_coordinate.v;
  relative_state_limit.v_max = std::max(relative_state_limit.v_max, kEpsilon);

  return VariableCoordinateTimeOptimalTrajectory(
      base_coordinate, relative_init_state, relative_state_limit, p_precision);
}

double VariableCoordinateTimeOptimalTrajectory::Evaluate(
    const int32_t order, const double param) const {
  const double base_s =
      relative_coordinate_param_.s_start + relative_coordinate_param_.v * param;
  const double base_v = relative_coordinate_param_.v;

  switch (order) {
    case 0:
      if (param > ParamLength()) {
        return state_limit_.p_end + base_s;
      }
      return (
          GetThirdOrderTrajectoryState(init_state_, third_order_param_, param)
              .p +
          base_s);
    case 1:
      if (param > ParamLength()) {
        return base_v;
      }
      return (
          GetThirdOrderTrajectoryState(init_state_, third_order_param_, param)
              .v +
          base_v);
    case 2:
      if (param > ParamLength()) {
        return 0.0;
      }
      return GetThirdOrderTrajectoryState(init_state_, third_order_param_,
                                          param)
          .a;
    case 3:
      if (param > ParamLength()) {
        return 0.0;
      }
      return GetThirdOrderTrajectoryState(init_state_, third_order_param_,
                                          param)
          .j;
    default:
      return 0.0;
  }
}

}  // namespace planning

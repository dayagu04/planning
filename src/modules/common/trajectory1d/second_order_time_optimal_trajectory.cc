#include "second_order_time_optimal_trajectory.h"

#include <cmath>
#include <iostream>
#include <limits>

#include "status/status.h"

namespace planning {

namespace {

inline int GetSign(double num) {
  if (std::fabs(num) < std::numeric_limits<double>::epsilon()) {
    return 0;
  }
  if (num > 0.0) {
    return 1;
  }
  return -1;
}

}  // namespace

SecondOrderTimeOptimalTrajectory::SecondOrderTimeOptimalTrajectory(
    const double p0, const double v0, const double a0, const double v_end,
    const double acc_min, const double acc_max, const double jerk_min,
    const double jerk_max) {
  const bool is_input_limit_valid =
      (acc_min < 0.0 && acc_max > 0.0 && jerk_min < 0.0 && jerk_max > 0.0);
  if (!is_input_limit_valid) {
    // throw Status(
    //     StatusCode::COMMON_ERROR,
    //     "invalid input limit for second order time optimal trajectory");
  }

  init_state_.p = p0;
  init_state_.v = v0;
  init_state_.a = a0;

  state_limit_.v_end = v_end;
  state_limit_.a_min = acc_min;
  state_limit_.a_max = acc_max;
  state_limit_.j_min = jerk_min;
  state_limit_.j_max = jerk_max;

  second_order_param_ = VelocityTargetSolver(init_state_, state_limit_);
}

SecondOrderTimeOptimalTrajectory::SecondOrderTimeOptimalTrajectory(
    const LonState& init_state, const StateLimit& state_limit)
    : init_state_(init_state), state_limit_(state_limit) {
  const bool is_input_limit_valid =
      (state_limit.a_min < 0.0 && state_limit.a_max > 0.0 &&
       state_limit.j_min < 0.0 && state_limit.j_max > 0.0);
  if (!is_input_limit_valid) {
    // throw Status(
    //     StatusCode::COMMON_ERROR,
    //     "invalid input limit for second order time optimal trajectory");
  }
  second_order_param_ = VelocityTargetSolver(init_state_, state_limit_);
}

double SecondOrderTimeOptimalTrajectory::Evaluate(const int32_t order,
                                                  const double param) const {
  switch (order) {
    case 0:
      return GetSecondOrderTrajectoryState(init_state_, second_order_param_,
                                           param)
          .p;
    case 1:
      if (param > ParamLength()) {
        return state_limit_.v_end;
      }
      return GetSecondOrderTrajectoryState(init_state_, second_order_param_,
                                           param)
          .v;
    case 2:
      if (param > ParamLength()) {
        return 0.0;
      }
      return GetSecondOrderTrajectoryState(init_state_, second_order_param_,
                                           param)
          .a;
    case 3:
      if (param > ParamLength()) {
        return 0.0;
      }
      return GetSecondOrderTrajectoryState(init_state_, second_order_param_,
                                           param)
          .j;
    default:
      return 0.0;
  }
}

double SecondOrderTimeOptimalTrajectory::ParamLength() const {
  return second_order_param_.t3;
}

SecondOrderParam SecondOrderTimeOptimalTrajectory::VelocityTargetSolver(
    const LonState& init_state, const StateLimit& state_limit) {
  const double j_min_inverse = 1.0 / state_limit.j_min;
  const double j_max_inverse = 1.0 / state_limit.j_max;

  double ve = 0.0;
  if (init_state.a > 0.0) {
    ve = init_state.v +
         init_state.a * std::fabs(init_state.a * j_min_inverse) * 0.5;
  } else {
    ve = init_state.v +
         init_state.a * std::fabs(init_state.a * j_max_inverse) * 0.5;
  }
  const int da = GetSign(state_limit.v_end - ve);

  double ac = 0.0;
  if (da == 1) {
    ac = state_limit.a_max;
  } else if (da == -1) {
    ac = state_limit.a_min;
  }

  double t1 = 0.0;
  double j1 = 0.0;
  if (ac - init_state.a >= 0.0) {
    t1 = (ac - init_state.a) * j_max_inverse;
    j1 = state_limit.j_max;
  } else {
    t1 = (ac - init_state.a) * j_min_inverse;
    j1 = state_limit.j_min;
  }
  double v1 = init_state.v + init_state.a * t1 + j1 * t1 * t1 * 0.5;

  double t2 = 0.0;
  double t3 = 0.0;
  double j3 = 0.0;
  if (-ac >= 0.0) {
    t3 = -ac * j_max_inverse;
    j3 = state_limit.j_max;
  } else {
    t3 = -ac * j_min_inverse;
    j3 = state_limit.j_min;
  }
  double v3 = ac * t3 + j3 * t3 * t3 * 0.5;

  double v2 = state_limit.v_end - v1 - v3;

  if (da == 0) {
    t2 = 0.0;
  } else {
    t2 = v2 / ac;
  }

  if (t2 < 0.0) {
    if (da == 1) {
      double an = std::sqrt((2.0 * (state_limit.v_end - init_state.v) +
                             init_state.a * init_state.a * j_max_inverse) /
                            (j_max_inverse - j_min_inverse));
      t1 = std::fabs((an - init_state.a) * j_max_inverse);
      t2 = 0.0;
      t3 = std::fabs(-an * j_min_inverse);
    } else if (da == -1) {
      double an = -std::sqrt((2.0 * (state_limit.v_end - init_state.v) +
                              init_state.a * init_state.a * j_min_inverse) /
                             (j_min_inverse - j_max_inverse));
      t1 = std::fabs((an - init_state.a) * j_min_inverse);
      t2 = 0.0;
      t3 = std::fabs(-an * j_max_inverse);
    }
  }

  SecondOrderParam second_order_param;
  second_order_param.t1 = t1;
  second_order_param.t2 = t1 + t2;
  second_order_param.t3 = t1 + t2 + t3;
  second_order_param.j1 = j1;
  second_order_param.j2 = 0.0;
  second_order_param.j3 = j3;

  return second_order_param;
}

LonState SecondOrderTimeOptimalTrajectory::GetSecondOrderTrajectoryState(
    const LonState& init_state, const SecondOrderParam& second_order_param,
    const double t) {
  if (t <= 0.0) {
    return init_state;
  }
  LonState result_state = init_state;

  LonState tmp_state;
  double t_remain = t;

  double t1 = std::min(t_remain, second_order_param.t1);
  tmp_state.p = init_state.p;
  tmp_state.v = init_state.v;
  tmp_state.a = init_state.a;
  tmp_state.j = second_order_param.j1;
  tmp_state.t = t1;

  result_state = EvaluatePolyState(tmp_state);
  t_remain = t_remain - t1;

  if (t_remain > 0.0) {
    double t2 =
        std::min(t_remain, second_order_param.t2 - second_order_param.t1);

    tmp_state.p = result_state.p;
    tmp_state.v = result_state.v;
    tmp_state.a = result_state.a;
    tmp_state.j = second_order_param.j2;
    tmp_state.t = t2;

    result_state = EvaluatePolyState(tmp_state);
    t_remain = t_remain - t2;
  }

  if (t_remain > 0.0) {
    double t3 =
        std::min(t_remain, second_order_param.t3 - second_order_param.t2);

    tmp_state.p = result_state.p;
    tmp_state.v = result_state.v;
    tmp_state.a = result_state.a;
    tmp_state.j = second_order_param.j3;
    tmp_state.t = t3;

    result_state = EvaluatePolyState(tmp_state);
    t_remain = t_remain - t3;
  }

  if (t_remain > 0.0) {
    tmp_state.p = result_state.p;
    tmp_state.v = result_state.v;
    tmp_state.a = 0.0;
    tmp_state.j = 0.0;
    tmp_state.t = t_remain;

    result_state = EvaluatePolyState(tmp_state);
  }

  result_state.t = t;
  return result_state;
}

LonState SecondOrderTimeOptimalTrajectory::EvaluatePolyState(
    const LonState& input_state) {
  const double p0 = input_state.p;
  const double v0 = input_state.v;
  const double a0 = input_state.a;
  const double jerk = input_state.j;
  const double t = input_state.t;
  const double t_square = t * t;
  const double t_cube = t * t * t;

  LonState result_state;
  result_state.j = jerk;
  result_state.a = a0 + jerk * t;
  result_state.v = v0 + a0 * t + 0.5 * jerk * t_square;
  result_state.p = p0 + v0 * t + 0.5 * a0 * t_square + jerk * t_cube / 6.0;

  return result_state;
}
}  // namespace planning

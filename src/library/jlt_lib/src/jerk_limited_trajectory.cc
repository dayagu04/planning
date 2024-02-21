#include "jerk_limited_trajectory.h"
#include <math.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include "jerk_limited_trajectory_define.h"
#include "src/common/log.h"
namespace {

constexpr double kEpsilon = 1e-10;
}

/* Jerk limited trajectry algorithm provides two systems to solve problem
 *  (1) Second-order system: bringing system from an arbitrary initial state to
 * a velocity setpoint (2) Third-order system: bringing system from an arbitrary
 * initial state to a position setpoint
 *
 * Note:
 *  (1) The vaule of v_min, a_min, j_min must be negative.
 *  (2) The vaule of v_max, a_max, j_max must be positive.
 *  (3) When JltType is SOLVE_RELATIVE_POS, the algorithm can compute a
 * trajectory to reach end state in a relative coordinate.
 */

namespace planning {
namespace jlt {

bool JerkLimitedTrajectory::Update(const PointState &init_point_state,
                                   const StateLimitParam &state_limit,
                                   const JltType jlt_type, const double dt) {
  init_point_state_ = init_point_state;
  state_limit_ = state_limit;
  bool is_input_limit_valid = false;

  switch (jlt_type) {
    case JltType::SOLVE_VEL: {
      is_solve_velocity_ = true;
      is_input_limit_valid =
          (state_limit_.v_desire >= 0.0 && state_limit_.a_min < 0.0 &&
           state_limit_.a_max > 0.0 && state_limit_.j_min < 0.0 &&
           state_limit_.j_max > 0.0);
      if (!is_input_limit_valid) {
        LOG_ERROR("Invalid jlt input\n");
        return false;
      }
      velocity_param_ = VelocityTargetSolver(state_limit_, init_point_state_,
                                             state_limit_.v_desire);
      break;
    }
    case JltType::SOLVE_POS: {
      is_solve_position_ = true;
      is_input_limit_valid =
          (state_limit_.v_min < 0.0 && state_limit_.v_max > 0.0 &&
           state_limit_.a_min < 0.0 && state_limit_.a_max > 0.0 &&
           state_limit_.j_min < 0.0 && state_limit_.j_max > 0.0);
      if (!is_input_limit_valid) {
        LOG_ERROR("Invalid jlt input\n");
        return false;
      }
      position_param_ = PositionTargetSolver(state_limit_, init_point_state_,
                                             state_limit_.p_desire);
      break;
    }
    default:
      return false;
  }

  s_curve_.clear();
  v_curve_.clear();
  a_curve_.clear();
  j_curve_.clear();
  GenerateCurve(dt);
  return true;
}

bool JerkLimitedTrajectory::Update(const PointState &init_point_state,
                                   const StateLimitParam &state_limit,
                                   const JltType jlt_type,
                                   const CoordinateParam &base_coordinate) {
  init_point_state_ = init_point_state;
  state_limit_ = state_limit;
  bool is_input_limit_valid = false;
  is_input_limit_valid =
      (state_limit_.v_min < 0.0 && state_limit_.v_max > 0.0 &&
       state_limit_.a_min < 0.0 && state_limit_.a_max > 0.0 &&
       state_limit_.j_min < 0.0 && state_limit_.j_max > 0.0);
  if (!is_input_limit_valid) {
    LOG_ERROR("invalid jlt input\n");
    return false;
  }
  PointState relative_init_state = init_point_state_;
  relative_init_state.p -= base_coordinate.s_start;
  relative_init_state.v -= base_coordinate.v;

  StateLimitParam relative_state_limit = state_limit;
  relative_state_limit.v_min -= base_coordinate.v;
  relative_state_limit.v_min = std::min(relative_state_limit.v_min, -kEpsilon);
  relative_state_limit.v_max -= base_coordinate.v;
  relative_state_limit.v_max = std::max(relative_state_limit.v_max, kEpsilon);

  position_param_ = PositionTargetSolver(
      relative_state_limit, relative_init_state, relative_state_limit.p_desire);
  relative_coordinate_param_ = base_coordinate;
  return true;
}

VelocityParam JerkLimitedTrajectory::VelocityTargetSolver(
    const StateLimitParam &state_limit, const PointState &point_state,
    const double &v_des) {
  double v_end = 0.0;
  double a_cruise = 0.0;
  int dir_a = 1;
  double a_0 = point_state.a;
  double v_0 = point_state.v;

  if (a_0 >= 0) {
    v_end = v_0 + a_0 * std::fabs(a_0 / state_limit.j_min) / 2;
  } else {
    v_end = v_0 + a_0 * std::fabs(a_0 / state_limit.j_max) / 2;
  }

  ((v_des - v_end) > 0) ? dir_a = 1
                        : (((v_des - v_end) < 0) ? dir_a = -1 : dir_a = 0);

  (dir_a == 1) ? a_cruise = state_limit.a_max
               : ((dir_a == -1) ? a_cruise = state_limit.a_min : a_cruise = 0);

  VelocityParam velocity_param;
  double delt_a = a_cruise - a_0;
  double t1 = 0.0;
  double delt_v_1 = 0.0;
  if (delt_a >= 0) {
    t1 = delt_a / state_limit.j_max;
    velocity_param.j1 = state_limit.j_max;
  } else {
    t1 = delt_a / state_limit.j_min;
    velocity_param.j1 = state_limit.j_min;
  }
  double t1_square = t1 * t1;
  delt_v_1 = v_0 + a_0 * t1 + velocity_param.j1 * t1_square / 2;

  double t3 = 0.0;
  if (-a_cruise >= 0) {
    t3 = -a_cruise / state_limit.j_max;
    velocity_param.j3 = state_limit.j_max;
  } else {
    t3 = -a_cruise / state_limit.j_min;
    velocity_param.j3 = state_limit.j_min;
  }

  double t3_square = t3 * t3;
  double delt_v_3 =
      (a_cruise * t3 + velocity_param.j3 * t3_square / 2);  // 符号待定
  double delt_v_2 = v_des - delt_v_1 - delt_v_3;

  double t2 = 0.0;
  if (dir_a == 0) {
    t2 = 0.0;
  } else {
    t2 = delt_v_2 / a_cruise;
  }

  double a_0_square = a_0 * a_0;
  if (t2 < 0) {
    if (dir_a == 1) {
      double a_n = sqrt((2 * (v_des - v_0) + a_0_square / state_limit.j_max) /
                        (1.0 / state_limit.j_max - 1.0 / state_limit.j_min));
      t1 = (a_n - a_0) / state_limit.j_max;
      t2 = 0.0;
      t3 = -a_n / state_limit.j_min;
    } else if (dir_a == -1) {
      double a_n = -sqrt((2 * (v_des - v_0) + a_0_square / state_limit.j_min) /
                         (1.0 / state_limit.j_min - 1.0 / state_limit.j_max));
      t1 = (a_n - a_0) / state_limit.j_min;
      t2 = 0.0;
      t3 = -a_n / state_limit.j_max;
    }
  }

  velocity_param.T1 = t1;
  velocity_param.T2 = t2 + t1;
  velocity_param.T3 = t3 + t2 + t1;

  return velocity_param;
}

PointState JerkLimitedTrajectory::UpdateTrajectory(
    const PointState &init_point_state, const double &t,
    const VelocityParam &velocity_param) {
  if (t < 0) {
    return init_point_state;
  }

  PointState result_state;
  double p_0 = init_point_state.p;
  double v_0 = init_point_state.v;
  double a_0 = init_point_state.a;

  double t_remain = t;
  double t1 = std::min(t, velocity_param.T1);
  result_state = UpdatePoint(p_0, v_0, a_0, velocity_param.j1, t1);

  t_remain = t - t1;
  if (t_remain > 0) {
    double t2 = std::min(t_remain, (velocity_param.T2 - velocity_param.T1));
    result_state =
        UpdatePoint(result_state.p, result_state.v, result_state.a, 0, t2);
    t_remain = t_remain - t2;
  }

  if (t_remain > 0) {
    double t3 = std::min(t_remain, (velocity_param.T3 - velocity_param.T2));
    result_state = UpdatePoint(result_state.p, result_state.v, result_state.a,
                               velocity_param.j3, t3);
    t_remain = t_remain - t3;
  }

  if (t_remain > 0) {
    result_state = UpdatePoint(result_state.p, result_state.v, 0, 0, t_remain);
  }

  return result_state;
}

PointState JerkLimitedTrajectory::UpdatePoint(const double &x, const double &v,
                                              const double &a, const double &j,
                                              const double &t) {
  PointState ps;
  double t_square = t * t;
  double t_cube = t * t * t;
  ps.j = j;
  ps.a = a + j * t;
  ps.v = v + a * t + 0.5 * j * t_square;
  ps.p = x + v * t + 0.5 * a * t_square + 1.0 / 6 * j * t_cube;
  return ps;
}

PositionParam JerkLimitedTrajectory::PositionTargetSolver(
    const StateLimitParam &state_limit, const PointState &init_state,
    const double &p_des) {
  PositionParam position_param;
  VelocityParam vel_sp;
  PointState point_state_sp;
  point_state_sp.v = init_state.v;
  point_state_sp.a = init_state.a;
  vel_sp = VelocityTargetSolver(state_limit, point_state_sp, 0);
  point_state_sp = UpdateTrajectory(init_state, vel_sp.T3, vel_sp);

  int dir_p = 1;
  double v_cruise = 0.0;

  ((p_des - point_state_sp.p) > 0)
      ? dir_p = 1
      : (((p_des - point_state_sp.p) < 0) ? dir_p = -1 : dir_p = 0);

  (dir_p == 1) ? v_cruise = state_limit.v_max
               : ((dir_p == -1) ? v_cruise = state_limit.v_min : v_cruise = 0);

  VelocityParam velocity_param_a;
  VelocityParam velocity_param_b;
  VelocityParam velocity_param_pb;
  PointState point_state_fa;
  PointState point_state_fb;
  PointState point_state_ab;
  PointState point_state_pb;

  point_state_fa.v = init_state.v;
  // PS_fa.p = init_state.p;
  point_state_fa.a = init_state.a;

  velocity_param_a =
      VelocityTargetSolver(state_limit, point_state_fa, v_cruise);
  point_state_fa =
      UpdateTrajectory(init_state, velocity_param_a.T3, velocity_param_a);

  point_state_fb.v = v_cruise;
  point_state_fb.a = 0.0;

  point_state_ab.p = point_state_fa.p;
  point_state_ab.v = v_cruise;
  point_state_ab.a = 0.0;

  velocity_param_b = VelocityTargetSolver(state_limit, point_state_fb, 0);
  point_state_fb =
      UpdateTrajectory(point_state_ab, velocity_param_b.T3, velocity_param_b);
  double delt_p = (point_state_fb.p - p_des);

  double t_cruise = 0.0;
  int dir_pf = 1;
  double t_pb = 0.0;
  (delt_p > 0) ? dir_pf = 1 : ((delt_p < 0) ? dir_pf = -1 : dir_pf = 0);

  if (dir_pf * dir_p <= 0) {
    t_cruise = -delt_p / (v_cruise + kEpsilon);
    t_pb = velocity_param_a.T3;
    position_param.switch_time = t_pb;
    position_param.velocity_param_b = velocity_param_b;
  } else {
    t_cruise = 0.0;
    double t_H = velocity_param_a.T3;
    double t_L = 0.0;

    for (int i = 0; i < 100; ++i) {
      t_pb = (t_H + t_L) / 2;
      point_state_pb = UpdateTrajectory(init_state, t_pb, velocity_param_a);
      velocity_param_pb = VelocityTargetSolver(state_limit, point_state_pb, 0);
      point_state_fb = UpdateTrajectory(point_state_pb, velocity_param_pb.T3,
                                        velocity_param_pb);
      double delt_p = (point_state_fb.p - p_des);

      (delt_p > 0) ? dir_pf = 1 : ((delt_p < 0) ? dir_pf = -1 : dir_pf = 0);

      if (dir_pf * dir_p < 0) {
        t_L = t_pb;
      } else {
        t_H = t_pb;
      }
      if (std::fabs(delt_p) < 0.001) {
        position_param.switch_time = t_pb;
        position_param.velocity_param_b = velocity_param_pb;
        break;
      }
    }
  }
  position_param.velocity_param_a = velocity_param_a;
  position_param.curise_time = t_cruise;
  position_param.curise_velocity = v_cruise;
  return position_param;
}

PointState JerkLimitedTrajectory::UpdateFinalTrajectory(
    const PointState &init_state, const PositionParam &position_param,
    const double &t) {
  if (t <= 0.0) {
    return init_state;
  }

  PointState point_state_result;

  if (t >= 0 && t < position_param.switch_time) {
    point_state_result =
        UpdateTrajectory(init_state, t, position_param.velocity_param_a);
  } else if (t >= position_param.switch_time &&
             t < (position_param.switch_time + position_param.curise_time)) {
    point_state_result =
        UpdateTrajectory(init_state, position_param.velocity_param_a.T3,
                         position_param.velocity_param_a);
    point_state_result.p =
        point_state_result.p + position_param.curise_velocity *
                                   (t - position_param.velocity_param_a.T3);
  } else if (t >= (position_param.switch_time + position_param.curise_time)) {
    if (position_param.curise_time > 0) {
      point_state_result =
          UpdateTrajectory(init_state, position_param.switch_time,
                           position_param.velocity_param_a);
      double p_c = point_state_result.p +
                   position_param.curise_velocity * position_param.curise_time;
      point_state_result.p = p_c;
      point_state_result.v = position_param.curise_velocity;
      point_state_result.a = 0;
      point_state_result = UpdateTrajectory(
          point_state_result,
          t - (position_param.switch_time + position_param.curise_time),
          position_param.velocity_param_b);
    } else {
      point_state_result =
          UpdateTrajectory(init_state, position_param.switch_time,
                           position_param.velocity_param_a);
      point_state_result = UpdateTrajectory(
          point_state_result,
          t - (position_param.switch_time + position_param.curise_time),
          position_param.velocity_param_b);
    }
  }
  return point_state_result;
}

double JerkLimitedTrajectory::Evaluate(const int order, const double param) {
  if (is_solve_velocity_) {
    double sum_time = ParamLength();
    switch (order) {
      case 0:
        return UpdateTrajectory(init_point_state_, param, velocity_param_).p;
      case 1:
        if (param > sum_time) {
          return state_limit_.v_desire;
        }
        return UpdateTrajectory(init_point_state_, param, velocity_param_).v;
      case 2:
        if (param > sum_time) {
          return 0.0;
        }
        return UpdateTrajectory(init_point_state_, param, velocity_param_).a;
      case 3:
        if (param > sum_time) {
          return 0.0;
        }
        return UpdateTrajectory(init_point_state_, param, velocity_param_).j;
      default:
        return 0.0;
    }
  } else if (is_solve_position_) {
    double sum_time = ParamLength();
    switch (order) {
      case 0:
        if (param > sum_time) {
          return state_limit_.p_desire;
        }
        return UpdateFinalTrajectory(init_point_state_, position_param_, param)
            .p;
      case 1:
        if (param > sum_time) {
          return 0.0;
        }
        return UpdateFinalTrajectory(init_point_state_, position_param_, param)
            .v;
      case 2:
        if (param > sum_time) {
          return 0.0;
        }
        return UpdateFinalTrajectory(init_point_state_, position_param_, param)
            .a;
      case 3:
        if (param > sum_time) {
          return 0.0;
        }
        return UpdateFinalTrajectory(init_point_state_, position_param_, param)
            .j;
      default:
        return 0.0;
    }
  } else {
    double sum_time = ParamLength();
    const double base_s = relative_coordinate_param_.s_start +
                          relative_coordinate_param_.v * param;
    const double base_v = relative_coordinate_param_.v;
    switch (order) {
      case 0:
        if (param > sum_time) {
          return state_limit_.p_desire + base_s;
        }
        return UpdateFinalTrajectory(init_point_state_, position_param_, param)
                   .p +
               base_s;
      case 1:
        if (param > sum_time) {
          return base_v;
        }
        return UpdateFinalTrajectory(init_point_state_, position_param_, param)
                   .v +
               base_v;
      case 2:
        if (param > sum_time) {
          return 0.0;
        }
        return UpdateFinalTrajectory(init_point_state_, position_param_, param)
            .a;
      case 3:
        if (param > sum_time) {
          return 0.0;
        }
        return UpdateFinalTrajectory(init_point_state_, position_param_, param)
            .j;
      default:
        return 0.0;
    }
  }
}

double JerkLimitedTrajectory::ParamLength() const {
  if (is_solve_velocity_) {
    return velocity_param_.T3;
  } else {
    return position_param_.velocity_param_a.T3 + position_param_.curise_time +
           position_param_.velocity_param_b.T3;
  }
}

bool JerkLimitedTrajectory::GenerateCurve(const double delta_t) {
  double total_time = ParamLength();
  if (total_time < 1e-3) {
    return false;
  }
  double time_stamp = 0.0;
  s_curve_.reserve(total_time / delta_t + 1);
  v_curve_.reserve(total_time / delta_t + 1);
  a_curve_.reserve(total_time / delta_t + 1);
  j_curve_.reserve(total_time / delta_t + 1);
  for (int i = 0; i < total_time / delta_t; ++i) {
    double s = Evaluate(0, time_stamp);
    double v = Evaluate(1, time_stamp);
    double a = Evaluate(2, time_stamp);
    double j = Evaluate(3, time_stamp);
    s_curve_.emplace_back(s);
    v_curve_.emplace_back(v);
    a_curve_.emplace_back(a);
    j_curve_.emplace_back(j);
    time_stamp = time_stamp + delta_t;
    // std::cout << "position = " << s_curve_[i] << " "
    //           << "velocity = " << v_curve_[i] << " "
    //           << "acc = " << a_curve_[i] << " "
    //           << "jerk = " << j_curve_[i] << std::endl;
  }
  return true;
}

}  // namespace jlt
}  // namespace planning
#include "uniform_jerk_curve.h"
#include "algorithm"
#include "cmath"

namespace planning {
UniformJerkCurve::UniformJerkCurve(const StateLimit& state_limit,
                                   const LonState& lon_state,
                                   bool is_upper_limit) {
  jerk_curve_coffi_.clear();
  lon_state_ = lon_state;
  state_limit_ = state_limit;
  state_limit_.v_end = is_upper_limit ? state_limit_.v_max : state_limit_.v_min;
  BuildUniformJerkTraj();
}

void UniformJerkCurve::BuildUniformJerkTraj() {
  jerk_curve_coffi_.clear();
  SecondOrderParam second_order_param =
      SecondOrderTimeOptimalTrajectory::VelocityTargetSolver(lon_state_,
                                                             state_limit_);
  if (second_order_param.t1 > kZeroEpsilon) {
    jerk_curve_coffi_.cofficients.push_back({lon_state_.p, lon_state_.v,
                                             lon_state_.a / 2.0,
                                             second_order_param.j1 / 6.0});
    jerk_curve_coffi_.time_points.push_back(0.0);
  }
  if ((second_order_param.t2 - second_order_param.t1) > kZeroEpsilon) {
    if (jerk_curve_coffi_.time_points.size() > 0) {
      jerk_curve_coffi_.cofficients.push_back(
          {CalcS(second_order_param.t1), CalcV(second_order_param.t1),
           CalcAcc(second_order_param.t1) / 2.0, second_order_param.j2 / 6.0});
      jerk_curve_coffi_.time_points.push_back(second_order_param.t1);
    } else {
      jerk_curve_coffi_.cofficients.push_back({lon_state_.p, lon_state_.v,
                                               lon_state_.a / 2.0,
                                               second_order_param.j2 / 6.0});
      jerk_curve_coffi_.time_points.push_back(0.0);
    }
  }
  if ((second_order_param.t3 - second_order_param.t2) > kZeroEpsilon) {
    if (jerk_curve_coffi_.time_points.size() > 0) {
      jerk_curve_coffi_.cofficients.push_back(
          {CalcS(second_order_param.t2), CalcV(second_order_param.t2),
           CalcAcc(second_order_param.t2) / 2.0, second_order_param.j3 / 6.0});
      jerk_curve_coffi_.time_points.push_back(second_order_param.t2);
    } else {
      jerk_curve_coffi_.time_points.push_back(0.0);
      jerk_curve_coffi_.cofficients.push_back({lon_state_.p, lon_state_.v,
                                               lon_state_.a / 2.0,
                                               second_order_param.j3 / 6.0});
    }
  }
  if (second_order_param.t3 > kZeroEpsilon) {
    if (jerk_curve_coffi_.time_points.size() > 0) {
      jerk_curve_coffi_.cofficients.push_back(
          {CalcS(second_order_param.t3), state_limit_.v_end, 0.0, 0.0});
      jerk_curve_coffi_.time_points.push_back(second_order_param.t3);
    } else {
      jerk_curve_coffi_.cofficients.push_back(
          {lon_state_.p, lon_state_.v, 0.0, 0.0});
      jerk_curve_coffi_.time_points.push_back(0.0);
    }
  }
  arrived_t_ = second_order_param.t3;
  arrived_v_ = state_limit_.v_end;
  arrived_s_ = CalcS(arrived_t_);
  arrived_a_ = 0.0;
}

double UniformJerkCurve::CalcS(const double t) const {
  double s = 0.0;
  for (int i = 0; i < jerk_curve_coffi_.time_points.size(); ++i) {
    if (i == jerk_curve_coffi_.time_points.size() - 1) {
      double during_t = t - jerk_curve_coffi_.time_points[i];
      s = jerk_curve_coffi_.cofficients[i][0] +
          jerk_curve_coffi_.cofficients[i][1] * during_t +
          jerk_curve_coffi_.cofficients[i][2] * during_t * during_t +
          jerk_curve_coffi_.cofficients[i][3] * during_t * during_t * during_t;
    } else {
      if (t < jerk_curve_coffi_.time_points[i + 1] + kZeroEpsilon) {
        double during_t = t - jerk_curve_coffi_.time_points[i];
        s = jerk_curve_coffi_.cofficients[i][0] +
            jerk_curve_coffi_.cofficients[i][1] * during_t +
            jerk_curve_coffi_.cofficients[i][2] * during_t * during_t +
            jerk_curve_coffi_.cofficients[i][3] * during_t * during_t *
                during_t;
        break;
      }
    }
  }
  return s;
}

double UniformJerkCurve::CalcV(const double t) const {
  double v = 0.0;
  for (int i = 0; i < jerk_curve_coffi_.time_points.size(); ++i) {
    if (i == jerk_curve_coffi_.time_points.size() - 1) {
      double during_t = t - jerk_curve_coffi_.time_points[i];
      v = jerk_curve_coffi_.cofficients[i][1] +
          jerk_curve_coffi_.cofficients[i][2] * during_t * 2.0 +
          jerk_curve_coffi_.cofficients[i][3] * during_t * during_t * 3.0;
    } else {
      if (t < jerk_curve_coffi_.time_points[i + 1] + kZeroEpsilon) {
        double during_t = t - jerk_curve_coffi_.time_points[i];
        v = jerk_curve_coffi_.cofficients[i][1] +
            jerk_curve_coffi_.cofficients[i][2] * during_t * 2.0 +
            jerk_curve_coffi_.cofficients[i][3] * during_t * during_t * 3.0;
        break;
      }
    }
  }
  return v;
}

double UniformJerkCurve::CalcAcc(const double t) const {
  double a = 0.0;
  for (int i = 0; i < jerk_curve_coffi_.time_points.size(); ++i) {
    if (i == jerk_curve_coffi_.time_points.size() - 1) {
      double during_t = t - jerk_curve_coffi_.time_points[i];
      a = jerk_curve_coffi_.cofficients[i][2] * 2.0 +
          jerk_curve_coffi_.cofficients[i][3] * during_t * 6.0;
    } else {
      if (t < jerk_curve_coffi_.time_points[i + 1] + kZeroEpsilon) {
        double during_t = t - jerk_curve_coffi_.time_points[i];
        a = jerk_curve_coffi_.cofficients[i][2] * 2.0 +
            jerk_curve_coffi_.cofficients[i][3] * during_t * 6.0;
        break;
      }
    }
  }
  return a;
}

double UniformJerkCurve::CalcJerk(const double t) const {
  double jerk = 0.0;
  for (int i = 0; i < jerk_curve_coffi_.time_points.size(); ++i) {
    if (i == jerk_curve_coffi_.time_points.size() - 1) {
      jerk = jerk_curve_coffi_.cofficients[i][3] * 6.0;
    } else {
      if (t < jerk_curve_coffi_.time_points[i + 1] + kZeroEpsilon) {
        jerk = jerk_curve_coffi_.cofficients[i][3] * 6.0;
        break;
      }
    }
  }
  return jerk;
}
}  // namespace planning
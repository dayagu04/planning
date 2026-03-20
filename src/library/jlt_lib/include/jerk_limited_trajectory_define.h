#pragma once

namespace planning {
namespace jlt {

enum JltType {
  SOLVE_VEL = 0,
  SOLVE_POS = 1,
  SOLVE_RELATIVE_POS = 2,
  JLT_TYPE_MAX = 3
};

struct StateLimitParam {
  double p_desire = 0.0;
  double v_desire = 0.0;
  double v_max = 0.0;
  double v_min = 0.0;
  double a_max = 0.0;
  double a_min = 0.0;
  double j_max = 0.0;
  double j_min = 0.0;
};

struct VelocityParam {
  double j1 = 0.0;
  double j2 = 0.0;
  double j3 = 0.0;
  double T1 = 0.0;
  double T2 = 0.0;
  double T3 = 0.0;
};

struct PositionParam {
  VelocityParam velocity_param_a;
  VelocityParam velocity_param_b;
  double cruise_time = 0.0;
  double switch_time = 0.0;
  double curise_velocity = 0.0;
};

struct PointState {
  double p = 0.0;
  double v = 0.0;
  double a = 0.0;
  double j = 0.0;
};

struct CoordinateParam {
  double s_start = 0.0;
  double v = 0.0;
};

}  // namespace jlt
}  // namespace planning
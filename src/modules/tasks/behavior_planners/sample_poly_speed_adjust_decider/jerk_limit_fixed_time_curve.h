/**
 * @file
 * @brief 用于快速生成存在jerk,v约束的轨迹ref
 */
#pragma once

#include <vector>

#include "jerk_limited_trajectory_define.h"

namespace planning {
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

class JerkLimitedFixedTimeCurve {
 public:
  JerkLimitedFixedTimeCurve() = default;

  ~JerkLimitedFixedTimeCurve() = default;

  bool Update(const PointState &init_point_state,
              const StateLimitParam &state_limit, const JltType jlt_type,
              const double dt);
  bool Update(const PointState &init_point_state,
              const StateLimitParam &state_limit, const JltType jlt_type,
              const CoordinateParam &base_coordinate);

  double Evaluate(const int order, const double param);

  double ParamLength() const;

  const VelocityParam &GetVelocityParam() const { return velocity_param_; };
  const PositionParam &GetPositionParam() const { return position_param_; };
  const std::vector<double> &GetSCurve() const { return s_curve_; };
  const std::vector<double> &GetVelCurve() const { return v_curve_; };
  const std::vector<double> &GetAccCurve() const { return a_curve_; };
  const std::vector<double> &GetJerkCurve() const { return j_curve_; };

  void DebugString() const;

 private:
  VelocityParam VelocityTargetSolver(const StateLimitParam &state_limit,
                                     const PointState &init_state,
                                     const double v_des);

  PointState UpdateTrajectory(const PointState &init_state, const double t,
                              const VelocityParam &velocity_param);

  PointState UpdatePoint(const double x, const double v, const double a,
                         const double j, const double t);

  PositionParam PositionTargetSolver(const StateLimitParam &state_limit,
                                     const PointState &init_state,
                                     const double p_f);

  PointState UpdateFinalTrajectory(const PointState &init_state,
                                   const PositionParam &position_param,
                                   const double t);

  bool GenerateCurve(const double delta_t);

 private:
  // hard constraints
  StateLimitParam state_limit_;
  // init state
  PointState init_point_state_;
  // position solver result
  PositionParam position_param_;
  // velocity solver result
  VelocityParam velocity_param_;
  // relative position
  CoordinateParam relative_coordinate_param_;
  // solve state
  bool is_solve_velocity_ = false;
  bool is_solve_position_ = false;
  // curve
  std::vector<double> s_curve_;
  std::vector<double> v_curve_;
  std::vector<double> a_curve_;
  std::vector<double> j_curve_;
};

}  // namespace planning
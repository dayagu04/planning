/**
 * @file
 * @brief 用于快速生成存在jerk,v约束的轨迹ref
 */
#pragma once

#include <vector>

#include "jerk_limited_trajectory_define.h"

namespace planning {
namespace jlt {

/* Jerk limited trajectry algorithm provides two systems to solve problem
 *  (1) Second-order system: bringing system from an arbitrary initial state to
 * a velocity setpoint;
 *  (2) Third-order system: bringing system from an arbitrary
 * initial state to a position setpoint
 *
 * Note:
 *  (1) The vaule of v_min, a_min, j_min must be negative.
 *  (2) The vaule of v_max, a_max, j_max must be positive.
 *  (3) When JltType is SOLVE_RELATIVE_POS, the algorithm can compute a
 * trajectory to reach end state in a relative coordinate.
 * ref: Safe navigation of quadrotors with jerk limited trajectory
 */
class JerkLimitedTrajectory {
 public:
  JerkLimitedTrajectory() = default;

  ~JerkLimitedTrajectory() = default;

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

}  // namespace jlt
}  // namespace planning
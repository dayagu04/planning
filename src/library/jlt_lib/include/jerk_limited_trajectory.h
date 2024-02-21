/**
 * @file
 * @brief 用于快速生成存在jerk,v约束的轨迹ref
 */
#pragma once

#include <vector>
#include "jerk_limited_trajectory_define.h"

namespace planning {
namespace jlt {

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
  // std::vector<std::vector<double>> GetOutput();

  double Evaluate(const int order, const double param);

  double ParamLength() const;

  VelocityParam GetVelocityParam() { return velocity_param_; };
  PositionParam GetPositionParam() { return position_param_; };
  std::vector<double> GetSCurve() { return s_curve_; };
  std::vector<double> GetVelCurve() { return v_curve_; };
  std::vector<double> GetAccCurve() { return a_curve_; };
  std::vector<double> GetJerkCurve() { return j_curve_; };

 private:
  VelocityParam VelocityTargetSolver(const StateLimitParam &state_limit,
                                     const PointState &init_state,
                                     const double &v_des);

  PointState UpdateTrajectory(const PointState &init_state, const double &t,
                              const VelocityParam &velocity_param);

  PointState UpdatePoint(const double &x, const double &v, const double &a,
                         const double &j, const double &t);

  PositionParam PositionTargetSolver(const StateLimitParam &state_limit,
                                     const PointState &init_state,
                                     const double &p_f);

  PointState UpdateFinalTrajectory(const PointState &init_state,
                                   const PositionParam &position_param,
                                   const double &t);

  bool GenerateCurve(const double delta_t);

 private:
  StateLimitParam state_limit_;
  // init state
  PointState init_point_state_;
  // position
  PositionParam position_param_;
  // velocity
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
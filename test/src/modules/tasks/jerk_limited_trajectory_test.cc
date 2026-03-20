#include "jerk_limited_trajectory.h"
#include <math.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include "gtest/gtest.h"
#include "jerk_limited_trajectory_define.h"

TEST(TestVelJerkLimitedTrajectory, Vel_Jerk_Limited_Trajectory) {
  // init state
  planning::jlt::PointState init_state = {0.0, 0.0, 2.0, 0.0};
  planning::jlt::StateLimitParam limit_state = {0.0, 20.0, 10.0, -1.0,
                                                5.0, -5.0, 7.0,  -7.0};
  // generate curve
  planning::jlt::JerkLimitedTrajectory curve_by_vel;
  curve_by_vel.Update(init_state, limit_state, planning::jlt::SOLVE_VEL, 0.1);

  planning::jlt::VelocityParam VP = curve_by_vel.GetVelocityParam();
  std::vector<double> s_ref = curve_by_vel.GetSCurve();
  std::vector<double> v_ref = curve_by_vel.GetVelCurve();
  std::vector<double> a_ref = curve_by_vel.GetAccCurve();
  std::vector<double> j_ref = curve_by_vel.GetJerkCurve();
  std::cout << "t3 = " << VP.T3 << std::endl;
  for (int i = 0; i < s_ref.size(); ++i) {
    std::cout << "position = " << s_ref[i] << " "
              << "velocity = " << v_ref[i] << " "
              << "acc = " << a_ref[i] << " "
              << "jerk = " << j_ref[i] << std::endl;
  }
}

TEST(TestPosJerkLimitedTrajectory, Pos_Jerk_Limited_Trajectory) {
  // init state
  planning::jlt::PointState init_state = {0.0, 5.0, 0.0, 0.0};
  planning::jlt::StateLimitParam limit_state = {100.0, 0.0,  10.0, -1.0,
                                                5.0,   -5.0, 7.0,  -7.0};
  // generate curve
  planning::jlt::JerkLimitedTrajectory curve_by_pos;
  curve_by_pos.Update(init_state, limit_state, planning::jlt::SOLVE_POS, 0.1);

  planning::jlt::PositionParam PP = curve_by_pos.GetPositionParam();
  std::vector<double> s_ = curve_by_pos.GetSCurve();
  std::vector<double> v_ = curve_by_pos.GetVelCurve();
  std::vector<double> a_ = curve_by_pos.GetAccCurve();
  std::vector<double> j_ = curve_by_pos.GetJerkCurve();
  std::cout << "pa.t3 = " << PP.velocity_param_a.T3 << " "
            << "pb.t3 = " << PP.velocity_param_b.T3 << " "
            << "curise.t = " << PP.curise_time << std::endl;
  for (int i = 0; i < s_.size(); ++i) {
    std::cout << "position = " << s_[i] << " "
              << "velocity = " << v_[i] << " "
              << "acc = " << a_[i] << " "
              << "jerk = " << j_[i] << std::endl;
  }
}

TEST(TestRelativePosJerkLimitedTrajectory,
     Relative_Pos_Jerk_Limited_Trajectory) {
  planning::jlt::PointState init_state;
  init_state.p = 0.0;
  init_state.v = 10.0;
  init_state.a = 0.0;

  const double kJerkUpperBound = 2.0;
  const double kJerkLowerBound = -2.0;
  const double kAccelUpperBound = 2.0;
  const double kAccelLowerBound = -2.0;

  const double kSpeedBuffer = 5.0;
  const double kVelUpperBound = init_state.v + kSpeedBuffer;
  const double epsilon = 1e-4;

  double yield_s = 50;
  double yield_v = 5.0;
  std::cout << "yield v: " << yield_v << " "
            << "yield s: " << yield_s << "\n";

  planning::jlt::CoordinateParam upper_coordinate_param;
  upper_coordinate_param.s_start = yield_s;
  upper_coordinate_param.v = yield_v;

  planning::jlt::StateLimitParam upper_state_limit;
  upper_state_limit.p_desire = 0.0;
  upper_state_limit.v_max = kVelUpperBound;
  upper_state_limit.v_min = -epsilon;
  upper_state_limit.a_max = kAccelUpperBound;
  upper_state_limit.a_min = kAccelLowerBound;
  upper_state_limit.j_max = kJerkUpperBound;
  upper_state_limit.j_min = kJerkLowerBound;

  planning::jlt::JerkLimitedTrajectory curve_by_relat_pos;
  curve_by_relat_pos.Update(init_state, upper_state_limit,
                            planning::jlt::SOLVE_RELATIVE_POS,
                            upper_coordinate_param);

  std::vector<std::pair<double, double>> st_path_ego;
  std::vector<std::pair<double, double>> st_path_obstacle;
  std::vector<std::pair<double, double>> vt_ego;
  std::vector<std::pair<double, double>> vt_obstacle;
  const double end_time = curve_by_relat_pos.ParamLength();
  std::cout << "Total_time: " << end_time << std::endl;
  const double delta_time = 0.1;
  for (double time = 0.0; time < end_time; time += delta_time) {
    double s_ego = curve_by_relat_pos.Evaluate(0, time);
    st_path_ego.emplace_back(time, s_ego);
    double s_obstacle = yield_s + time * yield_v;
    st_path_obstacle.emplace_back(time, s_obstacle);

    double v_ego = curve_by_relat_pos.Evaluate(1, time);
    double v_obstacle = yield_v;
    vt_ego.emplace_back(time, v_ego);
    vt_obstacle.emplace_back(time, v_obstacle);
    std::cout << "ego_pos = " << s_ego << " "
              << "ego_vel = " << v_ego << " "
              << "obs_pos = " << s_obstacle << " "
              << "obs_vel = " << v_obstacle << std::endl;
  }
}

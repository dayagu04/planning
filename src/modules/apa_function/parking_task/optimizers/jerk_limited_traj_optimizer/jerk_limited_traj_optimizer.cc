#include "jerk_limited_traj_optimizer.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include "debug_info_log.h"

namespace planning {

#define DEBUG_OPTIMIZER (1)

bool JerkLimitedTrajOptimizer::Init() { return true; }

void JerkLimitedTrajOptimizer::Execute(const SVPoint& init_point,
                                       const double path_len) {
  delta_time_ = 0.05;
  speed_data_.clear();

  if (path_len < 0.01) {
    solver_state_ = SpeedOptimizerState::NONE;
    return;
  }

  UpdateSolver(init_point, path_len, 0.0);

  RecordDebugInfo();

  return;
}

void JerkLimitedTrajOptimizer::UpdateSolver(const SVPoint& init_point,
                                            const double s_des,
                                            const double v_des) {
  planning::jlt::StateLimitParam state_limit;
  planning::jlt::PointState init_point_state;

  const apa_planner::ParkingSpeedConfig& speed_config =
      apa_param.GetParam().speed_config;

  state_limit.v_max = speed_config.default_cruise_speed;
  state_limit.v_min = -0.1;

  if (init_point.v < 0.1) {
    state_limit.a_max = speed_config.acc_upper;
  } else {
    state_limit.a_max = 0.3;
  }
  state_limit.j_max = speed_config.jerk_upper;
  state_limit.j_min = speed_config.jerk_lower;
  state_limit.p_desire = s_des;
  state_limit.v_desire = v_des;
  state_limit.a_min = -0.2;

  init_point_state.p = init_point.s;
  init_point_state.v = init_point.v;
  init_point_state.a = init_point.acc;

  jlt::JerkLimitedTrajectory solver;
  double acc_step = -0.03;

  // update min deceleration
  double min_dec = -10.0;
  int sampling_num = std::ceil(min_dec / acc_step);

  bool valid_resolution = false;

  for (int i = 0; i < sampling_num; i++) {
    if (SamplingAccLowerBound(init_point_state, state_limit, &solver)) {
      valid_resolution = true;
      break;
    }

#if DEBUG_OPTIMIZER
    DebugJLTSpeed(&solver, &state_limit);
#endif

    state_limit.a_min += acc_step;
  }

  solver_state_ = SpeedOptimizerState::SUCCESS;
  if (valid_resolution) {
    CopySpeedData(&solver);
  } else {
    UpdateFallbackTraj(init_point, s_des);
  }

  return;
}

bool JerkLimitedTrajOptimizer::SamplingAccLowerBound(
    const planning::jlt::PointState& init_point,
    const planning::jlt::StateLimitParam& state_limit,
    jlt::JerkLimitedTrajectory* solver) {
  bool state = solver->Update(init_point, state_limit, planning::jlt::SOLVE_POS,
                              delta_time_);

  // check valid
  const std::vector<double>& s_profile = solver->GetSCurve();
  const std::vector<double>& v_profile = solver->GetVelCurve();

  if (s_profile.empty() || v_profile.empty()) {
#if DEBUG_OPTIMIZER
    ILOG_INFO << "jlt fail";
#endif
    return false;
  }

  // check speed
  for (size_t i = v_profile.size() - 1; i > 0; i--) {
    if (v_profile[i] < -0.001) {
#if DEBUG_OPTIMIZER
      ILOG_INFO << "jlt speed is less than 0";
#endif
      return false;
    }
  }

  for (size_t i = s_profile.size() - 1; i > 0; i--) {
    if (s_profile[i] > state_limit.p_desire + 0.001) {
#if DEBUG_OPTIMIZER
      ILOG_INFO << "jlt distance is bigger than path s";
#endif
      return false;
    }
  }

  return true;
}

void JerkLimitedTrajOptimizer::CopySpeedData(
    jlt::JerkLimitedTrajectory* solver) {
  const std::vector<double>& s_profile = solver->GetSCurve();
  const std::vector<double>& v_profile = solver->GetVelCurve();
  const std::vector<double>& acc_profile = solver->GetAccCurve();
  const std::vector<double>& jerk_profile = solver->GetJerkCurve();

  for (int i = 0; i < s_profile.size(); ++i) {
    speed_data_.AppendSpeedPoint(s_profile[i], delta_time_ * i, v_profile[i],
                                 acc_profile[i],
                                 jerk_profile[i]);

#if DEBUG_OPTIMIZER
    ILOG_INFO << "i = " << i << ",s = " << s_profile[i]
              << ", t = " << delta_time_ * i << ",v = " << v_profile[i];

#endif
  }

  return;
}

void JerkLimitedTrajOptimizer::DebugJLTSpeed(
    jlt::JerkLimitedTrajectory* solver,
    const planning::jlt::StateLimitParam* constraints) {
  ILOG_INFO << "acc lower = " << constraints->a_min;

  const std::vector<double>& s_profile = solver->GetSCurve();
  const std::vector<double>& v_profile = solver->GetVelCurve();
  const std::vector<double>& acc_profile = solver->GetAccCurve();
  const std::vector<double>& jerk_profile = solver->GetJerkCurve();

  for (int i = 0; i < s_profile.size(); ++i) {
    ILOG_INFO << "i = " << i << ",s = " << s_profile[i]
              << ", t = " << delta_time_ * i << ",v = " << v_profile[i]
              << ", acc =" << acc_profile[i] << ",jerk = " << jerk_profile[i];

  }

  return;
}

void JerkLimitedTrajOptimizer::UpdateFallbackTraj(const SVPoint& init_point,
                                                  const double s_des) {
  if (init_point.v < 0.01) {
    double half_s = s_des/2.0;
    double acc = 0.3;
    double dec = -0.2;

    double s1 = (init_point.v * init_point.v + 2 * acc * s_des) /
                (2.0 * acc - 2.0 * dec);
    double max_v = std::sqrt(2 * acc * s1 + init_point.v * init_point.v);

    double time1 = (max_v- init_point.v)/acc;
    double time2   = -max_v / dec + time1;

    speed_data_.clear();
    double time = 0.0;
    double p;
    double v;

    // speed up
    int point_size = std::ceil(time1 / delta_time_);
    for (int i = 0; i < point_size; i++) {
      p = init_point.v * time + 0.5 * acc * time * time;
      v = init_point.v + acc * time;

      speed_data_.AppendSpeedPoint(p, time, v, acc, 0.0);

      time += delta_time_;
      time = std::min(time, time1);
    }

    // speed down
    time += delta_time_;
    point_size = std::ceil(time2 - time1 / delta_time_);
    for (int i = 0; i < point_size; i++) {
      p = max_v * time + 0.5 * dec * time * time;
      v = max_v + dec * time;

      speed_data_.AppendSpeedPoint(p, time, v, dec, 0.0);

      time += delta_time_;
      time = std::min(time, time2);
    }

  } else {
    double dec = -init_point.v * init_point.v / 2.0 / s_des;
    double total_time = -init_point.v / dec;

    speed_data_.clear();
    double time = 0.0;
    double p;
    double v;
    double acc;
    int point_size = std::ceil(total_time / delta_time_);
    for (int i = 0; i < point_size; i++) {
      p = init_point.v * time + 0.5 * dec * time * time;
      v = init_point.v + dec * time;

      speed_data_.AppendSpeedPoint(p, time, v, dec, 0.0);

      time += delta_time_;
      time = std::min(time, total_time);
    }
  }

  return;
}

void JerkLimitedTrajOptimizer::RecordDebugInfo() {
  auto& debug_ = DebugInfoManager::GetInstance().GetDebugInfoPb();
  common::ApaSpeedDebug* speed_debug = debug_->mutable_apa_speed_debug();

  common::StPoint2D proto_point;
  for (size_t i = 0; i < speed_data_.size(); i++) {
    const SpeedPoint& point = speed_data_[i];
    proto_point.set_s(point.s);
    proto_point.set_t(point.t);
    proto_point.set_vel(point.v);
    proto_point.set_acc(point.a);
    proto_point.set_jerk(point.da);

    speed_debug->add_jlt_profile()->CopyFrom(proto_point);
  }

  return;
}

}  // namespace planning
#include "jerk_limited_traj_optimizer.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "debug_info_log.h"
#include "ifly_time.h"
#include "log_glog.h"

namespace planning {
namespace apa_planner {

#define DEBUG_OPTIMIZER (0)
#define DEBUG_SAMPLING (0)
#define DEBUG_RESULT (1)

bool JerkLimitedTrajOptimizer::Init() {
  config_.Init();
  speed_data_.clear();
  ClearDebugInfo();

  state_ = TaskExcuteState::NONE;
  fail_code_ = JLTFailCode::NONE;

  return true;
}

void JerkLimitedTrajOptimizer::Execute(
    const SVPoint& stitch_speed_point, const SVPoint& ego_speed_point,
    const std::vector<pnc::geometry_lib::PathPoint>& path,
    const SpeedDecisions* speed_decisions) {
  Init();

  // sanity check
  double total_s = 0.0;
  if (path.size() > 1) {
    total_s = path.back().s;
  }

  stop_decision_ = GetCloseStopDecision(speed_decisions);
  if (stop_decision_ != nullptr) {
    total_s = std::min(
        total_s, stop_decision_->path_s - stop_decision_->lon_decision_buffer);
  }
  ILOG_INFO << "total s = " << total_s;

  double opt_start_time = IflyTime::Now_ms();

  if (total_s < config_.min_path_dist_for_veh_starting) {
    GenerateStoppingTraj(ego_speed_point, total_s);
  } else {
    UpdatePositionTargetSolver(stitch_speed_point, total_s, 0.0);
  }

  if (speed_data_.empty()) {
    FallbackTrajByExpectState();
  }

  RecordDebugInfo();
  state_ = TaskExcuteState::SUCCESS;

#if DEBUG_RESULT
  // TaskDebug();
#endif

  ILOG_INFO << "jlt optimizer time = " << IflyTime::Now_ms() - opt_start_time;

  return;
}

const bool JerkLimitedTrajOptimizer::GenerateJLTSpeed(const SVPoint& init_point,
                                                      const double s_des,
                                                      const double v_des) {
  // sanity check
  if (s_des < 0.05) {
    fail_code_ = JLTFailCode::SHORT_PATH;
    return false;
  }

  planning::jlt::StateLimitParam state_limit;
  planning::jlt::PointState init_point_state;

  const ParkingSpeedConfig& speed_config = apa_param.GetParam().speed_config;

  state_limit.v_max = speed_config.default_cruise_speed;
  state_limit.v_min = -0.1;

  if (s_des > speed_config.path_thresh_for_acc_bound) {
    state_limit.a_max = speed_config.long_path_acc_upper;
  } else {
    state_limit.a_max = speed_config.short_path_acc_upper;
  }
  state_limit.j_max = 10.0;
  state_limit.j_min = -10.0;
  state_limit.p_desire = s_des;
  state_limit.v_desire = v_des;
  state_limit.a_min = -0.1;

  init_point_state.p = init_point.s;
  init_point_state.v = init_point.v;
  // add acc slack variable.
  double acc_slack_variable = 0.1;
  if (init_point.acc > acc_slack_variable) {
    init_point_state.a = init_point.acc - acc_slack_variable;
  } else if (init_point.acc < -acc_slack_variable) {
    init_point_state.a = init_point.acc + acc_slack_variable;
  }
  init_point_state.a = 0.0;

  // update min deceleration
  double dec_step = -0.03;
  double min_dec = -2.0;
  int sampling_num = std::ceil(min_dec / dec_step);

  bool valid_solution = false;

  for (int i = 0; i < sampling_num; i++) {
    if (SamplingAccLowerBound(init_point_state, state_limit, &solver_)) {
      valid_solution = true;
      break;
    }

#if DEBUG_SAMPLING
    DebugJLTSpeed(&solver_, &state_limit);
#endif

    state_limit.a_min += dec_step;
  }

  if (!valid_solution) {
    ILOG_INFO << "init, p = " << init_point_state.p
              << ", v = " << init_point_state.v
              << ", a = " << init_point_state.a;
    ILOG_INFO << "end , p = " << state_limit.p_desire
              << ",v = " << state_limit.v_desire;
    ILOG_INFO << "a min = " << state_limit.a_min;
  }

  return valid_solution;
}

void JerkLimitedTrajOptimizer::UpdatePositionTargetSolver(
    const SVPoint& init_point, const double s_des, const double v_des) {
  if (GenerateJLTSpeed(init_point, s_des, v_des)) {
    CopySpeedData(&solver_);
  } else {
    GenerateFallbackTraj(init_point, s_des);

    ILOG_INFO << "JLT fail reason = " << static_cast<int>(fail_code_);
  }

  return;
}

bool JerkLimitedTrajOptimizer::SamplingAccLowerBound(
    const planning::jlt::PointState& init_point,
    const planning::jlt::StateLimitParam& state_limit,
    jlt::JerkLimitedTrajectory* solver) {
  bool state = solver->Update(init_point, state_limit, planning::jlt::SOLVE_POS,
                              config_.delta_time);

  // check valid
  const std::vector<double>& s_profile = solver->GetSCurve();
  const std::vector<double>& v_profile = solver->GetVelCurve();

  if (s_profile.empty() || v_profile.empty()) {
#if DEBUG_SAMPLING
    ILOG_INFO << "jlt fail";
#endif
    fail_code_ = JLTFailCode::SOLVER_FAIL;
    return false;
  }

  // check speed
  for (int i = v_profile.size() - 1; i > 0; i--) {
    if (v_profile[i] < -0.001) {
#if DEBUG_SAMPLING
      ILOG_INFO << "jlt speed is less than 0";
#endif
      fail_code_ = JLTFailCode::NEGATIVE_SPEED;
      return false;
    }
  }

  for (int i = s_profile.size() - 1; i > 0; i--) {
    if (s_profile[i] > state_limit.p_desire + 0.001) {
#if DEBUG_SAMPLING
      ILOG_INFO << "jlt distance is bigger than path s";
#endif
      fail_code_ = JLTFailCode::PATH_OVERSHOOT;
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
    speed_data_.AppendSpeedPoint(s_profile[i], config_.delta_time * i,
                                 v_profile[i], acc_profile[i], jerk_profile[i]);

#if DEBUG_OPTIMIZER
    ILOG_INFO << "i = " << i << ",s = " << s_profile[i]
              << ", t = " << config_.delta_time * i << ",v = " << v_profile[i];

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
              << ", t = " << config_.delta_time * i << ",v = " << v_profile[i]
              << ", acc =" << acc_profile[i] << ",jerk = " << jerk_profile[i];
  }

  return;
}

void JerkLimitedTrajOptimizer::GenerateStoppingTraj(
    const SVPoint& init_point, const double s_des) {
  speed_data_.clear();

  // path is none, need emergency brake.
  double dec;
  if (s_des <= 1e-2) {
    dec = -0.5;
    // todo: use remain dist. will be retired.
    if (stop_decision_ != nullptr &&
        stop_decision_->reason_code == LonDecisionReason::REMAIN_DIST) {
      dec = -1.0;
    }
  } else {
    dec = -init_point.v * init_point.v / 2.0 / s_des;
    // limit dec
    if (stop_decision_ != nullptr &&
        stop_decision_->reason_code == LonDecisionReason::REMAIN_DIST) {
      dec = std::max(-1.0, dec);
    } else {
      dec = std::max(-0.5, dec);
    }
  }

  double total_time = 0.0;
  if (std::abs(dec) < 1e-5) {
    total_time = 1.0;
  } else {
    total_time = -init_point.v / dec;
  }
  total_time = std::min(total_time, 3.0);
  total_time = std::max(total_time, config_.delta_time);

  double time = 0.0;
  double p;
  double v;
  int point_size = std::ceil(total_time / config_.delta_time) + 1;

  for (int i = 0; i < point_size; i++) {
    p = init_point.v * time + 0.5 * dec * time * time;
    p = std::max(p, init_point.s);

    v = init_point.v + dec * time;
    v = std::max(0.0, v);

    speed_data_.AppendSpeedPoint(p, time, v, dec, 0.0);

    time += config_.delta_time;
    time = std::min(time, total_time);
    if (v <= 0.0) {
      break;
    }
  }

  // todo: if path is long, need speed up or cruise.

  return;
}

void JerkLimitedTrajOptimizer::GenerateStartingAndStoppingTraj(
    const SVPoint& init_point, const double s_des) {
  speed_data_.clear();

  // second order system.
  // first phase is acc;
  // second phase is cruise;
  // third phase is dec;

  // constraints
  double acc = 0.3;
  double dec = -0.2;
  double max_cruise_speed = 0.5;

  // first phase
  double s1 = -(init_point.v * init_point.v + 2 * dec * s_des) /
              (2.0 * acc - 2.0 * dec);
  double phase1_end_v = std::sqrt(2 * acc * s1 + init_point.v * init_point.v);

  bool need_second_phase = false;
  if (phase1_end_v > max_cruise_speed) {
    need_second_phase = true;
  }

  if (!need_second_phase) {
    double first_phase_total_time = (phase1_end_v - init_point.v) / acc;

    double time = 0.0;
    double p;
    double v;
    // speed up
    int point_size = std::ceil(first_phase_total_time / config_.delta_time);
    for (int i = 0; i < point_size; i++) {
      p = init_point.v * time + 0.5 * acc * time * time;
      v = init_point.v + acc * time;

      speed_data_.AppendSpeedPoint(p, time, v, acc, 0.0);

      time += config_.delta_time;
      time = std::min(time, first_phase_total_time);
    }

    // speed down
    // third phase
    double third_phase_total_time = -phase1_end_v / dec;
    time = config_.delta_time;
    point_size = std::ceil(third_phase_total_time / config_.delta_time);
    for (int i = 0; i < point_size; i++) {
      p = s1 + phase1_end_v * time + 0.5 * dec * time * time;
      v = phase1_end_v + dec * time;

      speed_data_.AppendSpeedPoint(p, first_phase_total_time + time, v, dec,
                                   0.0);

      time += config_.delta_time;
      time = std::min(time, third_phase_total_time);
    }
  } else {
    double first_phase_time = (max_cruise_speed - init_point.v) / acc;
    double first_phase_dist =
        (max_cruise_speed * max_cruise_speed - init_point.v * init_point.v) /
        2.0 / acc;

    double time = 0.0;
    double p;
    double v;
    // speed up
    int point_size = std::ceil(first_phase_time / config_.delta_time);
    for (int i = 0; i < point_size; i++) {
      p = init_point.v * time + 0.5 * acc * time * time;
      v = init_point.v + acc * time;

      speed_data_.AppendSpeedPoint(p, time, v, acc, 0.0);

      time += config_.delta_time;
      time = std::min(time, first_phase_time);
    }

    double third_phase_dist = -max_cruise_speed * max_cruise_speed / 2.0 / dec;

    double second_phase_dist = s_des - third_phase_dist - p;
    double second_phase_time = second_phase_dist / max_cruise_speed;

    point_size = std::ceil(second_phase_time / config_.delta_time);
    time = config_.delta_time;
    for (int i = 0; i < point_size; i++) {
      p = first_phase_dist + max_cruise_speed * time;
      v = max_cruise_speed;

      speed_data_.AppendSpeedPoint(p, first_phase_time + time, v, 0, 0.0);

      time += config_.delta_time;
      time = std::min(time, first_phase_time + second_phase_time);
    }

    // speed down
    // third phase
    double third_phase_total_time = -max_cruise_speed / dec;
    time = config_.delta_time;
    point_size = std::ceil(third_phase_total_time / config_.delta_time);
    for (int i = 0; i < point_size; i++) {
      p = first_phase_dist + second_phase_dist + max_cruise_speed * time +
          0.5 * dec * time * time;
      v = max_cruise_speed + dec * time;
      speed_data_.AppendSpeedPoint(
          p, first_phase_time + second_phase_time + time, v, dec, 0.0);

      time += config_.delta_time;
      time = std::min(
          time, first_phase_time + second_phase_time + third_phase_total_time);
    }
  }

  return;
}

void JerkLimitedTrajOptimizer::GenerateFallbackTraj(const SVPoint& init_point,
                                                    const double s_des) {
  speed_data_.clear();

  if (init_point.v < 0.1) {
    GenerateStartingAndStoppingTraj(init_point, s_des);
  } else {
    GenerateStoppingTraj(init_point, s_des);
  }

  return;
}

void JerkLimitedTrajOptimizer::GenerateStandstillTraj(const SVPoint& init_point,
                                                      const double s_des) {
  speed_data_.clear();

  double time = 0.0;
  double v;
  double delta_s = 0.02;
  double s = 0.0;
  int point_size = std::ceil(s_des / delta_s);
  point_size = std::max(point_size, 1);

  for (int i = 0; i < point_size; i++) {
    v = 0;
    speed_data_.AppendSpeedPoint(s, time, v, -0.3, 0.0);

    s += delta_s;
    time += config_.delta_time;
  }

  return;
}

void JerkLimitedTrajOptimizer::FallbackTrajByExpectState() {
  speed_data_.clear();
  speed_data_.AppendSpeedPoint(0, 0, 0, -0.1, 0.0);

  return;
}

void JerkLimitedTrajOptimizer::ClearDebugInfo() {
  auto& debug_ = DebugInfoManager::GetInstance().GetDebugInfoPb();
  common::ApaSpeedDebug* speed_debug = debug_->mutable_apa_speed_debug();
  speed_debug->clear_jlt_profile();
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

  speed_debug->set_speed_type(common::SpeedProfileType::JLT);

  return;
}

void JerkLimitedTrajOptimizer::TaskDebug() {
  for (size_t i = 0; i < speed_data_.size(); i++) {
    const SpeedPoint& point = speed_data_[i];

    ILOG_INFO << "s = " << point.s << ", t = " << point.t
              << ", velocity = " << point.v << ", acc = " << point.a
              << ", jerk = " << point.da;
  }
  return;
}
}  // namespace apa_planner
}  // namespace planning
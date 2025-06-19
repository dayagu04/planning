#include "piecewise_jerk_qp_speed_optimizer.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "apa_param_config.h"
#include "debug_info_log.h"
#include "dp_speed_common.h"
#include "ifly_time.h"
#include "log_glog.h"

namespace planning {
namespace apa_planner {

#define DECIDER_DEBUG (0)

PiecewiseJerkSpeedQPOptimizer::PiecewiseJerkSpeedQPOptimizer() {}

bool PiecewiseJerkSpeedQPOptimizer::Init() {
  qp_config_.Init();
  state_ = TaskExcuteState::NONE;
  ClearDebugInfo();

  return true;
}

void PiecewiseJerkSpeedQPOptimizer::GenerateInitState(
    const SVPoint& init_point, std::array<double, 3>& init_state) {
  init_state[0] = init_point.s;
  init_state[1] = init_point.v;
  init_state[2] = init_point.acc;

  // 速度已经超调. 如果不限制acc值，osqp肯定无解.
  if (init_point.v >= qp_config_.max_cruise_speed) {
    init_state[2] = 0;
    return;
  }

  // To let osqp solve success, so add acc filt mechanism to ignore measure
  // error. v, a, j: is a second system
  // a = a_0 + j*t;
  // v= v_0 + a_0 *t+1/2 * j *t^2

  // 如果init acc >0，预测速度即将超调。如果速度超调，需要降低acc值;
  // dv = 0, 计算速度最高值
  if (init_point.acc > 0.0) {
    double jerk = -5.0;
    double prediction_v =
        init_point.v - 0.5 * init_point.acc * init_point.acc / jerk;

    // 速度超调，需要修改acc
    if (prediction_v >= qp_config_.max_cruise_speed) {
      double acc =
          std::sqrt((init_point.v - qp_config_.max_cruise_speed) * jerk * 2.0);

      init_state[2] = acc;
      return;
    }
  }

  if (init_point.v < 1e-2 && init_point.acc < 0) {
    init_state[2] = 0;
    return;
  }

  // 如果init acc小于0，预测速度是否会负值. 如果为负值，限制acc幅值.
  if (init_point.acc < 0.0) {
    double jerk = 5.0;
    double prediction_v =
        init_point.v - 0.5 * init_point.acc * init_point.acc / jerk;
    if (prediction_v < 0.0) {
      double acc = -std::sqrt(init_point.v * jerk * 2.0);

      init_state[2] = acc;
      return;
    }
  }

  return;
}

void PiecewiseJerkSpeedQPOptimizer::Execute(
    const SVPoint& init_point, const SpeedLimitProfile* speed_limit_profile,
    const SpeedData& dp_speed_data, const SpeedDecisions* speed_decisions) {
  Init();
  speed_limit_profile_ = speed_limit_profile;
  qp_speed_data_ = dp_speed_data;

  if (speed_limit_profile == nullptr || dp_speed_data.size() < 2) {
    ILOG_INFO << "speed data is null";
    return;
  }

  double opt_start_time = IflyTime::Now_ms();
  ILOG_INFO << "speed qp optimizer";

  std::array<double, 3> init_state;
  GenerateInitState(init_point, init_state);

  init_point.DebugString("stitch start point");
  ILOG_INFO << "smooth init, v = " << init_state[1]
            << ", a = " << init_state[2];

  double total_length = dp_speed_data.back().s;
  const ParkLonDecision* stop_decision = GetCloseStopDecision(speed_decisions);
  if (stop_decision != nullptr) {
    total_length =
        std::min(total_length,
                 stop_decision->path_s - stop_decision->lon_decision_buffer);
  }

  if (total_length < qp_config_.enable_qp_by_path_length) {
    return;
  }

  double total_time = dp_speed_data.back().t;
  if (total_time < 0.1) {
    return;
  }

  if (total_time > qp_config_.time_horizon) {
    delta_time_ = qp_config_.time_resolution;
    num_of_knots_ = std::ceil(qp_config_.time_horizon / delta_time_);
    total_time = qp_config_.time_horizon;
  } else if (total_time > 1.0) {
    delta_time_ = qp_config_.time_resolution;
    num_of_knots_ = std::ceil(total_time / delta_time_);
  } else {
    num_of_knots_ = 20;
    delta_time_ = total_time / num_of_knots_;
  }

  // model
  // f = 1/2 * x^T * H * x + Q * x
  // Ax<=U

  PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_knots_, delta_time_,
                                                   init_state);

  const ParkingSpeedConfig& speed_config = apa_param.GetParam().speed_config;

  // set weight
  piecewise_jerk_problem.set_weight_ddx(qp_config_.acc_weight);
  piecewise_jerk_problem.set_weight_dddx(qp_config_.jerk_weight);

  // set s boundary, no need add s boundary vector.
  piecewise_jerk_problem.set_x_bounds(0.0, total_length + 1e-3);

  // set acc boundary
  if (total_length > speed_config.path_thresh_for_acc_bound) {
    piecewise_jerk_problem.set_ddx_bounds(speed_config.acc_lower,
                                          speed_config.long_path_acc_upper);
  } else {
    piecewise_jerk_problem.set_ddx_bounds(speed_config.acc_lower,
                                          speed_config.short_path_acc_upper);
  }

  // set jerk boundary
  piecewise_jerk_problem.set_dddx_bound(speed_config.jerk_lower,
                                        speed_config.jerk_upper);

  // update ref v, ref_s.
  std::vector<double> x_ref;
  std::vector<double> dx_ref;
  std::vector<double> t_ref;
  std::vector<std::pair<double, double>> ds_bounds;
  SpeedPoint speed_point;
  const double v_lower_bound = 0.0;
  double v_upper_bound;
  double curr_time = 0.0;

  for (int i = 0; i < num_of_knots_; ++i) {
    // get path
    dp_speed_data.EvaluateByTime(curr_time, &speed_point);

    t_ref.emplace_back(speed_point.t);
    x_ref.emplace_back(speed_point.s);
    dx_ref.emplace_back(speed_point.v);

    // get v_upper_bound
    v_upper_bound =
        speed_limit_profile_->GetSpeedLimitByRange(speed_point.s, 0.1);
    v_upper_bound =
        std::max(v_upper_bound, ComputeFastBrakeProfile(init_state, curr_time));

    ds_bounds.emplace_back(v_lower_bound, v_upper_bound);

    curr_time += delta_time_;
    curr_time = std::min(curr_time, total_time);
  }
  if (x_ref.empty() || dx_ref.empty()) {
    ILOG_INFO << "size is null";

    state_ = TaskExcuteState::FAIL;
    return;
  }

  // set x ref
  piecewise_jerk_problem.set_x_ref(qp_config_.ref_s_weight, x_ref);

  // set dx ref
  piecewise_jerk_problem.set_dx_ref(qp_config_.ref_v_weight, dx_ref);

  // set dx boundary
  piecewise_jerk_problem.set_dx_bounds(ds_bounds);

  // todo: set acc ref

  // set end state
  if (dp_speed_data.back().t > qp_config_.time_horizon) {
    std::array<double, 3> weight_end_state = {{0.1, 5.0, 0.0}};
    std::array<double, 3> end_state_ref = {{x_ref.back(), dx_ref.back(), 0.0}};
    piecewise_jerk_problem.set_end_state_ref(weight_end_state, end_state_ref);
  } else {
    std::array<double, 3> end_state_ref = {{total_length, 0.0, 0.0}};
    piecewise_jerk_problem.set_end_state_constriants(end_state_ref);
  }

  // debug info
#if DECIDER_DEBUG
  DebugRef(t_ref, x_ref, dx_ref);
  DebugLinearConstraints(x_ref, ds_bounds);
#endif

  // Solve the problem
  if (!piecewise_jerk_problem.Optimize(6000, qp_config_.optimizer_time_limit)) {
    ILOG_INFO << "Piecewise jerk speed optimizer failed!";
    state_ = TaskExcuteState::FAIL;
    return;
  }

  // Extract output
  const std::vector<double>& s = piecewise_jerk_problem.opt_x();
  const std::vector<double>& ds = piecewise_jerk_problem.opt_dx();
  const std::vector<double>& dds = piecewise_jerk_problem.opt_ddx();

#if DECIDER_DEBUG
  DebugPiecewiseJerkProblem(piecewise_jerk_problem);
#endif

  // sanity check
  if (s.size() <= 2 || ds.size() <= 2 || dds.size() <= 2) {
    ILOG_INFO << "Piecewise jerk speed optimizer failed!";
    state_ = TaskExcuteState::FAIL;
    return;
  }

  qp_speed_data_.clear();
  double jerk_start = (dds[1] - dds[0]) / delta_time_;
  qp_speed_data_.AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], jerk_start);
  for (int i = 1; i < num_of_knots_; ++i) {
    // Avoid the very last points when already stopped
    if (qp_speed_data_.back().v <= 0.0 && ds[i] <= 0.0) {
      break;
    }
    qp_speed_data_.AppendSpeedPoint(s[i], delta_time_ * i, ds[i], dds[i],
                                    (dds[i] - dds[i - 1]) / delta_time_);
  }

  RecordDebugInfo(x_ref, ds_bounds);
  state_ = TaskExcuteState::SUCCESS;

  double opt_time_ms = IflyTime::Now_ms() - opt_start_time;
  ILOG_INFO << "speed qp opt finish, optimizer time = " << opt_time_ms;

  return;
}

void PiecewiseJerkSpeedQPOptimizer::DebugPiecewiseJerkProblem(
    const PiecewiseJerkSpeedProblem& pwj) {
  const std::vector<double>& s = pwj.opt_x();
  const std::vector<double>& ds = pwj.opt_dx();
  const std::vector<double>& dds = pwj.opt_ddx();

  ILOG_INFO << "debug pwj speed point";
  for (int i = 0; i < num_of_knots_; ++i) {
    ILOG_INFO << "For t[" << i * delta_time_ << "], s = " << s[i]
              << ", v = " << ds[i] << ", a = " << dds[i];
  }
  return;
}

void PiecewiseJerkSpeedQPOptimizer::ClearDebugInfo() {
  auto& debug_ = DebugInfoManager::GetInstance().GetDebugInfoPb();
  common::ApaSpeedDebug* speed_debug = debug_->mutable_apa_speed_debug();
  speed_debug->clear_qp_profile();
  speed_debug->clear_qp_speed_constraint();
  return;
}

void PiecewiseJerkSpeedQPOptimizer::RecordDebugInfo(
    const std::vector<double>& x_ref,
    const std::vector<std::pair<double, double>>& s_dot_bounds) {
  auto& debug_ = DebugInfoManager::GetInstance().GetDebugInfoPb();
  common::ApaSpeedDebug* speed_debug = debug_->mutable_apa_speed_debug();
  common::SVGraphSpeedConstraint* qp_speed_constraint_debug =
      speed_debug->mutable_qp_speed_constraint();

  for (size_t i = 0; i < x_ref.size(); i++) {
    qp_speed_constraint_debug->add_s(x_ref[i]);
    qp_speed_constraint_debug->add_v_upper_bound(s_dot_bounds[i].second);
  }

  common::StPoint2D proto_point;
  for (size_t i = 0; i < qp_speed_data_.size(); i++) {
    const SpeedPoint& point = qp_speed_data_[i];
    proto_point.set_s(point.s);
    proto_point.set_t(point.t);
    proto_point.set_vel(point.v);
    proto_point.set_acc(point.a);
    proto_point.set_jerk(point.da);

    speed_debug->add_qp_profile()->CopyFrom(proto_point);
  }

  speed_debug->set_speed_type(common::SpeedProfileType::PWJ_QP);

  return;
}

void PiecewiseJerkSpeedQPOptimizer::DebugRef(const std::vector<double>& t_ref,
                                             const std::vector<double>& x_ref,
                                             const std::vector<double>& v_ref) {
  for (size_t i = 0; i < x_ref.size(); i++) {
    ILOG_INFO << "i = " << i << ",s = " << x_ref[i] << ", t = " << t_ref[i]
              << ",v ref = " << v_ref[i];
  }

  return;
}

void PiecewiseJerkSpeedQPOptimizer::DebugLinearConstraints(
    const std::vector<double>& x_ref,
    const std::vector<std::pair<double, double>>& s_dot_bounds) {
  for (size_t i = 0; i < x_ref.size(); i++) {
    ILOG_INFO << "i = " << i << ",s = " << x_ref[i]
              << ",v lower = " << s_dot_bounds[i].first
              << ", v upper = " << s_dot_bounds[i].second;
  }

  const ParkingSpeedConfig& speed_config = apa_param.GetParam().speed_config;

  double total_length = 0;
  if (!qp_speed_data_.empty()) {
    total_length = qp_speed_data_.back().s;
  }

  ILOG_INFO << "s lower = " << 0.0 << ", upper = " << total_length;
  ILOG_INFO << "acc lower = " << speed_config.acc_lower
            << ", upper = " << speed_config.long_path_acc_upper;
  ILOG_INFO << "jerk lower = " << speed_config.jerk_lower
            << ", upper = " << speed_config.jerk_upper;

  return;
}

double PiecewiseJerkSpeedQPOptimizer::ComputeFastBrakeProfile(
    const std::array<double, 3>& init, const double time) {
  double jerk = 0.0;
  double dec = -0.3;
  double speed_error = 0.1;

  // second system
  // a = a0+j*t;
  // v = v0+a0*t+1/2 * j * t^2;

  // generate brake speed profile.
  // non zero jerk phase
  double phase1_time;
  if (dec > init[2]) {
    jerk = 5.0;
    phase1_time = (dec - init[2]) / jerk;
  } else {
    jerk = -5.0;
    phase1_time = (dec - init[2]) / jerk;
  }
  phase1_time = std::max(0.0, phase1_time);

  double phase1_speed;
  if (time <= phase1_time) {
    phase1_speed = init[1] + init[2] * time + 0.5 * jerk * time * time;
    return std::max(0.0, phase1_speed) + speed_error;
  }

  // zero jerk phase
  double phase2_start_speed;
  phase2_start_speed =
      init[1] + init[2] * phase1_time + 0.5 * jerk * phase1_time * phase1_time;
  phase2_start_speed = std::max(phase2_start_speed, 0.0);

  double phase2_end_speed;
  phase2_end_speed = phase2_start_speed + dec * (time - phase1_time);
  phase2_end_speed = std::max(phase2_end_speed, 0.0);

  return phase2_end_speed + speed_error;
}

}  // namespace apa_planner
}  // namespace planning

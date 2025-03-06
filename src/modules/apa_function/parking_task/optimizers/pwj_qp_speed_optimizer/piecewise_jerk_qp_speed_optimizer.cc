#include "piecewise_jerk_qp_speed_optimizer.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "apa_param_config.h"
#include "debug_info_log.h"
#include "dp_speed_common.h"
#include "log_glog.h"

namespace planning {

#define DECIDER_DEBUG (0)

PiecewiseJerkSpeedQPOptimizer::PiecewiseJerkSpeedQPOptimizer() {}

void PiecewiseJerkSpeedQPOptimizer::Execute(
    const SVPoint& init_point, const SpeedLimitProfile* speed_limit_profile,
    const SpeedData& dp_speed_data) {
  qp_config_.Init();
  speed_limit_profile_ = speed_limit_profile;
  qp_speed_data_ = dp_speed_data;

  if (speed_limit_profile == nullptr || dp_speed_data.size() < 2) {
    ILOG_INFO << "speed data is null";
    return;
  }

  ILOG_INFO << "speed qp optimizer";

  std::array<double, 3> init_state = {init_point.s, init_point.v,
                                      init_point.acc};
  init_point.DebugString();

  double delta_time = qp_config_.delta_time;
  double total_length = qp_speed_data_.back().s;
  double total_time = qp_speed_data_.back().t;
  num_of_knots_ = std::ceil(total_time / delta_time) + 1;

  // model
  // f = 1/2 * x^T * H * x + Q * x
  // Ax<=U

  PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_knots_, delta_time,
                                                   init_state);

  const apa_planner::ParkingSpeedConfig& speed_config =
      apa_param.GetParam().speed_config;

  // set weight
  piecewise_jerk_problem.set_weight_ddx(qp_config_.acc_weight);
  piecewise_jerk_problem.set_weight_dddx(qp_config_.jerk_weight);

  // set s boundary, no need add s boundary vector.
  piecewise_jerk_problem.set_x_bounds(0.0, total_length);

  // set acc boundary
  piecewise_jerk_problem.set_ddx_bounds(speed_config.acc_lower,
                                        speed_config.acc_upper);

  // set jerk boundary
  piecewise_jerk_problem.set_dddx_bound(speed_config.jerk_lower,
                                        speed_config.jerk_upper);

  // update ref v, ref_s.
  std::vector<double> x_ref;
  std::vector<double> dx_ref;
  std::vector<double> t_ref;
  std::vector<std::pair<double, double>> s_dot_bounds;
  SpeedPoint speed_point;
  const double v_lower_bound = 0.0;
  double v_upper_bound;
  double curr_time = 0.0;

  for (int i = 0; i < num_of_knots_; ++i) {
    // get path
    qp_speed_data_.EvaluateByTime(curr_time, &speed_point);

    t_ref.emplace_back(speed_point.t);
    x_ref.emplace_back(speed_point.s);
    dx_ref.emplace_back(speed_point.v);

    // get v_upper_bound
    v_upper_bound = speed_limit_profile_->GetSpeedLimitByRange(
        speed_point.s - 0.05, speed_point.s + 0.05);
    v_upper_bound = std::max(v_upper_bound, init_point.v + 0.001);

    s_dot_bounds.emplace_back(v_lower_bound, v_upper_bound);

    curr_time += delta_time;
    curr_time = std::min(curr_time, total_time);
  }
  if (x_ref.empty() || dx_ref.empty()) {
    ILOG_INFO << "size is null";
    return;
  }

  // set x ref
  piecewise_jerk_problem.set_x_ref(qp_config_.ref_s_weight, x_ref);

  // set dx ref
  piecewise_jerk_problem.set_dx_ref(qp_config_.ref_v_weight, dx_ref);

  // set dx boundary
  piecewise_jerk_problem.set_dx_bounds(s_dot_bounds);

  // todo: set acc ref

  // set end state
  std::array<double, 3> end_state_ref = {{total_length, 0.0, 0.0}};
  // set weight 0, do not consider this soft constraint in Q, but we will add a
  // hard constraint for end state in A matrix.
  std::array<double, 3> end_state_weight = {{0.0, 0.0, 0.0}};
  piecewise_jerk_problem.set_end_state_ref(end_state_weight, end_state_ref);

  // debug info
#if DECIDER_DEBUG
  DebugRef(t_ref, x_ref, dx_ref);
  DebugLinearConstraints(x_ref, s_dot_bounds);
#endif

  // Solve the problem
  if (!piecewise_jerk_problem.Optimize(10000)) {
    ILOG_INFO << "Piecewise jerk speed optimizer failed!";
    return;
  }

  // Extract output
  const std::vector<double>& s = piecewise_jerk_problem.opt_x();
  const std::vector<double>& ds = piecewise_jerk_problem.opt_dx();
  const std::vector<double>& dds = piecewise_jerk_problem.opt_ddx();

#if DECIDER_DEBUG
  DebugPiecewiseJerkProblem(piecewise_jerk_problem);
#endif

  qp_speed_data_.clear();
  qp_speed_data_.AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
  for (int i = 1; i < num_of_knots_; ++i) {
    // Avoid the very last points when already stopped
    if (ds[i] <= 0.0) {
      break;
    }
    qp_speed_data_.AppendSpeedPoint(s[i], delta_time * i, ds[i], dds[i],
                                    (dds[i] - dds[i - 1]) / delta_time);
  }

  RecordDebugInfo(x_ref, s_dot_bounds);

  ILOG_INFO << "speed qp opt finish";

  return;
}

void PiecewiseJerkSpeedQPOptimizer::DebugPiecewiseJerkProblem(
    const PiecewiseJerkSpeedProblem& pwj) {
  const std::vector<double>& s = pwj.opt_x();
  const std::vector<double>& ds = pwj.opt_dx();
  const std::vector<double>& dds = pwj.opt_ddx();

  ILOG_INFO << "debug pwj speed point";
  for (int i = 0; i < num_of_knots_; ++i) {
    ILOG_INFO << "For t[" << i * qp_config_.delta_time << "], s = " << s[i]
              << ", v = " << ds[i] << ", a = " << dds[i];
  }
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

  const apa_planner::ParkingSpeedConfig& speed_config =
      apa_param.GetParam().speed_config;

  double total_length = 0;
  if (!qp_speed_data_.empty()) {
    total_length = qp_speed_data_.back().s;
  }

  ILOG_INFO << "s lower = " << 0.0 << ", upper = " << total_length;
  ILOG_INFO << "acc lower = " << speed_config.acc_lower
            << ", upper = " << speed_config.acc_upper;
  ILOG_INFO << "jerk lower = " << speed_config.jerk_lower
            << ", upper = " << speed_config.jerk_upper;

  return;
}

}  // namespace planning

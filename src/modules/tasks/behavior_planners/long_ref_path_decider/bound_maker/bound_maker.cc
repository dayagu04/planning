#include "bound_maker.h"

namespace planning {
namespace {
constexpr double IsoAccLimitUpper = -3.5;
constexpr double IsoAccLimitLower = -5.0;
constexpr double IsoAccLimitSpeedUpper = 20.0;
constexpr double IsoAccLimitSpeedLower = 5.0;

constexpr double IsoJerkLimitUpper = -3.5;
constexpr double IsoJerkLimitLower = -5.0;
constexpr double IsoJerkLimitSpeedUpper = 20.0;
constexpr double IsoJerkLimitSpeedLower = 5.0;

constexpr double kFollowBuffer = 0.2;
constexpr double kOvertakeBuffer = 2.0;

constexpr double kSpeedBoundFactor = 1.1;
constexpr double kPerSecondPlanLenth = 50.0;

}  // namespace
BoundMaker::BoundMaker(const SpeedPlannerConfig& speed_planning_config,
                       framework::Session* session)
    : speed_planning_config_(speed_planning_config), session_(session) {
  dt_ = speed_planning_config_.dt;
  plan_time_ = speed_planning_config_.planning_time;
  plan_points_num_ = static_cast<int32_t>(plan_time_ / dt_) + 1;
}

common::Status BoundMaker::Run() {
  LOG_DEBUG("=======LongRefPathDecider: BoundMaker======= \n");
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto& init_point = ego_state_manager->planning_init_point();
  init_lon_state_ = {0, init_point.v, init_point.a};

  // 1. acc bound
  MakeAccBound();

  // 2. s bound
  MakeSBound();

  // 3. v bound
  MakeVBound();

  // 4. jerk bound
  MakeJerkBound();

  return common::Status::OK();
}

void BoundMaker::MakeAccBound() {
  // @gpxu 待补充
  const auto& speed_planning_bound =
      speed_planning_config_.speed_planning_bound;
  double lower_speed_acc_upper_bound =
      speed_planning_bound.low_speed_acc_upper_bound;
  double high_speed_acc_upper_bound =
      speed_planning_bound.high_speed_acc_upper_bound;
  const double low_speed_threshold_with_acc_upper_bound =
      speed_planning_bound.low_speed_threshold_with_acc_upper_bound;
  const double high_speed_threshold_with_acc_upper_bound =
      speed_planning_bound.high_speed_threshold_with_acc_upper_bound;
  // for lane change,we need larger acc bound

  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto& lane_change_status = lane_change_decider_output.curr_state;
  bool is_in_lane_change_execution =
      lane_change_status == kLaneChangeExecution ||
      lane_change_status == kLaneChangeComplete;

  if (is_in_lane_change_execution) {
    lower_speed_acc_upper_bound =
        speed_planning_bound.lane_change_low_speed_acc_upper_bound;
    high_speed_acc_upper_bound =
        speed_planning_bound.lane_change_high_speed_acc_upper_bound;
  }
  // TODO: adjust high_speed_acc_upper_bound by driving style
  // MatchAccLimitWithTable(is_in_lane_change_execution,
  // &lower_speed_acc_upper_bound,
  //                        &high_speed_acc_upper_bound);
  acc_upper_bound_with_speed_ = planning_math::LerpWithLimit(
      lower_speed_acc_upper_bound, low_speed_threshold_with_acc_upper_bound,
      high_speed_acc_upper_bound, high_speed_threshold_with_acc_upper_bound,
      init_lon_state_[1]);
  const double config_acc_lower_bound = speed_planning_bound.acc_lower_bound;
  acc_upper_bound_ = std::vector<double>(
      plan_points_num_,
      std::fmax(init_lon_state_[2], acc_upper_bound_with_speed_));
  acc_lower_bound_ = std::vector<double>(
      plan_points_num_, std::fmin(init_lon_state_[2], config_acc_lower_bound));
}

void BoundMaker::MakeSBound() {
  // @gpxu 待补充
  auto max_acceration_curve = GenerateMaxAccelerationCurve();
  auto max_deceleration_curve = GenerateMaxDecelerationCurve();

  const auto& environmental_model = session_->environmental_model();
  const auto& ego_lane =
      environmental_model.get_virtual_lane_manager()->get_current_lane();

  if (ego_lane == nullptr) {
    return;
  }
  // get reference path from ego lane
  const auto& ego_reference_path = ego_lane->get_reference_path();
  if (ego_reference_path == nullptr) {
    return;
  }
  const auto& ego_lane_coord = ego_reference_path->get_frenet_coord();
  if (ego_lane_coord == nullptr) {
    return;
  }

  const double path_length = ego_lane_coord->Length();
  const double default_upper_bound =
      std::fmax(path_length, plan_time_ * kPerSecondPlanLenth);
  s_lower_bound_ = std::vector<double>(plan_points_num_, -0.1);
  s_upper_bound_ = std::vector<double>(plan_points_num_, default_upper_bound);

  const auto ptr_st_graph_helper =
      session_->planning_context().st_graph_helper();
  constexpr double kUpperBoundBuffer = 1.0;
  constexpr double kLowSpeedBuffer = 0.2;

  double ego_speed = init_lon_state_[1];
  bool is_ego_low_speed = init_lon_state_[1] < kLowSpeedBuffer;

  double ego_acc = init_lon_state_[2];
  double add_buffer_start_time = 2.5;
  double add_speed_buffer_start_time = 2.0;

  constexpr double kBaseBuffer = 1.0;
  constexpr double kSpeedBufferThreshold = 4.0;
  const double preview_time = 0.5;
  constexpr double kSafeVelBuffer = 1.0;

  double future_speed = ego_speed + ego_acc * preview_time;
  future_speed = std::fmax(0.0, future_speed);
  double speed_buffer = future_speed * preview_time;
  speed_buffer = std::fmin(kSpeedBufferThreshold, speed_buffer);

  auto virtual_acc_curve = MakeVirtualZeroAccCurve();

  for (int32_t i = 0; i < plan_points_num_; ++i) {
    const double relative_t = i * dt_;
    // std::cout << "bound t: " << relative_t << std::endl;
    const auto corridor_upper_point =
        ptr_st_graph_helper->GetPassCorridorUpperBound(relative_t);
    const auto corridor_lower_point =
        ptr_st_graph_helper->GetPassCorridorLowerBound(relative_t);
    double& upper_bound = s_upper_bound_[i];
    double& lower_bound = s_lower_bound_[i];
    if (corridor_upper_point.valid() && corridor_upper_point.agent_id() != -1) {
      const double upper_s_with_buffer =
          corridor_upper_point.s() - kFollowBuffer;
      upper_bound = upper_s_with_buffer;
      if (is_ego_low_speed) {
        continue;
      }
      // for base case
      if (relative_t > add_buffer_start_time &&
          max_deceleration_curve.Evaluate(0, relative_t) <
              upper_bound - kBaseBuffer) {
        upper_bound -= kBaseBuffer;
      }
      // for ego speed buffer
      bool is_safe_vel =
          virtual_acc_curve->Evaluate(1, relative_t) + kSafeVelBuffer >
          corridor_upper_point.velocity();
      if (relative_t > add_speed_buffer_start_time &&
          max_deceleration_curve.Evaluate(0, relative_t) <
              upper_bound - speed_buffer &&
          is_safe_vel) {
        upper_bound -= speed_buffer;
      }
    }

    if (corridor_lower_point.valid() && corridor_lower_point.agent_id() != -1) {
      lower_bound = corridor_lower_point.s();
    }
  }
}

void BoundMaker::MakeVBound() {
  // @gpxu 待补充
  v_lower_bound_ = std::vector<double>(plan_points_num_, -0.1);
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const double cruise_speed = ego_state_manager->ego_v_cruise();
  constexpr double kRefSpeedBuffer = 0.1;
  const double cruise_speed_uppper_bound = cruise_speed * kSpeedBoundFactor;
  const double ego_speed_upper_bound = init_lon_state_[1] * kSpeedBoundFactor;
  const double max_speed = init_lon_state_[1] < cruise_speed_uppper_bound
                               ? cruise_speed_uppper_bound
                               : ego_speed_upper_bound;
  v_upper_bound_ = std::vector<double>(plan_points_num_, max_speed);
}

void BoundMaker::MakeJerkBound() {
  // @gpxu 待补充
  double jerk_upper_bound =
      speed_planning_config_.speed_planning_bound.jerk_upper_bound;
  if (init_lon_state_[2] > 0.0) {
    jerk_upper_bound = speed_planning_config_.slow_jerk_upper_bound;
  }
  double jerk_lower_bound =
      speed_planning_config_.speed_planning_bound.jerk_lower_bound;
  jerk_upper_bound_ = std::vector<double>(plan_points_num_, jerk_upper_bound);
  jerk_lower_bound_ = std::vector<double>(plan_points_num_, jerk_lower_bound);
}

SecondOrderTimeOptimalTrajectory BoundMaker::GenerateMaxAccelerationCurve()
    const {
  LonState init_state;
  init_state.p = init_lon_state_[0];
  init_state.v = init_lon_state_[1];
  init_state.a = init_lon_state_[2];

  StateLimit state_limit;
  constexpr double kSpeedBuffer = 100.0;
  constexpr double kAccBuffer = 0.2;
  constexpr double kSlowJerkUpperBound = 6.0;
  constexpr double kSlowJerkLowerBound = -3.0;
  state_limit.v_end = init_lon_state_[1] + kSpeedBuffer;
  state_limit.a_max = acc_upper_bound_with_speed_ - kAccBuffer;
  state_limit.a_min =
      speed_planning_config_.speed_planning_bound.acc_lower_bound;
  state_limit.j_max = kSlowJerkUpperBound;
  state_limit.j_min = kSlowJerkLowerBound;
  return SecondOrderTimeOptimalTrajectory(init_state, state_limit);
}

SecondOrderTimeOptimalTrajectory BoundMaker::GenerateMaxDecelerationCurve()
    const {
  LonState init_state;
  init_state.p = init_lon_state_[0];
  init_state.v = init_lon_state_[1];
  init_state.a = init_lon_state_[2];
  StateLimit state_limit;

  //
  //   iso_acc_limit_upper : -3.5
  // iso_acc_limit_lower : -5.0
  // iso_acc_limit_speed_upper : 20.0
  // iso_acc_limit_speed_lower : 5.0
  //
  const double acc_lower_bound = planning_math::LerpWithLimit(
      IsoAccLimitLower, IsoAccLimitSpeedLower, IsoAccLimitUpper,
      IsoAccLimitSpeedUpper, init_lon_state_[1]);

  const double jerk_lower_bound = planning_math::LerpWithLimit(
      IsoJerkLimitLower, IsoJerkLimitSpeedLower, IsoJerkLimitUpper,
      IsoJerkLimitSpeedUpper, init_lon_state_[1]);

  constexpr double kSlowAccLowerBound = -3.0;
  state_limit.a_max = acc_upper_bound_with_speed_;
  state_limit.a_min = acc_lower_bound;
  state_limit.j_max =
      speed_planning_config_.speed_planning_bound.jerk_upper_bound;
  state_limit.j_min = jerk_lower_bound;
  return SecondOrderTimeOptimalTrajectory(init_state, state_limit);
}

std::unique_ptr<Trajectory1d> BoundMaker::MakeVirtualZeroAccCurve() {
  auto virtual_zero_acc_curve =
      std::make_unique<PiecewiseJerkAccelerationTrajectory1d>(
          init_lon_state_[0], init_lon_state_[1]);
  virtual_zero_acc_curve->AppendSegment(init_lon_state_[2], dt_);

  const double zero_acc_jerk_max = speed_planning_config_.zero_acc_jerk_max;
  const double zero_acc_jerk_min = speed_planning_config_.zero_acc_jerk_min;
  for (double t = dt_; t <= plan_time_; t += dt_) {
    const double acc = virtual_zero_acc_curve->Evaluate(2, t);
    const double vel = virtual_zero_acc_curve->Evaluate(1, t);

    double a_next = 0.0;
    // if init acc < 0.0, move a to zero with jerk max
    // if init acc >0.0,move a to zero with jerk min
    if (init_lon_state_[2] < 0.0) {
      a_next = acc + dt_ * zero_acc_jerk_max;
    } else {
      a_next = acc + dt_ * zero_acc_jerk_min;
    }

    if (init_lon_state_[2] * acc <= 0.0) {
      a_next = 0.0;
    }

    if (vel <= 0.0) {
      a_next = 0.0;  //??
    }
    virtual_zero_acc_curve->AppendSegment(a_next, dt_);
  }
  return virtual_zero_acc_curve;
}

double BoundMaker::s_lower_bound(const double t) const {
  int32_t index = static_cast<int32_t>(std::round(t / dt_));
  return s_lower_bound_[index];
}
double BoundMaker::s_upper_bound(const double t) const {
  int32_t index = static_cast<int32_t>(std::round(t / dt_));
  return s_upper_bound_[index];
}

double BoundMaker::v_lower_bound(const double t) const {
  int32_t index = static_cast<int32_t>(std::round(t / dt_));
  return v_lower_bound_[index];
}

double BoundMaker::v_upper_bound(const double t) const {
  int32_t index = static_cast<int32_t>(std::round(t / dt_));
  return v_upper_bound_[index];
}

double BoundMaker::a_lower_bound(const double t) const {
  int32_t index = static_cast<int32_t>(std::round(t / dt_));
  return acc_lower_bound_[index];
}

double BoundMaker::a_upper_bound(const double t) const {
  int32_t index = static_cast<int32_t>(std::round(t / dt_));
  return acc_upper_bound_[index];
}

double BoundMaker::jerk_lower_bound(const double t) const {
  int32_t index = static_cast<int32_t>(std::round(t / dt_));
  return jerk_lower_bound_[index];
}

double BoundMaker::jerk_upper_bound(const double t) const {
  int32_t index = static_cast<int32_t>(std::round(t / dt_));
  return jerk_upper_bound_[index];
}

}  // namespace planning
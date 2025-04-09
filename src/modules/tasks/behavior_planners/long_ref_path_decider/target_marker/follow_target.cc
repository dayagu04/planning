#include "follow_target.h"
#include <cstdint>

#include "config/basic_type.h"
#include "debug_info_log.h"
#include "ego_planning_config.h"
#include "environmental_model.h"
#include "planning_context.h"
#include "target.h"

namespace planning {

namespace {
constexpr double kLargeAgentLengthM = 8.0;
constexpr double default_headway = 1.5;
constexpr double min_follow_distance_gap_cut_in = 1.0;
constexpr int32_t check_time_idx = 8;

// stable jlt
constexpr double stable_vehicle_max_acc = 0.3;
constexpr double stable_vehicle_min_acc = -0.5;
constexpr double v_min = -0.1;
constexpr double v_max = 40.0;
constexpr double a_min = -1.0;
constexpr double a_max = 1.8;
constexpr double j_min = -1.0;
constexpr double j_max = 1.0;
constexpr double p_precision = 0.1;

// farslow jlt
constexpr double check_time = 1.5;
constexpr double kMinFarTimeGap = 1.8;
constexpr double min_far_distance_threshold = 15.0;
constexpr double kDefaultFollowMinDist = 3.0;
constexpr double kPreviewTime = 0.5;
constexpr double kSpeedBuffer = 3.0;
constexpr double kFarDistFollowTimeGap = 1.5;
constexpr double kNearDistFollowTimeGap = 1.2;
constexpr double kFarDistanceThreshold = 20.0;
constexpr double kNearDistanceThreshold = 10.0;
constexpr double kFarPreviewTime = 6.0;
constexpr double kNearPreviewTime = 2.0;
constexpr double kSpeedLower = 6.0;
constexpr double kSpeedUpper = 20.0;
constexpr double kAccMinLower = -0.7;
constexpr double kAccMinUpper = -0.3;
constexpr double kAccMax = 1.5;
constexpr double kJerkMin = -1.0;
constexpr double kJerkMax = 0.5;
constexpr double AccMinThreshold = -3.0;
constexpr double kSafeTimeGap = 1.4;
constexpr double kMinFollowDistance = 3.0;
constexpr int32_t kContinuousNumFarSlow = 3;
constexpr int32_t kContinuousNumStable = 5;
}  // namespace

FollowTarget::FollowTarget(const SpeedPlannerConfig config,
                           framework::Session* session)
    : Target(config, session) {
  follow_target_pb_.Clear();
  upper_bound_infos_ =
      std::vector<UpperBoundInfo>(plan_points_num_, UpperBoundInfo());
  GenerateUpperBoundInfo();

  // far slow car case
  // bool is_far_slow_car = JudgeFarSlowCar();
  // if (is_far_slow_car) {
  //   target_follow_curve_ = MakeTargetFollowCurve();
  //   enable_target_follow_curve_ = target_follow_curve_ != nullptr;
  // }

  MakeMinFollowDistance();

  GenerateFollowTarget();

  AddFollowTargetDataToProto();
}

void FollowTarget::GenerateUpperBoundInfo() {
  const auto* st_graph = session_->planning_context().st_graph_helper();
  if (st_graph == nullptr) {
    return;
  }
  for (size_t i = 0; i < plan_points_num_; i++) {
    const double t = i * dt_;
    const auto& upper_bound = st_graph->GetPassCorridorUpperBound(t);
    if (upper_bound.agent_id() != speed::kNoAgentId) {
      if (i == 0) {
        cipv_info_.agent_id = upper_bound.agent_id();
        cipv_info_.upper_bound_s = upper_bound.s();
        cipv_info_.vel = upper_bound.velocity();
        auto* agent =
            session_->environmental_model().get_agent_manager()->GetAgent(
                cipv_info_.agent_id);
        if (agent != nullptr) {
          cipv_info_.is_large_vehicle = agent->length() > kLargeAgentLengthM;
          cipv_info_.type = agent->type();
          cipv_info_.is_tfl_virtual_obs = agent->is_tfl_virtual_obs();
        }
      }
      upper_bound_infos_[i].s = upper_bound.s();
      upper_bound_infos_[i].t = t;
      upper_bound_infos_[i].v = upper_bound.velocity();
      upper_bound_infos_[i].target_type = TargetType::kFollow;
      upper_bound_infos_[i].agent_id = upper_bound.agent_id();
      upper_bound_infos_[i].st_boundary_id = upper_bound.boundary_id();
    }
  }
}

void FollowTarget::MakeMinFollowDistance() {
  const double large_vehicle_min_follow_distance =
      config_.large_vehicle_min_follow_distance_gap;
  double min_follow_distance_lower =
      config_.lower_speed_min_follow_distance_gap;
  double cone_min_follow_distance = config_.cone_min_follow_distance_gap;
  const double traffic_light_min_follow_distance_gap =
      config_.traffic_light_min_follow_distance_gap;
  if (cipv_info_.agent_id != -1 && cipv_info_.is_large_vehicle) {
    min_follow_distance_lower = large_vehicle_min_follow_distance;
  }
  if (cipv_info_.agent_id != -1 &&
      cipv_info_.type == agent::AgentType::TRAFFIC_CONE) {
    min_follow_distance_m_ = cone_min_follow_distance;
    return;
  }
  if (cipv_info_.agent_id != -1 && cipv_info_.is_tfl_virtual_obs) {
    min_follow_distance_lower = traffic_light_min_follow_distance_gap;
  }
  const double min_follow_distance_upper =
      config_.high_speed_min_follow_distance_gap;
  const double low_speed_threshold = config_.low_speed_threshold_kmph / 3.6;
  const double high_speed_threshold = config_.high_speed_threshold_kmph / 3.6;

  min_follow_distance_m_ = planning_math::LerpWithLimit(
      min_follow_distance_lower, low_speed_threshold, min_follow_distance_upper,
      high_speed_threshold, init_lon_state_[1]);
}

void FollowTarget::GenerateFollowTarget() {
  double matched_desired_headway = default_headway;
  const double default_t = 0.0;
  const bool default_has_target = false;
  const double default_s_target = 0.0;
  const double default_v_target = 0.0;
  const TargetType default_target_type = TargetType::kNotSet;
  auto default_target_value =
      TargetValue(default_t, default_has_target, default_s_target,
                  default_v_target, default_target_type);
  target_values_ =
      std::vector<TargetValue>(plan_points_num_, default_target_value);

  const auto& agent_headway_decider_output =
      session_->planning_context().agent_headway_decider_output();
  const auto& agents_headway_map =
      agent_headway_decider_output.agents_headway_Info();

  auto lon_ref_path_decider_output =
      session_->planning_context().lon_ref_path_decider_output();
  auto mutable_lon_ref_path_decider_output =
      session_->mutable_planning_context()
          ->mutable_lon_ref_path_decider_output();

  // stable target
  auto enable_stable_jlt_count_record =
      lon_ref_path_decider_output.target_maker_info.follow_target_info
          .enable_stable_jlt_count;
  const bool enable_stable_follow = JudgeStableCar(matched_desired_headway);

  // 1) multi-frame judgment
  int32_t enable_stable_jlt_count =
      enable_stable_follow ? std::min(enable_stable_jlt_count_record + 1, 5)
                           : 0;
  bool enable_stable_jlt = enable_stable_jlt_count >= kContinuousNumStable;

  // 2) run stable jlt
  auto stable_follow_trajectory =
      GenerateStableFollowSlowCurve(matched_desired_headway, enable_stable_jlt);

  // 3) judge headway sref or jlt sref ?
  if (stable_follow_trajectory != nullptr) {
    enable_stable_jlt = JudgeSrefValid(stable_follow_trajectory);
  }

  // 4) store enable_stable_jlt_count in planning_context
  mutable_lon_ref_path_decider_output->target_maker_info.follow_target_info
      .enable_stable_jlt_count = enable_stable_jlt_count;

  // far slow target
  // 1) multi-frame judgment
  auto enable_farslow_jlt_count_record =
      session_->planning_context()
          .lon_ref_path_decider_output()
          .target_maker_info.follow_target_info.enable_far_slow_jlt_count;
  const bool enable_far_slow_follow = JudgeFarSlowCar(matched_desired_headway);
  int32_t enable_farslow_jlt_count =
      enable_far_slow_follow ? std::min(enable_farslow_jlt_count_record + 1, 3)
                             : 0;
  bool enable_far_slow_jlt = enable_farslow_jlt_count >= kContinuousNumFarSlow;

  // 2) run far slow jlt
  auto far_slow_follow_trajectory =
      GenerateFarFollowSlowCurve(enable_far_slow_jlt);

  // 3) judge headway sref or jlt sref ?
  if (far_slow_follow_trajectory != nullptr) {
    enable_far_slow_jlt = JudgeSrefValid(far_slow_follow_trajectory);
  }

  // 4) store enable_farslow_jlt_count in planning_context
  mutable_lon_ref_path_decider_output->target_maker_info.follow_target_info
      .enable_far_slow_jlt_count = enable_farslow_jlt_count;

  // JSON DEBUG
  JSON_DEBUG_VALUE("has_target_follow_curve", 0)
  JSON_DEBUG_VALUE("has_stable_follow_target", 0)
  JSON_DEBUG_VALUE("has_farslow_follow_target", 0)

  for (int32_t i = 0; i < plan_points_num_; i++) {
    const double t = i * dt_;
    auto& target_value = target_values_[i];
    target_value.set_relative_t(t);
    if (upper_bound_infos_[i].target_type == TargetType::kNotSet) {
      target_values_[i] = target_value;
      double s_value = target_value.s_target_val();
      if (MakeSValueWithTargetFollowCurve(i, false, &s_value)) {
        target_value.set_has_target(true);
        target_value.set_s_target_val(s_value);
        target_value.set_target_type(TargetType::kFollow);
      }
      continue;
    }
    target_value.set_has_target(true);
    const double vel = virtual_zero_acc_curve_->Evaluate(1, t);
    const auto& agent_id = upper_bound_infos_[i].agent_id;
    double follow_time_gap = min_follow_distance_gap_cut_in;
    auto iter = agents_headway_map.find(agent_id);
    if (iter != agents_headway_map.end()) {
      follow_time_gap = iter->second.current_headway;
    }
    double target_s_disatnce = std::max(
        vel * follow_time_gap + min_follow_distance_m_, min_follow_distance_m_);

    double upper_bound_s =
        std::max(upper_bound_infos_[i].s - min_follow_distance_m_, 0.0);
    double target_s =
        std::max(upper_bound_infos_[i].s - target_s_disatnce, 0.0);
    const double s_target_value = std::min(upper_bound_s, target_s);

    target_value.set_s_target_val(s_target_value);
    target_value.set_target_type(upper_bound_infos_[i].target_type);

    double s_value = target_value.s_target_val();
    if (enable_stable_jlt) {
      target_value.set_s_target_val(stable_follow_trajectory->Evaluate(0, t));
      target_value.set_target_type(upper_bound_infos_[i].target_type);
      JSON_DEBUG_VALUE("has_stable_follow_target", 1.0)
    }

    if (enable_far_slow_jlt) {
      s_value = std::fmin(far_slow_follow_trajectory->Evaluate(0, t), s_value);
      target_value.set_s_target_val(s_value);
      target_value.set_target_type(upper_bound_infos_[i].target_type);
      JSON_DEBUG_VALUE("has_farslow_follow_target", 1.0)
    }

    // if (MakeSValueWithTargetFollowCurve(i, true, &s_value)) {
    //   target_value.set_has_target(true);
    //   target_value.set_s_target_val(s_value);
    //   target_value.set_target_type(TargetType::kFollow);
    //   JSON_DEBUG_VALUE("has_target_follow_curve", 1.0)
    // }
  }
}

bool FollowTarget::MakeSValueWithTargetFollowCurve(
    const int32_t index, const bool has_valid_s_value,
    double* const target_s_value) const {
  if (target_follow_curve_ == nullptr) {
    return false;
  }
  auto st_points = target_follow_curve_->get_target_st_curve().get_st_points();
  if (st_points.size() - 1 < index) {
    return false;
  }

  double target_follow_value = st_points[index].s;
  if (target_follow_value < *target_s_value || !has_valid_s_value) {
    *target_s_value = target_follow_value;
    return true;
  }
  return false;
}

double FollowTarget::MakeSlowerFollowSTarget(const double speed,
                                             const double upper_bound_s,
                                             const double time_gap) const {
  constexpr double kFollowDistanceBuffer = 2.0;
  // make small follow time gap follow target
  double follow_target_distance = std::fmax(
      speed * time_gap + kFollowDistanceBuffer, min_follow_distance_m_);
  double s_target = upper_bound_s - follow_target_distance;
  return s_target;
}

std::shared_ptr<VariableCoordinateTimeOptimalTrajectory>
FollowTarget::GenerateStableFollowSlowCurve(
    const double matched_desired_headway, const bool enable_stable_jlt) const {
  if (enable_stable_jlt) {
    CoordinateParam relative_coordinate_param;
    if (!GenerateRelativeCoordinate(matched_desired_headway,
                                    &relative_coordinate_param)) {
      return nullptr;
    }

    LonState init_state;
    init_state.p = init_lon_state_[0];
    init_state.v = init_lon_state_[1];
    init_state.a = init_lon_state_[2];

    StateLimit state_limit;
    state_limit.p_end = 0.0;
    state_limit.v_min = v_min;
    state_limit.v_max = v_max;
    state_limit.a_min = a_min;
    state_limit.a_max = a_max;
    state_limit.j_min = j_min;
    state_limit.j_max = j_max;

    VariableCoordinateTimeOptimalTrajectory follow_stable_target_trajectory =
        VariableCoordinateTimeOptimalTrajectory::ConstructInstance(
            init_state, state_limit, relative_coordinate_param, p_precision);

    return std::make_shared<VariableCoordinateTimeOptimalTrajectory>(
        follow_stable_target_trajectory);
  }
  return nullptr;
}

std::shared_ptr<VariableCoordinateTimeOptimalTrajectory>
FollowTarget::GenerateFarFollowSlowCurve(const bool enable_far_slow_jlt) const {
  if (enable_far_slow_jlt) {
    const double far_preview_dist = kFarPreviewTime * init_lon_state_[1];
    const double near_preview_dist = kNearPreviewTime * init_lon_state_[1];
    const double far_distance =
        std::fmax(kFarDistanceThreshold, far_preview_dist);
    const double near_distance =
        std::fmax(kNearDistanceThreshold, near_preview_dist);
    const double init_dist = upper_bound_infos_[0].s;

    const double far_slow_follow_time_gap = planning_math::LerpWithLimit(
        kNearDistFollowTimeGap, near_distance, kFarDistFollowTimeGap,
        far_distance, init_dist);

    CoordinateParam relative_coordinate_param;
    const auto& preview_upper_bound_info = upper_bound_infos_[check_time_idx];
    int64_t preview_st_boundary_id = preview_upper_bound_info.st_boundary_id;
    if (!GenerateFarSlowCarRelativeCoordinate(preview_st_boundary_id,
                                              far_slow_follow_time_gap,
                                              &relative_coordinate_param)) {
      return nullptr;
    }

    // Make variable time optimal traj
    auto follow_stable_target_trajectory =
        MakeSafeFarSlowCurve(relative_coordinate_param);

    return std::make_shared<VariableCoordinateTimeOptimalTrajectory>(
        follow_stable_target_trajectory);
  }
  return nullptr;
}

bool FollowTarget::GenerateRelativeCoordinate(
    const double follow_time_gap,
    CoordinateParam* const relative_coordinate_param) const {
  constexpr int32_t kCheckFrontTimeIndex = 20;
  for (int32_t i = kCheckFrontTimeIndex; i >= 0; i--) {
    const double t = i * dt_;
    const auto& upper_bound = upper_bound_infos_[i];
    if (upper_bound.target_type == TargetType::kNotSet) {
      continue;
    }
    const double vel = upper_bound.v;
    const double target_distance =
        std::max(init_lon_state_[1] * follow_time_gap + min_follow_distance_m_,
                 min_follow_distance_m_);
    const double s_target = upper_bound.s - t * vel - target_distance;
    relative_coordinate_param->s_start = s_target;
    relative_coordinate_param->v = vel;
    return true;
  }
  return false;
}

bool FollowTarget::GenerateFarSlowCarRelativeCoordinate(
    const int64_t st_boundary_id, const double follow_time_gap,
    CoordinateParam* const relative_coordinate_param) const {
  speed::STBoundary st_boundary;
  const auto* st_graph = session_->planning_context().st_graph_helper();
  if (!st_graph->GetStBoundary(st_boundary_id, &st_boundary)) {
    return false;
  }
  for (int32_t i = plan_points_num_ - 1; i >= 0; i--) {
    const double t = i * dt_;
    speed::STPoint lower_point;
    speed::STPoint upper_point;
    if (!st_boundary.GetBoundaryBounds(t, &lower_point, &upper_point)) {
      continue;
    }
    // const auto& upper_bound = upper_bound_infos_[i];
    // if (upper_bound.target_type == TargetType::kNotSet) {
    //   continue;
    // }

    const double vel = lower_point.velocity();
    const double target_distance =
        std::max(init_lon_state_[1] * follow_time_gap + min_follow_distance_m_,
                 min_follow_distance_m_);
    const double s_target = lower_point.s() - t * vel - target_distance;
    relative_coordinate_param->s_start = s_target;
    relative_coordinate_param->v = vel;
    return true;
  }
  return false;
}

VariableCoordinateTimeOptimalTrajectory FollowTarget::MakeSafeFarSlowCurve(
    const CoordinateParam& relative_coordinate_param) const {
  double acc_min = planning_math::LerpWithLimit(
      kAccMinLower, kSpeedLower, kAccMinUpper, kSpeedUpper, init_lon_state_[1]);
  LonState init_state;
  init_state.p = init_lon_state_[0];
  init_state.v = init_lon_state_[1];
  init_state.a = std::fmax(AccMinThreshold, init_lon_state_[2]);

  StateLimit state_limit;
  state_limit.p_end = 0.0;
  state_limit.v_min = v_min;
  state_limit.v_max = v_max;
  state_limit.a_min = acc_min;
  state_limit.a_max = kAccMax;
  state_limit.j_min = kJerkMin;
  state_limit.j_max = kJerkMax;

  const int32_t kMaxLoopNum = 4;
  constexpr double kLatPositionPrecision = 0.3;

  double acc_min_lower = -1.5;
  double acc_min_upper = state_limit.a_min;
  double a_min = 0.5 * (acc_min_upper + acc_min_lower);

  for (int loop_num = 0; loop_num < kMaxLoopNum; ++loop_num) {
    state_limit.a_min = a_min;
    auto far_slow_curve =
        VariableCoordinateTimeOptimalTrajectory::ConstructInstance(
            init_state, state_limit, relative_coordinate_param,
            kLatPositionPrecision);
    if (loop_num == kMaxLoopNum - 1) {
      return far_slow_curve;
    }
    if (IsSafeFarSlowCurve(far_slow_curve)) {
      return far_slow_curve;
    } else {
      acc_min_upper = a_min;
    }
    a_min = 0.5 * (acc_min_upper + acc_min_lower);
  }
  return VariableCoordinateTimeOptimalTrajectory::ConstructInstance(
      init_state, state_limit, relative_coordinate_param,
      kLatPositionPrecision);
}

bool FollowTarget::IsSafeFarSlowCurve(
    const VariableCoordinateTimeOptimalTrajectory& far_slow_curve) const {
  for (int i = 0; i < plan_points_num_; ++i) {
    double relative_t = i * dt_;
    double object_s = 0.0;
    double object_v = 0.0;

    if (upper_bound_infos_[i].target_type == TargetType::kNotSet) {
      continue;
    }
    object_s = upper_bound_infos_[i].s;
    object_v = upper_bound_infos_[i].v;

    double ego_s = far_slow_curve.Evaluate(0, relative_t);
    double ego_v = far_slow_curve.Evaluate(1, relative_t);
    if (ego_v < object_v) {
      continue;
    }
    double safe_distance = std::fmax(ego_v * kSafeTimeGap + kMinFollowDistance,
                                     kMinFollowDistance);
    if (object_s - ego_s < safe_distance) {
      return false;
    }
  }
  return true;
}

bool FollowTarget::JudgeStableCar(const double matched_desired_headway) const {
  // 1.check upper bound
  if (upper_bound_infos_.size() <= check_time_idx) {
    return false;
  }

  // 2.check lane change status
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto& lane_change_status = lane_change_decider_output.curr_state;
  bool is_in_lane_change = lane_change_status == kLaneChangeExecution ||
                           lane_change_status == kLaneChangeComplete;
  if (is_in_lane_change) {
    return false;
  }

  // 3.check st graph
  const auto* st_graph = session_->planning_context().st_graph_helper();
  if (st_graph == nullptr) {
    return false;
  }
  const double check_front_time = 4.0;
  for (double t = 0.0; t < check_front_time; t += dt_) {
    const auto& current_upper_bound = st_graph->GetPassCorridorUpperBound(t);
    const auto& next_upper_bound = st_graph->GetPassCorridorUpperBound(t + dt_);
    if (current_upper_bound.agent_id() != next_upper_bound.agent_id()) {
      return false;
    }
  }

  // 4.check stable acc
  const auto& init_upper_bound = st_graph->GetPassCorridorUpperBound(0.0);
  const double init_s_distance = init_upper_bound.s() - init_lon_state_[0];
  const double front_vel = init_upper_bound.velocity();
  const double front_acc = init_upper_bound.acceleration();
  if (front_acc > stable_vehicle_max_acc ||
      front_acc < stable_vehicle_min_acc) {
    // front vehicle acc not stable
    return false;
  }

  // 5.check vel diff
  const double vel_diff_threshold = std::min(3.0, init_lon_state_[1] * 0.3);
  if (std::fabs(front_vel - init_lon_state_[1]) > vel_diff_threshold) {
    // vel diff too large
    return false;
  }

  // 6.check init_s_distance
  const double follow_time_gap = matched_desired_headway;
  const double target_s_distance =
      std::max(init_lon_state_[1] * follow_time_gap + min_follow_distance_m_,
               min_follow_distance_m_);
  const double distance_ratio = 0.3;
  if (target_s_distance - init_s_distance >
      target_s_distance * distance_ratio) {
    // init_s_distance too small than target_s_distance
    return false;
  }
  return true;
}

bool FollowTarget::JudgeFarSlowCar(const double matched_desired_headway) const {
  // Judge far slow case
  const auto& preview_upper_bound_info = upper_bound_infos_[check_time_idx];
  int32_t agent_id = preview_upper_bound_info.agent_id;
  int64_t preview_st_boundary_id = preview_upper_bound_info.st_boundary_id;
  if (upper_bound_infos_.size() <= check_time_idx ||
      preview_upper_bound_info.target_type == TargetType::kNotSet) {
    return false;
  }

  // 1.check s_diff
  const double add_time_gap = std::fmax(0.0, matched_desired_headway - 1.2);
  const double far_time_gap = kMinFarTimeGap + add_time_gap;
  const double ego_move_distance =
      std::fmax(init_lon_state_[1] * far_time_gap + kDefaultFollowMinDist,
                min_far_distance_threshold);
  const double upper_bound_s_diff =
      preview_upper_bound_info.s - check_time * preview_upper_bound_info.v;
  if (upper_bound_s_diff < ego_move_distance) {
    return false;
  }

  // 2.check v_diff
  const double preview_ego_v =
      init_lon_state_[1] + init_lon_state_[2] * kPreviewTime;
  if (preview_ego_v - preview_upper_bound_info.v < kSpeedBuffer) {
    return false;
  }

  // 3.check init_v
  constexpr double kEgoSpeedThreshold = 36.0 / 3.6;
  if (init_lon_state_[1] < kEgoSpeedThreshold) {
    return false;
  }
  return true;
}

bool FollowTarget::JudgeSrefValid(
    std::shared_ptr<VariableCoordinateTimeOptimalTrajectory> jlt_curve) const {
  const auto& agents_headway_map = session_->planning_context()
                                       .agent_headway_decider_output()
                                       .agents_headway_Info();
  for (int32_t i = plan_points_num_ - 1; i >= 0; --i) {
    const double t = i * dt_;
    const double vel = virtual_zero_acc_curve_->Evaluate(1, t);
    const auto& agent_id = upper_bound_infos_[i].agent_id;
    double follow_time_gap = min_follow_distance_gap_cut_in;
    auto iter = agents_headway_map.find(agent_id);
    if (iter != agents_headway_map.end()) {
      follow_time_gap = iter->second.current_headway;
    }
    double target_s_disatnce = std::max(
        vel * follow_time_gap + min_follow_distance_m_, min_follow_distance_m_);

    double upper_bound_s =
        std::max(upper_bound_infos_[i].s - min_follow_distance_m_, 0.0);
    double target_s =
        std::max(upper_bound_infos_[i].s - target_s_disatnce, 0.0);
    const double s_target_value = std::min(upper_bound_s, target_s);
    const double s_value_jlt = jlt_curve->Evaluate(0, t);
    if (s_value_jlt > s_target_value) {
      return false;
    }
  }
  return true;
}

void FollowTarget::AddFollowTargetDataToProto() {
  auto& debug_info_pb = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto mutable_follow_target_data =
      debug_info_pb->mutable_lon_target_s_ref()->mutable_follow_target();
  if (!target_values_.empty()) {
    for (const auto& value : target_values_) {
      auto* ptr = follow_target_pb_.add_follow_target_s_ref();
      ptr->set_s(value.s_target_val());
      ptr->set_t(value.relative_t());
      ptr->set_target_type(static_cast<int32_t>(value.target_type()));
    }
  }
  mutable_follow_target_data->CopyFrom(follow_target_pb_);
}

}  // namespace planning
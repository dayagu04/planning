#include "bound_maker.h"

#include <cmath>

#include "behavior_planners/long_ref_path_decider/long_ref_path_decider.h"
#include "debug_info_log.h"

namespace planning {
namespace {
constexpr double kEpsilon = 1e-6;
constexpr double kFollowBuffer = 0.2;
constexpr double kOvertakeBuffer = 2.0;

constexpr double kSpeedBoundFactor = 1.1;
constexpr double kPerSecondPlanLenth = 50.0;

constexpr double kJerkLowerComfortableBound = -1.2;
constexpr double kDecelJeck = -1.2;
constexpr double kAccLowerComfortableBound = -3.0;
constexpr double kAccJeck = 3.0;
constexpr double kBrakeDelayTimeBuffer = 0.3;

constexpr double kAccMaxLowerBound = -5.0;
constexpr double kJerkMaxLowerBound = -4.0;

constexpr double kLaneChangeAccUpperBound = 1.2;
constexpr double kLaneChangeFrontAgentHeadway = 0.8;

constexpr double kDecelMax = 5.0;
constexpr double kVirtualUpperBound = 250.0;
constexpr double kMaxTau = 1.0;
constexpr double s0 = 3.5;
constexpr double max_tau = 1.35;
constexpr double a_max = 2.0;
constexpr double b_max = 3.0;
}  // namespace

BoundMaker::BoundMaker(const SpeedPlannerConfig& speed_planning_config,
                       framework::Session* session)
    : speed_planning_config_(speed_planning_config), session_(session) {
  dt_ = speed_planning_config_.dt;
  plan_time_ = speed_planning_config_.planning_time;
  plan_points_num_ = static_cast<int32_t>(plan_time_ / dt_) + 1;
}

common::Status BoundMaker::Run(const TargetMaker& target_maker) {
  ILOG_INFO << "=======LongRefPathDecider: BoundMaker=======";
  max_decel_target_pb_.Clear();
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto& lateral_behavior_planner_output =
      session_->planning_context().lateral_behavior_planner_output();
  const auto& init_point = ego_state_manager->planning_init_point();
  const auto& v_ego = ego_state_manager->ego_v();

  init_lon_state_ = {0, init_point.v, init_point.a};
  GenerateUpperBoundInfo();

  // 1. acc bound
  // MakeAccBound();
  MakeAccBound(v_ego, lateral_behavior_planner_output.lc_request);

  // 2. s bound
  MakeSBound();

  // 3. v bound
  MakeVBound();

  // 4. jerk bound
  MakeJerkBound();

  // 5. Judge danger agent by max decel curve
  JudgeDangerAgentByMaxDecelCurve(target_maker);

  // 6. soft bound (IDM + CAH)
  MakeSoftBound();

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

void BoundMaker::MakeAccBound(const double v_ego,
                              const std::string& lc_request) {
  std::pair<double, double> acc_target;
  acc_upper_bound_.resize(plan_points_num_);
  acc_lower_bound_.resize(plan_points_num_);

  auto virtual_acc_curve = MakeVirtualZeroAccCurve();
  const auto& agent_headway_decider_output =
      session_->planning_context().agent_headway_decider_output();
  const auto& agents_headway_map =
      agent_headway_decider_output.agents_headway_Info();
  const auto start_stop_decider_output =
      session_->planning_context().start_stop_decider_output();
  const auto cipv_info = session_->planning_context().cipv_decider_output();
  const auto& ego_state_mgr =
      session_->environmental_model().get_ego_state_manager();
  const auto& agent_mgr = session_->environmental_model().get_agent_manager();
  const auto& lane_borrow_output =
      session_->planning_context().lane_borrow_decider_output();
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto lane_change_state = lane_change_decider_output.curr_state;
  const auto& lon_ref_path_decider_output =
      session_->planning_context().lon_ref_path_decider_output();
  const auto& lon_decision_decider_output =
      session_->planning_context().longitudinal_decision_decider_output();
  const auto& motion_planner_output =
      session_->planning_context().motion_planner_output();
  bool can_increase_acc_upper_bound = false;
  can_increase_acc_upper_bound =
      lon_decision_decider_output.determined_cruise_bound().acc_positive_mps2 >
              1.0 - kEpsilon
          ? true
          : false;
  // cruise acc target
  acc_target.first =
      interp(v_ego, speed_planning_config_.cruise_dec_bound_table.vel_table,
             speed_planning_config_.cruise_dec_bound_table.acc_table);
  if (can_increase_acc_upper_bound) {
    acc_target.second =
        interp(v_ego, speed_planning_config_.cruise_acc_bound_table.vel_table,
               speed_planning_config_.cruise_acc_bound_table.acc_table);
  } else {
    acc_target.second = interp(v_ego, _A_CRUISE_MAX_BP, _A_CRUISE_MAX_V);
  }

  std::pair<double, double> acc_target_with_upper_bound{acc_target.first,
                                                        acc_target.second};
  for (size_t i = 0; i < plan_points_num_; i++) {
    const double t = i * dt_;
    if (upper_bound_infos_[i].agent_id == -1) {
      if (start_stop_decider_output.ego_start_stop_info().state() ==
              common::StartStopInfo::START &&
          lane_borrow_output.is_in_lane_borrow_status == false &&
          lane_change_state == kLaneKeeping) {
        acc_lower_bound_[i] = std::fmin(init_lon_state_[2], acc_target.first);
        acc_upper_bound_[i] = std::fmax(
            speed_planning_config_.lane_keeping_non_cipv_start_acc_bound,
            acc_target.second);
        continue;
      }
      acc_lower_bound_[i] = std::fmin(init_lon_state_[2], acc_target.first);
      if (can_increase_acc_upper_bound) {
        acc_upper_bound_[i] = acc_target.second;
      } else {
        acc_upper_bound_[i] =
            std::fmin(std::fmax(init_lon_state_[2], acc_target.second), 0.8);
      }

      continue;
    }

    double follow_time_gap = 1.0;
    const auto upper_bound_info = upper_bound_infos_[i];
    auto iter = agents_headway_map.find(upper_bound_info.agent_id);
    if (iter != agents_headway_map.end()) {
      follow_time_gap = iter->second.current_headway;
    }
    // const double desire_distance =
    //     GetCalibratedDistance(upper_bound_info.v, v_ego, lc_request);
    const double vel = virtual_acc_curve->Evaluate(1, t);
    const double desire_distance = std::max(
        vel * follow_time_gap + min_follow_distance_m_, min_follow_distance_m_);
    const double desire_velocity = CalcDesiredVelocity(
        upper_bound_info.s, desire_distance, upper_bound_info.v, v_ego);
    const double upper_bound_a = std::fmin(upper_bound_info.a + 0.5, 0.0);
    CalcAccLimits(upper_bound_info, desire_distance, desire_velocity, v_ego,
                  upper_bound_a, &acc_target_with_upper_bound);
    acc_lower_bound_[i] =
        std::fmin(init_lon_state_[2], acc_target_with_upper_bound.first);
    // acc_upper_bound is not relative with upper bound and use acc_target
    // calced by v interpolate
    /* if (start_stop_decider_output.ego_start_stop_info().state() ==
        common::StartStopInfo::START) {
      acc_upper_bound_[i] = std::fmax(acc_target.second,
    acc_target_with_upper_bound.second);
    } */
    if (can_increase_acc_upper_bound) {
      acc_upper_bound_[i] = acc_target.second;
    } else {
      acc_upper_bound_[i] = std::fmax(
          std::fmax(init_lon_state_[2], acc_target_with_upper_bound.second),
          0.3);
    }
    if (start_stop_decider_output.ego_start_stop_info().state() !=
            common::StartStopInfo::START &&
        !can_increase_acc_upper_bound) {
      acc_upper_bound_[i] = std::fmin(acc_upper_bound_[i], 0.8);
    }
  }
  if (lon_ref_path_decider_output.is_cross_vru_pre_handle) {
    for (int32_t i = 0; i < plan_points_num_; i++) {
      acc_lower_bound_[i] = std::fmin(acc_lower_bound_[i], kAccMaxLowerBound);
    }
  }
  if (motion_planner_output.is_limit_lon_acc_bound) {
    // 目前仅针对低速变道使用,避免急加速导致方向盘打不过来
    for (int32_t i = 0; i < plan_points_num_; i++) {
      acc_upper_bound_[i] = std::fmin(
          acc_upper_bound_[i], motion_planner_output.recommended_acc_bound);
    }
  }

  if (lane_change_decider_output.curr_state == kLaneChangeExecution ||
      lane_change_decider_output.curr_state == kLaneChangeComplete ||
      lane_change_decider_output.curr_state == kLaneChangePropose) {
    for (int32_t i = 0; i < plan_points_num_; i++) {
      acc_upper_bound_[i] =
          std::fmax(acc_upper_bound_[i], kLaneChangeAccUpperBound);
    }
  }

  double min_acc_bound_val = 10.0;
  auto min_it =
      std::min_element(acc_lower_bound_.begin(), acc_lower_bound_.end());
  min_acc_bound_val = *min_it;
  std::fill(acc_lower_bound_.begin(), acc_lower_bound_.end(),
            min_acc_bound_val);
}

void BoundMaker::MakeSBound() {
  const auto& environmental_model = session_->environmental_model();
  const auto& ego_lane =
      environmental_model.get_virtual_lane_manager()->get_current_lane();

  if (ego_lane == nullptr) {
    return;
  }

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
      std::fmin(path_length, plan_time_ * kPerSecondPlanLenth);

  s_lower_bound_ = std::vector<double>(plan_points_num_, -0.1);
  s_upper_bound_ = std::vector<double>(plan_points_num_, default_upper_bound);

  const auto ptr_st_graph_helper =
      session_->planning_context().st_graph_helper();

  for (int32_t i = 0; i < plan_points_num_; ++i) {
    const double relative_t = i * dt_;

    const auto corridor_upper_point =
        ptr_st_graph_helper->GetPassCorridorUpperBound(relative_t);
    const auto corridor_lower_point =
        ptr_st_graph_helper->GetPassCorridorLowerBound(relative_t);

    double& upper_bound = s_upper_bound_[i];
    double& lower_bound = s_lower_bound_[i];

    if (corridor_upper_point.valid() && corridor_upper_point.agent_id() != -1) {
      const double confidence = LongRefPathDecider::CalcUpperBoundConfidence(
          corridor_upper_point.s());
      upper_bound = confidence * corridor_upper_point.s() +
                    (1.0 - confidence) * upper_bound;
    }

    if (corridor_lower_point.valid() && corridor_lower_point.agent_id() != -1) {
      lower_bound = corridor_lower_point.s();
    }
  }

  const auto& lon_ref_path_decider_output =
      session_->planning_context().lon_ref_path_decider_output();

  if ((lon_ref_path_decider_output.is_lon_cipv_emergency_stop ||
       lon_ref_path_decider_output.is_joint_danger_emergency_stop) &&
      lon_ref_path_decider_output.comfort_target_upper_bound_infos.size() ==
          plan_points_num_) {
    for (int32_t i = 0; i < plan_points_num_; ++i) {
      s_upper_bound_[i] =
          lon_ref_path_decider_output.comfort_target_upper_bound_infos[i].s;
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
  double jerk_upper_bound =
      speed_planning_config_.speed_planning_bound.jerk_upper_bound;
  if (init_lon_state_[2] > 0.0) {
    jerk_upper_bound = speed_planning_config_.slow_jerk_upper_bound;
  }
  double jerk_lower_bound =
      speed_planning_config_.speed_planning_bound.jerk_lower_bound;
  jerk_upper_bound_ = std::vector<double>(plan_points_num_, jerk_upper_bound);
  jerk_lower_bound_ = std::vector<double>(plan_points_num_, jerk_lower_bound);

  auto max_decel_trajectory = GenerateMaxDecelerationCurve();
  AddMaxDecelCurveDataToProto(max_decel_trajectory);

  const auto& lon_ref_path_decider_output =
      session_->planning_context().lon_ref_path_decider_output();
  if (lon_ref_path_decider_output.is_cross_vru_pre_handle ||
      lon_ref_path_decider_output.is_lon_cipv_emergency_stop ||
      lon_ref_path_decider_output.is_joint_danger_emergency_stop) {
    return;
  }

  const auto& st_graph = session_->planning_context().st_graph_helper();
  if (!st_graph) {
    return;
  }

  bool is_need_comfortable_decel = true;
  for (int32_t i = plan_points_num_ - 1; i >= 0; --i) {
    const double t = i * dt_;
    const auto corridor_upper_point = st_graph->GetPassCorridorUpperBound(t);
    const auto upper_s = corridor_upper_point.s();
    const double ego_s = max_decel_trajectory.Evaluate(0, t);
    const double ego_vel = max_decel_trajectory.Evaluate(1, t);
    const double brake_buffer = ego_vel * kBrakeDelayTimeBuffer;
    if (upper_s - brake_buffer < ego_s) {
      is_need_comfortable_decel = false;
      break;
    }
  }

  if (is_need_comfortable_decel) {
    jerk_lower_bound_ =
        std::vector<double>(plan_points_num_, kJerkLowerComfortableBound);
  }
}

void BoundMaker::MakeSoftBound() {
  const auto& ego_state_mgr =
      session_->environmental_model().get_ego_state_manager();
  auto virtual_acc_curve = MakeVirtualZeroAccCurve();
  const auto& agents_headway_Info = session_->planning_context()
                                        .agent_headway_decider_output()
                                        .agents_headway_Info();

  std::vector<double> s_soft_bound(plan_points_num_, kVirtualUpperBound);

  int32_t gap_front_agent_id = -1;
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto curr_state = lane_change_decider_output.curr_state;
  if (curr_state == StateMachineLaneChangeStatus::kLaneChangeExecution ||
      curr_state == StateMachineLaneChangeStatus::kLaneChangeHold ||
      curr_state == StateMachineLaneChangeStatus::kLaneChangeComplete) {
    const auto front_node_id =
        lane_change_decider_output.lc_gap_info.front_node_id;
    const auto dynamic_world =
        session_->environmental_model().get_dynamic_world();
    if (dynamic_world != nullptr) {
      const auto node_ptr = dynamic_world->GetNode(front_node_id);
      if (node_ptr != nullptr) {
        gap_front_agent_id = node_ptr->node_agent_id();
      }
    }
  }

  for (int32_t i = 0; i < plan_points_num_; ++i) {
    double t = i * dt_;
    const double v_ego = virtual_acc_curve->Evaluate(1, t);
    const auto& upper_bound_info = upper_bound_infos_[i];

    if (upper_bound_info.agent_id == -1) {
      s_soft_bound[i] = s_upper_bound_[i];
      continue;
    }

    const int32_t upper_bound_id = upper_bound_info.agent_id;
    const double v_upper_bound = upper_bound_info.v;
    auto iter = agents_headway_Info.find(upper_bound_id);
    double tau = max_tau;
    if (iter != agents_headway_Info.end()) {
      tau = std::min(tau, iter->second.current_headway);
    }

    if (upper_bound_id == gap_front_agent_id) {
      tau = kLaneChangeFrontAgentHeadway;
    }

    double delta_v = v_ego - v_upper_bound;

    double s_safety =
        s0 + std::max(0.0, tau * v_ego + v_ego * delta_v /
                                             (2.0 * std::sqrt(a_max * b_max)));
    s_soft_bound[i] = std::max(0.0, s_upper_bound_[i] - s_safety);
  }
  s_soft_bound_ = s_soft_bound;
  JSON_DEBUG_VALUE("soft_bound_distance", s_upper_bound_[0] - s_soft_bound_[0]);
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

  double min_acc_bound = acc_lower_bound_[0];
  double max_acc_bound = acc_upper_bound_[0];
  for (int32_t i = 0; i < plan_points_num_; ++i) {
    min_acc_bound = std::min(min_acc_bound, acc_lower_bound_[i]);
    max_acc_bound = std::max(max_acc_bound, acc_upper_bound_[i]);
  }

  min_acc_bound = std::max(kAccLowerComfortableBound, min_acc_bound);

  state_limit.a_max = max_acc_bound;
  state_limit.a_min = min_acc_bound;
  state_limit.j_max = kAccJeck;
  state_limit.j_min = kDecelJeck;

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

void BoundMaker::GenerateUpperBoundInfo() {
  upper_bound_infos_ =
      std::vector<UpperBoundInfo>(plan_points_num_, UpperBoundInfo());
  const auto* st_graph = session_->planning_context().st_graph_helper();
  if (st_graph == nullptr) {
    return;
  }
  const auto agent_manager =
      session_->environmental_model().get_agent_manager();
  if (agent_manager == nullptr) {
    return;
  }
  const double virtual_front_s = plan_time_ * kPerSecondPlanLenth;
  const auto current_lane = session_->environmental_model()
                                .get_virtual_lane_manager()
                                ->get_current_lane();
  const auto current_frenet_coord = current_lane->get_lane_frenet_coord();

  for (size_t i = 0; i < plan_points_num_; i++) {
    const double t = i * dt_;
    const auto& upper_bound = st_graph->GetPassCorridorUpperBound(t);
    const auto agent_id = upper_bound.agent_id();
    if (agent_id != speed::kNoAgentId) {
      const double confidence =
          LongRefPathDecider::CalcUpperBoundConfidence(upper_bound.s());
      upper_bound_infos_[i].s =
          confidence * upper_bound.s() + (1.0 - confidence) * virtual_front_s;
      upper_bound_infos_[i].t = t;
      upper_bound_infos_[i].v = upper_bound.velocity();
      upper_bound_infos_[i].a = upper_bound.acceleration();
      upper_bound_infos_[i].agent_id = agent_id;
      const auto* agent = agent_manager->GetAgent(agent_id);
      if (agent != nullptr) {
        upper_bound_infos_[i].d_path = agent->d_path();
        upper_bound_infos_[i].d_rel = agent->d_rel();
      }
    }
  }
  const auto& lon_ref_path_decider_output =
      session_->planning_context().lon_ref_path_decider_output();
  if (lon_ref_path_decider_output.is_lat_follow ||
      lon_ref_path_decider_output.is_lon_cutin ||
      lon_ref_path_decider_output.is_joint_danger) {
    for (size_t i = 0; i < plan_points_num_ &&
                       i < lon_ref_path_decider_output
                               .comfort_target_upper_bound_infos.size();
         i++) {
      const auto& comfort_upper_bound_info =
          lon_ref_path_decider_output.comfort_target_upper_bound_infos[i];
      upper_bound_infos_[i].s = comfort_upper_bound_info.s;
      upper_bound_infos_[i].t = comfort_upper_bound_info.t;
      upper_bound_infos_[i].v = comfort_upper_bound_info.v;
      upper_bound_infos_[i].agent_id = comfort_upper_bound_info.agent_id;
      const auto* agent =
          agent_manager->GetAgent(comfort_upper_bound_info.agent_id);
      if (agent != nullptr) {
        upper_bound_infos_[i].d_path = agent->d_path();
        upper_bound_infos_[i].d_rel = agent->d_rel();
      }
    }
  }
}

void BoundMaker::CalcAccLimits(const UpperBoundInfo& upper_bound_info,
                               const double desired_distance,
                               const double v_target, const double v_ego,
                               const double lead_one_a_processed,
                               std::pair<double, double>* acc_target) {
  double agent_v_rel = v_ego - upper_bound_info.v;
  double a_lead_contr =
      lead_one_a_processed *
      interp(upper_bound_info.v, _A_LEAD_LOW_SPEED_BP, _A_LEAD_LOW_SPEED_V) *
      interp(desired_distance, _A_LEAD_DISTANCE_BP, _A_LEAD_DISTANCE_V) * 0.8;
  acc_target->second =
      CalcPositiveAccLimit(v_ego, agent_v_rel, acc_target->second);
  // compute max decel
  // assume the car is 1m/s slower
  double v_offset =
      1.2 * std::min(std::max(2.5 - upper_bound_info.d_path, 0.0) / 1.5, 1.0);
  // assume the distance is 1m lower
  double d_offset = 0.5 + 0.2 * v_ego;
  if (v_target < v_ego) {
    // add small value to avoid by zero divisions
    // compute needed accel to get to 1m distance with -1m/s rel speed
    double decel_offset =
        interp(upper_bound_info.v, _DECEL_OFFSET_BP, _DECEL_OFFSET_V);

    double critical_decel =
        CalcCriticalDecel(upper_bound_info.s, agent_v_rel, d_offset, v_offset);
    acc_target->first = std::min(decel_offset + critical_decel + a_lead_contr,
                                 acc_target->first);
  }
  // a_min can't be higher than a_max
  acc_target->first = std::min(acc_target->first, acc_target->second);
  // final check on limits
  acc_target->first = clip(acc_target->first, _A_MAX, _A_MIN);
  acc_target->second = clip(acc_target->second, _A_MAX, _A_MIN);
}

double BoundMaker::CalcPositiveAccLimit(const double v_ego, const double v_rel,
                                        const double a_max_const) {
  double a_max = a_max_const;
  // same as cruise accel, plus add a small correction based on relative
  // lead speed if the lead car is faster, we can accelerate more, if the
  // car is slower, then we can reduce acceleration
  a_max = a_max + interp(v_ego, _A_CORR_BY_SPEED_BP, _A_CORR_BY_SPEED_V) *
                      clip(-v_rel / 4.0, 1.0, -0.5);
  return a_max;
}

double BoundMaker::CalcCriticalDecel(const double d_lead, const double v_rel,
                                     const double d_offset,
                                     const double v_offset) {
  // this function computes the required decel to avoid crashing, given
  // comfort offsets
  double a_critical = -std::pow(std::max(0.0, v_rel + v_offset), 2) /
                      std::max(2 * (d_lead - d_offset), 0.5);
  return a_critical;
}

double BoundMaker::GetCalibratedDistance(const double v_lead,
                                         const double v_ego,
                                         const std::string& lc_request) {
  double v_lead_clip = std::max(v_lead, 0.0);
  double t_gap = interp(v_ego, _T_GAP_VEGO_BP, _T_GAP_VEGO_V);
  // Brake hysteresis
  double v_relative = std::min(std::max(v_ego - v_lead_clip, 0.0), 5.0);
  double distance_hysteresis = v_relative * 0.3;
  // distance when at zero speed
  return min_follow_distance_m_ + v_lead_clip * t_gap + distance_hysteresis;
}

double BoundMaker::CalcDesiredVelocity(const double d_rel, const double d_des,
                                       const double v_lead,
                                       const double v_ego) {
  // *** compute desired speed ***
  // the desired speed curve is divided in 4 portions:
  // 1-constant
  // 2-linear to regain distance
  // 3-linear to shorten distance
  // 4-parabolic (constant decel)
  double v_lead_clip = std::max(v_lead, 0.0);
  const double max_runaway_speed = -2.;  // no slower than 2m/s over the lead
  //  interpolate the lookups to find the slopes for a give lead speed
  double l_slope = interp(v_lead, _L_SLOPE_BP, _L_SLOPE_V);
  double p_slope = interp(v_lead, _P_SLOPE_BP, _P_SLOPE_V);
  // this is where parabola && linear curves are tangents
  double x_linear_to_parabola = p_slope / std::pow(l_slope, 2);
  // parabola offset to have the parabola being tangent to the linear curve
  double x_parabola_offset = p_slope / (2 * std::pow(l_slope, 2));

  double v_rel = v_ego - v_lead;
  double v_rel_des = 0.0;
  double soft_brake_distance = 0.0;
  if (d_rel < d_des) {
    // calculate v_rel_des on the line that connects 0m at max_runaway_speed
    // to d_des
    double v_rel_des_1 = (-max_runaway_speed) / d_des * (d_rel - d_des);
    // calculate v_rel_des on one third of the linear slope
    double v_rel_des_2 = (d_rel - d_des) * l_slope / 3.0;
    // take the min of the 2 above
    v_rel_des = std::min(v_rel_des_1, v_rel_des_2);
    v_rel_des = std::max(v_rel_des, max_runaway_speed);
    soft_brake_distance = d_rel;
  } else if (d_rel < d_des + x_linear_to_parabola) {
    v_rel_des = (d_rel - d_des) * l_slope;
    v_rel_des = std::max(v_rel_des, max_runaway_speed);
    soft_brake_distance = v_rel / l_slope + d_des;
  } else {
    v_rel_des = std::sqrt(2 * (d_rel - d_des - x_parabola_offset) * p_slope);
    soft_brake_distance =
        std::pow(v_rel, 2) / (2 * p_slope) + x_parabola_offset + d_des;
  }
  // compute desired speed
  double v_target = v_rel_des + v_lead;
  return v_target;
}

void BoundMaker::JudgeDangerAgentByMaxDecelCurve(
    const TargetMaker& target_maker) {
  constexpr int32_t kConsiderNum = 25;  // 5.0s
  auto max_deceleration_curve = GenerateMaxDecelerationCurve();
  auto virtual_acc_curve = MakeVirtualZeroAccCurve();
  const auto& st_graph = session_->planning_context().st_graph_helper();

  if (!st_graph) {
    return;
  }

  auto mutable_lon_ref_path_decider_output =
      session_->mutable_planning_context()
          ->mutable_lon_ref_path_decider_output();
  if (!mutable_lon_ref_path_decider_output) {
    return;
  }
  auto& danger_info = mutable_lon_ref_path_decider_output->danger_agent_info;
  danger_info.agents_id_set.clear();

  for (int32_t i = kConsiderNum - 1; i >= 0; --i) {
    const double t = i * dt_;
    const double s_safe = max_deceleration_curve.Evaluate(0, t);
    const auto corridor_upper_point = st_graph->GetPassCorridorUpperBound(t);
    if (corridor_upper_point.agent_id() == speed::kNoAgentId) {
      continue;
    }
    const double agent_s = corridor_upper_point.s();
    const double vel = virtual_acc_curve->Evaluate(1, t);
    const double brake_buffer = vel * kBrakeDelayTimeBuffer;
    auto target_value = target_maker.target_value(t);
    if (target_value.target_type() == TargetType::kFollow ||
        target_value.target_type() == TargetType::kNeighborYield ||
        target_value.target_type() == TargetType::kCautionYield ||
        target_value.target_type() == TargetType::kComfort) {
      // check s_target by s_safe
      if (agent_s - brake_buffer < s_safe) {
        if (danger_info.agents_id_set.find(corridor_upper_point.agent_id()) ==
            danger_info.agents_id_set.end()) {
          danger_info.agents_id_set.insert(corridor_upper_point.agent_id());
        }
      }
    }
  }
  std::vector<double> ids_double(danger_info.agents_id_set.begin(),
                                 danger_info.agents_id_set.end());
  JSON_DEBUG_VECTOR("lon_danger_agent_ids", ids_double, 0)
}

void BoundMaker::AddMaxDecelCurveDataToProto(
    const SecondOrderTimeOptimalTrajectory& max_deceleration_curve) {
// store max decel s curve in proto
#ifdef ENABLE_PROTO_LOG
  auto& debug_info_pb = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto mutable_max_decel_target_data =
      debug_info_pb->mutable_lon_target_s_ref()->mutable_max_decel_target();
  for (int32_t i = 0; i < plan_points_num_; i++) {
    auto* ptr = max_decel_target_pb_.add_max_decel_s_ref();
    const double t = i * dt_;
    const auto s = max_deceleration_curve.Evaluate(0, t);
    const auto v = max_deceleration_curve.Evaluate(1, t);
    const auto a = max_deceleration_curve.Evaluate(2, t);
    const auto j = max_deceleration_curve.Evaluate(3, t);
    ptr->set_s(s);
    ptr->set_v(v);
    ptr->set_a(a);
    ptr->set_j(j);
    ptr->set_t(t);
  }
  mutable_max_decel_target_data->CopyFrom(max_decel_target_pb_);
#endif
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

double BoundMaker::s_soft_bound(const double t) const {
  int32_t index = static_cast<int32_t>(std::round(t / dt_));
  return s_soft_bound_[index];
}

}  // namespace planning
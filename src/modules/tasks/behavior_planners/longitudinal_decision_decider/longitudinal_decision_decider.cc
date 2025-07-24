#include "longitudinal_decision_decider.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <memory>
#include <set>
#include <type_traits>
#include <unordered_map>
#include <utility>

#include "agent/agent.h"
#include "agent/agent_decision.h"
#include "common/utils/kd_path.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "define/geometry.h"
#include "log.h"
#include "longitudinal_decision_decider_output.h"
#include "src/modules/context/environmental_model.h"
#include "src/modules/context/planning_context.h"
#include "st_graph/st_boundary.h"
#include "task_basic_types.h"
#include "task_basic_types.pb.h"
#include "utils_math.h"

namespace planning {

namespace {
double kEpsilon = 1e-6;
bool CalculateAgentSLBoundary(
    const std::shared_ptr<planning_math::KDPath> &planned_path,
    const planning_math::Box2d &agent_box, double *const ptr_min_s,
    double *const ptr_max_s, double *const ptr_min_l, double *const ptr_max_l) {
  if (nullptr == ptr_min_s || nullptr == ptr_max_s || nullptr == ptr_min_l ||
      nullptr == ptr_max_l) {
    return false;
  }
  const auto &all_corners = agent_box.GetAllCorners();
  for (const auto &corner : all_corners) {
    planning::Point2D agent_sl{200.0, 100.0};
    planning::Point2D corner_sl{corner.x(), corner.y()};
    if (!planned_path->XYToSL(corner_sl, agent_sl)) {
      return false;
    }
    *ptr_min_s = std::fmin(*ptr_min_s, agent_sl.x);
    *ptr_max_s = std::fmax(*ptr_max_s, agent_sl.x);
    *ptr_min_l = std::fmin(*ptr_min_l, agent_sl.y);
    *ptr_max_l = std::fmax(*ptr_max_l, agent_sl.y);
  }
  // if (*ptr_min_s > 200.0 || *ptr_max_s < -200.0 || *ptr_min_l > 50.0 ||
  //     *ptr_max_l < -50.0) {
  //   return false;
  // }
  return true;
}

bool CalculateAgentSLBoundary(
    const std::shared_ptr<planning_math::KDPath> &planned_path,
    const planning::agent::Agent &agent, double *const ptr_min_s,
    double *const ptr_max_s, double *const ptr_min_l, double *const ptr_max_l) {
  const auto &agent_box = agent.box();
  bool is_success = CalculateAgentSLBoundary(planned_path, agent_box, ptr_min_s,
                                             ptr_max_s, ptr_min_l, ptr_max_l);
  return is_success;
}
}  // namespace

LongitudinalDecisionDecider::LongitudinalDecisionDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session),
      config_(config_builder->cast<LongitudinalDecisionDeciderConfig>()) {
  name_ = "LongitudinalDecisionDecider";
  plan_time_ = config_.trajectory_time_length;
  dt_ = config_.planning_dt;
  plan_points_num_ = static_cast<size_t>(plan_time_ / dt_) + 1;
}

void LongitudinalDecisionDecider::Reset() {
  cruise_accelerate_count_.first = 0.0;
  cruise_accelerate_count_.second = 0.0;
}

bool LongitudinalDecisionDecider::Execute() {
  LOG_DEBUG("=======LongitudinalDecisionDecider======= \n");
  const auto start_timestamp = IflyTime::Now_ms();
  if (!PreCheck()) {
    LOG_DEBUG("PreCheck failed\n");
    return false;
  }

  bool need_reset = false;  // binwang33 TBD: Add reset conditions
  if (need_reset) {
    Reset();
  }
  DetermineKinematicBoundForCruiseScenario();

  if (!config_.mute_invade_neighbor_decision) {
    UpdateInvadeNeighborResultsForEgoMotionSimPath();
    UpdateInvadeNeighborResults();
  } else {
    LOG_DEBUG("mute invade neighbor decision\n");
  }

  UpdateLaneChangeNeighborResults();

  const auto mutable_st_graph =
      session_->mutable_planning_context()->st_graph();
  if (!has_lon_decision_to_invade_agents_ &&
      !has_lon_decision_to_invade_agents_beside_ego_motion_sim_path_) {
    mutable_st_graph->UpdateNeighborAgentResultsForEgoMotionSimPath();
  }

  MakeDebugMessage();
  const auto end_timestamp = IflyTime::Now_ms();
  LOG_DEBUG("LongitudinalDecisionDecider time cost: [%f]ms \n",
            end_timestamp - start_timestamp);

  return true;
}

void LongitudinalDecisionDecider::DetermineKinematicBoundForCruiseScenario() {
  const auto &environmental_model = session_->environmental_model();
  const auto &ego_state_mgr = environmental_model.get_ego_state_manager();
  const auto &dynamic_world = environmental_model.get_dynamic_world();
  const auto &planning_context = session_->planning_context();
  const auto &mutable_planning_context = session_->mutable_planning_context();

  // 获取init point
  const auto &planning_init_point = ego_state_mgr->planning_init_point();
  const double ego_vel = planning_init_point.v;
  bool can_increase_acc_bound = true;

  // 1.lane change
  const auto lane_change_status =
      planning_context.lane_change_decider_output().curr_state;
  const bool is_in_lane_keeping =
      lane_change_status == StateMachineLaneChangeStatus::kLaneKeeping;
  if (!is_in_lane_keeping) {
    can_increase_acc_bound = false;
  }

  // 2.cruise speed
  const double cruise_speed = ego_state_mgr->ego_v_cruise();
  if (cruise_speed < kCruiseSpeedMinThd) {
    can_increase_acc_bound = false;
  }

  // 3.speed diff
  const double ego_vel_diff_cruise = ego_vel - cruise_speed;
  if (ego_vel_diff_cruise > -kEgoSpeedWithCruiseSpeedDiffThd) {
    can_increase_acc_bound = false;
  }

  // 4.cutin
  if (dynamic_world == nullptr) {
    return;
  }
  const auto *agent_manager = dynamic_world->agent_manager();
  if (agent_manager == nullptr) {
    return;
  }
  const auto &agents = agent_manager->GetAllCurrentAgents();
  for (const auto ptr_agent : agents) {
    if (ptr_agent == nullptr) {
      continue;
    }
    if (ptr_agent->is_cutin()) {
      can_increase_acc_bound = false;
      break;
    }
  }

  // 5.path curv
  const auto &planned_path =
      planning_context.motion_planner_output().lateral_path_coord;
  // binwang33: 翼闻添加接口合入后，此处统一掉
  const double k_preview_distance_thd = ego_vel * kEgoPreviewTimeThd;
  double sample_distance = 0.0;
  while (sample_distance < k_preview_distance_thd) {
    sample_distance += kPreviewDistanceStep;
    auto path_point = planned_path->GetPathPointByS(sample_distance);
    const double curr_kappa = path_point.kappa();
    if (curr_kappa > kMaxCurvThd) {
      can_increase_acc_bound = false;
      break;
    }
  }

  // 6.agents average speed
  const double agent_around_average_speed =
      CalculateAgentsAverageSpeedAroundEgo();
  if (agent_around_average_speed <
      cruise_speed * kAgentsAverageSpeedRatioByCruiseThd) {
    can_increase_acc_bound = false;
  }

  // 7.max_acc_curv VS st_corridor
  // 需要获取st信息，判断最大减速度时，和前车是否安全，前车从 agents_headway_map
  // 中获取
  const bool is_max_acc_curv_safe = IsMaxAccCurvSafeInStGraph();
  if (!is_max_acc_curv_safe) {
    can_increase_acc_bound = false;
  }

  // 8.计算增加后的acc bound
  if (can_increase_acc_bound) {
    cruise_accelerate_count_.first =
        std::min(++(cruise_accelerate_count_.first), kIncreaseAccBoundCountThd);
  } else {
    cruise_accelerate_count_.first =
        std::max(--(cruise_accelerate_count_.first), 0);
  }

  // update flag
  if (cruise_accelerate_count_.first == kIncreaseAccBoundCountThd) {
    cruise_accelerate_count_.second = 1;
  }
  if (cruise_accelerate_count_.first == 0) {
    cruise_accelerate_count_.second = 0;
  }

  if (cruise_accelerate_count_.second > 0) {
    KinematicsBound determined_cruise_bound;
    determined_cruise_bound.acc_positive_mps2 = kCruiseAccelerateThd;
    auto *mutable_longitudinal_decision_decider_output =
        mutable_planning_context
            ->mutable_longitudinal_decision_decider_output();
    if (mutable_longitudinal_decision_decider_output == nullptr) {
      return;
    }
    mutable_longitudinal_decision_decider_output->set_determined_cruise_bound(
        determined_cruise_bound);
  }
}

double LongitudinalDecisionDecider::CalculateAgentsAverageSpeedAroundEgo()
    const {
  const auto &environmental_model = session_->environmental_model();
  const auto &ego_state_mgr = environmental_model.get_ego_state_manager();
  const auto &dynamic_world = environmental_model.get_dynamic_world();
  const auto &virtual_lane_manager =
      environmental_model.get_virtual_lane_manager();
  const auto &agent_manager = dynamic_world->agent_manager();

  if (dynamic_world == nullptr || virtual_lane_manager == nullptr) {
    LOG_ERROR("dynamic_world || virtual_lane_manager is nullptr");
    return 0.0;
  }
  if (agent_manager == nullptr) {
    LOG_ERROR("agent_manager is nullptr");
    return 0.0;
  }

  const auto &planning_init_point = ego_state_mgr->planning_init_point();
  const double ego_vel = planning_init_point.v;
  const double cruise_speed = ego_state_mgr->ego_v_cruise();
  const double k_longitudinal_preview_distance =
      ego_vel * kAroundEgoLongitudinalPreviewTimeThd;
  const double k_longitudinal_backward_distance =
      ego_vel * kAroundEgoLongitudinalBackwardTimeThd;

  const auto &ego_lane = virtual_lane_manager->get_current_lane();
  if (ego_lane == nullptr || ego_lane->get_reference_path() == nullptr) {
    return 0.0;
  }

  const auto &ego_lane_coord =
      ego_lane->get_reference_path()->get_frenet_coord();
  double ego_s_base_ego_lane = 0.0, ego_l_base_ego_lane = 0.0;
  if (!ego_lane_coord->XYToSL(planning_init_point.x, planning_init_point.y,
                              &ego_s_base_ego_lane, &ego_l_base_ego_lane)) {
    return 0.0;
  }

  const auto &agents = agent_manager->GetAllCurrentAgents();
  if (agents.empty()) {
    return cruise_speed;
  }

  std::vector<int64_t> agents_around_ego;
  agents_around_ego.reserve(agents.size());
  for (const auto ptr_agent : agents) {
    double agent_s_base_ego_lane = 0.0, agent_l_base_ego_lane = 0.0;
    if (!ego_lane_coord->XYToSL(ptr_agent->x(), ptr_agent->y(),
                                &agent_s_base_ego_lane,
                                &agent_l_base_ego_lane)) {
      continue;
    }
    const double agent_distance_to_ego =
        agent_s_base_ego_lane - ego_s_base_ego_lane;
    if (std::fabs(agent_l_base_ego_lane) > kAroundEgoLateralDistanceThd) {
      continue;
    }
    if (agent_distance_to_ego > k_longitudinal_preview_distance ||
        agent_distance_to_ego < -k_longitudinal_backward_distance) {
      continue;
    }
    agents_around_ego.emplace_back(ptr_agent->agent_id());
  }
  if (agents_around_ego.empty()) {
    return cruise_speed;
  }

  double agent_around_speed_total = 0.0;

  for (const auto &agent_id : agents_around_ego) {
    const auto *around_agent = agent_manager->GetAgent(agent_id);
    if (around_agent->is_vru()) {
      // Interrupt whenever there is a VRU
      return 0.0;
    }
    agent_around_speed_total += around_agent->speed();
  }
  double agent_around_average_speed =
      agent_around_speed_total / agents_around_ego.size();

  return agent_around_average_speed;
}

bool LongitudinalDecisionDecider::IsMaxAccCurvSafeInStGraph() const {
  constexpr double kMinFollowDistance = 3.0;
  constexpr double kFollowTimeGap = 1.5;
  constexpr double kCruiseAccelerateThd = 1.0;

  const auto &environmental_model = session_->environmental_model();
  const auto &planning_context = session_->planning_context();
  const auto &ego_state_mgr = environmental_model.get_ego_state_manager();

  const auto *st_graph = planning_context.st_graph_helper();
  if (st_graph == nullptr) {
    return false;
  }
  const auto &planning_init_point = ego_state_mgr->planning_init_point();
  const double ego_vel = planning_init_point.v;

  const auto &agents_headway_map =
      planning_context.agent_headway_decider_output().agents_headway_Info();
  auto max_deceleration_curve =
      GenerateMaxDecelerationCurve(planning_init_point);
  for (size_t i = 0; i < plan_points_num_; ++i) {
    const double t = i * dt_;
    const auto &upper_bound = st_graph->GetPassCorridorUpperBound(t);
    if (upper_bound.agent_id() != speed::kNoAgentId) {
      auto iter = agents_headway_map.find(upper_bound.agent_id());
      double follow_time_gap = kFollowTimeGap;
      if (iter != agents_headway_map.end()) {
        follow_time_gap = iter->second.current_headway;
      }
      const double target_s_disatnce = std::max(
          ego_vel * follow_time_gap + kMinFollowDistance, kMinFollowDistance);
      const double max_curve_s = max_deceleration_curve.Evaluate(0, t);
      if (max_curve_s + target_s_disatnce > upper_bound.s()) {
        return false;
      }
    }
  }

  return true;
}

SecondOrderTimeOptimalTrajectory
LongitudinalDecisionDecider::GenerateMaxDecelerationCurve(
    const PlanningInitPoint &init_point) const {
  LonState init_state;
  init_state.p = init_point.lon_init_state.s();
  init_state.v = init_point.lon_init_state.v();
  init_state.a = init_point.lon_init_state.a();

  StateLimit state_limit;
  // acc:[-1.0, 2.0]  jerk:[-2.0, 4.0]
  const double acc_upper_bound = 2.0;
  const double acc_lower_bound = -1.0;
  const double jerk_upper_bound = 4.0;
  const double jerk_lower_bound = -2.0;

  // state_limit.v_end = agent_vel;
  state_limit.v_end = 0.0;
  state_limit.a_max = acc_upper_bound;
  state_limit.a_min = acc_lower_bound;
  state_limit.j_max = jerk_upper_bound;
  state_limit.j_min = jerk_lower_bound;

  return SecondOrderTimeOptimalTrajectory(init_state, state_limit);
}

void LongitudinalDecisionDecider::UpdateInvadeNeighborResults() {
  has_lon_decision_to_invade_agents_ = false;
  LOG_DEBUG(
      "=== LongitudinalDecisionDecider::UpdateInvadeNeighborResults ===\n");
  const auto &lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto &lane_borrow_output =
      session_->planning_context().lane_borrow_decider_output();
  if (lane_change_decider_output.curr_state !=
      StateMachineLaneChangeStatus::kLaneKeeping) {
    LOG_DEBUG(
        "LongitudinalDecisionDecider::UpdateInvadeNeighborResults: "
        "Not in lane keeping, return\n");
    JSON_DEBUG_VALUE("lon_decision_to_invade",
                     has_lon_decision_to_invade_agents_)
    int default_value = -1;
    JSON_DEBUG_VALUE("invade_neighbor_front_agent_id", default_value)
    JSON_DEBUG_VALUE("invade_neighbor_decision", default_value)
    return;
  }

  if (lane_borrow_output.is_in_lane_borrow_status) {
    LOG_DEBUG(
        "LongitudinalDecisionDecider::UpdateInvadeNeighborResults: "
        "In lane borrow status, return\n");
    JSON_DEBUG_VALUE("lon_decision_to_invade",
                     has_lon_decision_to_invade_agents_)
    int default_value = -1;
    JSON_DEBUG_VALUE("invade_neighbor_front_agent_id", default_value)
    JSON_DEBUG_VALUE("invade_neighbor_decision", default_value)
    return;
  }
  const auto &environmental_model = session_->environmental_model();
  const auto &planning_context = session_->planning_context();
  //  const auto &ego_state_mgr =
  // environmental_model.get_ego_state_manager();
  const auto &dynamic_world = environmental_model.get_dynamic_world();
  const auto &virtual_lane_manager =
      environmental_model.get_virtual_lane_manager();
  const auto &ego_cur_lane = virtual_lane_manager->get_current_lane();
  const auto &ego_state_manager = environmental_model.get_ego_state_manager();
  const double planning_init_x =
      ego_state_manager->planning_init_point().lat_init_state.x();
  const double planning_init_y =
      ego_state_manager->planning_init_point().lat_init_state.y();
  const double planned_path_length = session_->planning_context()
                                         .motion_planner_output()
                                         .lateral_path_coord->Length();
  const auto &agent_manager = environmental_model.get_agent_manager();
  const auto &blocked_obs_id_vec = lane_borrow_output.blocked_obs_id;
  const std::set<int32_t> lane_borrow_blocked_obs_id_set(
      blocked_obs_id_vec.begin(), blocked_obs_id_vec.end());
  const auto &planned_path =
      planning_context.motion_planner_output().lateral_path_coord;
  if (!ego_cur_lane) {
    LOG_DEBUG(
        "LongitudinalDecisionDecider::UpdateInvadeNeighborResults: "
        "no current lane, return\n");
    JSON_DEBUG_VALUE("lon_decision_to_invade",
                     has_lon_decision_to_invade_agents_)
    int default_value = -1;
    JSON_DEBUG_VALUE("invade_neighbor_front_agent_id", default_value)
    JSON_DEBUG_VALUE("invade_neighbor_decision", default_value)
    return;
  }
  if (!agent_manager) {
    LOG_DEBUG(
        "LongitudinalDecisionDecider::UpdateInvadeNeighborResults: "
        "no agent manager, return\n");
    JSON_DEBUG_VALUE("lon_decision_to_invade",
                     has_lon_decision_to_invade_agents_)
    int default_value = -1;
    JSON_DEBUG_VALUE("invade_neighbor_front_agent_id", default_value)
    JSON_DEBUG_VALUE("invade_neighbor_decision", default_value)
    return;
  }

  const auto &lat_obstacle_decision = session_->planning_context()
                                          .lateral_obstacle_decider_output()
                                          .lat_obstacle_decision;
  if (lat_obstacle_decision.empty()) {
    LOG_DEBUG(
        "LongitudinalDecisionDecider::UpdateInvadeNeighborResults: "
        "lateral obstacle decision empty, return\n");
    JSON_DEBUG_VALUE("lon_decision_to_invade",
                     has_lon_decision_to_invade_agents_)
    int default_value = -1;
    JSON_DEBUG_VALUE("invade_neighbor_front_agent_id", default_value)
    JSON_DEBUG_VALUE("invade_neighbor_decision", default_value)
    return;
  }
  // 在路口中不启用
  planning::common::IntersectionState intersection_state =
      virtual_lane_manager->GetIntersectionState();
  if (intersection_state == planning::common::IN_INTERSECTION) {
    LOG_DEBUG(
        "LongitudinalDecisionDecider::UpdateInvadeNeighborResults: "
        "in intersection, return\n");
    JSON_DEBUG_VALUE("lon_decision_to_invade",
                     has_lon_decision_to_invade_agents_)
    int default_value = -1;
    JSON_DEBUG_VALUE("invade_neighbor_front_agent_id", default_value)
    JSON_DEBUG_VALUE("invade_neighbor_decision", default_value)
    return;
  }
  if (dynamic_world == nullptr) {
    LOG_DEBUG(
        "LongitudinalDecisionDecider::UpdateInvadeNeighborResults: "
        "no dynamic world, return\n");
    JSON_DEBUG_VALUE("lon_decision_to_invade",
                     has_lon_decision_to_invade_agents_)
    int default_value = -1;
    JSON_DEBUG_VALUE("invade_neighbor_front_agent_id", default_value)
    JSON_DEBUG_VALUE("invade_neighbor_decision", default_value)
    return;
  }
  const auto *agent_manger = dynamic_world->agent_manager();
  if (agent_manger == nullptr) {
    LOG_DEBUG(
        "LongitudinalDecisionDecider::UpdateInvadeNeighborResults: "
        "no agent manager, return\n");
    JSON_DEBUG_VALUE("lon_decision_to_invade",
                     has_lon_decision_to_invade_agents_)
    int default_value = -1;
    JSON_DEBUG_VALUE("invade_neighbor_front_agent_id", default_value)
    JSON_DEBUG_VALUE("invade_neighbor_decision", default_value)
    return;
  }

  auto *st_graph_helper = planning_context.st_graph_helper();
  if (st_graph_helper == nullptr) {
    LOG_DEBUG(
        "LongitudinalDecisionDecider::UpdateInvadeNeighborResults: "
        "no st graph helper, return\n");
    JSON_DEBUG_VALUE("lon_decision_to_invade",
                     has_lon_decision_to_invade_agents_)
    int default_value = -1;
    JSON_DEBUG_VALUE("invade_neighbor_front_agent_id", default_value)
    JSON_DEBUG_VALUE("invade_neighbor_decision", default_value)
    return;
  }

  if (planned_path == nullptr) {
    LOG_DEBUG(
        "LongitudinalDecisionDecider::UpdateInvadeNeighborResults: "
        "no planned path, return\n");
    JSON_DEBUG_VALUE("lon_decision_to_invade",
                     has_lon_decision_to_invade_agents_)
    int default_value = -1;
    JSON_DEBUG_VALUE("invade_neighbor_front_agent_id", default_value)
    JSON_DEBUG_VALUE("invade_neighbor_decision", default_value)
    return;
  }

  // get closet invade neighbor gap's agents id
  DetermineClosestInvadeNeighborGapInfo(
      ego_cur_lane, planning_init_x, planning_init_y, planned_path_length,
      lat_obstacle_decision, lane_borrow_blocked_obs_id_set, agent_manager,
      st_graph_helper, planned_path);
  const auto invade_neighbor_front_agent_id =
      closest_neighbor_invade_gap_agents_id_.second;
  const auto invade_neighbor_rear_agent_id =
      closest_neighbor_invade_gap_agents_id_.first;
  // <bool, bool> : <ignore invade-gap-rear-agent, ignore
  // invade-gap-front-agent>
  // const std::pair<bool, bool> ignore_invade_gap_rear_agent_front_agent_pair =
  //     IgnoreInvadeNeighborAgents(
  //         agent_manager->GetAgent(invade_neighbor_rear_agent_id),
  //         agent_manager->GetAgent(invade_neighbor_front_agent_id),
  //         session_->planning_context()
  //             .motion_planner_output()
  //             .lateral_path_coord);
  CalculateInvadeNeighborAgentsDecisionInfo(
      agent_manager->GetAgent(invade_neighbor_rear_agent_id),
      agent_manager->GetAgent(invade_neighbor_front_agent_id),
      session_->planning_context().motion_planner_output().lateral_path_coord);
  const bool ignore_front_invade_agent =
      invade_neighbor_agents_decision_info_.ignore_front_agent;

  std::unordered_map<int32_t, speed::STBoundary::DecisionType>
      neighbor_agents_decision_table;
  const auto &agent_id_st_boundaries_map =
      st_graph_helper->GetAgentIdSTBoundariesMap();
  const auto &neighbor_agent_id_st_boundraies_map =
      st_graph_helper->GetNeighborAgentIdSTBoundariesMap();
  if (invade_neighbor_front_agent_id != -1 &&
      agent_id_st_boundaries_map.find(invade_neighbor_front_agent_id) ==
          agent_id_st_boundaries_map.end() &&
      /*neighbor_agent_id_st_boundraies_map.find(
          invade_neighbor_front_agent_id) ==
          neighbor_agent_id_st_boundraies_map.end() &&*/
      !ignore_front_invade_agent) {
    const auto front_invade_agent =
        agent_manager->GetAgent(invade_neighbor_front_agent_id);
    ConstructNeighborLaneStGraph(front_invade_agent);
    neighbor_agents_decision_table[invade_neighbor_front_agent_id] =
        invade_neighbor_agents_decision_info_.current_front_agent_decision;
  }

  const auto mutable_st_graph =
      session_->mutable_planning_context()->st_graph();
  if (!neighbor_agents_decision_table.empty()) {
    mutable_st_graph->UpdateNeighborAgentResults(
        neighbor_agents_decision_table);
    has_lon_decision_to_invade_agents_ = true;
  }
  JSON_DEBUG_VALUE("lon_decision_to_invade", has_lon_decision_to_invade_agents_)
  JSON_DEBUG_VALUE("invade_neighbor_front_agent_id",
                   invade_neighbor_front_agent_id)
  JSON_DEBUG_VALUE(
      "invade_neighbor_decision",
      static_cast<int>(
          invade_neighbor_agents_decision_info_.current_front_agent_decision))
}

void LongitudinalDecisionDecider::
    UpdateInvadeNeighborResultsForEgoMotionSimPath() {
  has_lon_decision_to_invade_agents_beside_ego_motion_sim_path_ = false;
  LOG_DEBUG(
      "=== LongitudinalDecisionDecider::UpdateInvadeNeighborResults ===\n");
  const auto &lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto &lane_borrow_output =
      session_->planning_context().lane_borrow_decider_output();
  if (lane_change_decider_output.curr_state !=
      StateMachineLaneChangeStatus::kLaneKeeping) {
    LOG_DEBUG(
        "LongitudinalDecisionDecider::UpdateInvadeNeighborResults: "
        "Not in lane keeping, return\n");
    JSON_DEBUG_VALUE(
        "lon_decision_to_invade_ego_motion_sim_path",
        has_lon_decision_to_invade_agents_beside_ego_motion_sim_path_)
    int default_value = -1;
    JSON_DEBUG_VALUE("invade_neighbor_front_agent_id_ego_motion_sim_path",
                     default_value)
    JSON_DEBUG_VALUE("invade_neighbor_decision_ego_motion_sim_path",
                     default_value)
    return;
  }

  if (lane_borrow_output.is_in_lane_borrow_status) {
    LOG_DEBUG(
        "LongitudinalDecisionDecider::UpdateInvadeNeighborResults: "
        "In lane borrow status, return\n");
    JSON_DEBUG_VALUE(
        "lon_decision_to_invade_ego_motion_sim_path",
        has_lon_decision_to_invade_agents_beside_ego_motion_sim_path_)
    int default_value = -1;
    JSON_DEBUG_VALUE("invade_neighbor_front_agent_id_ego_motion_sim_path",
                     default_value)
    JSON_DEBUG_VALUE("invade_neighbor_decision_ego_motion_sim_path",
                     default_value)
    return;
  }
  const auto &environmental_model = session_->environmental_model();
  const auto &planning_context = session_->planning_context();
  //  const auto &ego_state_mgr =
  // environmental_model.get_ego_state_manager();
  const auto &dynamic_world = environmental_model.get_dynamic_world();
  const auto &virtual_lane_manager =
      environmental_model.get_virtual_lane_manager();
  const auto &ego_cur_lane = virtual_lane_manager->get_current_lane();
  const auto &ego_state_manager = environmental_model.get_ego_state_manager();
  const double planning_init_x =
      ego_state_manager->planning_init_point().lat_init_state.x();
  const double planning_init_y =
      ego_state_manager->planning_init_point().lat_init_state.y();
  const double planned_path_length = session_->planning_context()
                                         .motion_planner_output()
                                         .lateral_path_coord->Length();
  const auto &agent_manager = environmental_model.get_agent_manager();
  const auto &blocked_obs_id_vec = lane_borrow_output.blocked_obs_id;
  const std::set<int32_t> lane_borrow_blocked_obs_id_set(
      blocked_obs_id_vec.begin(), blocked_obs_id_vec.end());
  const auto ego_motion_sim_path =
      planning_context.ego_motion_preplanner_output()
          .ego_motion_simulation_result()
          ->lat_lon_vehicle_motion_path_ptr;
  if (!ego_cur_lane) {
    LOG_DEBUG(
        "LongitudinalDecisionDecider::UpdateInvadeNeighborResults: "
        "no current lane, return\n");
    JSON_DEBUG_VALUE(
        "lon_decision_to_invade_ego_motion_sim_path",
        has_lon_decision_to_invade_agents_beside_ego_motion_sim_path_)
    int default_value = -1;
    JSON_DEBUG_VALUE("invade_neighbor_front_agent_id_ego_motion_sim_path",
                     default_value)
    JSON_DEBUG_VALUE("invade_neighbor_decision_ego_motion_sim_path",
                     default_value)
    return;
  }
  if (!agent_manager) {
    LOG_DEBUG(
        "LongitudinalDecisionDecider::UpdateInvadeNeighborResults: "
        "no agent manager, return\n");
    JSON_DEBUG_VALUE(
        "lon_decision_to_invade_ego_motion_sim_path",
        has_lon_decision_to_invade_agents_beside_ego_motion_sim_path_)
    int default_value = -1;
    JSON_DEBUG_VALUE("invade_neighbor_front_agent_id_ego_motion_sim_path",
                     default_value)
    JSON_DEBUG_VALUE("invade_neighbor_decision_ego_motion_sim_path",
                     default_value)
    return;
  }

  const auto &lat_obstacle_decision = session_->planning_context()
                                          .lateral_obstacle_decider_output()
                                          .lat_obstacle_decision;
  if (lat_obstacle_decision.empty()) {
    LOG_DEBUG(
        "LongitudinalDecisionDecider::UpdateInvadeNeighborResults: "
        "lateral obstacle decision empty, return\n");
    JSON_DEBUG_VALUE(
        "lon_decision_to_invade_ego_motion_sim_path",
        has_lon_decision_to_invade_agents_beside_ego_motion_sim_path_)
    int default_value = -1;
    JSON_DEBUG_VALUE("invade_neighbor_front_agent_id_ego_motion_sim_path",
                     default_value)
    JSON_DEBUG_VALUE("invade_neighbor_decision_ego_motion_sim_path",
                     default_value)
    return;
  }
  // 在路口中不启用
  planning::common::IntersectionState intersection_state =
      virtual_lane_manager->GetIntersectionState();
  // if (intersection_state == planning::common::IN_INTERSECTION) {
  //   LOG_DEBUG(
  //       "LongitudinalDecisionDecider::UpdateInvadeNeighborResults: "
  //       "in intersection, return\n");
  //   JSON_DEBUG_VALUE(
  //       "lon_decision_to_invade_ego_motion_sim_path",
  //       has_lon_decision_to_invade_agents_beside_ego_motion_sim_path_)
  //   int default_value = -1;
  //   JSON_DEBUG_VALUE("invade_neighbor_front_agent_id_ego_motion_sim_path",
  //                    default_value)
  //   return;
  // }
  if (dynamic_world == nullptr) {
    LOG_DEBUG(
        "LongitudinalDecisionDecider::UpdateInvadeNeighborResults: "
        "no dynamic world, return\n");
    JSON_DEBUG_VALUE(
        "lon_decision_to_invade_ego_motion_sim_path",
        has_lon_decision_to_invade_agents_beside_ego_motion_sim_path_)
    int default_value = -1;
    JSON_DEBUG_VALUE("invade_neighbor_front_agent_id_ego_motion_sim_path",
                     default_value)
    JSON_DEBUG_VALUE("invade_neighbor_decision_ego_motion_sim_path",
                     default_value)
    return;
  }
  const auto *agent_manger = dynamic_world->agent_manager();
  if (agent_manger == nullptr) {
    LOG_DEBUG(
        "LongitudinalDecisionDecider::UpdateInvadeNeighborResults: "
        "no agent manager, return\n");
    JSON_DEBUG_VALUE(
        "lon_decision_to_invade_ego_motion_sim_path",
        has_lon_decision_to_invade_agents_beside_ego_motion_sim_path_)
    int default_value = -1;
    JSON_DEBUG_VALUE("invade_neighbor_front_agent_id_ego_motion_sim_path",
                     default_value)
    JSON_DEBUG_VALUE("invade_neighbor_decision_ego_motion_sim_path",
                     default_value)
    return;
  }

  auto *st_graph_helper = planning_context.st_graph_helper();
  if (st_graph_helper == nullptr) {
    LOG_DEBUG(
        "LongitudinalDecisionDecider::UpdateInvadeNeighborResults: "
        "no st graph helper, return\n");
    JSON_DEBUG_VALUE(
        "lon_decision_to_invade_ego_motion_sim_path",
        has_lon_decision_to_invade_agents_beside_ego_motion_sim_path_)
    int default_value = -1;
    JSON_DEBUG_VALUE("invade_neighbor_front_agent_id_ego_motion_sim_path",
                     default_value)
    JSON_DEBUG_VALUE("invade_neighbor_decision_ego_motion_sim_path",
                     default_value)
    return;
  }

  if (ego_motion_sim_path == nullptr) {
    LOG_DEBUG(
        "LongitudinalDecisionDecider::UpdateInvadeNeighborResults: "
        "no ego motion sim path, return\n");
    JSON_DEBUG_VALUE(
        "lon_decision_to_invade_ego_motion_sim_path",
        has_lon_decision_to_invade_agents_beside_ego_motion_sim_path_)
    int default_value = -1;
    JSON_DEBUG_VALUE("invade_neighbor_front_agent_id_ego_motion_sim_path",
                     default_value)
    JSON_DEBUG_VALUE("invade_neighbor_decision_ego_motion_sim_path",
                     default_value)
    return;
  }

  // get closet invade neighbor gap's agents id
  DetermineClosestInvadeNeighborGapInfo(
      ego_cur_lane, planning_init_x, planning_init_y, planned_path_length,
      lat_obstacle_decision, lane_borrow_blocked_obs_id_set, agent_manager,
      st_graph_helper, ego_motion_sim_path, true);
  const auto invade_neighbor_front_agent_id =
      closest_neighbor_invade_gap_agents_id_.second;
  const auto invade_neighbor_rear_agent_id =
      closest_neighbor_invade_gap_agents_id_.first;
  // <bool, bool> : <ignore invade-gap-rear-agent, ignore
  // invade-gap-front-agent>
  // const std::pair<bool, bool> ignore_invade_gap_rear_agent_front_agent_pair =
  //     IgnoreInvadeNeighborAgents(
  //         agent_manager->GetAgent(invade_neighbor_rear_agent_id),
  //         agent_manager->GetAgent(invade_neighbor_front_agent_id),
  //         ego_motion_sim_path, true);
  CalculateInvadeNeighborAgentsDecisionInfo(
      agent_manager->GetAgent(invade_neighbor_rear_agent_id),
      agent_manager->GetAgent(invade_neighbor_front_agent_id),
      ego_motion_sim_path, true);
  const bool ignore_front_invade_agent =
      invade_neighbor_agents_decision_info_.ignore_front_agent;

  std::unordered_map<int32_t, speed::STBoundary::DecisionType>
      neighbor_agents_decision_table;
  const auto &agent_id_st_boundaries_map =
      st_graph_helper->GetAgentIdSTBoundariesMap();
  const auto &neighbor_agent_id_st_boundraies_map =
      st_graph_helper->GetNeighborAgentIdSTBoundariesMap();
  if (invade_neighbor_front_agent_id != -1 &&
      agent_id_st_boundaries_map.find(invade_neighbor_front_agent_id) ==
          agent_id_st_boundaries_map.end() &&
      /*neighbor_agent_id_st_boundraies_map.find(
          invade_neighbor_front_agent_id) ==
          neighbor_agent_id_st_boundraies_map.end() &&*/
      !ignore_front_invade_agent) {
    const auto front_invade_agent =
        agent_manager->GetAgent(invade_neighbor_front_agent_id);
    ConstructNeighborLaneStGraph(front_invade_agent);
    neighbor_agents_decision_table[invade_neighbor_front_agent_id] =
        invade_neighbor_agents_decision_info_.current_front_agent_decision;
  }

  const auto mutable_st_graph =
      session_->mutable_planning_context()->st_graph();
  if (!neighbor_agents_decision_table.empty()) {
    mutable_st_graph->UpdateNeighborAgentResults(
        neighbor_agents_decision_table);
    has_lon_decision_to_invade_agents_beside_ego_motion_sim_path_ = true;
  }
  JSON_DEBUG_VALUE(
      "lon_decision_to_invade_ego_motion_sim_path",
      has_lon_decision_to_invade_agents_beside_ego_motion_sim_path_)
  JSON_DEBUG_VALUE("invade_neighbor_front_agent_id_ego_motion_sim_path",
                   invade_neighbor_front_agent_id)
  JSON_DEBUG_VALUE(
      "invade_neighbor_decision_ego_motion_sim_path",
      static_cast<int>(
          invade_neighbor_agents_decision_info_.current_front_agent_decision))
}

void LongitudinalDecisionDecider::CalculateInvadeNeighborAgentsDecisionInfo(
    const agent::Agent *invade_gap_rear_agent,
    const agent::Agent *invade_gap_front_agent,
    const std::shared_ptr<planning_math::KDPath> &planned_path,
    const bool use_ego_motion_sim_path) {
  RestInvadeNeighborAgentsDecisionInfo();
  if (invade_gap_rear_agent == nullptr && invade_gap_front_agent == nullptr) {
    if (!use_ego_motion_sim_path) {
      JSON_DEBUG_VALUE("ego_ttc_to_front_invade_agent",
                       std::numeric_limits<double>::max())
    } else {
      JSON_DEBUG_VALUE("ego_ttc_to_front_invade_agent_ego_sim_path",
                       std::numeric_limits<double>::max())
    }
    return;
  }
  const auto &ego_state_mgr =
      session_->environmental_model().get_ego_state_manager();
  const double planning_init_vel =
      ego_state_mgr->planning_init_point().lon_init_state.v();
  // ignore invade agents as default, then judge
  bool ignore_gap_rear_agent = true;
  bool ignore_gap_front_agent = true;
  speed::STBoundary::DecisionType front_agent_decision =
      speed::STBoundary::DecisionType::UNKNOWN;
  const auto &ego_vehi_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();

  // check front agent
  if (invade_gap_front_agent) {
    double planning_init_s = session_->planning_context()
                                 .motion_planner_output()
                                 .path_backward_appended_length;
    Point2D invade_gap_front_agent_xy(invade_gap_front_agent->x(),
                                      invade_gap_front_agent->y());
    Point2D invade_gap_front_agent_sl;
    const bool access_to_front_agent_center_sl = planned_path->XYToSL(
        invade_gap_front_agent_xy, invade_gap_front_agent_sl);
    const double front_agent_center_s = invade_gap_front_agent_sl.x;
    const double front_agent_center_l = invade_gap_front_agent_sl.y;
    if (!access_to_front_agent_center_sl) {
      ignore_gap_rear_agent = true;
      if (!use_ego_motion_sim_path) {
        JSON_DEBUG_VALUE("ego_ttc_to_front_invade_agent",
                         std::numeric_limits<double>::max())
      } else {
        JSON_DEBUG_VALUE("ego_ttc_to_front_invade_agent_ego_sim_path",
                         std::numeric_limits<double>::max())
      }
    } else {
      const auto agent_center_matched_point =
          planned_path->GetPathPointByS(front_agent_center_s);
      const double agent_target_lane_heading_diff =
          invade_gap_front_agent->box().heading() -
          agent_center_matched_point.theta();
      const double agent_vel_frenet = invade_gap_front_agent->speed() *
                                      std::cos(agent_target_lane_heading_diff);
      double agent_relative_s_to_ego = std::numeric_limits<double>::max();
      if (!use_ego_motion_sim_path) {
        agent_relative_s_to_ego =
            front_agent_center_s - 0.5 * invade_gap_front_agent->length() -
            planning_init_s -
            (ego_vehi_param.length - ego_vehi_param.rear_edge_to_rear_axle);
      } else {
        // use ego motion sim path, so the planning init s is 0
        agent_relative_s_to_ego =
            front_agent_center_s - 0.5 * invade_gap_front_agent->length() -
            (ego_vehi_param.length - ego_vehi_param.rear_edge_to_rear_axle);
      }

      double vel_difference_to_agent = planning_init_vel - agent_vel_frenet;
      if (vel_difference_to_agent > -kEpsilon &&
          vel_difference_to_agent < 0.0) {
        vel_difference_to_agent = -kEpsilon;
      } else if (vel_difference_to_agent < kEpsilon &&
                 vel_difference_to_agent > 0.0) {
        vel_difference_to_agent = kEpsilon;
      }

      double ego_ttc_to_front_agent =
          agent_relative_s_to_ego / vel_difference_to_agent;
      if (!use_ego_motion_sim_path) {
        JSON_DEBUG_VALUE("ego_ttc_to_front_invade_agent",
                         ego_ttc_to_front_agent)
      } else {
        JSON_DEBUG_VALUE("ego_ttc_to_front_invade_agent_ego_sim_path",
                         ego_ttc_to_front_agent)
      }

      const bool ego_ttc_to_front_agent_dangerous =
          ego_ttc_to_front_agent < config_.ignore_ego_ttc_to_agent_thrd &&
          ego_ttc_to_front_agent > 0.0;

      // const bool has_distance_buffer_to_predeceleration =
      //     agent_relative_s_to_ego >
      //     config_.ego_predeceleration_distance_to_front_agent_threshold;

      if (ego_ttc_to_front_agent_dangerous) {
        ignore_gap_front_agent = false;
        front_agent_decision = speed::STBoundary::DecisionType::NEIGHBOR_YIELD;
      }

      if (vel_difference_to_agent <
              config_.close_to_same_velocity_difference_buffer &&
          (invade_neighbor_agents_decision_info_.last_front_agent_decision ==
               speed::STBoundary::DecisionType::NEIGHBOR_YIELD ||
           invade_neighbor_agents_decision_info_.last_front_agent_decision ==
               speed::STBoundary::DecisionType::NEIGHBOR_OVERTAKE)) {
        // if the front agent has been yielded, then overtake it
        ignore_gap_front_agent = false;
        front_agent_decision =
            speed::STBoundary::DecisionType::NEIGHBOR_OVERTAKE;
      }
    }
  } else {
    if (!use_ego_motion_sim_path) {
      JSON_DEBUG_VALUE("ego_ttc_to_front_invade_agent",
                       std::numeric_limits<double>::max())
    } else {
      JSON_DEBUG_VALUE("ego_ttc_to_front_invade_agent_ego_sim_path",
                       std::numeric_limits<double>::max())
    }
  }

  // assemble invade neighbor agents decision info
  invade_neighbor_agents_decision_info_.ignore_rear_agent =
      ignore_gap_rear_agent;
  invade_neighbor_agents_decision_info_.ignore_front_agent =
      ignore_gap_front_agent;
  invade_neighbor_agents_decision_info_.front_agent_id =
      invade_gap_front_agent ? invade_gap_front_agent->agent_id()
                             : agent::AgentDefaultInfo::kNoAgentId;
  invade_neighbor_agents_decision_info_.rear_agent_id =
      invade_gap_rear_agent ? invade_gap_rear_agent->agent_id()
                            : agent::AgentDefaultInfo::kNoAgentId;
  invade_neighbor_agents_decision_info_.current_front_agent_decision =
      front_agent_decision;
  invade_neighbor_agents_decision_info_.last_front_agent_decision =
      ignore_gap_front_agent ? speed::STBoundary::DecisionType::UNKNOWN
                             : front_agent_decision;
}

void LongitudinalDecisionDecider::RestInvadeNeighborAgentsDecisionInfo() {
  invade_neighbor_agents_decision_info_.current_front_agent_decision =
      speed::STBoundary::DecisionType::UNKNOWN;
  invade_neighbor_agents_decision_info_.ignore_rear_agent = true;
  invade_neighbor_agents_decision_info_.ignore_front_agent = true;
  invade_neighbor_agents_decision_info_.front_agent_id =
      agent::AgentDefaultInfo::kNoAgentId;
  invade_neighbor_agents_decision_info_.rear_agent_id =
      agent::AgentDefaultInfo::kNoAgentId;
  // invade_neighbor_agents_decision_info_.ego_ttc_to_front_agent =
  //     std::numeric_limits<double>::max();
  // invade_neighbor_agents_decision_info_.current_distance_ego_to_front_agent =
  //     std::numeric_limits<double>::max();
}

std::pair<bool, bool> LongitudinalDecisionDecider::IgnoreInvadeNeighborAgents(
    const agent::Agent *invade_gap_rear_agent,
    const agent::Agent *invade_gap_front_agent,
    const std::shared_ptr<planning_math::KDPath> &planned_path,
    const bool use_ego_motion_sim_path) const {
  if (invade_gap_rear_agent == nullptr && invade_gap_front_agent == nullptr) {
    if (!use_ego_motion_sim_path) {
      JSON_DEBUG_VALUE("ego_ttc_to_front_invade_agent",
                       std::numeric_limits<double>::max())
    } else {
      JSON_DEBUG_VALUE("ego_ttc_to_front_invade_agent_ego_sim_path",
                       std::numeric_limits<double>::max())
    }
    return std::make_pair(true, true);
  }
  // ignore invade agents as default, then judge
  std::pair<bool, bool> ignore_invade_gap_rear_front_agents_pair{true, true};

  auto &ignore_gap_rear_agent = ignore_invade_gap_rear_front_agents_pair.first;
  auto &ignore_gap_front_agent =
      ignore_invade_gap_rear_front_agents_pair.second;
  const auto &ego_vehi_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();

  // check front agent
  if (invade_gap_front_agent) {
    double planning_init_s = session_->planning_context()
                                 .motion_planner_output()
                                 .path_backward_appended_length;
    Point2D invade_gap_front_agent_xy(invade_gap_front_agent->x(),
                                      invade_gap_front_agent->y());
    Point2D invade_gap_front_agent_sl;
    const bool access_to_front_agent_center_sl = planned_path->XYToSL(
        invade_gap_front_agent_xy, invade_gap_front_agent_sl);
    const double front_agent_center_s = invade_gap_front_agent_sl.x;
    const double front_agent_center_l = invade_gap_front_agent_sl.y;
    if (!access_to_front_agent_center_sl) {
      ignore_gap_rear_agent = true;
      if (!use_ego_motion_sim_path) {
        JSON_DEBUG_VALUE("ego_ttc_to_front_invade_agent",
                         std::numeric_limits<double>::max())
      } else {
        JSON_DEBUG_VALUE("ego_ttc_to_front_invade_agent_ego_sim_path",
                         std::numeric_limits<double>::max())
      }
    } else {
      const auto agent_center_matched_point =
          planned_path->GetPathPointByS(front_agent_center_s);
      const double agent_target_lane_heading_diff =
          invade_gap_front_agent->box().heading() -
          agent_center_matched_point.theta();
      const double agent_vel_frenet = invade_gap_front_agent->speed() *
                                      std::cos(agent_target_lane_heading_diff);
      double agent_relative_s_to_ego = std::numeric_limits<double>::max();
      if (!use_ego_motion_sim_path) {
        agent_relative_s_to_ego =
            front_agent_center_s - 0.5 * invade_gap_front_agent->length() -
            planning_init_s -
            (ego_vehi_param.length - ego_vehi_param.rear_edge_to_rear_axle);
      } else {
        // use ego motion sim path, so the planning init s is 0
        agent_relative_s_to_ego =
            front_agent_center_s - 0.5 * invade_gap_front_agent->length() -
            (ego_vehi_param.length - ego_vehi_param.rear_edge_to_rear_axle);
      }

      const auto &planning_init_point = session_->environmental_model()
                                            .get_ego_state_manager()
                                            ->planning_init_point();
      const double planning_init_vel = planning_init_point.lon_init_state.v();
      double vel_difference_to_agent = planning_init_vel - agent_vel_frenet;
      if (vel_difference_to_agent > -kEpsilon &&
          vel_difference_to_agent < 0.0) {
        vel_difference_to_agent = -kEpsilon;
      } else if (vel_difference_to_agent < kEpsilon &&
                 vel_difference_to_agent > 0.0) {
        vel_difference_to_agent = kEpsilon;
      }

      double ego_ttc_to_front_agent =
          agent_relative_s_to_ego / vel_difference_to_agent;
      if (!use_ego_motion_sim_path) {
        JSON_DEBUG_VALUE("ego_ttc_to_front_invade_agent",
                         ego_ttc_to_front_agent)
      } else {
        JSON_DEBUG_VALUE("ego_ttc_to_front_invade_agent_ego_sim_path",
                         ego_ttc_to_front_agent)
      }
      if (ego_ttc_to_front_agent < config_.ignore_ego_ttc_to_agent_thrd &&
          ego_ttc_to_front_agent > 0.0) {
        ignore_gap_front_agent = false;
      }
    }
  } else {
    if (!use_ego_motion_sim_path) {
      JSON_DEBUG_VALUE("ego_ttc_to_front_invade_agent",
                       std::numeric_limits<double>::max())
    } else {
      JSON_DEBUG_VALUE("ego_ttc_to_front_invade_agent_ego_sim_path",
                       std::numeric_limits<double>::max())
    }
  }
  return ignore_invade_gap_rear_front_agents_pair;
}

// NOTE: ignored agents in lateral decision mean agents invade into ego cur lane
// and do not overlap with planned path probably, need consider the gap between
// them
void LongitudinalDecisionDecider::DetermineClosestInvadeNeighborGapInfo(
    const std::shared_ptr<VirtualLane> &ego_cur_lane,
    const double planning_init_x, const double planning_init_y,
    const double planned_path_length,
    const std::unordered_map<uint32_t, LatObstacleDecisionType>
        &lat_obstacle_decision,
    const std::set<int32_t> &lane_borrow_blocked_obs_id_set,
    const std::shared_ptr<agent::AgentManager> &agent_manager,
    const speed::StGraphHelper *st_graph_helper,
    const std::shared_ptr<planning_math::KDPath> &planned_path,
    const bool use_ego_motion_sim_path) {
  closest_neighbor_invade_gap_agents_id_ = std::pair<int32_t, int32_t>(-1, -1);
  const auto &ego_cur_lane_frenet_coord = ego_cur_lane->get_lane_frenet_coord();
  const auto &ego_vehi_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();

  double planning_init_s = session_->planning_context()
                               .motion_planner_output()
                               .path_backward_appended_length;
  double planning_init_l = 0.0;

  const auto &agent_id_st_boundaries_map =
      st_graph_helper->GetAgentIdSTBoundariesMap();
  std::map<double, int32_t> invade_agents_s_id_map;
  invade_agents_s_id_map[planning_init_s] = 0;

  // already on ego motion sim path, need consider
  if (use_ego_motion_sim_path) {
    const auto &st_graph = session_->planning_context().st_graph_helper();
    const auto &neighbor_agent_id_st_boundaries_map =
        st_graph->GetNeighborAgentIdSTBoundariesMap();
    for (const auto &[neighbor_agent_id, st_boundary] :
         neighbor_agent_id_st_boundaries_map) {
      double agent_min_s = std::numeric_limits<double>::max(),
             agent_max_s = -10.0;
      double agent_min_l = std::numeric_limits<double>::max(),
             agent_max_l = -50.0;
      const auto agent = agent_manager->GetAgent(neighbor_agent_id);
      if (!CalculateAgentSLBoundary(planned_path, *agent, &agent_min_s,
                                    &agent_max_s, &agent_min_l, &agent_max_l)) {
        LOG_DEBUG(
            "agent (ID: %d) corners projects to ego motion sim path failed , "
            "skip\n",
            neighbor_agent_id);
        continue;
      }
      invade_agents_s_id_map[agent_min_s] = neighbor_agent_id;
    }
  }

  for (const auto [id, lat_decision_type] : lat_obstacle_decision) {
    if (/*lat_decision_type != LatObstacleDecisionType::IGNORE ||*/
        agent_id_st_boundaries_map.find(id) !=
        agent_id_st_boundaries_map.end()) {  // already in st graph(like cipv)
      LOG_DEBUG("agent (ID: %d) is already in st graph, skip\n", id);
      continue;
    }
    if (lane_borrow_blocked_obs_id_set.find(id) !=
        lane_borrow_blocked_obs_id_set.end()) {  // ignore lane borrow agents
      LOG_DEBUG("agent (ID: %d) is lane borrow target, skip\n", id);
      continue;
    }
    const auto agent = agent_manager->GetAgent(id);
    if (agent && agent->is_static()) {
      LOG_DEBUG("agent (ID: %d) is static, skip\n", id);
      continue;
    }

    // const auto invade_agent = agent_manager->GetAgent(id);
    // Point2D invade_agent_sl;
    // Point2D invade_agent_xy(invade_agent->x(), invade_agent->y());
    double agent_min_s = std::numeric_limits<double>::max(),
           agent_max_s = -10.0;
    double agent_min_l = std::numeric_limits<double>::max(),
           agent_max_l = -50.0;
    if (!CalculateAgentSLBoundary(planned_path, *agent, &agent_min_s,
                                  &agent_max_s, &agent_min_l, &agent_max_l)) {
      LOG_DEBUG(
          "agent (ID: %d) corners projects to planned_path failed , skip\n",
          id);
      continue;
    }
    // if (!planned_path->XYToSL(invade_agent_xy,
    //                           invade_agent_sl)) {  // far from ego in lon s
    //   LOG_DEBUG("agent (ID: %d) is far from ego in lon s, skip\n", id);
    //   continue;
    // }

    const static double lat_distance_thrd =
        config_.lat_distance_close_enough_to_planned_path_thrd;
    // const double invade_agent_s = invade_agent_sl.x;
    // const double invade_agent_l = invade_agent_sl.y;
    // const double invade_agent_lat_distance_to_path =
    //     std::fabs(invade_agent_l) - 0.5 * invade_agent->width() -
    //     0.5 * ego_vehi_param.width;
    const double invade_agent_lat_distance_to_path =
        std::min(std::fabs(agent_min_l), std::fabs(agent_max_l)) -
        0.5 * ego_vehi_param.width;
    if (invade_agent_lat_distance_to_path >
        lat_distance_thrd) {  // far from ego in lat l
      LOG_DEBUG("agent (ID: %d) is far from ego in lat l(%fm), skip\n", id,
                invade_agent_lat_distance_to_path);
      continue;
    }
    if (agent->agent_decision().agent_decision_type() ==
        agent::AgentDecisionType::IGNORE) {
      LOG_DEBUG("agent (ID: %d) is ignored in agent decision, skip\n", id);
      continue;
    }
    // invade_agents_s_id_map[invade_agent_s] = id;
    invade_agents_s_id_map[agent_min_s] = id;
  }
  // no invade agents
  if (invade_agents_s_id_map.size() == 1) {
    return;
  }

  auto iterator_ego = invade_agents_s_id_map.find(planning_init_s);
  if (iterator_ego ==
      invade_agents_s_id_map.begin()) {  // only has upper invade agent
    closest_neighbor_invade_gap_agents_id_.second =
        std::next(iterator_ego)->second;
    closest_neighbor_invade_gap_agents_id_.first = -1;
  } else {
    if (std::next(iterator_ego) ==
        invade_agents_s_id_map.end()) {  // only has lower invade agent
      closest_neighbor_invade_gap_agents_id_.second = -1;
      closest_neighbor_invade_gap_agents_id_.first =
          std::prev(iterator_ego)->second;
    } else {  // both has upper and lower invade agents
      closest_neighbor_invade_gap_agents_id_.second =
          std::next(iterator_ego)->second;
      closest_neighbor_invade_gap_agents_id_.first =
          std::prev(iterator_ego)->second;
    }
  }
}

void LongitudinalDecisionDecider::UpdateLaneChangeNeighborResults() {
  const auto &environmental_model = session_->environmental_model();
  const auto &virtual_lane_manager =
      environmental_model.get_virtual_lane_manager();
  const auto &planning_context = session_->planning_context();
  const auto &lane_change_decider_output =
      planning_context.lane_change_decider_output();
  const auto &dynamic_world = environmental_model.get_dynamic_world();
  const auto &agent_manager = dynamic_world->agent_manager();
  const auto st_graph = planning_context.st_graph();
  const auto mutable_st_graph =
      session_->mutable_planning_context()->st_graph();
  const auto agent_id_st_boundaries_map =
      st_graph->agent_id_st_boundaries_map();
  const auto neighbor_agent_id_st_boundraies_map =
      st_graph->neighbor_agent_id_st_boundaries_map();
  const auto gap_front_node_id =
      lane_change_decider_output.lc_gap_info.front_node_id;
  const auto gap_rear_node_id =
      lane_change_decider_output.lc_gap_info.rear_node_id;
  int32_t gap_front_agent_id = -1;
  if (dynamic_world->GetNode(gap_front_node_id)) {
    gap_front_agent_id =
        dynamic_world->GetNode(gap_front_node_id)->node_agent_id();
  }
  int32_t gap_rear_agent_id = -1;
  if (dynamic_world->GetNode(gap_rear_node_id)) {
    gap_rear_agent_id =
        dynamic_world->GetNode(gap_rear_node_id)->node_agent_id();
  }

  const auto target_lane_id = lane_change_decider_output.target_lane_virtual_id;

  const auto lane_change_status = lane_change_decider_output.curr_state;
  JSON_DEBUG_VALUE("lane_change_status", static_cast<int>(lane_change_status))
  JSON_DEBUG_VALUE("gap_front_agent_id", gap_front_agent_id & 0xFFFF)
  JSON_DEBUG_VALUE("gap_rear_agent_id", gap_rear_agent_id & 0xFFFF)

  if (gap_front_agent_id == -1 && gap_rear_agent_id == -1) {
    LOG_DEBUG(
        "LongitudinalDecisionDecider::UpdateLaneChangeNeighborResults: No gap "
        "agents\n");
    int default_value = -1;
    JSON_DEBUG_VALUE("gap_lon_decision_update", default_value)
    JSON_DEBUG_VALUE("ignore_gap_rear_agent", default_value)
    return;
  }
  const auto gap_front_agent = agent_manager->GetAgent(gap_front_agent_id);
  const auto gap_rear_agent = agent_manager->GetAgent(gap_rear_agent_id);
  bool ignore_rear_agent = false;
  const auto target_lane_ptr =
      virtual_lane_manager->get_lane_with_virtual_id(target_lane_id);
  if (target_lane_ptr == nullptr) {
    LOG_DEBUG(
        "LongitudinalDecisionDecider::UpdateLaneChangeNeighborResults: No "
        "target lane\n");
    int default_value = -1;
    JSON_DEBUG_VALUE("gap_lon_decision_update", default_value)
    JSON_DEBUG_VALUE("ignore_gap_rear_agent", default_value)
    return;
  }
  const auto target_lane_frenet_coord =
      target_lane_ptr->get_lane_frenet_coord();
  const bool ignore_gap_rear_agent =
      IgnoreLaneChangeGapRearAgent(gap_rear_agent, target_lane_frenet_coord);
  JSON_DEBUG_VALUE("ignore_gap_rear_agent", ignore_gap_rear_agent)

  std::unordered_map<int32_t, speed::STBoundary::DecisionType>
      neighbor_agents_decision_table;
  if (lane_change_status ==
          StateMachineLaneChangeStatus::kLaneChangeExecution ||
      lane_change_status == StateMachineLaneChangeStatus::kLaneChangeComplete) {
    if (gap_front_agent_id != -1 &&
        neighbor_agent_id_st_boundraies_map.count(gap_front_agent_id) == 0) {
      ConstructNeighborLaneStGraph(gap_front_agent);
      neighbor_agents_decision_table[gap_front_agent_id] =
          speed::STBoundary::DecisionType::NEIGHBOR_YIELD;
    }
    if (gap_rear_agent_id != -1 &&
        neighbor_agent_id_st_boundraies_map.count(gap_rear_agent_id) == 0 &&
        !ignore_gap_rear_agent) {
      ConstructNeighborLaneStGraph(gap_rear_agent);
      neighbor_agents_decision_table[gap_rear_agent_id] =
          speed::STBoundary::DecisionType::NEIGHBOR_OVERTAKE;
    }
  }

  if (!neighbor_agents_decision_table.empty()) {
    mutable_st_graph->UpdateNeighborAgentResults(
        neighbor_agents_decision_table);
    LOG_DEBUG(
        "LongitudinalDecisionDecider::UpdateLaneChangeNeighborResults: Update "
        "neighbor agents decision table\n");
  }
  JSON_DEBUG_VALUE("gap_lon_decision_update",
                   !neighbor_agents_decision_table.empty())
}

// ignore rear agent if the ttc to ego is bigger than threshold
bool LongitudinalDecisionDecider::IgnoreLaneChangeGapRearAgent(
    const agent::Agent *gap_rear_agent,
    const std::shared_ptr<planning_math::KDPath> &target_lane_frenet_coord)
    const {
  if (gap_rear_agent == nullptr || target_lane_frenet_coord == nullptr) {
    int default_value = -1;
    JSON_DEBUG_VALUE("rear_agent_ttc_to_ego", default_value)
    return true;
  }
  const auto &planning_init_point = session_->environmental_model()
                                        .get_ego_state_manager()
                                        ->planning_init_point();
  const auto planning_init_x = planning_init_point.lat_init_state.x();
  const auto planning_init_y = planning_init_point.lat_init_state.y();
  const auto planning_init_theta = planning_init_point.lat_init_state.theta();
  const auto planning_init_v = planning_init_point.lon_init_state.v();
  const auto &ego_vehi_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();

  // agent info in target lane
  const auto &rear_agent_center = gap_rear_agent->box().center();
  Point2D rear_agent_center_xy(rear_agent_center.x(), rear_agent_center.y());
  Point2D rear_agent_center_sl;
  if (!target_lane_frenet_coord->XYToSL(rear_agent_center_xy,
                                        rear_agent_center_sl)) {
    int default_value = -1;
    JSON_DEBUG_VALUE("rear_agent_ttc_to_ego", default_value)
    return true;
  }
  const double rear_agent_center_s = rear_agent_center_sl.x;
  const double rear_agent_center_l = rear_agent_center_sl.y;
  const auto agent_center_matched_point =
      target_lane_frenet_coord->GetPathPointByS(rear_agent_center_s);
  const double agent_target_lane_heading_diff =
      gap_rear_agent->box().heading() - agent_center_matched_point.theta();
  const double agent_vel_frenet =
      gap_rear_agent->speed() * std::cos(agent_target_lane_heading_diff);

  // planning init info in target lane
  Point2D planning_init_xy(planning_init_x, planning_init_y);
  Point2D planning_init_sl;
  if (!target_lane_frenet_coord->XYToSL(planning_init_xy, planning_init_sl)) {
    int default_value = -1;
    JSON_DEBUG_VALUE("rear_agent_ttc_to_ego", default_value)
    return true;
  }
  double planning_init_s = planning_init_sl.x;
  double planning_init_l = planning_init_sl.y;
  const auto planning_init_matched_point =
      target_lane_frenet_coord->GetPathPointByS(planning_init_s);
  const double planning_init_target_lane_heading_diff =
      planning_init_theta - planning_init_matched_point.theta();
  const double planning_init_vel_frenet =
      planning_init_v * std::cos(planning_init_target_lane_heading_diff);

  // aproximate the distance between ego and rear agent on target lane
  const double agent_reletive_s_target_lane_frenet =
      planning_init_s - rear_agent_center_s -
      ego_vehi_param.rear_edge_to_rear_axle - 0.5 * gap_rear_agent->length();

  double vel_difference_to_ego = agent_vel_frenet - planning_init_vel_frenet;
  if (vel_difference_to_ego > -kEpsilon && vel_difference_to_ego < 0.0) {
    vel_difference_to_ego = -kEpsilon;
  } else if (vel_difference_to_ego < kEpsilon && vel_difference_to_ego > 0.0) {
    vel_difference_to_ego = kEpsilon;
  }
  double rear_agent_ttc_to_ego =
      agent_reletive_s_target_lane_frenet / vel_difference_to_ego;
  JSON_DEBUG_VALUE("rear_agent_ttc_to_ego", rear_agent_ttc_to_ego)
  if (rear_agent_ttc_to_ego < config_.ignore_agent_ttc_to_ego_thrd &&
      rear_agent_ttc_to_ego > 0.0) {
    return false;
  }

  return true;
}

bool LongitudinalDecisionDecider::ConstructNeighborLaneStGraph(
    const agent::Agent *const neighbor_agent) {
  const auto &mutable_planning_context = session_->mutable_planning_context();
  auto *mutable_st_graph = mutable_planning_context->st_graph();
  if (nullptr == neighbor_agent) {
    return false;
  }
  return mutable_st_graph->InsertAgent(*neighbor_agent,
                                       speed::StBoundaryType::NEIGHBOR);
}

void LongitudinalDecisionDecider::MakeDebugMessage() {
  // auto planning_debug_msg = planning_data->mutable_planning_debug_message();
  // if (planning_debug_msg == nullptr) {
  // }
  // auto* ptr_debug_string = planning_debug_msg->mutable_debug_string();
  // if (ptr_debug_string == nullptr) {
  //   return;
  // }

  // std::string cruise_acc_count_info = " DetermineKinematicBound: count ";
  // cruise_acc_count_info += std::to_string(cruise_accelerate_count_.first);
  // cruise_acc_count_info += " increase_acc ";
  // cruise_acc_count_info += std::to_string(cruise_accelerate_count_.second);
  // std::stringstream acc_bound;
  // acc_bound << std::fixed << std::setprecision(2) << " acc_bound "
  //           << planning_data->decision_output()
  //                  .longitudinal_decision_decider_output()
  //                  .determined_cruise_bound()
  //                  .acc_positive_mps2;
  // cruise_acc_count_info += acc_bound.str();
  // ptr_debug_string->append(cruise_acc_count_info);
}

}  // namespace planning

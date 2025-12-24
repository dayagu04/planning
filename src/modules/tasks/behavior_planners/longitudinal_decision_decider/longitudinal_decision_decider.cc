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
      nullptr == ptr_max_l || planned_path == nullptr) {
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
  ILOG_DEBUG << "=======LongitudinalDecisionDecider=======";
  const auto start_timestamp = IflyTime::Now_ms();
  if (!PreCheck()) {
    ILOG_DEBUG << "PreCheck failed";
    return false;
  }

  bool need_reset = false;  // binwang33 TBD: Add reset conditions
  if (need_reset) {
    Reset();
  }
  DetermineKinematicBoundForCruiseScenario();

  UpdateLaneChangeNeighborResults();

  UpdateParallelLongitudinalAvoidResults();

  const auto mutable_st_graph =
      session_->mutable_planning_context()->st_graph();

  const auto end_timestamp = IflyTime::Now_ms();
  ILOG_DEBUG << "LongitudinalDecisionDecider time cost:"
             << end_timestamp - start_timestamp;

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
  /* const auto lane_change_status =
      planning_context.lane_change_decider_output().curr_state;
  const bool is_in_lane_keeping =
      lane_change_status == StateMachineLaneChangeStatus::kLaneKeeping;
  if (!is_in_lane_keeping) {
    can_increase_acc_bound = false;
  } */

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
    return 0.0;
  }
  if (agent_manager == nullptr) {
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
  JSON_DEBUG_VALUE("gap_front_agent_id", gap_front_agent_id)
  JSON_DEBUG_VALUE("gap_rear_agent_id", gap_rear_agent_id)

  if (gap_front_agent_id == -1 && gap_rear_agent_id == -1) {
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

void LongitudinalDecisionDecider::UpdateParallelLongitudinalAvoidResults() {
  const auto &parallel_longitudinal_avoid_output =
      session_->planning_context().parallel_longitudinal_avoid_decider_output();

  bool is_parallel_longitudinal_avoid_active =
      parallel_longitudinal_avoid_output.is_need_parallel_longitudinal_avoid();

  const int32_t parallel_target_agent_id =
      parallel_longitudinal_avoid_output.parallel_target_agent_id();
  const bool is_parallel_overtake =
      parallel_longitudinal_avoid_output.is_parallel_overtake();
  const bool is_parallel_yield =
      parallel_longitudinal_avoid_output.is_parallel_yield();

  const bool is_lead_and_target_is_truck =
      parallel_longitudinal_avoid_output.is_lead_and_target_is_truck();

  const int32_t current_state =
      parallel_longitudinal_avoid_output.current_state();
  const int32_t running_frame_count =
      parallel_longitudinal_avoid_output.running_frame_count();
  const int32_t cooldown_frame_count =
      parallel_longitudinal_avoid_output.cooldown_frame_count();

  const double parallel_lateral_distance =
      parallel_longitudinal_avoid_output.lateral_distance();

  JSON_DEBUG_VALUE("parallel_longitudinal_avoid_active",
                   is_parallel_longitudinal_avoid_active)
  JSON_DEBUG_VALUE("parallel_target_agent_id", parallel_target_agent_id)
  JSON_DEBUG_VALUE("is_parallel_overtake", is_parallel_overtake)
  JSON_DEBUG_VALUE("is_parallel_yield", is_parallel_yield)
  JSON_DEBUG_VALUE("is_lead_and_target_is_truck", is_lead_and_target_is_truck)
  JSON_DEBUG_VALUE("parallel_decider_state", current_state)
  JSON_DEBUG_VALUE("parallel_running_frames", running_frame_count)
  JSON_DEBUG_VALUE("parallel_cooldown_frames", cooldown_frame_count)
  JSON_DEBUG_VALUE("parallel_lateral_distance", parallel_lateral_distance)

  auto *mutable_st_graph = session_->mutable_planning_context()->st_graph();
  std::unordered_map<int32_t, speed::STBoundary::DecisionType>
      neighbor_agents_decision_table;

  if (!is_parallel_longitudinal_avoid_active) {
    return;
  }

  if (is_parallel_overtake) {
    neighbor_agents_decision_table[parallel_target_agent_id] =
        speed::STBoundary::DecisionType::NEIGHBOR_OVERTAKE;
  } else if (is_parallel_yield || is_lead_and_target_is_truck) {
    neighbor_agents_decision_table[parallel_target_agent_id] =
        speed::STBoundary::DecisionType::NEIGHBOR_YIELD;
  }

  if (!neighbor_agents_decision_table.empty()) {
    mutable_st_graph->UpdateNeighborAgentResults(
        neighbor_agents_decision_table);
  }
}

}  // namespace planning

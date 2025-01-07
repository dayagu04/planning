#include "longitudinal_decision_decider.h"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <unordered_map>

#include "config/basic_type.h"
#include "debug_info_log.h"
#include "longitudinal_decision_decider_output.h"
#include "src/modules/context/environmental_model.h"
#include "src/modules/context/planning_context.h"
#include "st_graph/st_boundary.h"
#include "utils_math.h"

namespace planning {

LongitudinalDecisionDecider::LongitudinalDecisionDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  name_ = "LongitudinalDecisionDecider";
  // 读取配置文件
  config_ = config_builder->cast<EgoPlanningConfig>();
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

  // put agents that do not yield in lateral intrusion into the ST graph
  UpdateInvadeNeighborResults();

  UpdateLaneChangeNeighborResults();

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
  for (const auto *ptr_agent : agents) {
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
  for (const auto *ptr_agent : agents) {
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
  LOG_DEBUG(
      "=== LongitudinalDecisionDecider::UpdateInvadeNeighborResults ===\n");

  const auto &environmental_model = session_->environmental_model();
  const auto &mutable_planning_context = session_->mutable_planning_context();
  //  const auto &ego_state_mgr =
  // environmental_model.get_ego_state_manager();
  const auto &dynamic_world = environmental_model.get_dynamic_world();
  const auto &virtual_lane_manager =
      environmental_model.get_virtual_lane_manager();

  // 在路口中不启用
  planning::common::IntersectionState intersection_state =
      virtual_lane_manager->GetIntersectionState();
  if (intersection_state == planning::common::IN_INTERSECTION) {
    return;
  }
  if (dynamic_world == nullptr) {
    return;
  }
  const auto *agent_manger = dynamic_world->agent_manager();
  if (agent_manger == nullptr) {
    return;
  }

  auto *mutable_st_graph = mutable_planning_context->st_graph();
  if (mutable_st_graph == nullptr) {
    return;
  }

  // binwang33: 待横向障碍物决策开发，横向侵入但是无法nudge
  // const auto& lateral_invade_agents_info =
  // lateral_decision_data->LateralInvadeAgentInfo();
  std::unordered_map<int32_t, speed::STBoundary::DecisionType>
      neighbor_agents_decision_table;
  const auto agent_id_st_boundaries_map =
      mutable_st_graph->agent_id_st_boundaries_map();
  const auto neighbor_agent_id_st_boundraies_map =
      mutable_st_graph->neighbor_agent_id_st_boundaries_map();
  // for (const auto &invade_agent_info : lateral_invade_agents_info) {
  //   const auto invade_agent_id = invade_agent_info.agent_id;
  //   if (agent_id_st_boundaries_map.count(invade_agent_id) > 0) {
  //     continue;
  //   }
  //   bool is_not_in_neighbor_agent_id =
  //       neighbor_agent_id_st_boundraies_map.count(invade_agent_id) == 0;
  //   LOG_DEBUG("is_not_in_neighbor_agent_id =  [%d] \n",
  //             is_not_in_neighbor_agent_id);
  //   if (is_not_in_neighbor_agent_id) {
  //     const agent::Agent *invade_agent =
  //         agent_manger->GetAgent(invade_agent_id);
  //     const bool is_succeeded_construct_neighbor_lane_st_graph =
  //         ConstructNeighborLaneStGraph(invade_agent);
  //     LOG_DEBUG("tis_succeeded_construct_neighbor_lane_st_graph =  [%d] \n",
  //               is_succeeded_construct_neighbor_lane_st_graph);
  //     if (!is_succeeded_construct_neighbor_lane_st_graph) {
  //       continue;
  //     }
  //   }

  //   neighbor_agents_decision_table[invade_agent_id] =
  //       speed::STBoundary::DecisionType::NEIGHBOR_YIELD;
  //   LOG_DEBUG("invade agent id =  [%d] \n", invade_agent_id);
  // }

  if (!neighbor_agents_decision_table.empty()) {
    LOG_DEBUG("mutable_st_graph->UpdateNeighborAgentResults \n");
    // binwang33: 待ST接口合入
    // mutable_st_graph->UpdateNeighborAgentResults(
    //     neighbor_agents_decision_table);
  }
}

void LongitudinalDecisionDecider::UpdateLaneChangeNeighborResults() {
  const auto &environmental_model = session_->environmental_model();
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

  const auto lane_change_status = lane_change_decider_output.curr_state;
  if (lane_change_status !=
          StateMachineLaneChangeStatus::kLaneChangeExecution &&
      lane_change_status != StateMachineLaneChangeStatus::kLaneChangeComplete) {
    LOG_DEBUG(
        "LongitudinalDecisionDecider::UpdateLaneChangeNeighborResults: No "
        "LaneChangeExecution and No LaneChangeComplete\n");
    int default_value = -1;
    JSON_DEBUG_VALUE("gap_lon_decision_update", default_value)
    JSON_DEBUG_VALUE("gap_front_agent_id", default_value)
    JSON_DEBUG_VALUE("gap_rear_agent_id", default_value)
    return;
  }

  const int32_t gap_front_agent_id =
      lane_change_decider_output.gap_info.front_agent_id;
  const int32_t gap_rear_agent_id =
      lane_change_decider_output.gap_info.rear_agent_id;
  if (gap_front_agent_id == -1 && gap_rear_agent_id == -1) {
    LOG_DEBUG(
        "LongitudinalDecisionDecider::UpdateLaneChangeNeighborResults: No gap "
        "agents\n");
    int default_value = -1;
    JSON_DEBUG_VALUE("gap_lon_decision_update", default_value)
    JSON_DEBUG_VALUE("gap_front_agent_id", default_value)
    JSON_DEBUG_VALUE("gap_rear_agent_id", default_value)
    return;
  }
  const auto gap_front_agent = agent_manager->GetAgent(gap_front_agent_id);
  const auto gap_rear_agent = agent_manager->GetAgent(gap_rear_agent_id);
  std::unordered_map<int32_t, speed::STBoundary::DecisionType>
      neighbor_agents_decision_table;
  if (gap_front_agent_id != -1 &&
      neighbor_agent_id_st_boundraies_map.count(gap_front_agent_id) == 0 /*&&
      agent_id_st_boundaries_map.count(gap_front_agent_id) == 0*/) {
    ConstructNeighborLaneStGraph(gap_front_agent);
    neighbor_agents_decision_table[gap_front_agent_id] =
        speed::STBoundary::DecisionType::NEIGHBOR_YIELD;
  }
  if (gap_rear_agent_id != -1 &&
      neighbor_agent_id_st_boundraies_map.count(gap_rear_agent_id) == 0 /*&&
      agent_id_st_boundaries_map.count(gap_rear_agent_id) == 0*/) {
    ConstructNeighborLaneStGraph(gap_rear_agent);
    neighbor_agents_decision_table[gap_rear_agent_id] =
        speed::STBoundary::DecisionType::NEIGHBOR_OVERTAKE;
  }
  if (!neighbor_agents_decision_table.empty()) {
    mutable_st_graph->UpdateNeighborAgentResults(
        neighbor_agents_decision_table);
    LOG_DEBUG(
        "LongitudinalDecisionDecider::UpdateLaneChangeNeighborResults: Update "
        "neighbor agents decision table\n");
  }
  JSON_DEBUG_VALUE("gap_lon_decision_update",
                   neighbor_agents_decision_table.empty())
  JSON_DEBUG_VALUE("gap_front_agent_id", gap_front_agent_id & 0xFFFF)
  JSON_DEBUG_VALUE("gap_rear_agent_id", gap_rear_agent_id & 0xFFFF)
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

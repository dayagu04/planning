#include "st_graph_searcher.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

#include "common/st_graph/st_graph_utils.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "environmental_model.h"
#include "ifly_time.h"
#include "log.h"
#include "minheap.h"
#include "planning_context.h"
#include "session.h"
#include "st_search_node.h"
#include "trajectory/trajectory.h"
#include "trajectory/trajectory_point.h"

namespace planning {

namespace {

// constexpr double kMinBuffer = 1.0;
constexpr double kEpsilon = 1e-10;
constexpr double kUpsamplingStep = 0.1;
// constexpr double kMilliSecondToSecond = 0.001;
constexpr double kBestNodeMinT = 4.5;
constexpr double kTimeResolution = 0.2;
constexpr double kMaxNodeSpeedLimit = 100.0;

trajectory::TrajectoryPoint PlanningInitPointToTrajectoryPoint(
    const PlanningInitPoint& init_point) {
  trajectory::TrajectoryPoint planning_init_point;
  planning_init_point.set_x(init_point.x);
  planning_init_point.set_y(init_point.y);
  planning_init_point.set_theta(init_point.heading_angle);
  planning_init_point.set_vel(init_point.v);
  planning_init_point.set_acc(init_point.a);
  planning_init_point.set_s(init_point.frenet_state.s);
  planning_init_point.set_l(init_point.frenet_state.r);
  planning_init_point.set_dkappa(init_point.dkappa);
  planning_init_point.set_kappa(init_point.curvature);
  planning_init_point.set_absolute_time(0.0);
  // planning_init_point.set_relative_time(init_point.relative_time);
  return planning_init_point;
}

std::pair<std::unordered_set<int32_t>, std::unordered_set<int32_t>>
MakeTargetLaneFrontRearAgents(framework::Session* session) {
  const auto& dynamic_world =
      session->environmental_model().get_dynamic_world();
  // get lane change status
  const auto& lane_change_decider_output =
      session->planning_context().lane_change_decider_output();
  // const auto& coarse_planning_info =
  //     lane_change_decider_output.coarse_planning_info;
  const auto lc_request_direction = lane_change_decider_output.lc_request;
  const auto lane_change_state = lane_change_decider_output.curr_state;
  // const auto is_in_lane_change_execution =
  //     lane_change_state == kLaneChangeExecution;
  // const auto is_in_lane_change_complete =
  //     lane_change_state == kLaneChangeComplete;

  // get front and rear agents
  int64_t target_lane_front_node_id = -1;
  int64_t target_lane_rear_node_id = -1;
  std::unordered_set<int32_t> target_lane_front_agents;
  std::unordered_set<int32_t> target_lane_rear_agents;

  if (lc_request_direction == LEFT_CHANGE) {
    if (lane_change_state == kLaneChangeExecution ||
        lane_change_state == kLaneChangeComplete) {
      target_lane_front_node_id = dynamic_world->ego_front_node_id();
      target_lane_rear_node_id = dynamic_world->ego_rear_node_id();
    } else {
      target_lane_front_node_id = dynamic_world->ego_left_front_node_id();
      target_lane_rear_node_id = dynamic_world->ego_left_rear_node_id();
    }
  } else if (lc_request_direction == RIGHT_CHANGE) {
    if (lane_change_state == kLaneChangeExecution ||
        lane_change_state == kLaneChangeComplete) {
      target_lane_front_node_id = dynamic_world->ego_front_node_id();
      target_lane_rear_node_id = dynamic_world->ego_rear_node_id();
    } else {
      target_lane_front_node_id = dynamic_world->ego_right_front_node_id();
      target_lane_rear_node_id = dynamic_world->ego_right_rear_node_id();
    }
  }

  if (target_lane_front_node_id != -1) {
    auto* target_lane_front_node =
        dynamic_world->GetNode(target_lane_front_node_id);
    if (target_lane_front_node != nullptr) {
      target_lane_front_agents.insert(target_lane_front_node->node_agent_id());
    }
  }

  if (target_lane_rear_node_id != -1) {
    auto* target_lane_rear_node =
        dynamic_world->GetNode(target_lane_rear_node_id);
    if (target_lane_rear_node != nullptr) {
      target_lane_rear_agents.insert(target_lane_rear_node->node_agent_id());
    }
  }
  return std::make_pair(target_lane_front_agents, target_lane_rear_agents);
}

void UpdateHeuristicTargetSInLaneChange(framework::Session* session,
                                        const double heuristic_t,
                                        double* const heuristic_s) {
  const auto* st_graph_helper = session->planning_context().st_graph_helper();
  if (st_graph_helper == nullptr) {
    return;
  }
  const auto target_lane_front_rear_agents =
      MakeTargetLaneFrontRearAgents(session);
  const auto& target_lane_front_agents = target_lane_front_rear_agents.first;
  const auto& target_lane_rear_agents = target_lane_front_rear_agents.second;
  if (target_lane_front_agents.empty() && target_lane_rear_agents.empty()) {
    return;
  }
  double front_lower_s = std::numeric_limits<double>::max();
  double rear_upper_s = std::numeric_limits<double>::lowest();
  auto GetBoundGivenT =
      [&st_graph_helper](const std::unordered_set<int32_t>& target_lane_agents,
                         const double heuristic_t) {
        double lowest = std::numeric_limits<double>::max();
        double upmost = std::numeric_limits<double>::lowest();
        for (const auto& agent_id : target_lane_agents) {
          std::vector<int64_t> boundaries;
          if (!st_graph_helper->GetAgentStBoundaries(agent_id, &boundaries)) {
            continue;
          }
          if (boundaries.empty()) {
            continue;
          }
          for (const auto boundary_id : boundaries) {
            speed::STBoundary st_boundary;
            if (!st_graph_helper->GetStBoundary(boundary_id, &st_boundary)) {
              continue;
            }
            double s_lower = 0.0;
            double s_upper = 0.0;
            if (!st_boundary.GetBoundarySRange(heuristic_t, &s_lower,
                                               &s_upper)) {
              continue;
            }
            upmost = std::fmax(s_upper, upmost);
            lowest = std::fmin(s_lower, lowest);
          }
        }
        return std::make_pair(lowest, upmost);
      };

  double dummy_s = 0.0;
  constexpr double distance_threshold = 5.0;
  std::tie(front_lower_s, dummy_s) =
      GetBoundGivenT(target_lane_front_agents, heuristic_t);
  std::tie(dummy_s, rear_upper_s) =
      GetBoundGivenT(target_lane_rear_agents, heuristic_t);
  if ((front_lower_s == std::numeric_limits<double>::max()) &&
      (rear_upper_s == std::numeric_limits<double>::lowest())) {
    return;
  }
  if (front_lower_s == std::numeric_limits<double>::max()) {
    *heuristic_s = std::max(*heuristic_s, rear_upper_s + distance_threshold);
  } else if (rear_upper_s == std::numeric_limits<double>::lowest()) {
    *heuristic_s =
        std::max(front_lower_s * 0.8, front_lower_s - distance_threshold);
  } else {
    *heuristic_s = (rear_upper_s + front_lower_s) * 0.5;
  }
}

void UpdateHeuristicTargetSInLaneChange(framework::Session* session,
                                        double* const heuristic_s) {
  const auto& lane_change_decider_output =
      session->planning_context().lane_change_decider_output();
  const auto lane_change_state = lane_change_decider_output.curr_state;
  if (!(lane_change_state == kLaneChangeExecution ||
        lane_change_state == kLaneChangeCancel ||
        lane_change_state == kLaneChangeHold ||
        lane_change_state == kLaneChangeComplete)) {
    return;
  }
  const auto ego_trajs_future = lane_change_decider_output.ego_trajs_future;
  if (ego_trajs_future.empty()) {
    return;
  }
  *heuristic_s = ego_trajs_future.back().s;
}

bool HasCollisionRisk(const StSearchNode& current_node,
                      const StSearchNode& succ_node) {
  const auto& current_table = current_node.current_decision_table();
  const auto& succ_table = succ_node.current_decision_table();
  if (current_table.size() != succ_table.size()) {
    return true;
  }
  for (const auto& [key, _] : current_table) {
    if (succ_table.find(key) == succ_table.end()) {
      return true;
    }
  }
  return false;
}

bool CheckCollisionAfterUpsampling(
    const speed::StGraphHelper* st_graph,
    const std::unordered_set<int64_t>& target_lane_agent_boundaries,
    const double lower_collision_dist, const StSearchNode& current_node,
    const StSearchNode& succ_node) {
  if (st_graph == nullptr) {
    return false;
  }
  const double t_min = std::fmin(current_node.t(), succ_node.t());
  const double t_max = std::fmax(current_node.t(), succ_node.t());
  for (double t_intermediate = t_min + kUpsamplingStep;
       t_intermediate < t_max - kEpsilon; t_intermediate += kUpsamplingStep) {
    speed::STPoint upper_bound;
    speed::STPoint lower_bound;
    const double s_itermediate =
        planning_math::lerp(current_node.s(), current_node.t(), succ_node.s(),
                            succ_node.t(), t_intermediate);
    if (!st_graph->GetBorderByStPoint(s_itermediate, t_intermediate,
                                      &lower_bound, &upper_bound)) {
      return true;
    }
  }
  return false;
}
}  // namespace

StGraphSearcher::StGraphSearcher(const EgoPlanningConfigBuilder* config_builder,
                                 framework::Session* session)
    : Task(config_builder, session),
      config_(config_builder->cast<StGraphSearcherConfig>()),
      search_style_context_(
          {AStarSearchStyle::ORDINARY, AStarSearchStyle::RADICAL}) {
  name_ = "StGraphSearcher";
  if (yield_front_vehicle_safe_utils_ == nullptr) {
    yield_front_vehicle_safe_utils_ =
        std::make_unique<YieldFrontVehicleSafeFunction>(session, config_);
  }
}

bool StGraphSearcher::Execute() {
  ILOG_INFO << "=======StGraphSearcher=======";

  if (!PreCheck()) {
    ILOG_DEBUG << "PreCheck failed";
    return false;
  }

  auto* mutable_output =
      session_->mutable_planning_context()->mutable_st_graph_searcher_output();
  if (mutable_output == nullptr) {
    ILOG_ERROR << "mutable st_graph_searcher_output is null";
    return false;
  }

  mutable_output->set_debounce_params(
      config_.st_search_overtake_debounce_min_consecutive_frames,
      config_.st_search_overtake_debounce_min_hold_time_ms);

  std::vector<StSearchNode> st_path;

  const auto& last_output =
      session_->planning_context().st_graph_searcher_output();
  // Use debounce strategy state for decision switch penalty
  prev_is_overtake_front_vehicle_on_target_lane_ =
      last_output.is_search_overtake_front_vehicle();
  prev_is_yield_back_vehicle_ = last_output.is_search_yield_back_vehicle();
  has_prev_strategy_ =
      prev_is_overtake_front_vehicle_on_target_lane_ ||
      prev_is_yield_back_vehicle_;

  cached_stabilized_rear_agent_id_ = GetStabilizedTargetLaneRearAgentId();

  // search success
  bool search_success = false;
  for (auto search_style : search_style_context_) {
    search_success = SearchStPath(&st_path, search_style);
    JSON_DEBUG_VALUE("search_style", static_cast<int32_t>(search_style))
    if (search_success) {
      break;
    }
  }

  // search fail
  if (!search_success) {
    SetSearchFailSafe();
    mutable_output->set_is_search_success(false);
    AddStGraphSearcherDataToProto(st_path);
    return true;
  }

  // update output to st graph
  const auto& last_node = st_path.back();
  session_->mutable_planning_context()
      ->st_graph()
      ->UpdateStBoundaryDecisionResults(last_node.decision_table());
  mutable_output->set_is_search_success(true);

  // store st path in proto
  st_graph_searcher_pb_.Clear();
  AddStGraphSearcherDataToProto(st_path);

  // Based on the decision of the last node, update lane change decision
  bool is_yield_back_vehicle =
      CheckYieldBackVehicle(last_node.decision_table());
  const bool is_overtake_front_vehicle_on_target_lane =
      CheckOvertakeFrontVehicleOnTargetLane(last_node.decision_table());
  // const auto traffic_light_decision_map =
  //     MakeDecisionForTrafficLightVirtualAgent(last_node);
  const bool is_yield_front_vehicle_safe = CheckIfFrontVehcileSafe();

  mutable_output->set_is_yield_front_vehicle_safe(is_yield_front_vehicle_safe);
  mutable_output->set_search_yield_back_vehicle(is_yield_back_vehicle);
  mutable_output->set_is_search_overtake_front_vehicle(
      is_overtake_front_vehicle_on_target_lane);
  // mutable_output->set_traffic_light_decision_map(traffic_light_decision_map);

  return true;
}

bool StGraphSearcher::SearchStPath(
    std::vector<StSearchNode>* const searched_path,
    AStarSearchStyle search_style) {
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto& planning_init_point_old =
      ego_state_manager->planning_init_point();
  const auto& planning_init_point =
      PlanningInitPointToTrajectoryPoint(planning_init_point_old);

  const auto& planned_kd_path =
      session_->planning_context().motion_planner_output().lateral_path_coord;
  const auto v_cruise = ego_state_manager->ego_v_cruise();
  const auto target_lane_rear_agent_st_boundaries =
      GetTargetLaneRearAgentStBoundaries();

  SetSearchConfigBySearchStyle(search_style);

  double planning_distance = planned_kd_path->Length();
  UpdateHeuristicTargetSInLaneChange(session_, &planning_distance);
  StSearchInput st_search_input_info(
      planning_init_point, planning_distance,
      search_config_.planning_time_horizon, v_cruise,
      search_config_.max_accel_limit, search_config_.min_accel_limit,
      search_config_.max_jerk_limit, search_config_.min_jerk_limit,
      search_config_.accel_sample_num, search_config_.s_step,
      search_config_.t_step, search_config_.vel_step);
  std::unordered_map<int64_t, StSearchNode> nodes;
  MinHeap<int64_t, double> open_set;
  std::unordered_set<int64_t> close_set;

  auto start_node =
      GenerateStartNode(planning_init_point, st_search_input_info);

  if (!start_node.is_valid()) {
    ILOG_ERROR << "st search, start node is not valid";
    return false;
  }
  nodes.insert(std::make_pair(start_node.id(), start_node));
  open_set.Push(start_node.id(), start_node.TotalCost());
  bool is_goal_reached = false;

  // record time
  const double start_time = IflyTime::Now_ms();
  const double max_search_time_ms = search_config_.max_search_time * 1e3;
  int count = 0;
  auto current_node = start_node;
  auto best_node = current_node;
  best_node.set_h_cost(std::numeric_limits<double>::max());
  farthest_node_ = start_node;
  farthest_node_.set_h_cost(std::numeric_limits<double>::max());
  // debug info
  std::vector<double> expanded_nodes_s_vec{};
  std::vector<double> expanded_nodes_t_vec{};
  std::vector<double> history_cur_nodes_s_vec{};
  std::vector<double> history_cur_nodes_t_vec{};
  // start A* search loop
  while (!open_set.IsEmpty()) {
    const double current_time = IflyTime::Now_ms();
    const double time_used = current_time - start_time;
    if (time_used > max_search_time_ms) {
      ILOG_DEBUG << "time out, time used:" << time_used;
      break;
    }

    count++;

    current_node = nodes.at(open_set.Top().first);
    history_cur_nodes_s_vec.emplace_back(current_node.s());
    history_cur_nodes_t_vec.emplace_back(current_node.t());
    // update best node
    if (current_node.t() > kBestNodeMinT - kEpsilon &&
        current_node.h_cost() < best_node.h_cost()) {
      best_node = current_node;
    }

    if (current_node.t() > farthest_node_.t()) {
      farthest_node_ = current_node;
    }

    // pop current node and store in close set
    open_set.Pop();
    close_set.insert(current_node.id());
    if (IsReachGoal(st_search_input_info, current_node)) {
      is_goal_reached = true;
      break;
    }

    // generate success nodes
    GenerateSuccessorNodes(st_search_input_info, current_node,
                           target_lane_rear_agent_st_boundaries);

    for (auto& child_node : successor_nodes_) {
      if (close_set.find(child_node.id()) != close_set.end()) {
        continue;
      }
      const double child_total_g_cost =
          current_node.g_cost() + child_node.cost();

      expanded_nodes_s_vec.emplace_back(child_node.s());
      expanded_nodes_t_vec.emplace_back(child_node.t());

      if (open_set.IsInHeap(child_node.id())) {
        auto& old_child = nodes.at(child_node.id());
        if (child_total_g_cost < old_child.g_cost()) {
          child_node.set_g_cost(child_total_g_cost);
          child_node.set_parent_id(current_node.id());
          old_child = child_node;
          open_set.Update(old_child.id(), old_child.TotalCost());
        }
      } else {
        child_node.set_g_cost(child_total_g_cost);
        child_node.set_parent_id(current_node.id());
        nodes[child_node.id()] = child_node;
        open_set.Push(child_node.id(), child_node.TotalCost());
      }
    }
  }
  JSON_DEBUG_VECTOR("expanded_nodes_s_vec", expanded_nodes_s_vec, 3)
  JSON_DEBUG_VECTOR("expanded_nodes_t_vec", expanded_nodes_t_vec, 3)
  JSON_DEBUG_VECTOR("history_cur_nodes_s_vec", history_cur_nodes_s_vec, 3)
  JSON_DEBUG_VECTOR("history_cur_nodes_t_vec", history_cur_nodes_t_vec, 3)
  JSON_DEBUG_VALUE("search_succeed", is_goal_reached)
  JSON_DEBUG_VALUE("expanded_nodes_size", expanded_nodes_s_vec.size())
  JSON_DEBUG_VALUE("history_cur_nodes_size", history_cur_nodes_s_vec.size())
  JSON_DEBUG_VALUE("open_set_empty", open_set.IsEmpty())
  // const double end_time = IflyTime::Now_ms();
  if (!is_goal_reached) {
    ILOG_DEBUG << "st search fail, goal not reached";
    bool is_visualize_all_vertexes = config_.is_visualize_st_search_process;
    if (is_visualize_all_vertexes) {
      // VisualizeStSearchVertexes(nodes);
    }
    if (best_node.h_cost() == std::numeric_limits<double>::max()) {
      ILOG_DEBUG << "st search fail, goal not reached";
      return false;
    } else {
      current_node = best_node;
      ILOG_DEBUG << "st search fail, but use best node to reconstruct path";
    }
  }

  // get the searched path by back trace from current_node using the parent_id
  searched_path->emplace_back(current_node);
  while (current_node.parent_id() != -1) {
    auto parent_id = current_node.parent_id();
    auto parent_entry = nodes.find(parent_id);
    if (parent_entry == nodes.end()) {
      // std::cout << "find parent node failed, parent_id: " << parent_id <<
      // std::endl;
      break;
    }
    current_node = parent_entry->second;
    searched_path->emplace_back(current_node);
  }
  std::reverse(searched_path->begin(), searched_path->end());

  AddAStarSearchCostDebugInfo(searched_path);
  if (searched_path->size() < 2) {
    ILOG_DEBUG << "st_search path size < 2";
    return false;
  }

  // add visualaization for all the generated nodes
  bool is_visualize_all_vertexes = config_.is_visualize_st_search_process;
  if (is_visualize_all_vertexes) {
    // VisualizeStSearchVertexes(nodes);
  }
  // std::cout << "print result st_path: " << std::endl;
  // for (size_t i = 0; i < searched_path->size(); ++i) {
  //   std::cout << "\tid: " << (*searched_path)[i].id() << "\ts: " <<
  //   (*searched_path)[i].s()
  //             << " \tt: " << (*searched_path)[i].t() << " \tvel: " <<
  //             (*searched_path)[i].vel()
  //             << " \ta: " << (*searched_path)[i].accel() << " \tj: " <<
  //             (*searched_path)[i].jerk()
  //             << std::endl;
  // }
  return true;
}

void StGraphSearcher::SetSearchConfigBySearchStyle(
    AStarSearchStyle search_style) {
  search_config_.planning_time_horizon = config_.planning_time_horizon;
  search_config_.max_search_time = config_.max_search_time_s;
  if (search_style == AStarSearchStyle::ORDINARY) {
    search_config_.max_accel_limit = config_.max_accel_limit;
    search_config_.min_accel_limit = config_.min_accel_limit;
    search_config_.max_jerk_limit = config_.max_jerk_limit;
    search_config_.min_jerk_limit = config_.min_jerk_limit;
    search_config_.accel_sample_num = config_.accel_sample_num;
    search_config_.s_step = config_.s_step;
    search_config_.t_step = config_.t_step;
    search_config_.vel_step = config_.vel_step;
  } else if (search_style == AStarSearchStyle::RADICAL) {
    search_config_.max_accel_limit = config_.max_accel_limit_radical_style;
    search_config_.min_accel_limit = config_.min_accel_limit_radical_style;
    search_config_.max_jerk_limit = config_.max_jerk_limit_radical_style;
    search_config_.min_jerk_limit = config_.min_jerk_limit_radical_style;
    search_config_.accel_sample_num = config_.accel_sample_num_radical_style;
    search_config_.s_step = config_.s_step_radical_style;
    search_config_.t_step = config_.t_step_radical_style;
    search_config_.vel_step = config_.vel_step_radical_style;
  }
}

std::unordered_set<int64_t>
StGraphSearcher::GetTargetLaneRearAgentStBoundaries() const {
  std::unordered_set<int64_t> st_boundaries_set;
  // const auto& dynamic_world =
  //     session_->environmental_model().get_dynamic_world();
  // get lane change status
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  // const auto& coarse_planning_info =
  //     lane_change_decider_output.coarse_planning_info;
  // const auto lc_request_direction = lane_change_decider_output.lc_request;
  const auto lane_change_state = lane_change_decider_output.curr_state;
  const auto is_in_lane_change_execution =
      lane_change_state == kLaneChangeExecution ||
      lane_change_state == kLaneChangeComplete;
  if (!is_in_lane_change_execution) {
    return st_boundaries_set;
  }

  // get rear agents
  // int64_t target_lane_rear_node_id = -1;
  std::unordered_set<int32_t> target_lane_rear_agents;
  const auto target_lane_front_rear_agents =
      MakeTargetLaneFrontRearAgents(session_);
  target_lane_rear_agents = target_lane_front_rear_agents.second;

  if (target_lane_rear_agents.empty()) {
    return st_boundaries_set;
  }
  const auto* st_graph_helper = session_->planning_context().st_graph_helper();
  if (st_graph_helper == nullptr) {
    return st_boundaries_set;
  }
  for (const auto& agent_id : target_lane_rear_agents) {
    std::vector<int64_t> boundaries;
    if (!st_graph_helper->GetAgentStBoundaries(agent_id, &boundaries)) {
      continue;
    }
    if (boundaries.empty()) {
      continue;
    }
    for (const auto boundary_id : boundaries) {
      st_boundaries_set.insert(boundary_id);
    }
  }
  return st_boundaries_set;
}

StSearchNode StGraphSearcher::GenerateStartNode(
    const trajectory::TrajectoryPoint planning_init_point,
    const StSearchInput& input_info) const {
  double s_step = input_info.s_step();
  double t_step = input_info.t_step();
  double vel_step = input_info.vel_step();

  StSearchNode start_node(input_info.init_s(), input_info.init_t(),
                          input_info.init_vel(), s_step, t_step, vel_step);
  start_node.set_accel(planning_init_point.acc());
  start_node.set_jerk(planning_init_point.jerk());

  // set decision table according to ST graph input
  speed::STPoint upper_bound;
  speed::STPoint lower_bound;
  const bool is_collision_free =
      session_->planning_context().st_graph_helper()->GetBorderByStPoint(
          input_info.init_s(), input_info.init_t(), &lower_bound, &upper_bound);

  std::unordered_map<int64_t, speed::STBoundary::DecisionType>
      start_decision_table;
  if (upper_bound.boundary_id() != speed::kNoAgentId) {
    start_decision_table.insert(std::make_pair(
        upper_bound.boundary_id(), speed::STBoundary::DecisionType::YIELD));
  }
  if (lower_bound.boundary_id() != speed::kNoAgentId) {
    start_decision_table.insert(std::make_pair(
        lower_bound.boundary_id(), speed::STBoundary::DecisionType::OVERTAKE));
  }

  start_node.set_upper_bound(upper_bound);
  start_node.set_lower_bound(lower_bound);
  start_node.set_decision_table(start_decision_table);
  start_node.set_current_decision_table(start_decision_table);
  start_node.set_is_valid(is_collision_free);
  return start_node;
}

bool StGraphSearcher::IsReachGoal(const StSearchInput& input_info,
                                  const StSearchNode& node) const {
  // A* end condition:
  // 1. search s reached target distance
  // 2. search t reached max time
  if (node.s() > input_info.planning_distance()) {
    return true;
  }
  const double time_tolerance = 0.1;
  if (std::fabs(node.t() - input_info.planning_time_horizon()) <
      time_tolerance) {
    return true;
  }
  return false;
}

void StGraphSearcher::GenerateSuccessorNodes(
    const StSearchInput& input_info, const StSearchNode& current_node,
    const std::unordered_set<int64_t>& target_lane_agent_boundaries) {
  successor_nodes_.clear();
  const double speed_limit_scale = config_.speed_limit_scale;
  const double min_lower_collision_dist = config_.min_lower_collision_dist;
  const double max_lower_collision_dist = config_.max_lower_collision_dist;
  const double speed_scale = config_.lower_collision_dist_speed_scale;
  const auto& st_graph_helper = session_->planning_context().st_graph_helper();
  successor_nodes_.reserve(input_info.accel_step().size());
  for (size_t i = 0; i < input_info.accel_step().size(); ++i) {
    const double accel_succ = input_info.accel_step().at(i);
    // ignore nodes with unreasonable jerk
    const double jerk_succ =
        (input_info.accel_step().at(i) - current_node.accel()) *
        input_info.t_step_inverse();
    if (jerk_succ > input_info.max_jerk_limit() ||
        jerk_succ < input_info.min_jerk_limit()) {
      continue;
    }
    // ignore nodes with unreasonable vel
    const double t_succ = current_node.t() + input_info.t_step();
    const double vel_succ =
        current_node.vel() + accel_succ * input_info.t_step();
    const double s_succ = current_node.s() +
                          current_node.vel() * input_info.t_step() +
                          0.5 * accel_succ * input_info.t_step_square();
    if (vel_succ > input_info.speed_limit() * speed_limit_scale ||
        vel_succ < 0.0) {
      continue;
    }

    if (s_succ < current_node.s()) {
      continue;
    }

    // check collision with ST curve
    speed::STPoint upper_bound;
    speed::STPoint lower_bound;
    bool is_collision_free_on_st_graph = st_graph_helper->GetBorderByStPoint(
        s_succ, t_succ, &lower_bound, &upper_bound);
    if (!is_collision_free_on_st_graph) {
      continue;
    }

    double lower_collision_dist = lower_bound.velocity() * speed_scale;
    lower_collision_dist =
        std::min(std::max(lower_collision_dist, min_lower_collision_dist),
                 max_lower_collision_dist);

    // get decision result for current (s, t) point
    std::unordered_map<int64_t, speed::STBoundary::DecisionType>
        succ_decision_table;
    if (upper_bound.boundary_id() != speed::kNoAgentId) {
      succ_decision_table.insert(std::make_pair(
          upper_bound.boundary_id(), speed::STBoundary::DecisionType::YIELD));
    }
    if (lower_bound.boundary_id() != speed::kNoAgentId) {
      succ_decision_table.insert(
          std::make_pair(lower_bound.boundary_id(),
                         speed::STBoundary::DecisionType::OVERTAKE));
    }
    const auto succ_current_decision_table = succ_decision_table;
    // check decision with parent node
    const auto& parent_decision_table = current_node.decision_table();
    bool is_decision_conflict = false;
    if (succ_decision_table.empty()) {
      succ_decision_table = parent_decision_table;
    } else {
      for (const auto& entry : parent_decision_table) {
        const auto boundary_id = entry.first;
        const auto parent_decision = entry.second;
        auto succ_iter = succ_decision_table.find(boundary_id);
        if (succ_iter == succ_decision_table.end()) {
          continue;
        }
        if (parent_decision != succ_iter->second) {
          is_decision_conflict = true;
          break;
        }
      }

      if (!is_decision_conflict && upper_bound.boundary_id() != speed::kNoAgentId &&
          upper_bound.agent_id() != speed::kNoAgentId) {
        const int32_t yield_agent_id = upper_bound.agent_id();
        for (const auto& entry : parent_decision_table) {
          if (entry.second == speed::STBoundary::DecisionType::OVERTAKE &&
              static_cast<int32_t>(entry.first >> 8) == yield_agent_id) {
            is_decision_conflict = true;
            break;
          }
        }
      }

      if (!is_decision_conflict) {
        for (const auto& entry : parent_decision_table) {
          if (succ_decision_table.find(entry.first) == succ_decision_table.end()) {
            succ_decision_table[entry.first] = entry.second;
          }
        }
      }
    }
    if (is_decision_conflict) {
      // succ node decision conflict with parent node, means collision with
      // st_boundary, node not valid
      continue;
    }

    // generate succ node
    StSearchNode succ_node = StSearchNode(
        s_succ, t_succ, vel_succ, input_info.s_step(), input_info.t_step(),
        input_info.vel_step(), config_.enable_only_s_t_hash);

    succ_node.set_parent_id(current_node.id());
    succ_node.set_upper_bound(upper_bound);
    succ_node.set_lower_bound(lower_bound);
    succ_node.set_decision_table(succ_decision_table);
    succ_node.set_current_decision_table(succ_current_decision_table);
    succ_node.set_accel(accel_succ);
    succ_node.set_jerk(jerk_succ);
    if (HasCollisionRisk(current_node, succ_node) &&
        CheckCollisionAfterUpsampling(
            st_graph_helper, target_lane_agent_boundaries, lower_collision_dist,
            current_node, succ_node)) {
      continue;
    }

    // compute cost
    ComputeNodeCost(input_info, current_node, &succ_node);
    successor_nodes_.emplace_back(std::move(succ_node));
  }
}

void StGraphSearcher::ComputeNodeCost(const StSearchInput& input_info,
                                      const StSearchNode& current_node,
                                      StSearchNode* const succ_node) {
  // all weight should be tuned
  const double weight_yield = config_.weight_yield;
  const double weight_overtake = config_.weight_overtake;
  const double weight_vel = config_.weight_vel;
  const double weight_accel = config_.weight_accel;
  const double weight_accel_sign = config_.weight_accel_sign;
  const double weight_accel_sign_overtake = config_.weight_accel_sign_overtake;
  const double weight_jerk = config_.weight_jerk;
  // const double weight_virtual_yield = config_.weight_virtual_yield;

  double cost_yield = ComputeYieldCost(input_info, *succ_node);

  double cost_overtake = ComputeOvertakeCost(input_info, *succ_node);

  double cost_vel = ComputeVelocityCost(input_info, *succ_node);

  double cost_accel =
      ComputeAccelerationCost(input_info, current_node, *succ_node);

  double cost_accel_sign_changed =
      ComputeAccelerationSignCost(input_info, current_node, *succ_node);

  double cost_jerk = ComputeJerkCost(input_info, *succ_node);

  double cost_length = ComputeLengthCost(input_info, current_node, *succ_node);

  // double cost_virtual_yield = ComputeVirtualYieldCost(input_info,
  // *succ_node);

  double weighted_cost_accel_sign_changed =
      cost_overtake == config_.cost_ego_overtake_has_collision_with_lower_bound
          ? cost_accel_sign_changed * weight_accel_sign_overtake
          : cost_accel_sign_changed * weight_accel_sign;
  // decision switch penalty
  double cost_decision_switch = 0.0;
  if (has_prev_strategy_) {
    const auto& decision_table = succ_node->decision_table();
    const bool curr_is_yield_back = CheckYieldBackVehicle(decision_table);
    const bool curr_is_overtake_front =
        CheckOvertakeFrontVehicleOnTargetLane(decision_table);
    if (curr_is_yield_back != prev_is_yield_back_vehicle_) {
      cost_decision_switch += config_.decision_switch_penalty;
    }
    if (curr_is_overtake_front !=
        prev_is_overtake_front_vehicle_on_target_lane_) {
      cost_decision_switch += config_.decision_switch_penalty;
    }
  }

  double edge_cost =
      cost_yield * weight_yield + cost_overtake * weight_overtake +
      cost_vel * weight_vel + cost_accel * weight_accel +
      weighted_cost_accel_sign_changed + cost_jerk * weight_jerk +
      cost_length + cost_decision_switch;
  StSearchNode::EdgeSubCost edge_sub_cost{
      .fathernode_to_childnode_cost_yield = cost_yield * weight_yield,
      .fathernode_to_childnode_edge_cost_overtake =
          cost_overtake * weight_overtake,
      .fathernode_to_childnode_edge_cost_vel = cost_vel * weight_vel,
      .fathernode_to_childnode_edge_cost_accel = cost_accel * weight_accel,
      .fathernode_to_childnode_edge_cost_accel_sign_changed =
          weighted_cost_accel_sign_changed,
      .fathernode_to_childnode_edge_cost_jerk = cost_jerk * weight_jerk,
      .fathernode_to_childnode_edge_cost_length = cost_length};

  succ_node->set_edge_sub_cost(edge_sub_cost);
  succ_node->NodeSubCostAccumulate(current_node);
  succ_node->set_cost(edge_cost);

  // print cost result
  // std::cout << "\tcompute node id: " << succ_node->id() << std::endl;
  // std::cout << "\tcompute node cost: " << std::endl;
  // std::cout << "\t\tcost_yield:      " << cost_yield << std::endl;
  // std::cout << "\t\tcost_overtake:   " << cost_overtake << std::endl;
  // std::cout << "\t\tcost_vel:        " << cost_vel << std::endl;
  // std::cout << "\t\tcost_accel:      " << cost_accel << std::endl;
  // std::cout << "\t\tcost_accel_sign: " << cost_accel_sign_changed <<
  // std::endl; std::cout << "\t\tcost_jerk:       " << cost_jerk << std::endl;
  // std::cout << "\t\tcost_length:     " << cost_length << std::endl;
  // std::cout << "\t\tcost_virtual_yield:     " << std::endl;
  // std::cout << "\t\tedge_cost:       " << edge_cost << std::endl;

  double cost_h = ComputeHeuristicCost(input_info, *succ_node);
  succ_node->set_h_cost(cost_h);

  // std::cout << "\t\t\t\tparent_g_cost:   " << current_node.g_cost() <<
  // std::endl; std::cout << "\t\t\t\ttol_cost:        " <<
  // current_node.g_cost() + edge_cost + cost_h
  //           << std::endl;
}

double StGraphSearcher::ComputeYieldCost(const StSearchInput& input_info,
                                         const StSearchNode& node) const {
  const auto& upper_bound = node.upper_bound();
  if (upper_bound.boundary_id() == speed::kNoAgentId) {
    return 0.0;
  }

  const double upper_truncation_time_buffer =
      config_.upper_truncation_time_buffer;
  const double min_upper_distance_buffer = config_.min_upper_distance_buffer;
  const double upper_truncation_distance = std::max(
      node.vel() * upper_truncation_time_buffer, min_upper_distance_buffer);

  const double distance_to_front = upper_bound.s() - node.s();
  if (distance_to_front < 0.0) {
    return 0.0;
  }

  double base_cost = 0.0;
  if (distance_to_front < upper_truncation_distance) {
    base_cost = (1.0 - (distance_to_front / upper_truncation_distance));
  } else {
    return 0.0;
  }

  const int32_t agent_id = upper_bound.agent_id();
  if (agent_id == speed::kNoAgentId) {
    return base_cost;
  }

  if (IsSpecialYieldAgent(agent_id)) {
    return base_cost * 0.1;
  }

  return base_cost;
}

double StGraphSearcher::ComputeVirtualYieldCost(
    const StSearchInput& input_info, const StSearchNode& node) const {
  double virtual_yield_cost = 0.0;
  double cur_limit_s_upper = -100.0;
  double cur_limit_s_lower = -100.0;
  constexpr double s_buffer_tiny = 0.5;

  constexpr double kTimeBuffer = 0.5;
  constexpr double kMinBuffer = 10.0;
  const double upper_truncation_distance =
      std::max(node.vel() * kTimeBuffer, kMinBuffer);

  // virtual agent?
  // const auto& extend_st_boundary =
  //     planning_data.decision_output().expand_st_boundaries_decider_output();

  speed::STPoint lower_bound, upper_bound;
  // const bool has_no_collision = false;
  // const bool has_no_collision =
  //     extend_st_boundary.GetBorderByStPoint(node.s(), node.t(), &lower_bound,
  //     &upper_bound);
  // std::cout << "\t\t\t\thas_no_collision: " << has_no_collision
  //           << " virtual upper, bound_id: " << upper_bound.boundary_id()
  //           << " agent_id: " << upper_bound.agent_id() << " s: " <<
  //           upper_bound.s()
  //           << " lower, bound_id: " << lower_bound.boundary_id()
  //           << " agent_id: " << lower_bound.agent_id() << " s: " <<
  //           lower_bound.s() << std::endl;

  if (lower_bound.boundary_id() == speed::kNoAgentId &&
      upper_bound.boundary_id() == speed::kNoAgentId) {
    return 0.0;
  }
  if (lower_bound.boundary_id() == upper_bound.boundary_id()) {
    const double s_buffer =
        std::min(upper_bound.velocity() * kTimeBuffer, 10.0);
    cur_limit_s_upper = upper_bound.s() + s_buffer;
    cur_limit_s_lower = lower_bound.s() - s_buffer_tiny;
  } else if (lower_bound.boundary_id() == speed::kNoAgentId) {
    return 0.0;
  } else {
    speed::STPoint tmp_lower_bound, tmp_upper_bound;
    // extend_st_boundary.GetBorderByStPoint(lower_bound.s() - s_buffer_tiny,
    // node.t(),
    //                                       &tmp_lower_bound,
    //                                       &tmp_upper_bound);
    const double s_buffer =
        std::min(tmp_upper_bound.velocity() * kTimeBuffer, 10.0);
    cur_limit_s_upper = tmp_upper_bound.s() + s_buffer;
    cur_limit_s_lower = tmp_lower_bound.s() - s_buffer_tiny;
  }
  if (node.s() > cur_limit_s_upper || node.s() < cur_limit_s_lower) {
    return 0.0;
  }

  const double distance_to_front = cur_limit_s_upper - node.s();
  return 1.0 - distance_to_front / (cur_limit_s_upper - cur_limit_s_lower);
}

double StGraphSearcher::ComputeOvertakeCost(const StSearchInput& input_info,
                                            const StSearchNode& node) const {
  const auto& lower_bound = node.lower_bound();
  if (lower_bound.boundary_id() == speed::kNoAgentId) {
    return 0.0;
  }
  const double lower_truncation_time_buffer =
      config_.lower_truncation_time_buffer;
  const double min_lower_distance_buffer = config_.min_lower_distance_buffer;
  const double lower_truncation_distance = std::max(
      node.vel() * lower_truncation_time_buffer, min_lower_distance_buffer);
  auto cur_lane_change_state =
      session_->planning_context().lane_change_decider_output().curr_state;

  const double distance_to_rear = node.s() - lower_bound.s();
  if (distance_to_rear < 0.0) {
    return 0.0;
  }

  // TODO: need consider lane change hold status in the future
  if (node.t() <= config_.cutin_time_st_graph_threshold &&
      (cur_lane_change_state !=
           StateMachineLaneChangeStatus::kLaneChangeComplete &&
       cur_lane_change_state !=
           StateMachineLaneChangeStatus::kLaneChangeExecution) &&
      distance_to_rear <
          vehicle_param_.length +
              config_.distance_ego_rear_edge_to_lower_bound_when_overtake) {
    return config_.cost_ego_overtake_has_collision_with_lower_bound;
  }

  double base_cost = 0.0;
  if (distance_to_rear < lower_truncation_distance) {
    base_cost = (1.0 - (distance_to_rear / lower_truncation_distance));
  } else {
    return 0.0;
  }

  const int32_t agent_id = lower_bound.agent_id();
  if (agent_id == speed::kNoAgentId) {
    return base_cost;
  }

  if (IsSpecialOvertakeAgent(agent_id)) {
    return base_cost * 0.1;
  }

  return base_cost;
}

double StGraphSearcher::ComputeVelocityCost(const StSearchInput& input_info,
                                            const StSearchNode& node) const {
  const double speed = node.vel();
  const double speed_limit = input_info.speed_limit();
  const double cruise_speed = input_info.cruise_speed();

  double cost = 0.0;
  const double det_speed = (speed - speed_limit) / std::max(speed_limit, kEpsilon);
  if (det_speed > 0.0) {
    cost += config_.exceed_speed_penalty * det_speed * det_speed;
  }
  else if (det_speed < 0.0) {
    cost += config_.low_speed_penalty * (-det_speed);
  }

  const double diff_speed =
      (speed - cruise_speed) / std::max(cruise_speed, kEpsilon);
  cost += config_.reference_speed_penalty * std::fabs(diff_speed);

  return cost;
}

double StGraphSearcher::ComputeAccelerationCost(
    const StSearchInput& input_info, const StSearchNode& current_node,
    const StSearchNode& node) const {
  const double accel = node.accel();
  const double max_acc = input_info.max_accel_limit();
  const double min_acc = input_info.min_accel_limit();

  const double accel_sq = accel * accel;
  double cost = 0.0;

  constexpr double accel_penalty = 0.5;
  constexpr double decel_penalty = 1.0;

  if (accel > 0.0) {
    cost = accel_penalty * accel_sq;
  } else {
    cost = decel_penalty * accel_sq;
  }

  cost += accel_sq * decel_penalty * decel_penalty /
              (1.0 + std::exp(1.0 * (accel - min_acc))) +
          accel_sq * accel_penalty * accel_penalty /
              (1.0 + std::exp(-1.0 * (accel - max_acc)));

  const double acc_range = max_acc - min_acc;
  return cost / acc_range;
}

double StGraphSearcher::ComputeAccelerationSignCost(
    const StSearchInput& input_info, const StSearchNode& current_node,
    const StSearchNode& node) const {
  bool is_sign_changed = (current_node.accel() * node.accel() < 0.0);
  if (!is_sign_changed) {
    return 0.0;
  }
  double cost = std::fabs(current_node.accel() - node.accel());
  return cost;
}

double StGraphSearcher::ComputeJerkCost(const StSearchInput& input_info,
                                        const StSearchNode& node) const {
  const double jerk = node.jerk();
  const double jerk_sq = jerk * jerk;

  constexpr double positive_jerk_coeff = 1.5;
  constexpr double negative_jerk_coeff = 1.0;

  double cost = 0.0;
  if (jerk > 0.0) {
    cost = positive_jerk_coeff * jerk_sq;
  } else {
    cost = negative_jerk_coeff * jerk_sq;
  }

  const double jerk_range =
      input_info.max_jerk_limit() - input_info.min_jerk_limit();
  return cost / jerk_range;
}

double StGraphSearcher::ComputeLengthCost(const StSearchInput& input_info,
                                          const StSearchNode& current_node,
                                          const StSearchNode& node) const {
  double s_length_cost =
      (node.s() - current_node.s()) * input_info.planning_distance_inverse();
  double t_length_cost = (node.t() - current_node.t()) *
                         input_info.planning_time_horizon_inverse();

  const double weight_s = config_.weight_length_s;
  const double weight_t = config_.weight_length_t;

  double cost_length = s_length_cost * weight_s + t_length_cost * weight_t;

  return cost_length;
}

double StGraphSearcher::ComputeLaneChangeHeuristicCost(
    const StSearchNode& node) const {
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto lane_change_state = lane_change_decider_output.curr_state;
  if (!(lane_change_state == kLaneChangeExecution ||
        lane_change_state == kLaneChangeCancel ||
        lane_change_state == kLaneChangeHold ||
        lane_change_state == kLaneChangeComplete)) {
    return 0.0;
  }

  const auto& ego_trajs_future = lane_change_decider_output.ego_trajs_future;
  if (ego_trajs_future.empty()) {
    return 0.0;
  }

  const double node_t = node.t();
  double target_s = 0.0;

  if (node_t <= ego_trajs_future.front().t) {
    target_s = ego_trajs_future.front().s;
  } else if (node_t >= ego_trajs_future.back().t) {
    target_s = ego_trajs_future.back().s;
  } else {
    auto it = std::lower_bound(
        ego_trajs_future.begin(), ego_trajs_future.end(), node_t,
        [](const auto& pt, double t) { return pt.t < t; });

    if (it == ego_trajs_future.end()) {
      target_s = ego_trajs_future.back().s;
    } else if (std::fabs(it->t - node_t) < 1e-6 ||
               it == ego_trajs_future.begin()) {
      target_s = it->s;
    } else {
      const auto& prev = *std::prev(it);
      const double dt = it->t - prev.t;
      if (dt > 1e-6) {
        target_s = prev.s + (it->s - prev.s) * (node_t - prev.t) / dt;
      } else {
        target_s = it->s;
      }
    }
  }

  const double s_diff = std::fabs(node.s() - target_s);
  return s_diff / std::fmax(target_s, 1.0);
}

double StGraphSearcher::ComputeHeuristicCost(const StSearchInput& input_info,
                                             const StSearchNode& node) const {
  double time_cost = std::fabs(input_info.planning_time_horizon() - node.t()) *
                     input_info.planning_time_horizon_inverse();
  double s_cost = std::fabs(input_info.planning_distance() - node.s()) *
                  input_info.planning_distance_inverse();

  const double weight_t = config_.weight_hcost_t;
  const double weight_s = config_.weight_hcost_s;

  double cost_h = time_cost * weight_t + s_cost * weight_s;

  double lane_change_cost = ComputeLaneChangeHeuristicCost(node);
  const double weight_lane_change = config_.weight_hcost_lane_change;
  cost_h += lane_change_cost * weight_lane_change;

  return cost_h;
}

void StGraphSearcher::SetSearchFailSafe() const {
  std::unordered_map<int64_t, speed::STBoundary::DecisionType>
      succ_decision_table;
  const auto& st_graph = session_->planning_context().st_graph();
  const auto& boundary_id_st_boundaries_map =
      st_graph->boundary_id_st_boundaries_map();
  const auto& agent_id_st_boundaries_map =
      st_graph->agent_id_st_boundaries_map();
  const auto& st_graph_input =
      session_->planning_context().st_graph()->st_graph_input();
  const auto& cipv_info = session_->planning_context().cipv_decider_output();
  SetStSearchFailSafeDecisionTable(boundary_id_st_boundaries_map,
                                   agent_id_st_boundaries_map, st_graph_input,
                                   cipv_info, &succ_decision_table);
  session_->mutable_planning_context()
      ->st_graph()
      ->UpdateStBoundaryDecisionResults(succ_decision_table);
}

void StGraphSearcher::SetStSearchFailSafeDecisionTable(
    const std::unordered_map<int64_t, std::unique_ptr<speed::STBoundary>>&
        boundary_id_st_boundaries_map,
    const std::unordered_map<int32_t, std::vector<int64_t>>&
        agent_id_st_boundaries_map,
    const std::shared_ptr<speed::StGraphInput>& st_graph_input,
    const CIPVInfo& cipv_info,
    std::unordered_map<int64_t, speed::STBoundary::DecisionType>*
        succ_decision_table) const {
  int64_t cipv_boundary_id = -1;
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto lane_change_state = lane_change_decider_output.curr_state;
  const auto is_in_lane_change_propose =
      lane_change_state == StateMachineLaneChangeStatus::kLaneChangePropose;
  if (st_graph_input == nullptr) {
    return;
  }

  const auto& parallel_longitudinal_avoid_output =
      session_->planning_context().parallel_longitudinal_avoid_decider_output();
  const bool is_parallel_overtake =
      parallel_longitudinal_avoid_output
          .is_need_parallel_longitudinal_avoid() &&
      parallel_longitudinal_avoid_output.is_parallel_overtake();
  const auto parallel_overtake_agent_id =
      is_parallel_overtake
          ? parallel_longitudinal_avoid_output.parallel_target_agent_id()
          : -1;

  const auto agent_manager =
      session_->environmental_model().get_agent_manager();
  for (const auto& agent_entry : agent_id_st_boundaries_map) {
    const int32_t agent_id = agent_entry.first;
    const auto& st_boundary_ids = agent_entry.second;

    if (agent_id == parallel_overtake_agent_id) {
      continue;
    }

    bool is_reverse = false;
    bool is_rear = false;
    if (agent_manager) {
      const auto* agent = agent_manager->GetAgent(agent_id);
      if (agent != nullptr && agent->is_reverse()) {
        is_reverse = true;
      }
      if (agent != nullptr && agent->d_rel() < 1e-6) {
        is_rear = true;
      }
    }

    if (!is_reverse && !is_rear && IsSpecialYieldAgent(agent_id)) {
      for (const auto boundary_id : st_boundary_ids) {
        succ_decision_table->insert(std::make_pair(
            boundary_id, speed::STBoundary::DecisionType::YIELD));
      }
    }
  }

  const auto rear_st_id = speed::StGraphUtils::GetAgentStBoundaryId(
      st_graph_input->rear_agent_of_target(), agent_id_st_boundaries_map);
  const speed::STBoundary* rear_st_boundary = nullptr;
  if (boundary_id_st_boundaries_map.find(rear_st_id) !=
      boundary_id_st_boundaries_map.end()) {
    rear_st_boundary = boundary_id_st_boundaries_map.at(rear_st_id).get();
    if (rear_st_boundary != nullptr) {
      succ_decision_table->insert(std::make_pair(
          rear_st_id, speed::STBoundary::DecisionType::OVERTAKE));
    }
  }

  if (st_graph_input->is_lane_keeping() ||
      st_graph_input->is_lane_change_cancle() ||
      (is_in_lane_change_propose &&
       !lane_change_decider_output.s_search_status) ||
      (rear_st_id == -1 || rear_st_boundary == nullptr)) {
    const auto cipv_id = cipv_info.cipv_id();
    if (cipv_id != -1 && agent_id_st_boundaries_map.find(cipv_id) !=
                             agent_id_st_boundaries_map.end()) {
      cipv_boundary_id = agent_id_st_boundaries_map.at(cipv_id).front();
      succ_decision_table->insert(std::make_pair(
          cipv_boundary_id, speed::STBoundary::DecisionType::YIELD));
    }
    return;
  }

  for (const auto& st_boundary_entry : boundary_id_st_boundaries_map) {
    const auto boundary_id = st_boundary_entry.first;
    const auto& st_boundary = *st_boundary_entry.second;
    if (boundary_id == rear_st_id || boundary_id == speed::kNoAgentId) {
      continue;
    }
    if (st_boundary.IsEmpty()) {
      continue;
    }
    if (st_boundary.min_t() > rear_st_boundary->max_t()) {
      continue;
    }
    if (speed::StGraphUtils::IsBoundaryAboveRearTargetBoundary(
            st_boundary, rear_st_boundary)) {
      succ_decision_table->insert(
          std::make_pair(boundary_id, speed::STBoundary::DecisionType::YIELD));
    }
  }
}

int32_t StGraphSearcher::GetStabilizedTargetLaneRearAgentId() {
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  if (lane_change_decider_output.lc_request == NO_CHANGE) {
    return -1;
  }
  const auto target_lane_front_rear_agents =
      MakeTargetLaneFrontRearAgents(session_);
  const auto& target_lane_rear_agents = target_lane_front_rear_agents.second;
  int32_t current_rear = -1;
  if (!target_lane_rear_agents.empty()) {
    current_rear = *target_lane_rear_agents.begin();
  }
  if (current_rear == -1) {
    last_target_lane_rear_agent_id_ = -1;
    candidate_rear_agent_id_ = -1;
    rear_agent_consecutive_cnt_ = 0;
    return -1;
  }
  if (current_rear == last_target_lane_rear_agent_id_) {
    candidate_rear_agent_id_ = -1;
    rear_agent_consecutive_cnt_ = 0;
    return last_target_lane_rear_agent_id_;
  }
  if (last_target_lane_rear_agent_id_ == -1) {
    last_target_lane_rear_agent_id_ = current_rear;
    return current_rear;
  }
  if (current_rear == candidate_rear_agent_id_) {
    rear_agent_consecutive_cnt_++;
    if (rear_agent_consecutive_cnt_ >= kRearAgentHysteresisFrames) {
      last_target_lane_rear_agent_id_ = candidate_rear_agent_id_;
      candidate_rear_agent_id_ = -1;
      rear_agent_consecutive_cnt_ = 0;
      return last_target_lane_rear_agent_id_;
    }
    return last_target_lane_rear_agent_id_;
  }
  candidate_rear_agent_id_ = current_rear;
  rear_agent_consecutive_cnt_ = 1;
  return last_target_lane_rear_agent_id_;
}

bool StGraphSearcher::CheckYieldBackVehicle(
    const std::unordered_map<int64_t, speed::STBoundary::DecisionType>&
        decision_table) {
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  if (lane_change_decider_output.lc_request == NO_CHANGE) {
    return false;
  }
  const int32_t agent_id = cached_stabilized_rear_agent_id_;
  if (agent_id == -1) {
    return false;
  }

  std::vector<int64_t> st_boundaries;
  const auto* st_graph_helper = session_->planning_context().st_graph_helper();
  if (st_graph_helper == nullptr) {
    return false;
  }
  st_graph_helper->GetAgentStBoundaries(agent_id, &st_boundaries);
  for (const auto boundary_id : st_boundaries) {
    speed::STBoundary st_boundary;
    st_graph_helper->GetStBoundary(boundary_id, &st_boundary);

    auto rear_node_front_point = st_boundary.lower_points().front();
    double rear_agent_max_start_yield_time_s =
        config_.rear_agent_max_start_yield_time_s;
    if (rear_node_front_point.t() > rear_agent_max_start_yield_time_s) {
      return false;
    }

    auto lower_it = decision_table.find(boundary_id);
    if (lower_it == decision_table.end()) {
      // int32_t id = agent_id;
      continue;
    } else {
      if (lower_it->second == speed::STBoundary::DecisionType::YIELD) {
        return true;
      } else {
        continue;
      }
    }
  }
  return false;
}

bool StGraphSearcher::CheckOvertakeFrontVehicleOnTargetLane(
    const std::unordered_map<int64_t, speed::STBoundary::DecisionType>&
        decision_table) {
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  if (lane_change_decider_output.lc_request == NO_CHANGE) {
    return false;
  }
  const int32_t agent_id = cached_stabilized_rear_agent_id_;
  if (agent_id == -1) {
    return false;
  }

  const auto* st_graph_helper = session_->planning_context().st_graph_helper();
  if (st_graph_helper == nullptr) {
    return false;
  }
  std::vector<int64_t> agent_st_boundaries;
  if (!st_graph_helper->GetAgentStBoundaries(agent_id, &agent_st_boundaries)) {
    return false;
  }
  for (const auto& boundary_id : agent_st_boundaries) {
    auto iter = decision_table.find(boundary_id);
    if (iter == decision_table.end()) {
      continue;
    }
    if (iter->second == speed::STBoundary::DecisionType::OVERTAKE) {
      return true;
    }
  }
  return false;
}

double StGraphSearcher::GetAgentMinPredictionSpeed(
    const int64_t agent_id) const {
  if (agent_id == planning_data::kInvalidId) {
    return std::numeric_limits<double>::max();
  }
  std::vector<int64_t> st_boundaries;
  double min_front_node_vel = std::numeric_limits<double>::max();
  const auto* st_graph_helper = session_->planning_context().st_graph_helper();
  if (st_graph_helper->GetAgentStBoundaries(agent_id, &st_boundaries)) {
    for (const auto boundary_id : st_boundaries) {
      speed::STBoundary front_st_boundary;
      if (st_graph_helper->GetStBoundary(boundary_id, &front_st_boundary)) {
        for (double time = front_st_boundary.min_t();
             time < front_st_boundary.max_t(); time += kTimeResolution) {
          speed::STPoint upper_point, lower_point;
          front_st_boundary.GetBoundaryBounds(time, &lower_point, &upper_point);
          min_front_node_vel =
              std::min(min_front_node_vel, lower_point.velocity());
        }
      }
    }
  }
  return min_front_node_vel;
}

bool StGraphSearcher::CheckIfFrontVehcileSafe() {
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto lc_request_direction = lane_change_decider_output.lc_request;
  if (lc_request_direction == NO_CHANGE) {
    return true;
  }
  const auto target_lane_front_rear_agents =
      MakeTargetLaneFrontRearAgents(session_);
  const auto target_lane_rear_agents = target_lane_front_rear_agents.second;
  int64_t agent_id = -1;
  if (!target_lane_rear_agents.empty()) {
    auto it = target_lane_rear_agents.begin();
    agent_id = *it;
  }

  double front_vel = GetAgentMinPredictionSpeed(agent_id);
  if (front_vel > kMaxNodeSpeedLimit) {
    return true;
  }

  const auto& agent_manager =
      session_->environmental_model().get_agent_manager();
  if (agent_manager == nullptr) {
    return true;
  }
  const auto* agent = agent_manager->GetAgent(agent_id);
  if (agent == nullptr) {
    return true;
  }
  front_vel = std::min(front_vel, agent->speed());

  const auto& min_acc_curve =
      yield_front_vehicle_safe_utils_->GenerateMaxDecelerationCurve(front_vel);
  const bool is_yield_safe =
      yield_front_vehicle_safe_utils_->IsYieldSafe(min_acc_curve, agent_id);
  if (!is_yield_safe) {
    return false;
  }
  return true;
}

void StGraphSearcher::AddStGraphSearcherDataToProto(
    const std::vector<StSearchNode>& st_search_path) {
#ifdef ENABLE_PROTO_LOG
  auto& debug_info_pb = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto mutable_st_graph_searcher_data =
      debug_info_pb->mutable_st_graph_searcher();
  if (!st_search_path.empty()) {
    for (const auto& search_node : st_search_path) {
      auto* p = st_graph_searcher_pb_.add_st_search_path();
      p->set_s(search_node.s());
      p->set_t(search_node.t());
      p->set_acc(search_node.accel());
      p->set_jerk(search_node.jerk());
      p->set_vel(search_node.vel());
    }
    mutable_st_graph_searcher_data->CopyFrom(st_graph_searcher_pb_);
  } else {
    mutable_st_graph_searcher_data->clear_st_search_path();
  }
#endif
}

void StGraphSearcher::AddAStarSearchCostDebugInfo(
    std::vector<StSearchNode>* const searched_path) const {
  std::vector<double> st_path_final_nodes_total_cost_vec{};
  std::vector<double> st_path_final_nodes_g_cost_vec{};
  std::vector<double> st_path_final_nodes_h_cost_vec{};
  std::vector<double> st_path_final_nodes_cost_yield_vec{};
  std::vector<double> st_path_final_nodes_cost_overtake_vec{};
  std::vector<double> st_path_final_nodes_cost_vel_vec{};
  std::vector<double> st_path_final_nodes_cost_accel_vec{};
  std::vector<double> st_path_final_nodes_cost_accel_sign_changed_vec{};
  std::vector<double> st_path_final_nodes_cost_jerk_vec{};
  std::vector<double> st_path_final_nodes_cost_length_vec{};
  std::vector<double> st_path_final_nodes_time_vec{};
  for (const auto& final_node : *searched_path) {
    st_path_final_nodes_total_cost_vec.emplace_back(final_node.TotalCost());
    st_path_final_nodes_g_cost_vec.emplace_back(final_node.g_cost());
    st_path_final_nodes_h_cost_vec.emplace_back(final_node.h_cost());
    st_path_final_nodes_cost_yield_vec.emplace_back(
        final_node.node_sub_cost().current_node_cost_yield_accumulated);
    st_path_final_nodes_cost_overtake_vec.emplace_back(
        final_node.node_sub_cost().current_node_cost_overtake_accumulated);
    st_path_final_nodes_cost_vel_vec.emplace_back(
        final_node.node_sub_cost().current_node_cost_vel_accumulated);
    st_path_final_nodes_cost_accel_vec.emplace_back(
        final_node.node_sub_cost().current_node_cost_accel_accumulated);
    st_path_final_nodes_cost_accel_sign_changed_vec.emplace_back(
        final_node.node_sub_cost()
            .current_node_cost_accel_sign_changed_accumulated);
    st_path_final_nodes_cost_jerk_vec.emplace_back(
        final_node.node_sub_cost().current_node_cost_jerk_accumulated);
    st_path_final_nodes_cost_length_vec.emplace_back(
        final_node.node_sub_cost().current_node_cost_length_accumulated);
    st_path_final_nodes_time_vec.emplace_back(final_node.t());
  }
  JSON_DEBUG_VECTOR("st_path_final_nodes_total_cost_vec",
                    st_path_final_nodes_total_cost_vec, 4)
  JSON_DEBUG_VECTOR("st_path_final_nodes_g_cost_vec",
                    st_path_final_nodes_g_cost_vec, 4)
  JSON_DEBUG_VECTOR("st_path_final_nodes_h_cost_vec",
                    st_path_final_nodes_h_cost_vec, 4)
  JSON_DEBUG_VECTOR("st_path_final_nodes_cost_yield_vec",
                    st_path_final_nodes_cost_yield_vec, 4)
  JSON_DEBUG_VECTOR("st_path_final_nodes_cost_overtake_vec",
                    st_path_final_nodes_cost_overtake_vec, 4)
  JSON_DEBUG_VECTOR("st_path_final_nodes_cost_vel_vec",
                    st_path_final_nodes_cost_vel_vec, 4)
  JSON_DEBUG_VECTOR("st_path_final_nodes_cost_accel_vec",
                    st_path_final_nodes_cost_accel_vec, 4)
  JSON_DEBUG_VECTOR("st_path_final_nodes_cost_accel_sign_changed_vec",
                    st_path_final_nodes_cost_accel_sign_changed_vec, 4)
  JSON_DEBUG_VECTOR("st_path_final_nodes_cost_jerk_vec",
                    st_path_final_nodes_cost_jerk_vec, 4)
  JSON_DEBUG_VECTOR("st_path_final_nodes_cost_length_vec",
                    st_path_final_nodes_cost_length_vec, 4)
  JSON_DEBUG_VECTOR("st_path_final_nodes_time_vec",
                    st_path_final_nodes_time_vec, 4)
}

bool StGraphSearcher::IsSpecialYieldAgent(const int32_t agent_id) const {
  const auto& agent_longitudinal_decider_output =
      session_->planning_context().agent_longitudinal_decider_output();
  const auto& cutin_ids = agent_longitudinal_decider_output.cutin_agent_ids;
  if (std::find(cutin_ids.begin(), cutin_ids.end(), agent_id) !=
      cutin_ids.end()) {
    return true;
  }

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
        const int32_t gap_front_agent_id = node_ptr->node_agent_id();
        if (gap_front_agent_id == agent_id) {
          return true;
        }
      }
    }
  }

  const auto& lat_lon_joint_planner_output =
      session_->planning_context().lat_lon_joint_planner_decider_output();
  const auto& danger_ids = lat_lon_joint_planner_output.GetDangerObstacleIds();
  if (std::find(danger_ids.begin(), danger_ids.end(), agent_id) !=
      danger_ids.end()) {
    return true;
  }

  return false;
}

bool StGraphSearcher::IsSpecialOvertakeAgent(const int32_t agent_id) const {
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto curr_state = lane_change_decider_output.curr_state;

  if (curr_state == StateMachineLaneChangeStatus::kLaneChangeExecution ||
      curr_state == StateMachineLaneChangeStatus::kLaneChangeHold ||
      curr_state == StateMachineLaneChangeStatus::kLaneChangeComplete) {
    const auto rear_node_id =
        lane_change_decider_output.lc_gap_info.rear_node_id;

    const auto dynamic_world =
        session_->environmental_model().get_dynamic_world();
    if (dynamic_world != nullptr) {
      const auto node_ptr = dynamic_world->GetNode(rear_node_id);
      if (node_ptr != nullptr) {
        const int32_t gap_rear_agent_id = node_ptr->node_agent_id();
        if (gap_rear_agent_id == agent_id) {
          return true;
        }
      }
    }
  }

  return false;
}

}  // namespace planning
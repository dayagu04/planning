#include "st_graph_searcher.h"

#include <utility>
#include <vector>

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
constexpr double kTimeResolution = 0.1;
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

  if (lane_change_state == kLaneChangeExecution) {
    if (lc_request_direction == LEFT_CHANGE) {
      target_lane_front_node_id = dynamic_world->ego_left_front_node_id();
      target_lane_rear_node_id = dynamic_world->ego_left_rear_node_id();
    } else if (lc_request_direction == RIGHT_CHANGE) {
      target_lane_front_node_id = dynamic_world->ego_right_front_node_id();
      target_lane_rear_node_id = dynamic_world->ego_right_rear_node_id();
    }
  } else if (lane_change_state == kLaneChangeComplete) {
    target_lane_front_node_id = dynamic_world->ego_front_node_id();
    target_lane_rear_node_id = dynamic_world->ego_rear_node_id();
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

bool HasCollisionRisk(const StSearchNode& current_node,
                      const StSearchNode& succ_node) {
  // Check if share same current_decision_table.
  if (current_node.current_decision_table().size() !=
      succ_node.current_decision_table().size()) {
    return true;
  }
  std::vector<int64_t> current_keys;
  std::vector<int64_t> succ_keys;
  for (const auto& elem : current_node.current_decision_table()) {
    current_keys.push_back(elem.first);
  }
  std::sort(current_keys.begin(), current_keys.end());
  for (const auto& elem : succ_node.current_decision_table()) {
    succ_keys.push_back(elem.first);
  }
  std::sort(succ_keys.begin(), succ_keys.end());
  return current_keys != succ_keys;
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
    : Task(config_builder, session) {
  name_ = "StGraphSearcher";
  config_ = config_builder->cast<StGraphSearcherConfig>();
  if (yield_front_vehicle_safe_utils_ == nullptr) {
    yield_front_vehicle_safe_utils_ =
        std::make_unique<YieldFrontVehicleSafeFunction>(session, config_);
  }
}

bool StGraphSearcher::Execute() {
  LOG_DEBUG("=======StGraphSearcher======= \n");

  if (!PreCheck()) {
    LOG_DEBUG("PreCheck failed\n");
    return false;
  }

  std::vector<StSearchNode> st_path;

  // search success
  auto res = SearchStPath(&st_path);

  // search fail
  if (!res) {
    SetSearchFailSafe();
    session_->mutable_planning_context()
        ->mutable_st_graph_searcher_output()
        ->set_is_search_success(false);
    return true;
  }

  // update output to st graph
  const auto& last_node = st_path.back();
  session_->mutable_planning_context()
      ->st_graph()
      ->UpdateStBoundaryDecisionResults(last_node.decision_table());
  session_->mutable_planning_context()
      ->mutable_st_graph_searcher_output()
      ->set_is_search_success(true);

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
  auto* mutable_output =
      session_->mutable_planning_context()->mutable_st_graph_searcher_output();
  mutable_output->set_is_yield_front_vehicle_safe(is_yield_front_vehicle_safe);
  mutable_output->set_search_yield_back_vehicle(is_yield_back_vehicle);
  mutable_output->set_is_search_overtake_front_vehicle(
      is_overtake_front_vehicle_on_target_lane);
  // mutable_output->set_traffic_light_decision_map(traffic_light_decision_map);

  return true;
}

bool StGraphSearcher::SearchStPath(
    std::vector<StSearchNode>* const searched_path) {
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

  // set config
  const double planning_time_horizon = config_.planning_time_horizon;
  const double max_accel_limit = config_.max_accel_limit;
  const double min_accel_limit = config_.min_accel_limit;
  const double max_jerk_limit = config_.max_jerk_limit;
  const double min_jerk_limit = config_.min_jerk_limit;
  const int64_t accel_sample_num = config_.accel_sample_num;
  const double s_step = config_.s_step;
  const double t_step = config_.t_step;
  const double vel_step = config_.vel_step;
  const double max_search_time = config_.max_search_time_s;

  double planning_distance = planned_kd_path->Length();
  UpdateHeuristicTargetSInLaneChange(session_, planning_time_horizon,
                                     &planning_distance);
  StSearchInput st_search_input_info(
      planning_init_point, planning_distance, planning_time_horizon, v_cruise,
      max_accel_limit, min_accel_limit, max_jerk_limit, min_jerk_limit,
      accel_sample_num, s_step, t_step, vel_step);
  std::unordered_map<int64_t, StSearchNode> nodes;
  MinHeap<int64_t, double> open_set;
  std::unordered_set<int64_t> close_set;

  auto start_node =
      GenerateStartNode(planning_init_point, st_search_input_info);

  if (!start_node.is_valid()) {
    LOG_ERROR("st search, start node is not valid");
    return false;
  }
  nodes.insert(std::make_pair(start_node.id(), start_node));
  open_set.Push(start_node.id(), start_node.TotalCost());
  bool is_goal_reached = false;

  // record time
  const double start_time = IflyTime::Now_ms();
  int count = 0;
  auto current_node = start_node;
  auto best_node = current_node;
  best_node.set_h_cost(std::numeric_limits<double>::max());
  std::vector<double> expanded_nodes_s_vec{};
  std::vector<double> expanded_nodes_t_vec{};
  std::vector<double> history_cur_nodes_s_vec{};
  std::vector<double> history_cur_nodes_t_vec{};
  // start A* search loop
  while (!open_set.IsEmpty()) {
    const double current_time = IflyTime::Now_ms();
    const double time_used = current_time - start_time;
    const double max_search_time_ms = max_search_time * 1e3;
    // max search time < 0.1s
    if (time_used > max_search_time_ms) {
      LOG_DEBUG("time out, time used: %.4f", time_used);
      break;
    }
    // if (count > 100) {
    //   break;
    // }

    count++;
    current_node = nodes.at(open_set.Top().first);
    history_cur_nodes_s_vec.emplace_back(current_node.s());
    history_cur_nodes_t_vec.emplace_back(current_node.t());
    // update best node
    if (current_node.t() > kBestNodeMinT - kEpsilon &&
        current_node.h_cost() < best_node.h_cost()) {
      best_node = current_node;
    }

    // pop current node and store in close set
    open_set.Pop();
    close_set.insert(current_node.id());
    if (IsReachGoal(st_search_input_info, current_node)) {
      is_goal_reached = true;
      break;
    }

    // generate success nodes
    auto successor_nodes =
        GenerateSuccessorNodes(st_search_input_info, current_node,
                               target_lane_rear_agent_st_boundaries);

    for (auto& child_node : successor_nodes) {
      if (close_set.find(child_node.id()) != close_set.end()) {
        continue;
      }
      const double child_total_g_cost =
          current_node.g_cost() + child_node.cost();

      if (open_set.IsInHeap(child_node.id())) {
        auto& old_child = nodes.at(child_node.id());
        const double old_child_total_g_cost = old_child.g_cost();
        if (child_total_g_cost < old_child_total_g_cost) {
          old_child = child_node;
          old_child.set_g_cost(child_total_g_cost);
          old_child.set_parent_id(current_node.id());
          open_set.Update(old_child.id(), old_child.TotalCost());
        }
        expanded_nodes_s_vec.emplace_back(child_node.s());
        expanded_nodes_t_vec.emplace_back(child_node.t());
      } else {
        child_node.set_g_cost(child_total_g_cost);
        child_node.set_parent_id(current_node.id());
        open_set.Push(child_node.id(), child_node.TotalCost());
        nodes[child_node.id()] = child_node;
        expanded_nodes_s_vec.emplace_back(child_node.s());
        expanded_nodes_t_vec.emplace_back(child_node.t());
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
  // const double end_time = IflyTime::Now_ms();
  if (!is_goal_reached) {
    LOG_DEBUG("st search fail, goal not reached \n");
    bool is_visualize_all_vertexes = config_.is_visualize_st_search_process;
    if (is_visualize_all_vertexes) {
      // VisualizeStSearchVertexes(nodes);
    }
    if (best_node.h_cost() == std::numeric_limits<double>::max()) {
      LOG_DEBUG("st search fail, goal not reached \n");
      return false;
    } else {
      current_node = best_node;
      LOG_DEBUG("st search fail, but use best node to reconstruct path\n");
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
  if (searched_path->size() < 2) {
    LOG_DEBUG("st_search path size < 2");
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
  if (!session_->planning_context().st_graph_helper()->GetBorderByStPoint(
          input_info.init_s(), input_info.init_t(), &lower_bound,
          &upper_bound)) {
    // collision
    // std::cout << "start node is collision, failed" << std::endl;
    start_node.set_is_valid(false);
  }

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

  start_node.set_decision_table(start_decision_table);
  // Same dicision table for start node.
  start_node.set_current_decision_table(start_decision_table);
  start_node.set_is_valid(true);

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

std::vector<StSearchNode> StGraphSearcher::GenerateSuccessorNodes(
    const StSearchInput& input_info, const StSearchNode& current_node,
    const std::unordered_set<int64_t>& target_lane_agent_boundaries) const {
  std::vector<StSearchNode> successor_nodes;

  const double speed_limit_scale = config_.speed_limit_scale;
  const double min_lower_collision_dist = config_.min_lower_collision_dist;
  const double max_lower_collision_dist = config_.max_lower_collision_dist;
  const double speed_scale = config_.lower_collision_dist_speed_scale;

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
    // ignore nodes which go back
    const double s_diff = s_succ - current_node.s();
    if (s_diff < -kEpsilon) {
      continue;
    }

    // check collision with ST curve
    speed::STPoint upper_bound;
    speed::STPoint lower_bound;
    const auto& st_graph_helper =
        session_->planning_context().st_graph_helper();
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
        auto boundary_id = entry.first;
        auto parent_decision = entry.second;
        auto succ_iter = succ_decision_table.find(boundary_id);
        if (succ_iter == succ_decision_table.end()) {
          // copy parent decision
          succ_decision_table[boundary_id] = parent_decision;
          continue;
        }
        is_decision_conflict = (parent_decision != succ_iter->second);
        if (is_decision_conflict) {
          break;
        }
      }
    }
    if (is_decision_conflict) {
      // succ node decision conflict with parent node, means collision with
      // st_boundary, node not valid
      continue;
    }

    // generate succ node
    StSearchNode succ_node =
        StSearchNode(s_succ, t_succ, vel_succ, input_info.s_step(),
                     input_info.t_step(), input_info.vel_step(), config_.enable_only_s_t_hash);

    succ_node.set_parent_id(current_node.id());
    succ_node.set_upper_bound(upper_bound);
    succ_node.set_lower_bound(lower_bound);
    // succ_decision_table: 更新了父节点中的decision
    // succ_current_decision_table: 当前node的decision
    succ_node.set_decision_table(succ_decision_table);
    succ_node.set_current_decision_table(succ_current_decision_table);
    succ_node.set_accel(accel_succ);
    succ_node.set_jerk(jerk_succ);

    // Check if there is collisoin between current_node and succ_node.
    // Check if there is risk: compare current_node and succ_node;
    // Sample more points on line between current_node and succ_node, check
    // collison on st_graph, if has collisoin, drop succ_node.
    if (HasCollisionRisk(current_node, succ_node) &&
        CheckCollisionAfterUpsampling(
            st_graph_helper, target_lane_agent_boundaries, lower_collision_dist,
            current_node, succ_node)) {
      continue;
    }

    // compute cost
    ComputeNodeCost(input_info, current_node, &succ_node);
    successor_nodes.emplace_back(std::move(succ_node));
  }
  return successor_nodes;
}

void StGraphSearcher::ComputeNodeCost(const StSearchInput& input_info,
                                      const StSearchNode& current_node,
                                      StSearchNode* const succ_node) const {
  // all weight should be tuned
  const double weight_yield = config_.weight_yield;
  const double weight_overtake = config_.weight_overtake;
  const double weight_vel = config_.weight_vel;
  const double weight_accel = config_.weight_accel;
  const double weight_accel_sign = config_.weight_accel_sign;
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

  double edge_cost =
      cost_yield * weight_yield + cost_overtake * weight_overtake +
      cost_vel * weight_vel + cost_accel * weight_accel +
      cost_accel_sign_changed * weight_accel_sign + cost_jerk * weight_jerk +
      cost_length /* + cost_virtual_yield * weight_virtual_yield8 */;

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
  if (distance_to_front < upper_truncation_distance) {
    // currently use proportional func, can be switched to exponential func
    return (1.0 - (distance_to_front / upper_truncation_distance));
  }
  return 0.0;
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

  if (distance_to_front < upper_truncation_distance) {
    virtual_yield_cost = 1.0 - distance_to_front / upper_truncation_distance;
  } else {
    virtual_yield_cost = 0.0;
  }

  return virtual_yield_cost;
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

  const double distance_to_rear = node.s() - lower_bound.s();
  if (distance_to_rear < 0.0) {
    return 0.0;
  }
  if (distance_to_rear < lower_truncation_distance) {
    // currently use proportional func, can be switched to exponential func
    return (1.0 - (distance_to_rear / lower_truncation_distance));
  }
  return 0.0;
}

double StGraphSearcher::ComputeVelocityCost(const StSearchInput& input_info,
                                            const StSearchNode& node) const {
  const double vel_diff = std::fabs(node.vel() - input_info.cruise_speed());
  // valid node's vel is always less than speed_limit
  return vel_diff * input_info.speed_limit_inverse();
}

double StGraphSearcher::ComputeAccelerationCost(
    const StSearchInput& input_info, const StSearchNode& current_node,
    const StSearchNode& node) const {
  double desired_accel = 0.0;
  const auto& upper_bound = node.upper_bound();
  const bool need_to_yield_rear_bound =
      upper_bound.boundary_id() != speed::kNoAgentId;

  if (need_to_yield_rear_bound) {
    desired_accel = (upper_bound.velocity() - current_node.vel()) /
                    (input_info.planning_time_horizon() - current_node.t());
  } else {
    double vel_tolerance = config_.velocity_tolerance;
    double propoper_accel_value = config_.proper_accel_value;
    const double node_vel = node.vel();
    if (node_vel > input_info.cruise_speed() + vel_tolerance) {
      desired_accel = -propoper_accel_value;
    } else if (node_vel < input_info.cruise_speed() - vel_tolerance) {
      desired_accel = propoper_accel_value;
    }
  }

  double acc_diff = std::fabs(node.accel() - desired_accel);
  double acc_normalize_value =
      std::max(std::fabs(input_info.min_accel_limit() - desired_accel),
               std::fabs(input_info.max_accel_limit() - desired_accel));

  const double cost_accel = acc_diff / acc_normalize_value;
  return cost_accel;
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
  const double desired_jerk = 0.0;
  const double jerk_normalize_value =
      std::max(std::fabs(input_info.min_jerk_limit() - desired_jerk),
               std::fabs(input_info.max_jerk_limit() - desired_jerk));
  double jerk_diff = std::fabs(node.jerk() - desired_jerk);
  double cost_jerk = jerk_diff / jerk_normalize_value;
  return cost_jerk;
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

double StGraphSearcher::ComputeHeuristicCost(const StSearchInput& input_info,
                                             const StSearchNode& node) const {
  double time_cost = std::fabs(input_info.planning_time_horizon() - node.t()) *
                     input_info.planning_time_horizon_inverse();
  double s_cost = std::fabs(input_info.planning_distance() - node.s()) *
                  input_info.planning_distance_inverse();

  const double weight_t = config_.weight_hcost_t;
  const double weight_s = config_.weight_hcost_s;

  double cost_h = time_cost * weight_t + s_cost * weight_s;

  // std::cout << "\t\t\t\th_time_cost:     " << time_cost << std::endl;
  // std::cout << "\t\t\t\th_s_cost:        " << s_cost << std::endl;
  // std::cout << "\t\t\t\tcost_h:          " << cost_h << std::endl;

  return cost_h;
}

void StGraphSearcher::SetSearchFailSafe() const {
  std::unordered_map<int64_t, speed::STBoundary::DecisionType>
      succ_decision_table;
  session_->mutable_planning_context()
      ->st_graph()
      ->SetStSearchFailSafeDecisionTable(&succ_decision_table);
  session_->mutable_planning_context()
      ->st_graph()
      ->UpdateStBoundaryDecisionResults(succ_decision_table);
}

bool StGraphSearcher::CheckYieldBackVehicle(
    const std::unordered_map<int64_t, speed::STBoundary::DecisionType>&
        decision_table) const {
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto lc_request_direction = lane_change_decider_output.lc_request;
  if (lc_request_direction == NO_CHANGE) {
    return false;
  }
  const auto target_lane_front_rear_agents =
      MakeTargetLaneFrontRearAgents(session_);
  const auto target_lane_rear_agents = target_lane_front_rear_agents.second;
  int64_t agent_id = -1;
  if (!target_lane_rear_agents.empty()) {
    auto it = target_lane_rear_agents.begin();
    agent_id = *it;
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
        decision_table) const {
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto lc_request_direction = lane_change_decider_output.lc_request;
  if (lc_request_direction == NO_CHANGE) {
    return false;
  }
  const auto target_lane_front_rear_agents =
      MakeTargetLaneFrontRearAgents(session_);
  const auto target_lane_rear_agents = target_lane_front_rear_agents.second;
  int64_t agent_id = -1;
  if (!target_lane_rear_agents.empty()) {
    auto it = target_lane_rear_agents.begin();
    agent_id = *it;
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
    const std::vector<StSearchNode> st_search_path) {
  auto& debug_info_pb = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto mutable_st_graph_searcher_data =
      debug_info_pb->mutable_st_graph_searcher();
  if (!st_search_path.empty()) {
    for (const auto& search_node : st_search_path) {
      auto* p = st_graph_searcher_pb_.add_st_search_path();
      p->set_s(search_node.s());
      p->set_t(search_node.t());
    }
  }
  mutable_st_graph_searcher_data->CopyFrom(st_graph_searcher_pb_);
}

}  // namespace planning
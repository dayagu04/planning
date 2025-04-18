#include "agent_headway_decider.h"

#include "debug_info_log.h"
#include "environmental_model.h"
#include "log.h"
#include "planning_context.h"
#include "utils/pose2d_utils.h"

namespace planning {

namespace {

// define headway params here
constexpr double user_time_gap = 1.5;
constexpr double lane_change_decrease_time_gap = 0.8;
constexpr double neighbor_valid_decrease_time_gap = 0.8;
constexpr double first_appear_time_gap = 1.0;
constexpr double kHighSpeedDiffThd = 2.78;
constexpr double kTflVirtualAgentHW = 1.5;
}  // namespace

AgentHeadwayDecider::AgentHeadwayDecider(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : Task(config_builder, session) {
  name_ = "AgentHeadwayDecider";
  config_ = config_builder->cast<AgentHeadwayConfig>();
  plan_time_ = config_.plan_time;
  dt_ = config_.dt;
  plan_points_num_ = static_cast<size_t>(plan_time_ / dt_) + 1;
}

void AgentHeadwayDecider::Reset() { agents_headway_map_.clear(); }

bool AgentHeadwayDecider::Execute() {
  LOG_DEBUG("=======AgentHeadwayDecider======= \n");
  auto res = UpdateAgentsHeadwayInfos();

  auto& mutable_output = session_->mutable_planning_context()
                             ->mutable_agent_headway_decider_output();

  mutable_output.set_agents_headway_Info(agents_headway_map_);

  return true;
}

bool AgentHeadwayDecider::UpdateAgentsHeadwayInfos() {
  std::set<int32_t> pass_corridor_agents;
  const double cutin_headway = config_.cutin_headway_threshold;
  const double smallest_headway = config_.smallest_headway_threshold;
  double headway_step = config_.headway_step;
  double gear_headway = smallest_headway;
  MatchHeadwayWithGearTable(&gear_headway);

  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto lane_change_state = lane_change_decider_output.curr_state;
  const bool is_in_lane_change_execution =
      lane_change_state == kLaneChangeExecution;
  const auto* st_graph_helper = session_->planning_context().st_graph_helper();
  const auto& dynamic_world =
      session_->environmental_model().get_dynamic_world();
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  if (st_graph_helper == nullptr) {
    LOG_DEBUG("[AgentHeadwayDecider] st_graph is nullptr");
    return false;
  }
  if (dynamic_world == nullptr) {
    LOG_DEBUG("[AgentHeadwayDecider] dynamic_world is nullptr");
    return false;
  }
  const auto* agent_manager = dynamic_world->agent_manager();
  if (agent_manager == nullptr) {
    LOG_DEBUG("[AgentHeadwayDecider] agent_manager is nullptr");
    return false;
  }
  if (ego_state_manager == nullptr) {
    LOG_DEBUG("[AgentHeadwayDecider] ego_state_manager is nullptr");
    return false;
  }

  // get cipv id from closest_in_path_vehicle_decider
  const int32_t cipv_id =
      session_->planning_context().cipv_decider_output().cipv_id();
  const bool is_neighbor_target_valid = IsNeighborTargetValid(st_graph_helper);

  for (const auto& st_agent_boundary_id_map :
       st_graph_helper->GetAgentIdSTBoundariesMap()) {
    const auto st_agent_id = st_agent_boundary_id_map.first;
    if (st_agent_boundary_id_map.second.empty()) {
      continue;
    }
    const int32_t agent_st_boundary_id =
        st_agent_boundary_id_map.second.front();
    speed::STBoundary st_boundary;
    if (!st_graph_helper->GetStBoundary(agent_st_boundary_id, &st_boundary)) {
      continue;
    }

    const auto* agent = agent_manager->GetAgent(st_agent_id);
    if (agent == nullptr) {
      continue;
    }
    const bool agent_is_cutin = agent->is_cutin();

    speed::STPoint lower_point;
    speed::STPoint upper_point;
    bool zero_projection = false;
    if (st_boundary.GetBoundaryBounds(0.0, &lower_point, &upper_point)) {
      zero_projection = true;
    } else {
      zero_projection = false;
    }

    pass_corridor_agents.emplace(st_agent_id);
    auto iter = agents_headway_map_.find(st_agent_id);
    const double init_headway_by_ego =
        CalcAgentInitHeadway(ego_state_manager, agent);
    const bool is_tfl_virtual_agent = agent->is_tfl_virtual_obs();
    if (is_tfl_virtual_agent) {
      gear_headway = kTflVirtualAgentHW;
    }
    const double agent_init_headway =
        std::fmin(std::fmax(init_headway_by_ego, cutin_headway), gear_headway);

    const double v_ego = ego_state_manager->ego_v();
    const double v_relative = agent->speed() - v_ego;
    if (v_relative > kHighSpeedDiffThd) {
      headway_step = 0.5 * headway_step;
    }

    // first appear
    if (iter == agents_headway_map_.end()) {
      const double init_headway_by_first_appear =
          user_time_gap - first_appear_time_gap;
      agents_headway_map_[st_agent_id].current_headway =
          init_headway_by_first_appear;
      continue;
    }

    double current_headway = agents_headway_map_[st_agent_id].current_headway;

    // if (agent_is_cutin) {
    //   const double delta_headway = planning_math::LerpWithLimit(
    //       config_.cut_in_headway_upper_bound,
    //       config_.cut_in_velocity_lower_bound,
    //       config_.cut_in_headway_lower_bound,
    //       config_.cut_in_velocity_upper_bound, ego_state_manager->ego_v());
    //   const double matched_cut_in_headway =
    //       std::fmin((cutin_headway - delta_headway), current_headway);
    //   agents_headway_map_[st_agent_id].current_headway =
    //       std::fmin(matched_cut_in_headway + headway_step, gear_headway);
    //   continue;
    // }

    if (is_neighbor_target_valid) {
      const double neighbor_target_headway = std::fmin(
          (user_time_gap - neighbor_valid_decrease_time_gap),
          current_headway);
      agents_headway_map_[st_agent_id].current_headway =
          std::fmin(neighbor_target_headway + headway_step, gear_headway);
      continue;
    }

    if (is_in_lane_change_execution) {
      const double lane_change_headway = std::fmin(
          (user_time_gap - lane_change_decrease_time_gap), current_headway);
      agents_headway_map_[st_agent_id].current_headway =
          std::fmin(lane_change_headway + headway_step, gear_headway);
      continue;
    }

    // if (is_need_reset) {
    //   const double lane_change_headway = user_time_gap -
    //   lane_change_decrease_time_gap; agents_headway_map_[st_agent_id] =
    //   {lane_change_headway}; continue;
    // }

    if (iter != agents_headway_map_.end()) {
      const double final_headway = std::fmin(
          agents_headway_map_[st_agent_id].current_headway + headway_step,
          gear_headway);
      agents_headway_map_[st_agent_id].current_headway = final_headway;
    }
  }

  // clear agent which is lost
  for (auto iter = agents_headway_map_.begin();
       iter != agents_headway_map_.end();) {
    if (pass_corridor_agents.count(iter->first)) {
      ++iter;
    } else {
      agents_headway_map_.erase(iter++);
    }
  }

  // json debug
  std::vector<double> agent_id;
  std::vector<double> agent_headway;
  for (const auto agent : agents_headway_map_) {
    agent_id.emplace_back(agent.first);
    agent_headway.emplace_back(agent.second.current_headway);
  }
  JSON_DEBUG_VECTOR("agents_headway_id", agent_id, 0)
  JSON_DEBUG_VECTOR("agents_headway_value", agent_headway, 3)
  return true;
}

void AgentHeadwayDecider::MatchHeadwayWithGearTable(
    double* const matched_desired_headway) const {
  static auto headway_table = config_.normal_headway_table;
  // const auto gear = planning_data->system_manager_info().navi_ttc_gear();
  // get ttc through different gears
  auto time_headway_level = session_->environmental_model()
                                .get_ego_state_manager()
                                ->time_headway_level();
  if (time_headway_level < 1) {
    time_headway_level = 1;
  } else if (time_headway_level > 5) {
    time_headway_level = 5;
  } else {
    time_headway_level = time_headway_level;
  }

  // const auto driving_style =
  //     planning_data->system_manager_info().driving_style();
  const auto driving_style = DrivingStyle::NORMAL;

  if (driving_style == DrivingStyle::AGGRESIVE) {
    headway_table = config_.aggressive_headway_table;
  } else if (driving_style == DrivingStyle::NORMAL) {
    headway_table = config_.normal_headway_table;
  } else if (driving_style == DrivingStyle::CONSERVATIVE) {
    headway_table = config_.conservative_headway_table;
  }

  if (time_headway_level > headway_table.size()) {
    return;
  }

  *matched_desired_headway = headway_table.at(time_headway_level - 1).second;
  JSON_DEBUG_VALUE("time_headway_level", time_headway_level);
  return;
}

bool AgentHeadwayDecider::IsNeighborTargetValid(
    const speed::StGraphHelper* st_graph) const {
  speed::STPoint first_neighbor_upper_bound = speed::STPoint::HighestSTPoint();
  bool is_yield_valid =
      st_graph->GetFirstNeighborUpperBound(&first_neighbor_upper_bound);
  speed ::STPoint first_neighbor_lower_bound = speed::STPoint::LowestSTPoint();
  bool is_overtake_valid =
      st_graph->GetFirstNeighborLowerBound(&first_neighbor_lower_bound);

  if (!(is_yield_valid || is_overtake_valid)) {
    return false;
  }

  double yield_s =
      first_neighbor_upper_bound.s() -
      first_neighbor_upper_bound.velocity() * first_neighbor_upper_bound.t();
  double overtake_s =
      first_neighbor_lower_bound.s() +
      first_neighbor_lower_bound.velocity() * first_neighbor_lower_bound.t();
  if (yield_s < overtake_s) {
    return false;
  }
  return true;
};

double AgentHeadwayDecider::CalcAgentInitHeadway(
    const std::shared_ptr<EgoStateManager>& ego_state_manager,
    const agent::Agent* agent) {
  constexpr double ego_vel_min_thd = 0.1;
  const double cutin_headway = config_.cutin_headway_threshold;
  const auto& planned_path =
      session_->planning_context().motion_planner_output().lateral_path_coord;
  const auto& init_point = ego_state_manager->planning_init_point();

  double ego_s = 0.0;
  double ego_l = 0.0;
  double agent_s = 0.0;
  double agent_l = 0.0;
  if (!planned_path->XYToSL(init_point.x, init_point.y, &ego_s, &ego_l) ||
      !planned_path->XYToSL(agent->x(), agent->y(), &agent_s, &agent_l)) {
    return cutin_headway;
  }

  double agent_to_ego_distance = agent_s - ego_s;
  if (agent_to_ego_distance < 0.0) {
    return cutin_headway;
  }

  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();

  double agent_back_edge_to_ego_front_edge_distance =
      agent_to_ego_distance - vehicle_param.front_edge_to_rear_axle -
      agent->length() * 0.5;

  const double ego_vel = ego_state_manager->ego_v();
  const double init_headway = (agent_back_edge_to_ego_front_edge_distance -
                               config_.lower_speed_min_follow_distance_gap) /
                              std::fmax(ego_vel, ego_vel_min_thd);
  return init_headway;
}

}  // namespace planning
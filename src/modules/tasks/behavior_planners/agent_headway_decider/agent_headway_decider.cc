#include "agent_headway_decider.h"

#include <math.h>

#include <vector>

#include "agent/agent.h"
#include "behavior_planners/sample_poly_speed_adjust_decider/sample_poly_const.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "environmental_model.h"
#include "log.h"
#include "planning_context.h"
#include "utils/pose2d_utils.h"

namespace planning {

namespace {

// define headway params here
constexpr double user_time_gap = 1.5;
constexpr double lane_change_decrease_time_gap = 0.5;
constexpr double neighbor_valid_decrease_time_gap = 0.8;
constexpr double k_first_appear_time_gap = 1.0;
constexpr double kEpsilon = 1e-6;
constexpr double kHighSpeedDiffThd = -0.5;
constexpr double kTflVirtualAgentHW = 1.5;
constexpr double kInterestPredictionTrajScope_s = 3.6;
constexpr double kTimeResolutionPredTraj = 0.2;
constexpr double kPlanningdt = 0.1;
constexpr double kVelDiffInLaneChangeThrd = 2.5;
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
  ILOG_INFO << "=======AgentHeadwayDecider=======";
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
  MatchHeadwayWithGearTable(gear_headway);

  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto lane_change_state = lane_change_decider_output.curr_state;
  const bool is_in_lane_change = lane_change_state == kLaneChangeExecution ||
                                 lane_change_state == kLaneChangeHold ||
                                 lane_change_state == kLaneChangeComplete;
  const bool is_in_lane_change_propose = lane_change_state == kLaneChangePropose;
  if (is_in_lane_change || is_in_lane_change_propose) {
    lc_to_lk_thw_is_init_ = false;
  }
  const auto* st_graph_helper = session_->planning_context().st_graph_helper();
  const auto& dynamic_world =
      session_->environmental_model().get_dynamic_world();
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto& yeild_agents_ids_periods_in_st_pass_corridor =
      st_graph_helper->yeild_agents_ids_periods_in_st_pass_corridor();

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

  if (st_graph_helper == nullptr) {
    ILOG_DEBUG << "[AgentHeadwayDecider] st_graph is nullptr";
    return false;
  }
  if (dynamic_world == nullptr) {
    ILOG_DEBUG << "[AgentHeadwayDecider] dynamic_world is nullptr";
    return false;
  }
  const auto* agent_manager = dynamic_world->agent_manager();
  if (agent_manager == nullptr) {
    ILOG_DEBUG << "[AgentHeadwayDecider] agent_manager is nullptr";
    return false;
  }
  if (ego_state_manager == nullptr) {
    ILOG_DEBUG << "[AgentHeadwayDecider] ego_state_manager is nullptr";
    return false;
  }

  // get cipv id from closest_in_path_vehicle_decider
  const int32_t cipv_id =
      session_->planning_context().cipv_decider_output().cipv_id();
  // const bool is_neighbor_target_valid =
  // IsNeighborTargetValid(st_graph_helper);

  const double v_ego =
      ego_state_manager->planning_init_point().lon_init_state.v();
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
    if (st_boundary.min_t() > 3.5 &&
        (st_boundary.max_t() - st_boundary.min_t()) < 0.7) {
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
    const bool is_lane_borrow_virtual_agent =
        agent->is_lane_borrow_virtual_obs();
    const bool is_vru_crossing_virtual_agent =
        agent->is_vru_crossing_virtual_obs();
    double first_appear_time_gap = k_first_appear_time_gap;
    if (is_tfl_virtual_agent) {
      gear_headway = kTflVirtualAgentHW;
      first_appear_time_gap = 0.5;
    }
    const double agent_init_headway =
        std::fmin(std::fmax(init_headway_by_ego, cutin_headway), gear_headway);

    constexpr double kHighRelativeVelThreshold = 2.0;
    constexpr double kLowRelativeVelThreshold = -0.5;
    constexpr double kVeryLowRelativeVelThreshold = -3.0;
    constexpr double kHighRelativeVelStepFactor = 0.3;
    constexpr double kMediumRelativeVelStepFactor = 0.5;
    constexpr double kLowRelativeVelStepFactor = 1.2;
    constexpr double kDefaultStepFactor = 1.0;

    const double v_relative = agent->speed() - v_ego;
    double step_factor = kDefaultStepFactor;
    if (v_relative > kHighRelativeVelThreshold) {
      step_factor = kHighRelativeVelStepFactor;
    } else if (v_relative > kLowRelativeVelThreshold) {
      step_factor = kMediumRelativeVelStepFactor;
    } else if (v_relative < kVeryLowRelativeVelThreshold) {
      step_factor = kLowRelativeVelStepFactor;
    }
    headway_step = step_factor * config_.headway_step;

    if (is_vru_crossing_virtual_agent) {
      agents_headway_map_[st_agent_id].current_headway = 0.0;
      continue;
    }

    if (gap_front_agent_id == st_agent_id && (lane_change_state == kLaneChangeExecution||
      lane_change_state == kLaneChangeComplete)) {
      CalculateTHWInLaneChange(st_agent_id, gear_headway);
      last_gap_front_agent_id_ = gap_front_agent_id;
      continue;
    } else if (yeild_agents_ids_periods_in_st_pass_corridor.find(
                   last_gap_front_agent_id_) !=
                   yeild_agents_ids_periods_in_st_pass_corridor.end() &&
               st_agent_id == last_gap_front_agent_id_ &&
               !is_in_lane_change_propose && !is_in_lane_change) {
      if (CalculateTHWInLaneChangeToLaneKeep(gear_headway)) {
        continue;
      }
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

    if (agent_is_cutin) {
      const double cutin_target_headway =
          CalculateCutinHeadway(agent, v_ego, current_headway);
      agents_headway_map_[st_agent_id].current_headway = cutin_target_headway;
      continue;
    }

    if (is_lane_borrow_virtual_agent) {
      agents_headway_map_[st_agent_id].current_headway = 0.0;
      continue;
    }

    // if (is_neighbor_target_valid) {
    //   const auto neighbor_yield_agent_id =
    //       st_graph_helper->GetFirstNeighborYieldAgentId();
    //   const auto neighbor_yield_agent =
    //       agent_manager->GetAgent(neighbor_yield_agent_id);

    //   const auto& neighbor_yield_agent_prediction_trajs =
    //       neighbor_yield_agent
    //           ? neighbor_yield_agent->trajectories_used_by_st_graph()
    //           : std::vector<trajectory::Trajectory>{};

    //   if (neighbor_yield_agent_prediction_trajs.empty() ||
    //       neighbor_yield_agent_prediction_trajs.front().empty()) {
    //   } else {
    //     const auto& neighbor_yield_agent_traj =
    //         neighbor_yield_agent_prediction_trajs.front();
    //     double sum_vel = 0.0;
    //     size_t traj_point_size = std::round(kInterestPredictionTrajScope_s /
    //                                         kTimeResolutionPredTraj);
    //     for (size_t i = 0; i < traj_point_size; ++i) {
    //       sum_vel += neighbor_yield_agent_traj[i].vel();
    //     }
    //     double average_vel = sum_vel / traj_point_size;
    //     if (average_vel - v_ego > 0.3 && is_in_lane_change) {
    //       agents_headway_map_[st_agent_id].current_headway = 0.8;
    //       continue;
    //     }
    //   }

    //   const double neighbor_target_headway = std::fmin(
    //       (user_time_gap - neighbor_valid_decrease_time_gap),
    //       current_headway);
    //   agents_headway_map_[st_agent_id].current_headway =
    //       std::fmin(neighbor_target_headway + headway_step, gear_headway);
    //   continue;
    // }


    if (is_in_lane_change_propose) {
      const double lane_change_headway = std::fmin(
          (user_time_gap - lane_change_decrease_time_gap), current_headway);
      agents_headway_map_[st_agent_id].current_headway =
          std::fmin(lane_change_headway + headway_step, gear_headway);
      continue;
    }

    const int32_t origin_lane_front_agent_id = GetOriginLaneFrontAgentId();
    if (is_in_lane_change && st_agent_id == origin_lane_front_agent_id) {
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

    // for destination stop(only virtual agent)
    const auto& stop_destination_decider_output =
        session_->planning_context().stop_destination_decider_output();
    const auto stop_destination_virtual_agent_id =
        stop_destination_decider_output.stop_destination_virtual_agent_id();
    const auto stop_destination_agent =
        agent_manager->GetAgent(stop_destination_virtual_agent_id);
    agent::AgentType stop_destination_agent_type = agent::AgentType::UNKNOWN;
    if (stop_destination_agent) {
      stop_destination_agent_type = stop_destination_agent->type();
    }
    if (agent::AgentDefaultInfo::kNoAgentId !=
            stop_destination_virtual_agent_id &&
        stop_destination_agent_type == agent::AgentType::VIRTUAL) {
      agents_headway_map_[st_agent_id].current_headway =
          stop_destination_decider_output
              .stop_destination_virtual_agent_time_headway();
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

void AgentHeadwayDecider::CalculateTHWInLaneChange(
    const int32_t gap_front_agent_id, const double thw_request) {
  const auto& dynamic_world =
      session_->environmental_model().get_dynamic_world();
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto* agent_manager = dynamic_world->agent_manager();
  const auto* st_graph_helper = session_->planning_context().st_graph_helper();
  const double v_ego =
      ego_state_manager->planning_init_point().lon_init_state.v();

  const auto& yeild_agents_ids_periods_in_st_pass_corridor =
      st_graph_helper->yeild_agents_ids_periods_in_st_pass_corridor();
  const auto gap_yield_agent_id_period_iter =
      yeild_agents_ids_periods_in_st_pass_corridor.find(gap_front_agent_id);
  const auto yield_gap_front_agent_decision_exsits =
      gap_yield_agent_id_period_iter !=
      yeild_agents_ids_periods_in_st_pass_corridor.end();

  if (yield_gap_front_agent_decision_exsits) {
    const auto neighbor_yield_agent =
        agent_manager->GetAgent(gap_front_agent_id);

    const auto& neighbor_yield_agent_prediction_trajs =
        neighbor_yield_agent
            ? neighbor_yield_agent->trajectories_used_by_st_graph()
            : std::vector<trajectory::Trajectory>{};

    if (neighbor_yield_agent_prediction_trajs.empty() ||
        neighbor_yield_agent_prediction_trajs.front().empty()) {
      if (gap_yield_agent_id_period_iter->second <= kPlanningdt) {
        thw_lane_change_slope_filter_.Init(
            0.0, 0.0, config_.thw_low_rate_lane_change,
            config_.thw_init_value_lane_change,
            config_.thw_target_low_value_lane_change, kPlanningdt);
        thw_lane_change_slope_filter_.Update(thw_request);
        agents_headway_map_[gap_front_agent_id].current_headway =
            thw_lane_change_slope_filter_.GetOutput();
      } else {
        thw_lane_change_slope_filter_.Update(thw_request);
        agents_headway_map_[gap_front_agent_id].current_headway =
            thw_lane_change_slope_filter_.GetOutput();
      }
    } else {
      const auto& neighbor_yield_agent_traj =
          neighbor_yield_agent_prediction_trajs.front();
      double sum_vel = 0.0;
      size_t traj_point_size =
          std::round(kInterestPredictionTrajScope_s / kTimeResolutionPredTraj);
      for (size_t i = 0; i < traj_point_size; ++i) {
        sum_vel += neighbor_yield_agent_traj[i].vel();
      }
      double average_vel = sum_vel / traj_point_size;
      const double vel_diff = average_vel - v_ego;
      if (vel_diff > 0.3) {
        if (gap_yield_agent_id_period_iter->second <= kPlanningdt) {
          thw_lane_change_slope_filter_.Init(
              0.0, 0.0, config_.thw_low_rate_lane_change,
              config_.thw_init_value_lane_change,
              config_.thw_target_low_value_lane_change, kPlanningdt);
          thw_lane_change_slope_filter_.Update(
              config_.thw_target_low_value_lane_change);
          agents_headway_map_[gap_front_agent_id].current_headway =
              thw_lane_change_slope_filter_.GetOutput();
        } else {
          const double slope_filter_max_limit =
              agents_headway_map_[gap_front_agent_id].current_headway >
                      config_.thw_target_low_value_lane_change
                  ? agents_headway_map_[gap_front_agent_id].current_headway
                  : config_.thw_target_low_value_lane_change;
          thw_lane_change_slope_filter_.SetLimit(
              config_.thw_init_value_lane_change, slope_filter_max_limit);
          thw_lane_change_slope_filter_.SetRate(
              -config_.thw_high_rate_lane_change,
              config_.thw_low_rate_lane_change);
          thw_lane_change_slope_filter_.Update(
              config_.thw_target_low_value_lane_change);
          agents_headway_map_[gap_front_agent_id].current_headway =
              thw_lane_change_slope_filter_.GetOutput();
        }
      } else {
        if (gap_yield_agent_id_period_iter->second <= kPlanningdt) {
          thw_lane_change_slope_filter_.Init(
              0.0, 0.0, config_.thw_low_rate_lane_change,
              config_.thw_init_value_lane_change, thw_request, kPlanningdt);
          if (vel_diff < -kVelDiffInLaneChangeThrd) {
            thw_lane_change_slope_filter_.SetRate(
                0.0, config_.thw_high_rate_lane_change);
          }
          thw_lane_change_slope_filter_.Update(thw_request);
          agents_headway_map_[gap_front_agent_id].current_headway =
              thw_lane_change_slope_filter_.GetOutput();
        } else {
          thw_lane_change_slope_filter_.Init(
              agents_headway_map_[gap_front_agent_id].current_headway, 0.0,
              config_.thw_low_rate_lane_change,
              config_.thw_init_value_lane_change, thw_request, kPlanningdt);
          if (vel_diff < -kVelDiffInLaneChangeThrd) {
            thw_lane_change_slope_filter_.SetRate(
                0.0, config_.thw_high_rate_lane_change);
          }
          thw_lane_change_slope_filter_.Update(thw_request);
          agents_headway_map_[gap_front_agent_id].current_headway =
              thw_lane_change_slope_filter_.GetOutput();
        }
      }
    }
  }
}

bool AgentHeadwayDecider::CalculateTHWInLaneChangeToLaneKeep(
    const double thw_request) {
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto* st_graph_helper = session_->planning_context().st_graph_helper();
  auto virtual_acc_curve = MakeVirtualZeroAccCurve();
  double max_dis_between_boundry_ego_curv = std::numeric_limits<double>::min();
  double current_ego_dis_to_low_boundry = std::numeric_limits<double>::max();
  std::map<int, double> boundry_time_idx_to_dis;
  std::vector<int64_t> boundaries;
  if (st_graph_helper->GetAgentStBoundaries(last_gap_front_agent_id_, &boundaries)) {
    for (const auto boundary_id : boundaries) {
      speed::STBoundary st_boundary;
      if (st_graph_helper->GetStBoundary(boundary_id, &st_boundary)) {
        const auto& lower_pts = st_boundary.lower_points();
        for (int idx = 0; idx < lower_pts.size(); ++idx) {
          const auto& pt = lower_pts[idx];
          if (std::fabs(pt.t() - 0.0) < kEpsilon) {
            if (current_ego_dis_to_low_boundry > pt.s()) {
              current_ego_dis_to_low_boundry = pt.s();
            }
          }
          max_dis_between_boundry_ego_curv = pt.s() - virtual_acc_curve->Evaluate(0, pt.t());
          double ego_vel_on_t = virtual_acc_curve->Evaluate(1, pt.t());
          if (max_dis_between_boundry_ego_curv < ego_vel_on_t * 0.3 + 3.0) {
            int time_idx = static_cast<int>(std::round(pt.t() / kTimeResolutionPredTraj));
            boundry_time_idx_to_dis.insert({time_idx, max_dis_between_boundry_ego_curv});
          }
        }
      }
    }
  }
  int boundry_time_idx_to_dis_map_size = boundry_time_idx_to_dis.size();
  double headway_expand_rate_scale = 1.0;
  double headway_init_rate_scale = 1.0;
  headway_expand_rate_scale = planning_math::LerpWithLimit(
    1.0, 5, 2.0,
    20, boundry_time_idx_to_dis_map_size);
  auto iter = boundry_time_idx_to_dis.upper_bound(10);
  if(iter != boundry_time_idx_to_dis.end() && iter != boundry_time_idx_to_dis.begin()) {
    headway_init_rate_scale = planning_math::LerpWithLimit(
      1.6, 4, 1.1,
      10, boundry_time_idx_to_dis.begin()->first);
  }

  double ego_current_pos_headway = (current_ego_dis_to_low_boundry - 4.0) / ego_state_manager->planning_init_point().v;
  ego_current_pos_headway = std::fmax(ego_current_pos_headway, 0.1);
  if (fabs(ego_current_pos_headway - thw_request) < 1e-3) {
    agents_headway_map_[last_gap_front_agent_id_].current_headway = thw_request;
    last_gap_front_agent_id_ = -1;
    lc_to_lk_thw_is_init_ = false;
    return true;
  } else {
    if (!lc_to_lk_thw_is_init_) {
      thw_lane_change_slope_filter_.Init(
        ego_current_pos_headway * headway_init_rate_scale, -config_.thw_low_rate_lane_change_to_lane_keep,
          config_.thw_low_rate_lane_change_to_lane_keep * headway_expand_rate_scale,
          config_.thw_init_value_lane_change, thw_request, kPlanningdt);
      lc_to_lk_thw_is_init_ = true;
    }

    thw_lane_change_slope_filter_.Update(thw_request);
    agents_headway_map_[last_gap_front_agent_id_].current_headway =
        thw_lane_change_slope_filter_.GetOutput();
    return true;
  }
}

void AgentHeadwayDecider::MatchHeadwayWithGearTable(
    double& matched_desired_headway) const {
  auto time_headway_table = config_.ego_normal_thw_table_level_3;
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  // const auto gear = planning_data->system_manager_info().navi_ttc_gear();
  // get ttc through different gears
  auto time_headway_level = session_->environmental_model()
                                .get_ego_state_manager()
                                ->time_headway_level();
  const auto has_time_headway_scale_up_request =
      ego_state_manager->has_time_headway_scale_up_request();
  const auto has_efficient_pass_request =
      ego_state_manager->has_efficient_pass_request();

  if (time_headway_level < 1) {
    time_headway_level = 1;
  } else if (time_headway_level > 5) {
    time_headway_level = 5;
  } else {
    time_headway_level = time_headway_level;
  }

  // const auto driving_style =
  //     planning_data->system_manager_info().driving_style();
  switch (time_headway_level) {
    case 1:
      time_headway_table = config_.ego_normal_thw_table_level_1;
      break;
    case 2:
      time_headway_table = config_.ego_normal_thw_table_level_2;
      break;
    case 3:
      time_headway_table = config_.ego_normal_thw_table_level_3;
      break;
    case 4:
      time_headway_table = config_.ego_normal_thw_table_level_4;
      break;
    case 5:
      time_headway_table = config_.ego_normal_thw_table_level_5;
      break;
    default:
      time_headway_table = config_.ego_normal_thw_table_level_3;
      break;
  }

  const double planning_init_vel =
      ego_state_manager->planning_init_point().lon_init_state.v();

  if (time_headway_level > time_headway_table.size()) {
    return;
  }

  matched_desired_headway = planning::interp(
      planning_init_vel, config_.ego_vel_table, time_headway_table);
  matched_desired_headway =
      has_time_headway_scale_up_request
          ? matched_desired_headway * config_.thw_scale_up_factor
          : matched_desired_headway;
  matched_desired_headway =
      has_efficient_pass_request
          ? matched_desired_headway * config_.thw_scale_down_factor
          : matched_desired_headway;
  JSON_DEBUG_VALUE("time_headway_level", time_headway_level);
  JSON_DEBUG_VALUE("THW", matched_desired_headway);
  JSON_DEBUG_VALUE("thw_scale_up_request", has_time_headway_scale_up_request)
  JSON_DEBUG_VALUE("thw_efficient_pass_request", has_efficient_pass_request)
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

double AgentHeadwayDecider::CalculateCutinHeadway(
    const agent::Agent* agent, const double ego_velocity,
    const double current_headway) {
  constexpr double kCutinMinHeadway = 0.5;
  constexpr double kCutinMaxHeadway = 1.2;
  constexpr double kCutinHeadwayStep = 0.1;
  constexpr double kHighSpeedCutinFactor = 0.8;
  constexpr double kHighSpeedThreshold = 80.0 / 3.6;

  double cutin_target_headway = kCutinMaxHeadway;
  if (ego_velocity > kHighSpeedThreshold) {
    cutin_target_headway = kCutinMaxHeadway * kHighSpeedCutinFactor;
  }

  constexpr double kCutinHighRelativeVelThreshold = 2.0;
  constexpr double kCutinLowRelativeVelThreshold = -5.0;
  constexpr double kCutinHighVelFactor = 0.7;
  constexpr double kCutinLowVelFactor = 1.3;
  constexpr double kCutinDefaultVelFactor = 1.0;
  constexpr double kCutinVelFactorRange = 7.0;  // (2.0 + 5.0)

  const double relative_velocity = agent->speed() - ego_velocity;
  double velocity_factor = kCutinDefaultVelFactor;
  if (relative_velocity >= kCutinHighRelativeVelThreshold) {
    velocity_factor = kCutinHighVelFactor;
  } else if (relative_velocity <= kCutinLowRelativeVelThreshold) {
    velocity_factor = kCutinLowVelFactor;
  } else {
    velocity_factor = kCutinLowVelFactor +
                      (relative_velocity - kCutinLowRelativeVelThreshold) *
                          (kCutinHighVelFactor - kCutinLowVelFactor) /
                          kCutinVelFactorRange;
  }
  cutin_target_headway *= velocity_factor;

  double final_headway;
  if (current_headway < cutin_target_headway) {
    final_headway =
        std::fmin(current_headway + kCutinHeadwayStep, cutin_target_headway);
  } else {
    constexpr double kCutinHeadwayDecreaseRatio = 0.5;
    final_headway = std::fmax(
        current_headway - kCutinHeadwayStep * kCutinHeadwayDecreaseRatio,
        cutin_target_headway);
  }

  final_headway =
      std::fmax(kCutinMinHeadway, std::fmin(final_headway, kCutinMaxHeadway));

  return final_headway;
}

int32_t AgentHeadwayDecider::GetOriginLaneFrontAgentId() {
  const auto& dynamic_world =
      session_->environmental_model().get_dynamic_world();

  if (dynamic_world == nullptr) {
    return -1;
  }

  // get lane change status
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto lc_request_direction = lane_change_decider_output.lc_request;
  const auto lane_change_state = lane_change_decider_output.curr_state;

  const bool is_lane_changing = (lane_change_state == kLaneChangeExecution ||
                                 lane_change_state == kLaneChangeComplete);

  int64_t current_lane_front_node_id = -1;

  if (lc_request_direction == LEFT_CHANGE) {
    if (is_lane_changing) {
      current_lane_front_node_id = dynamic_world->ego_right_front_node_id();
    } else {
      current_lane_front_node_id = dynamic_world->ego_front_node_id();
    }
  } else if (lc_request_direction == RIGHT_CHANGE) {
    if (is_lane_changing) {
      current_lane_front_node_id = dynamic_world->ego_left_front_node_id();
    } else {
      current_lane_front_node_id = dynamic_world->ego_front_node_id();
    }
  } else {
    current_lane_front_node_id = dynamic_world->ego_front_node_id();
  }

  if (current_lane_front_node_id != -1) {
    auto* current_lane_front_node =
        dynamic_world->GetNode(current_lane_front_node_id);
    if (current_lane_front_node != nullptr) {
      return current_lane_front_node->node_agent_id();
    }
  }

  return -1;
}

std::unique_ptr<Trajectory1d> AgentHeadwayDecider::MakeVirtualZeroAccCurve() {
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  std::array<double, 3> init_lon_state = {0., ego_state_manager->planning_init_point().v,
                                          ego_state_manager->planning_init_point().a};

  auto virtual_zero_acc_curve =
      std::make_unique<PiecewiseJerkAccelerationTrajectory1d>(
          init_lon_state[0], init_lon_state[1]);
  virtual_zero_acc_curve->AppendSegment(init_lon_state[2], dt_);

  const double zero_acc_jerk_max = 0.5;
  const double zero_acc_jerk_min = -1.0;
  for (double t = dt_; t <= plan_time_; t += dt_) {
    const double acc = virtual_zero_acc_curve->Evaluate(2, t);
    const double vel = virtual_zero_acc_curve->Evaluate(1, t);

    double a_next = 0.0;
    // if init acc < 0.0, move a to zero with jerk max
    // if init acc >0.0,move a to zero with jerk min
    if (init_lon_state[2] < 0.0) {
      a_next = acc + dt_ * zero_acc_jerk_max;
    } else {
      a_next = acc + dt_ * zero_acc_jerk_min;
    }

    if (init_lon_state[2] * acc <= 0.0) {
      a_next = 0.0;
    }

    if (vel <= 0.0) {
      a_next = 0.0;  //??
    }
    virtual_zero_acc_curve->AppendSegment(a_next, dt_);
  }
  return virtual_zero_acc_curve;
}

}  // namespace planning
#include "longtime_task_pipeline_v3.h"

#include "log.h"
#include "speed/st_graph_input.h"

namespace planning {

LongTimeTaskPipelineV3::LongTimeTaskPipelineV3(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : BaseTaskPipeline(config_builder, session) {
  ego_lane_road_right_decider_ =
      std::make_unique<EgoLaneRoadRightDecider>(config_builder, session);
  lane_change_decider_ =
      std::make_unique<LaneChangeDecider>(config_builder, session);
  speed_adjust_decider_ =
      std::make_unique<SpeedAdjustDecider>(config_builder, session);
  lateral_offset_decider_ =
      std::make_unique<LateralOffsetDecider>(config_builder, session);
  gap_selector_decider_ =
      std::make_unique<GapSelectorDecider>(config_builder, session);
  general_lateral_decider_ =
      std::make_unique<GeneralLateralDecider>(config_builder, session);
  traffic_light_decider_ =
      std::make_unique<TrafficLightDecider>(config_builder, session);
  lateral_motion_planner_ =
      std::make_unique<LateralMotionPlanner>(config_builder, session);

  // long pipeline V3
  virtual_obstacle_decider_ =
      std::make_unique<VirtualObstacleDecider>(config_builder, session);
  agent_longitudinal_decider_ =
      std::make_unique<AgentLongitudinalDecider>(config_builder, session);
  expand_st_boundaries_decider_ =
      std::make_unique<ExpandStBoundariesDecider>(config_builder, session);
  closest_in_path_vehicle_decider_ =
      std::make_unique<ClosestInPathVehicleDecider>(config_builder, session);
  cipv_lost_prohibit_start_decider_ =
      std::make_unique<CipvLostProhibitStartDecider>(config_builder, session);
  cipv_lost_prohibit_acceleration_decider_ =
      std::make_unique<CipvLostProhibitAccelerationDecider>(config_builder,
                                                            session);
  st_graph_input_ =
      std::make_shared<speed::StGraphInput>(config_builder, session);
  st_graph_ = std::make_shared<speed::STGraph>();
  st_graph_helper_ = std::make_shared<speed::StGraphHelper>(*st_graph_);
  st_graph_searcher_ =
      std::make_unique<StGraphSearcher>(config_builder, session);
  truck_longitudinal_avoid_decider_ =
      std::make_unique<TruckLongitudinalAvoidDecider>(config_builder, session);
  agent_headway_decider_ =
      std::make_unique<AgentHeadwayDecider>(config_builder, session);
  longitudinal_decision_decider_ =
      std::make_unique<LongitudinalDecisionDecider>(config_builder, session);
  speed_limit_decider_ =
      std::make_unique<SpeedLimitDecider>(config_builder, session);
  long_ref_path_decider_ =
      std::make_unique<LongRefPathDecider>(config_builder, session);
  scc_lon_behavior_planner_ =
      std::make_unique<SccLonBehaviorPlanner>(config_builder, session);
  scc_longitudinal_motion_planner_ =
      std::make_unique<SccLongitudinalMotionPlannerV3>(config_builder, session);
  result_trajectory_generator_ =
      std::make_unique<ResultTrajectoryGenerator>(config_builder, session);
}

bool LongTimeTaskPipelineV3::Run() {
  bool ok = traffic_light_decider_->Execute();
  if (!ok) {
    AddErrorInfo(traffic_light_decider_->Name());
    return false;
  }

  ok = ego_lane_road_right_decider_->Execute();
  if (!ok) {
    AddErrorInfo(ego_lane_road_right_decider_->Name());
    return false;
  }

  ok = lane_change_decider_->Execute();
  if (!ok) {
    AddErrorInfo(lane_change_decider_->Name());
    return false;
  }

  ok = speed_adjust_decider_->Execute();
  if (!ok) {
    AddErrorInfo(speed_adjust_decider_->Name());
    return false;
  }

  ok = lateral_offset_decider_->Execute();
  if (!ok) {
    AddErrorInfo(lateral_offset_decider_->Name());
    return false;
  }

  // cailiu2 的gap selector将来可以挪至下方
  ok = gap_selector_decider_->Execute();
  if (!ok) {
    AddErrorInfo(gap_selector_decider_->Name());
    return false;
  }

  ok = general_lateral_decider_->Execute();
  if (!ok) {
    AddErrorInfo(general_lateral_decider_->Name());
    return false;
  }

  ok = lateral_motion_planner_->Execute();
  if (!ok) {
    AddErrorInfo(lateral_motion_planner_->Name());
    return false;
  }

  // --↓↓↓↓↓↓--long behavior--↓↓↓↓↓↓--
  ok = virtual_obstacle_decider_->Execute();
  if (!ok) {
    AddErrorInfo(virtual_obstacle_decider_->Name());
    return false;
  }

  ok = agent_longitudinal_decider_->Execute();
  if (!ok) {
    AddErrorInfo(agent_longitudinal_decider_->Name());
    return false;
  }

  // auto *mutable_agent_manager = session_->mutable_environmental_model()
  //                                   ->get_dynamic_world()
  //                                   ->mutable_agent_manager();
  // planning_data->mutable_dynamic_world()->mutable_agent_manager();

  //-------------CIPV相关-----------------------
  // const bool last_cipv_lost =
  //     planning_data->history_info().cipv_lost_prohibit_start_decider_output().cipv_lost();
  // if (last_cipv_lost) {
  //   // insert last virtual agent
  //   cp_common::agent::Agent cipv_lost_virtual_agent;
  //   cp::cp_planning::CipvLostProhibitStartDecider::MakeVirtualAgentForCipvLost(
  //       planning_data->time_aligned_ego_state(), &cipv_lost_virtual_agent);
  //   std::unordered_map<int32_t, cp_common::agent::Agent> agent_table;
  //   agent_table.insert(std::pair<int32_t, cp_common::agent::Agent>(
  //       cipv_lost_virtual_agent.agent_id(), cipv_lost_virtual_agent));
  //   mutable_agent_manager->Append(agent_table);
  // }

  // 构建st input
  double time_start = IflyTime::Now_ms();
  st_graph_input_->Update();
  ok = st_graph_->Init(st_graph_input_);
  auto planning_context = session_->mutable_planning_context();
  planning_context->set_st_graph(st_graph_);
  planning_context->set_st_graph_helper(st_graph_helper_);
  double time_end = IflyTime::Now_ms();
  if (!ok) {
    LOG_ERROR("st graph init error");
    return false;
  }
  JSON_DEBUG_VALUE("construct_st_graph_cost", time_end - time_start);

  ok = expand_st_boundaries_decider_->Execute();
  if (!ok) {
    AddErrorInfo(expand_st_boundaries_decider_->Name());
    return false;
  }

  ok = closest_in_path_vehicle_decider_->Execute();
  if (!ok) {
    AddErrorInfo(closest_in_path_vehicle_decider_->Name());
    return false;
  }

  ok = cipv_lost_prohibit_start_decider_->Execute();
  if (!ok) {
    AddErrorInfo(cipv_lost_prohibit_start_decider_->Name());
    return false;
  }

  ok = cipv_lost_prohibit_acceleration_decider_->Execute();
  if (!ok) {
    AddErrorInfo(cipv_lost_prohibit_acceleration_decider_->Name());
    return false;
  }

  time_start = IflyTime::Now_ms();
  ok = st_graph_searcher_->Execute();
  time_end = IflyTime::Now_ms();
  if (!ok) {
    AddErrorInfo(st_graph_searcher_->Name());
    return false;
  }
  JSON_DEBUG_VALUE("st_graph_searcher_cost", time_end - time_start);

  ok = truck_longitudinal_avoid_decider_->Execute();
  if (!ok) {
    AddErrorInfo(truck_longitudinal_avoid_decider_->Name());
    return false;
  }

  // ok = gap_selector_decider_->Execute();
  // if (!ok) {
  //   AddErrorInfo(gap_selector_decider_->Name());
  //   return false;
  // }

  ok = agent_headway_decider_->Execute();
  if (!ok) {
    AddErrorInfo(agent_headway_decider_->Name());
    return false;
  }

  ok = longitudinal_decision_decider_->Execute();
  if (!ok) {
    AddErrorInfo(longitudinal_decision_decider_->Name());
    return false;
  }

  ok = speed_limit_decider_->Execute();
  if (!ok) {
    AddErrorInfo(speed_limit_decider_->Name());
    return false;
  }

  ok = long_ref_path_decider_->Execute();
  if (!ok) {
    AddErrorInfo(long_ref_path_decider_->Name());
    return false;
  }
  // --↑↑↑↑↑↑--long behavior--↑↑↑↑↑↑--

  // ------ long motion planner ------
  ok = scc_longitudinal_motion_planner_->Execute();
  if (!ok) {
    AddErrorInfo(scc_longitudinal_motion_planner_->Name());
    return false;
  }

  ok = result_trajectory_generator_->Execute();
  if (!ok) {
    AddErrorInfo(result_trajectory_generator_->Name());
    return false;
  }

  return true;
}

}  // namespace planning
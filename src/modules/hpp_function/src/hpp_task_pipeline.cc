#include "hpp_task_pipeline.h"

namespace planning {

HppTaskPipeline::HppTaskPipeline(const EgoPlanningConfigBuilder *config_builder,
                                 framework::Session *session)
    : BaseTaskPipeline(config_builder, session) {
  // Lateral
  lane_change_decider_ =
      std::make_unique<LaneChangeDecider>(config_builder, session);
  lateral_obstacle_decider_ =
      std::make_unique<HppLateralObstacleDecider>(config_builder, session);
  hpp_general_lateral_decider_ =
      std::make_unique<HppGeneralLateralDecider>(config_builder, session);
  lateral_motion_planner_ =
      std::make_unique<LateralMotionPlanner>(config_builder, session);

  // HPP Specific
  hpp_stop_decider_ =
      std::make_unique<HppStopDecider>(config_builder, session);
  hpp_obstacle_preprocess_decider_ =
      std::make_unique<HppObstaclePreprocessDecider>(config_builder, session);

  // V3 Longitudinal Pipeline
  stop_destination_decider_ =
      std::make_unique<StopDestinationDecider>(config_builder, session);
  mrc_brake_decider_ =
      std::make_unique<MRCBrakeDecider>(config_builder, session);
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

  // ST Graph Init
  st_graph_input_ =
      std::make_shared<speed::StGraphInput>(config_builder, session);
  st_graph_ = std::make_shared<speed::STGraph>();
  st_graph_helper_ = std::make_shared<speed::StGraphHelper>(*st_graph_);

  st_graph_searcher_ =
      std::make_unique<StGraphSearcher>(config_builder, session);
  parallel_longitudinal_avoid_decider_ =
      std::make_unique<ParallelLongitudinalAvoidDecider>(config_builder,
                                                         session);
  agent_headway_decider_ =
      std::make_unique<AgentHeadwayDecider>(config_builder, session);
  longitudinal_decision_decider_ =
      std::make_unique<LongitudinalDecisionDecider>(config_builder, session);
  hpp_speed_limit_decider_ =
      std::make_unique<HPPSpeedLimitDecider>(config_builder, session);
  start_stop_decider_ =
      std::make_unique<StartStopDecider>(config_builder, session);
  long_ref_path_decider_ =
      std::make_unique<LongRefPathDecider>(config_builder, session);

  // V3 Motion Planner
  scc_longitudinal_motion_planner_ =
      std::make_unique<SccLongitudinalMotionPlannerV3>(config_builder, session);

  // Post Processing
  result_trajectory_generator_ =
      std::make_unique<ResultTrajectoryGenerator>(config_builder, session);
  parking_switch_decider_ =
      std::make_unique<ParkingSwitchDecider>(config_builder, session);
}

bool HppTaskPipeline::Run() {
  auto time1 = IflyTime::Now_ms();
  bool ok = lane_change_decider_->Execute();
  if (!ok) {
    AddErrorInfo(lane_change_decider_->Name());
    return false;
  }
  auto time2 = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("LaneChangeDeciderTime", time2 - time1);

  ok = lateral_obstacle_decider_->Execute();
  if (!ok) {
    AddErrorInfo(lateral_obstacle_decider_->Name());
    return false;
  }
  auto time3 = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("LateralObstacleDeciderTime", time3 - time2);

  ok = hpp_general_lateral_decider_->Execute();
  if (!ok) {
    AddErrorInfo(hpp_general_lateral_decider_->Name());
    return false;
  }
  auto time4 = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("HppGeneralLateralDeciderTime", time4 - time3);

  ok = lateral_motion_planner_->Execute();
  if (!ok) {
    AddErrorInfo(lateral_motion_planner_->Name());
    return false;
  }
  auto time5 = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("LateralMotionPlannerTime", time5 - time4);

  ok = hpp_stop_decider_->Execute();
  if (!ok) {
    AddErrorInfo(hpp_stop_decider_->Name());
    return false;
  }
  auto time5_5 = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("HppStopDeciderTime", time5_5 - time5);

  // HPP Obstacle Preprocessing (Ground Lines -> Virtual Agents, Dynamic Obs Fix)
  ok = hpp_obstacle_preprocess_decider_->Execute();
  if (!ok) {
    AddErrorInfo(hpp_obstacle_preprocess_decider_->Name());
    return false;
  }
  auto time5_6 = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("HppObstaclePreprocessDeciderTime", time5_6 - time5_5);

  // --↓↓↓↓↓↓-- V3 Long Behavior --↓↓↓↓↓↓--
  ok = stop_destination_decider_->Execute();
  if (!ok) {
    AddErrorInfo(stop_destination_decider_->Name());
    return false;
  }

  ok = mrc_brake_decider_->Execute();
  if (!ok) {
    AddErrorInfo(mrc_brake_decider_->Name());
    return false;
  }

  ok = agent_longitudinal_decider_->Execute();
  if (!ok) {
    AddErrorInfo(agent_longitudinal_decider_->Name());
    return false;
  }

  // Build ST Input and Graph
  double time_start = IflyTime::Now_ms();
  st_graph_input_->Update();  // Update inputs (including new virtual agents)
  ok = st_graph_->Init(st_graph_input_);
  auto planning_context = session_->mutable_planning_context();
  planning_context->set_st_graph(st_graph_);
  planning_context->set_st_graph_helper(st_graph_helper_);
  double time_end = IflyTime::Now_ms();
  if (!ok) {
    ILOG_ERROR << "st graph init error";
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

  ok = parallel_longitudinal_avoid_decider_->Execute();
  if (!ok) {
    AddErrorInfo(parallel_longitudinal_avoid_decider_->Name());
    return false;
  }

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

  ok = hpp_speed_limit_decider_->Execute();
  if (!ok) {
    AddErrorInfo(hpp_speed_limit_decider_->Name());
    return false;
  }

  ok = start_stop_decider_->Execute();
  if (!ok) {
    AddErrorInfo(start_stop_decider_->Name());
    return false;
  }

  ok = long_ref_path_decider_->Execute();
  if (!ok) {
    AddErrorInfo(long_ref_path_decider_->Name());
    return false;
  }
  // --↑↑↑↑↑↑-- V3 Long Behavior --↑↑↑↑↑↑--

  // V3 Motion Planner
  ok = scc_longitudinal_motion_planner_->Execute();
  if (!ok) {
    AddErrorInfo(scc_longitudinal_motion_planner_->Name());
    return false;
  }
  auto time7 = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("SccLongitudinalMotionPlannerTime", time7 - time5_6);

  // Post Processing
  ok = result_trajectory_generator_->Execute();
  if (!ok) {
    AddErrorInfo(result_trajectory_generator_->Name());
    return false;
  }
  auto time8 = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("ResultTrajectoryGeneratorTime", time8 - time7);

  ok = parking_switch_decider_->Execute();
  if (!ok) {
    AddErrorInfo(parking_switch_decider_->Name());
    return false;
  }
  auto time9 = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("ParkingSwitchDeciderTime", time9 - time8);

  return true;
}

}  // namespace planning

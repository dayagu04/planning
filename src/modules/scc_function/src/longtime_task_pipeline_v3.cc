#include "longtime_task_pipeline_v3.h"

#include <memory>

#include "behavior_planners/lane_borrow_decider/lane_borrow_deciderv3.h"
#include "behavior_planners/mrc_brake_decider/mrc_brake_decider.h"
#include "ego_planning_config.h"
#include "speed/st_graph_input.h"
#include "context/function_switch_config_context.h"

namespace planning {
LongTimeTaskPipelineV3::LongTimeTaskPipelineV3(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : BaseTaskPipeline(config_builder, session) {
  ego_lane_road_right_decider_ =
      std::make_unique<EgoLaneRoadRightDecider>(config_builder, session);
  // construction_scene_decider_ =
  //     std::make_unique<ConstructionSceneDecider>(config_builder, session);
  spatio_temporal_planner_ =
      std::make_unique<SpatioTemporalPlanner>(config_builder, session);
  lane_change_decider_ =
      std::make_unique<LaneChangeDecider>(config_builder, session);
  sample_poly_speed_adjust_decider_ =
      std::make_unique<SamplePolySpeedAdjustDecider>(config_builder, session);
  lateral_obstacle_decider_ =
      std::make_unique<SccLateralObstacleDecider>(config_builder, session);
  // lane_borrow_deciderV1_ =
  //     std::make_unique<lane_borrow_deciderV1::LaneBorrowDecider>(config_builder,
  //                                                                session);
  // lane_borrow_deciderV2_ =
  //     std::make_unique<lane_borrow_deciderV2::LaneBorrowDecider>(config_builder,
  //                                                                session);
  lane_borrow_deciderV3_ =
      std::make_unique<lane_borrow_deciderV3::LaneBorrowDecider>(config_builder,
                                                                 session);
  potential_dangerous_agent_decider_ =
      std::make_unique<PotentialDangerousAgentDecider>(config_builder, session);
  lateral_offset_decider_ =
      std::make_unique<LateralOffsetDecider>(config_builder, session);
  // gap_selector_decider_ =
  //     std::make_unique<GapSelectorDecider>(config_builder, session);
  general_lateral_decider_ =
      std::make_unique<GeneralLateralDecider>(config_builder, session);
  traffic_light_decider_ =
      std::make_unique<TrafficLightDecider>(config_builder, session);
  lateral_motion_planner_ =
      std::make_unique<SCCLateralMotionPlanner>(config_builder, session);

  // long pipeline V3
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
  speed_limit_decider_ =
      std::make_unique<SpeedLimitDecider>(config_builder, session);
  long_ref_path_decider_ =
      std::make_unique<LongRefPathDecider>(config_builder, session);
  // scc_lon_behavior_planner_ =
  //     std::make_unique<SccLonBehaviorPlanner>(config_builder, session);
  start_stop_decider_ =
      std::make_unique<StartStopDecider>(config_builder, session);
  scc_longitudinal_motion_planner_ =
      std::make_unique<SccLongitudinalMotionPlannerV3>(config_builder, session);
  result_trajectory_generator_ =
      std::make_unique<ResultTrajectoryGenerator>(config_builder, session);
  auto lane_borrow_config = config_builder->cast<EgoPlanningConfig>();
  enable_lane_borrow_deciderV3_ =
      lane_borrow_config.enable_lane_borrow_deciderV3;

  hmi_decider_ = std::make_unique<SCCHMIDecider>(config_builder, session);

  lat_lon_joint_planner_decider_ =
      std::make_unique<LatLonJointPlannerDecider>(config_builder, session);
}

bool LongTimeTaskPipelineV3::Run() {
#ifdef PlanTimeBenchmark
  double start_time, end_time;
  start_time = IflyTime::Now_ms();
#endif

  bool ok = traffic_light_decider_->Execute();
  if (!ok) {
    AddErrorInfo(traffic_light_decider_->Name());
    return false;
  }

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("traffic_light_decider_cost", end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  ok = ego_lane_road_right_decider_->Execute();
  if (!ok) {
    AddErrorInfo(ego_lane_road_right_decider_->Name());
    return false;
  }

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("ego_lane_road_right_decider_cost", end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  ok = potential_dangerous_agent_decider_->Execute();
  if (!ok) {
    AddErrorInfo(potential_dangerous_agent_decider_->Name());
    return false;
  }

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("potential_dangerous_agent_decider_cost",
                   end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  // ok = construction_scene_decider_->Execute();
  // if (!ok) {
  //   AddErrorInfo(construction_scene_decider_->Name());
  //   return false;
  // }

  // ok = sample_poly_speed_adjust_decider_->Execute();
  // if (!ok) {
  //   AddErrorInfo(sample_poly_speed_adjust_decider_->Name());
  //   return false;
  // }

  ok = lane_change_decider_->Execute();
  if (!ok) {
    AddErrorInfo(lane_change_decider_->Name());
    return false;
  }
#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("lane_change_decider_cost", end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  ok = lat_lon_joint_planner_decider_->Execute();
  if (!ok) {
    AddErrorInfo(lat_lon_joint_planner_decider_->Name());
    return false;
  }

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("lat_lon_joint_planner_decider_cost", end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  ok = sample_poly_speed_adjust_decider_->Execute();
  if (!ok) {
    AddErrorInfo(sample_poly_speed_adjust_decider_->Name());
    return false;
  }
  // ok = sample_poly_speed_adjust_decider_->Execute();
  // if (!ok) {
  //   AddErrorInfo(sample_poly_speed_adjust_decider_->Name());
  //   return false;
  // }

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("sample_poly_speed_adjust_decider_cost",
                   end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  ok = lateral_obstacle_decider_->Execute();
  if (!ok) {
    AddErrorInfo(lateral_obstacle_decider_->Name());
    return false;
  }

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("lateral_obstacle_decider_cost", end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif
  // if (enable_lane_borrow_deciderV3_) {
  //   ok = lane_borrow_deciderV2_->Execute();
  //   if (!ok) {
  //     AddErrorInfo(lane_borrow_deciderV2_->Name());
  //     return false;
  //   }
  // } else {
  //   ok = lane_borrow_deciderV1_->Execute();
  //   if (!ok) {
  //     AddErrorInfo(lane_borrow_deciderV1_->Name());
  //     return false;
  //   }
  // }
  const auto& disable_lb_from_product = static_cast<bool>(FunctionSwitchConfigContext::Instance()
                                            ->get_function_switch_config()
                                            .disable_lane_borrow);
  if (enable_lane_borrow_deciderV3_ && !disable_lb_from_product) {
    ok = lane_borrow_deciderV3_->Execute();
    if (!ok) {
      AddErrorInfo(lane_borrow_deciderV3_->Name());
      return false;
    }
  }

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("lane_borrow_decider_cost", end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  ok = lateral_offset_decider_->Execute();
  if (!ok) {
    AddErrorInfo(lateral_offset_decider_->Name());
    return false;
  }

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("lateral_offset_decider_cost", end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  // cailiu2 的gap selector将来可以挪至下方
  // ok = gap_selector_decider_->Execute();
  // if (!ok) {
  //   AddErrorInfo(gap_selector_decider_->Name());
  //   return false;
  // }

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("gap_selector_decider_cost", end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  // 时空联合规划：暂时针对车道内保持
  ok = spatio_temporal_planner_->Execute();
  if (!ok) {
    AddErrorInfo(spatio_temporal_planner_->Name());
    return false;
  }
#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("spatio_temporal_planner_cost", end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  ok = general_lateral_decider_->Execute();
  if (!ok) {
    AddErrorInfo(general_lateral_decider_->Name());
    return false;
  }

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("general_lateral_decider_cost", end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  ok = lateral_motion_planner_->Execute();
  if (!ok) {
    AddErrorInfo(lateral_motion_planner_->Name());
    return false;
  }

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("lateral_motion_planner_cost", end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  // --↓↓↓↓↓↓--long behavior--↓↓↓↓↓↓--
  ok = stop_destination_decider_->Execute();
  if (!ok) {
    AddErrorInfo(stop_destination_decider_->Name());
    return false;
  }

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("stop_destination_decider_cost", end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  ok = mrc_brake_decider_->Execute();
  if (!ok) {
    AddErrorInfo(mrc_brake_decider_->Name());
    return false;
  }

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("mrc_brake_decider_cost", end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  ok = agent_longitudinal_decider_->Execute();
  if (!ok) {
    AddErrorInfo(agent_longitudinal_decider_->Name());
    return false;
  }

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("agent_longitudinal_decider_cost", end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif
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
  // double time_start = IflyTime::Now_ms();
  st_graph_input_->Update();  // 相关障碍物 轨迹延长 初始轨迹的车身边界 box
  ok = st_graph_->Init(st_graph_input_);
  auto planning_context = session_->mutable_planning_context();
  planning_context->set_st_graph(st_graph_);
  planning_context->set_st_graph_helper(st_graph_helper_);
  // double time_end = IflyTime::Now_ms();
  if (!ok) {
    ILOG_ERROR << "st graph init error";
    return false;
  }

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("construct_st_graph_cost", end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  ok = expand_st_boundaries_decider_->Execute();
  if (!ok) {
    AddErrorInfo(expand_st_boundaries_decider_->Name());
    return false;
  }

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("expand_st_boundaries_decider_cost", end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  ok = closest_in_path_vehicle_decider_->Execute();
  if (!ok) {
    AddErrorInfo(closest_in_path_vehicle_decider_->Name());
    return false;
  }

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("closest_in_path_vehicle_decider_cost",
                   end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  ok = cipv_lost_prohibit_start_decider_->Execute();
  if (!ok) {
    AddErrorInfo(cipv_lost_prohibit_start_decider_->Name());
    return false;
  }
#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("cipv_lost_prohibit_start_decider_cost",
                   end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  ok = cipv_lost_prohibit_acceleration_decider_->Execute();
  if (!ok) {
    AddErrorInfo(cipv_lost_prohibit_acceleration_decider_->Name());
    return false;
  }

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("cipv_lost_prohibit_acceleration_decider_cost",
                   end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  ok = st_graph_searcher_->Execute();

  if (!ok) {
    AddErrorInfo(st_graph_searcher_->Name());
    return false;
  }

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("st_graph_searcher_cost", end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  ok = parallel_longitudinal_avoid_decider_->Execute();
  if (!ok) {
    AddErrorInfo(parallel_longitudinal_avoid_decider_->Name());
    return false;
  }

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("parallel_longitudinal_avoid_decider_cost",
                   end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

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
#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("agent_headway_decider_cost", end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  ok = longitudinal_decision_decider_->Execute();
  if (!ok) {
    AddErrorInfo(longitudinal_decision_decider_->Name());
    return false;
  }

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("longitudinal_decision_decider_cost", end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  ok = speed_limit_decider_->Execute();
  if (!ok) {
    AddErrorInfo(speed_limit_decider_->Name());
    return false;
  }

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("speed_limit_decider_cost", end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  ok = start_stop_decider_->Execute();
  if (!ok) {
    AddErrorInfo(start_stop_decider_->Name());
    return false;
  }

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("start_stop_decider_cost", end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  ok = long_ref_path_decider_->Execute();
  if (!ok) {
    AddErrorInfo(long_ref_path_decider_->Name());
    return false;
  }
  // --↑↑↑↑↑↑--long behavior--↑↑↑↑↑↑--

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("long_ref_path_decider_cost", end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  // ------ long motion planner ------
  ok = scc_longitudinal_motion_planner_->Execute();
  if (!ok) {
    AddErrorInfo(scc_longitudinal_motion_planner_->Name());
    return false;
  }

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("scc_longitudinal_motion_planner_cost",
                   end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  ok = result_trajectory_generator_->Execute();
  if (!ok) {
    AddErrorInfo(result_trajectory_generator_->Name());
    return false;
  }

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("result_trajectory_generator_cost", end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  hmi_decider_->Execute();

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("hmi_decider_cost", end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  return true;
}

}  // namespace planning
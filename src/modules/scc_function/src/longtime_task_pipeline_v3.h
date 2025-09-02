/**
 * @file longtime_task_pipeline_v3.h
 * @brief This pipeline relies on localization & prediction module
 **/
#pragma once

#include <memory>

#include "base_task_pipeline.h"
#include "ego_planning_config.h"
#include "session.h"
#include "st_graph/st_graph.h"
#include "st_graph/st_graph_helper.h"
#include "st_graph/st_graph_input.h"
#include "st_graph/st_graph_utils.h"
#include "tasks/behavior_planners/spatio_temporal_planner/spatio_temporal_union_planner.h"
#include "tasks/behavior_planners/agent_headway_decider/agent_headway_decider.h"
#include "tasks/behavior_planners/agent_longitudinal_decider/agent_longitudinal_decider.h"
#include "tasks/behavior_planners/cipv_lost_prohibit_acceleration_decider/cipv_lost_prohibit_acceleration_decider.h"
#include "tasks/behavior_planners/cipv_lost_prohibit_start_decider/cipv_lost_prohibit_start_decider.h"
#include "tasks/behavior_planners/closest_in_path_vehicle_decider/closest_in_path_vehicle_decider.h"
#include "tasks/behavior_planners/ego_lane_road_right_decider/ego_lane_road_right_decider.h"
#include "tasks/behavior_planners/expand_st_boundaries_decider/expand_st_boundaries_decider.h"
#include "tasks/behavior_planners/gap_selector_decider/gap_selector_decider.h"
#include "tasks/behavior_planners/general_lateral_decider/general_lateral_decider.h"
#include "tasks/behavior_planners/lane_borrow_decider/lane_borrow_deciderv1.h"
#include "tasks/behavior_planners/lane_borrow_decider/lane_borrow_deciderv2.h"
#include "tasks/behavior_planners/lane_change_decider/lane_change_decider.h"
#include "tasks/behavior_planners/lateral_obstacle_decider/lateral_obstacle_decider.h"
#include "tasks/behavior_planners/lateral_offset_decider/lateral_offset_decider.h"
#include "tasks/behavior_planners/long_ref_path_decider/long_ref_path_decider.h"
#include "tasks/behavior_planners/longitudinal_decision_decider/longitudinal_decision_decider.h"
#include "tasks/behavior_planners/sample_poly_speed_adjust_decider/sample_poly_speed_adjust_decider.h"
#include "tasks/behavior_planners/speed_limit_decider/speed_limit_decider.h"
#include "tasks/behavior_planners/st_graph_decider/st_graph_searcher.h"
#include "tasks/behavior_planners/start_stop_decider/start_stop_decider.h"
#include "tasks/behavior_planners/stop_destination_decider/stop_destination_decider.h"
#include "tasks/behavior_planners/traffic_light_decider/traffic_light_decider.h"
#include "tasks/behavior_planners/truck_longitudinal_avoid_decider/truck_longitudinal_avoid_decider.h"
#include "tasks/motion_planners/lateral_motion_planner/lateral_motion_planner.h"
#include "tasks/motion_planners/scc_lon_motion_planner_v3/scc_longitudinal_motion_planner_v3.h"
#include "tasks/trajectory_generator/result_trajectory_generator.h"
#include "tasks/motion_planners/ego_motion_preplanner/ego_motion_preplanner.h"

namespace planning {
class LongTimeTaskPipelineV3 : public BaseTaskPipeline {
 public:
  explicit LongTimeTaskPipelineV3(
      const EgoPlanningConfigBuilder *config_builder,
      framework::Session *session);

  virtual ~LongTimeTaskPipelineV3() = default;

  bool Run() override;

 private:
  std::unique_ptr<EgoLaneRoadRightDecider> ego_lane_road_right_decider_;
  std::unique_ptr<SpatioTemporalPlanner> spatio_temporal_planner_;
  std::unique_ptr<LaneChangeDecider> lane_change_decider_;
  std::unique_ptr<LateralObstacleDecider> lateral_obstacle_decider_;
  std::unique_ptr<LateralOffsetDecider> lateral_offset_decider_;
  std::unique_ptr<GapSelectorDecider> gap_selector_decider_;
  std::unique_ptr<GeneralLateralDecider> general_lateral_decider_;
  std::unique_ptr<TrafficLightDecider> traffic_light_decider_;
  std::unique_ptr<lane_borrow_deciderV2::LaneBorrowDecider>
      lane_borrow_deciderV2_;
  std::unique_ptr<lane_borrow_deciderV1::LaneBorrowDecider>
      lane_borrow_deciderV1_;
  std::unique_ptr<SamplePolySpeedAdjustDecider>
      sample_poly_speed_adjust_decider_;

  std::unique_ptr<StopDestinationDecider> stop_destination_decider_;
  std::unique_ptr<AgentLongitudinalDecider> agent_longitudinal_decider_;
  std::unique_ptr<ExpandStBoundariesDecider> expand_st_boundaries_decider_;
  std::unique_ptr<ClosestInPathVehicleDecider> closest_in_path_vehicle_decider_;
  std::unique_ptr<CipvLostProhibitStartDecider>
      cipv_lost_prohibit_start_decider_;
  std::unique_ptr<CipvLostProhibitAccelerationDecider>
      cipv_lost_prohibit_acceleration_decider_;

  std::unique_ptr<StGraphSearcher> st_graph_searcher_;

  std::unique_ptr<TruckLongitudinalAvoidDecider>
      truck_longitudinal_avoid_decider_;
  std::unique_ptr<AgentHeadwayDecider> agent_headway_decider_;
  std::unique_ptr<LongitudinalDecisionDecider> longitudinal_decision_decider_;
  std::unique_ptr<SpeedLimitDecider> speed_limit_decider_;
  std::unique_ptr<StartStopDecider> start_stop_decider_;
  std::unique_ptr<LongRefPathDecider> long_ref_path_decider_;

  // V3后续要取消这个,单独s ref生成
//   std::unique_ptr<SccLonBehaviorPlanner> scc_lon_behavior_planner_;

  // Motion Planners
  std::unique_ptr<LateralMotionPlanner> lateral_motion_planner_;
  std::unique_ptr<SccLongitudinalMotionPlannerV3>
      scc_longitudinal_motion_planner_;
  std::unique_ptr<EgoMotionPreplanner> ego_motion_preplanner_;

  std::unique_ptr<ResultTrajectoryGenerator> result_trajectory_generator_;

  // ST Graph
  std::shared_ptr<speed::StGraphInput> st_graph_input_;
  std::shared_ptr<speed::STGraph> st_graph_;
  std::shared_ptr<speed::StGraphHelper> st_graph_helper_;
  bool enable_lane_borrow_deciderV2_ = false;
};

}  // namespace planning
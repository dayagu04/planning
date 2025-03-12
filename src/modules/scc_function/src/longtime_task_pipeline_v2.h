/**
 * @file longtime_task_pipeline_v2.h
 * @brief This pipeline relies on localization module
 *        but does not rely on prediction module
 **/

#pragma once

#include <memory>

#include "base_task_pipeline.h"
#include "behavior_planners/lane_borrow_decider/lane_borrow_deciderv1.h"
#include "ego_planning_config.h"
#include "session.h"
#include "tasks/behavior_planners/agent_longitudinal_decider/agent_longitudinal_decider.h"
#include "tasks/behavior_planners/cipv_lost_prohibit_acceleration_decider/cipv_lost_prohibit_acceleration_decider.h"
#include "tasks/behavior_planners/ego_lane_road_right_decider/ego_lane_road_right_decider.h"
#include "tasks/behavior_planners/gap_selector_decider/gap_selector_decider.h"
#include "tasks/behavior_planners/general_lateral_decider/general_lateral_decider.h"
#include "tasks/behavior_planners/lane_change_decider/lane_change_decider.h"
#include "tasks/behavior_planners/lateral_obstacle_decider/lateral_obstacle_decider.h"
#include "tasks/behavior_planners/lateral_offset_decider/lateral_offset_decider.h"
#include "tasks/behavior_planners/sample_poly_speed_adjust_decider/sample_poly_speed_adjust_decider.h"
#include "tasks/behavior_planners/scc_lon_behavior_planner/scc_lon_behavior_planner.h"
#include "tasks/behavior_planners/speed_search_decider/speed_adjust_decider.h"
#include "tasks/behavior_planners/steering_wheel_stationary_decider/steering_wheel_stationary_decider.h"
#include "tasks/behavior_planners/traffic_light_decider/traffic_light_decider.h"
#include "tasks/motion_planners/lateral_motion_planner/lateral_motion_planner.h"
#include "tasks/motion_planners/scc_lon_motion_planner/scc_longitudinal_motion_planner.h"
#include "tasks/trajectory_generator/result_trajectory_generator.h"

namespace planning {

class LongTimeTaskPipelineV2 : public BaseTaskPipeline {
 public:
  explicit LongTimeTaskPipelineV2(
      const EgoPlanningConfigBuilder *config_builder,
      framework::Session *session);

  virtual ~LongTimeTaskPipelineV2() = default;

  bool Run() override;

 private:
  std::unique_ptr<EgoLaneRoadRightDecider> ego_lane_road_right_decider_;
  std::unique_ptr<LaneChangeDecider> lane_change_decider_;
  std::unique_ptr<LateralOffsetDecider> lateral_offset_decider_;
  std::unique_ptr<GapSelectorDecider> gap_selector_decider_;
  std::unique_ptr<GeneralLateralDecider> general_lateral_decider_;
  std::unique_ptr<TrafficLightDecider> traffic_light_decider_;
  std::unique_ptr<LateralMotionPlanner> lateral_motion_planner_;
  std::unique_ptr<SccLonBehaviorPlanner> scc_lon_behavior_planner_;
  std::unique_ptr<SccLongitudinalMotionPlanner>
      scc_longitudinal_motion_planner_;
  std::unique_ptr<ResultTrajectoryGenerator> result_trajectory_generator_;
  std::unique_ptr<AgentLongitudinalDecider> agent_longitudinal_decider_;
  std::unique_ptr<CipvLostProhibitAccelerationDecider>
      cipv_lost_prohibit_acceleration_decider_;
  std::unique_ptr<SpeedAdjustDecider> speed_adjust_decider_;
  std::unique_ptr<LaneBorrowDecider> lane_borrow_decider_;
  std::unique_ptr<LateralObstacleDecider> lateral_obstacle_decider_;
  std::unique_ptr<SamplePolySpeedAdjustDecider>
      sample_poly_speed_adjust_decider_;
  std::unique_ptr<SteeringWheelStationaryDecider>
      steering_wheel_stationary_decider_;
};

}  // namespace planning
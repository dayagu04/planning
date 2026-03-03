#pragma once

#include <memory>

// #include "apa_function/apa_plan_interface.h"
#include "base_task_pipeline.h"
#include "behavior_planners/hpp_switch_to_parking_decider/parking_switch_decider.h"
#include "behavior_planners/hpp_stop_decider/hpp_stop_decider.h"
#include "ego_planning_config.h"
#include "session.h"
// #include "tasks/behavior_planners/general_longitudinal_decider/general_longitudinal_decider.h"
#include "tasks/behavior_planners/hpp_general_lateral_decider/hpp_general_lateral_decider/hpp_general_lateral_decider.h"
#include "tasks/behavior_planners/lane_change_decider/lane_change_decider.h"
#include "tasks/behavior_planners/lateral_obstacle_decider/hpp_lateral_obstacle_decider/hpp_lateral_obstacle_decider.h"
#include "tasks/motion_planners/lateral_motion_planner/lateral_motion_planner.h"
// #include "tasks/motion_planners/longitudinal_motion_planner/longitudinal_motion_planner.h"
#include "tasks/trajectory_generator/result_trajectory_generator.h"

// V3 Longitudinal Includes
#include "st_graph/st_graph.h"
#include "st_graph/st_graph_helper.h"
#include "st_graph/st_graph_input.h"
#include "tasks/behavior_planners/hpp_obstacle_preprocess_decider/hpp_obstacle_preprocess_decider.h"
#include "tasks/behavior_planners/stop_destination_decider/stop_destination_decider.h"
#include "tasks/behavior_planners/mrc_brake_decider/mrc_brake_decider.h"
#include "tasks/behavior_planners/agent_longitudinal_decider/agent_longitudinal_decider.h"
#include "tasks/behavior_planners/expand_st_boundaries_decider/expand_st_boundaries_decider.h"
#include "tasks/behavior_planners/closest_in_path_vehicle_decider/closest_in_path_vehicle_decider.h"
#include "tasks/behavior_planners/cipv_lost_prohibit_start_decider/cipv_lost_prohibit_start_decider.h"
#include "tasks/behavior_planners/cipv_lost_prohibit_acceleration_decider/cipv_lost_prohibit_acceleration_decider.h"
#include "tasks/behavior_planners/st_graph_decider/st_graph_searcher.h"
#include "tasks/behavior_planners/parallel_longitudinal_avoid_decider/parallel_longitudinal_avoid_decider.h"
#include "tasks/behavior_planners/agent_headway_decider/agent_headway_decider.h"
#include "tasks/behavior_planners/longitudinal_decision_decider/longitudinal_decision_decider.h"
#include "tasks/behavior_planners/hpp_speed_limit_decider/hpp_speed_limit_decider.h"
#include "tasks/behavior_planners/start_stop_decider/start_stop_decider.h"
#include "tasks/behavior_planners/long_ref_path_decider/long_ref_path_decider.h"
#include "tasks/motion_planners/scc_lon_motion_planner_v3/scc_longitudinal_motion_planner_v3.h"

namespace planning {

class HppTaskPipeline : public BaseTaskPipeline {
 public:
  HppTaskPipeline(const EgoPlanningConfigBuilder *config_builder,
                  framework::Session *session);

  virtual ~HppTaskPipeline() = default;

  bool Run() override;

 private:
  std::unique_ptr<LaneChangeDecider> lane_change_decider_;
  std::unique_ptr<HppLateralObstacleDecider> lateral_obstacle_decider_;
  std::unique_ptr<HppGeneralLateralDecider> hpp_general_lateral_decider_;
  std::unique_ptr<LateralMotionPlanner> lateral_motion_planner_;
  std::unique_ptr<HppStopDecider> hpp_stop_decider_;
  std::unique_ptr<ParkingSwitchDecider> parking_switch_decider_;
  std::unique_ptr<HppObstaclePreprocessDecider> hpp_obstacle_preprocess_decider_;

  // V3 Longitudinal Pipeline Components
  std::unique_ptr<StopDestinationDecider> stop_destination_decider_;
  std::unique_ptr<MRCBrakeDecider> mrc_brake_decider_;
  std::unique_ptr<AgentLongitudinalDecider> agent_longitudinal_decider_;
  std::unique_ptr<ExpandStBoundariesDecider> expand_st_boundaries_decider_;
  std::unique_ptr<ClosestInPathVehicleDecider> closest_in_path_vehicle_decider_;
  std::unique_ptr<CipvLostProhibitStartDecider> cipv_lost_prohibit_start_decider_;
  std::unique_ptr<CipvLostProhibitAccelerationDecider> cipv_lost_prohibit_acceleration_decider_;
  std::unique_ptr<StGraphSearcher> st_graph_searcher_;
  std::unique_ptr<ParallelLongitudinalAvoidDecider> parallel_longitudinal_avoid_decider_;
  std::unique_ptr<AgentHeadwayDecider> agent_headway_decider_;
  std::unique_ptr<LongitudinalDecisionDecider> longitudinal_decision_decider_;
  std::unique_ptr<HPPSpeedLimitDecider> hpp_speed_limit_decider_;
  std::unique_ptr<StartStopDecider> start_stop_decider_;
  std::unique_ptr<LongRefPathDecider> long_ref_path_decider_;

  // V3 Motion Planner
  std::unique_ptr<SccLongitudinalMotionPlannerV3> scc_longitudinal_motion_planner_;

  // ST Graph
  std::shared_ptr<speed::StGraphInput> st_graph_input_;
  std::shared_ptr<speed::STGraph> st_graph_;
  std::shared_ptr<speed::StGraphHelper> st_graph_helper_;

  // Post Processing
  std::unique_ptr<ResultTrajectoryGenerator> result_trajectory_generator_;
};

}  // namespace planning

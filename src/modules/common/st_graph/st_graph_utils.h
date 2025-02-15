#pragma once

#include <vector>

#include "agent/agent.h"
#include "math/box2d.h"
#include "math/line_segment2d.h"
#include "st_boundary.h"
#include "st_graph.h"
#include "st_graph_input.h"
#include "st_point.h"
#include "trajectory1d/second_order_time_optimal_trajectory.h"
#include "utils/kd_path.h"
// #include "cp_common/trajectory/trajectory.h"
#include "dynamic_world/dynamic_world.h"
#include "virtual_lane_manager.h"
// #include "planning_data.h"

namespace planning {
namespace speed {

// The utils class for st-graph
class StGraphUtils {
 public:
  StGraphUtils() = default;
  ~StGraphUtils() = default;

  static bool IsStaticAgent(const agent::Agent& agent);

  static const agent::Agent* GetFrontAgentOfTargetLane(
      const std::shared_ptr<planning_data::DynamicWorld>& dynamic_world,
      const std::string lane_change_status,
      const std::string& lane_change_request);

  static const agent::Agent* GetRearAgentOfTargetLane(
      const std::shared_ptr<planning_data::DynamicWorld>& dynamic_world,
      const std::string lane_change_status,
      const std::string& lane_change_request);

  static double CalculateLateralBufferForNormalLaneKeeping(
      const trajectory::TrajectoryPoint& init_point,
      const double lower_lateral_buffer_m, const double upper_lateral_buffer_m,
      const double lower_speed_kph, const double upper_speed_kph);

  static double CalculateLateralBufferForTimeRange(
      const double lower_lateral_buffer_m, const double upper_lateral_buffer_m,
      const double lower_t, const double upper_t, const double t);

  static void DetermineCautionYieldDecision(
      const std::shared_ptr<StGraphInput>& st_graph_input,
      const std::string lane_change_status,
      const std::string lane_change_request,
      const std::unordered_map<int32_t, std::vector<int64_t>>&
          agent_id_st_boundaries_map,
      const std::unordered_map<int64_t, std::unique_ptr<STBoundary>>&
          boundary_id_st_boundaries_map,
      std::vector<int32_t>& caution_yield_agent_ids);

  // TODO: 待Cross决策加入后添加
  static void SetCrossingAgentCautionYieldDecision(
      const std::shared_ptr<StGraphInput>& st_graph_input,
      const std::unordered_map<int32_t, std::vector<int64_t>>&
          agent_id_st_boundaries_map,
      const std::unordered_map<int64_t, std::unique_ptr<STBoundary>>&
          boundary_id_st_boundaries_map);

  static void PreUpdateStBoundaryForLaneChange(
      const std::unordered_map<int32_t, std::vector<int64_t>>&
          agent_id_st_boundaries_map,
      const agent::Agent* front_agent_of_target,
      const agent::Agent* rear_agent_of_target, const double safety_buffer_m,
      std::unordered_map<int64_t, std::unique_ptr<STBoundary>>* const
          boundary_id_st_boundaries_map);

  static void UpdateStBoundaryForLaneChange(
      const std::unordered_map<int32_t, std::vector<int64_t>>&
          agent_id_st_boundaries_map,
      const agent::Agent* front_agent_of_target,
      const agent::Agent* rear_agent_of_target, const double safety_buffer_m,
      std::unordered_map<int64_t, std::unique_ptr<STBoundary>>* const
          boundary_id_st_boundaries_map);

  static void UpdateStBoundaryForOvertaking(
      const std::shared_ptr<StGraphInput>& st_graph_input,
      const std::unordered_map<int32_t, std::vector<int64_t>>&
          agent_id_st_boundaries_map,
      const agent::Agent* rear_agent_of_target,
      std::unordered_map<int64_t, std::unique_ptr<STBoundary>>* const
          boundary_id_st_boundaries_map);

  static int32_t GetAgentStBoundaryId(
      const agent::Agent* agent,
      const std::unordered_map<int32_t, std::vector<int64_t>>&
          agent_id_st_boundaries_map);

  static bool IsLargeAgent(const agent::Agent& agent);

  static bool CheckCandicateAgentForClosePass(
      const bool is_lane_keeping, const trajectory::TrajectoryPoint& init_point,
      const std::shared_ptr<planning_math::KDPath>& planned_path,
      const std::shared_ptr<VirtualLane>& ego_lane,
      const std::shared_ptr<VirtualLane>& agent_lane, const agent::Agent& agent,
      const StBoundaryType& type);

  static double RecalculateLateralBufferForLargeAgent(
      const trajectory::TrajectoryPoint& planning_init_point,
      const agent::AgentManager& agent_manager, const agent::Agent& agent,
      const double expand_buffer, const double small_expand_buffer,
      const double default_lateral_buffer, int32_t* const prev_st_count);

  static void CalculateAgentSLBoundary(
      const std::shared_ptr<planning_math::KDPath>& kd_path,
      const planning_math::Box2d& obs_box,
      const std::pair<double, double>& path_range, const StBoundaryType type,
      std::vector<double>* const agent_sl_boundary,
      std::vector<std::pair<int32_t, planning_math::Vec2d>>* const
          considered_corners);

  static bool CalculateSRange(
      const std::shared_ptr<planning_math::KDPath>& kd_path,
      const PathBorderQuerier& path_border_querier,
      const planning_math::Box2d& obs_box, const StBoundaryType type,
      const std::pair<double, double>& path_range,
      const std::vector<double>& agent_sl_boundary,
      std::vector<std::pair<int32_t, planning_math::Vec2d>>& considered_corners,
      const planning_math::Box2d& planning_init_point_box,
      double* const lower_s, double* const upper_s);

  static void GetIntersectiveLineSegments(
      const std::shared_ptr<planning_math::KDPath>& kd_path, const bool is_left,
      const planning_math::Box2d& obs_box,
      const std::vector<double>& agent_sl_boundary,
      std::vector<planning_math::LineSegment2d>* const intersective_segments);

  static void CalculateIntersectS(
      const std::vector<planning_math::LineSegment2d>& intersective_edges,
      const planning_math::LineSegment2d& border, const double start_s,
      bool* const has_intersect_point, double* const lower_s,
      double* const upper_s);

  static void DetermineClosetStBoundary(
      const std::unordered_map<int64_t, std::unique_ptr<STBoundary>>&
          boundary_id_st_boundaries_map,
      int64_t& closest_boundary_id, double& closest_s);

  static bool CheckAdjustLateralBufferByT(
      const trajectory::TrajectoryPoint& init_point,
      const std::shared_ptr<VirtualLaneManager>& virtual_lane_manager,
      const std::shared_ptr<VirtualLane>& ego_lane,
      const std::shared_ptr<VirtualLane>& agent_lane, const agent::Agent& agent,
      const bool is_parallel_to_ego_lane, const bool is_lane_keeping);

  static bool NeedDynamicBufferForTimeRange(const agent::Agent& agent);

  static bool LinearExtendTrajectory(const trajectory::Trajectory& trajectory,
                                     const double time,
                                     trajectory::TrajectoryPoint* point);

  static double AdjustLateralBufferByT(
      const trajectory::TrajectoryPoint& agent_point,
      const double default_buffer,
      const std::shared_ptr<VirtualLane>& ptr_agent_lane);

  static bool CheckLonFarPositionSTBoundary(
      const agent::Agent& agent,
      const std::vector<std::pair<STPoint, STPoint>>& st_point_pairs,
      const std::shared_ptr<StGraphInput>& st_graph_input,
      const bool is_parallel, const std::shared_ptr<VirtualLane>& ego_lane,
      const std::shared_ptr<VirtualLane>& ptr_agent_lane,
      const std::shared_ptr<VirtualLaneManager>& virtual_lane_manager);

  static bool IsLargeAgentByLength(const agent::Agent& agent);

  static bool CheckLateralFarCutinAgent(
      const agent::Agent& agent,
      const std::vector<std::pair<STPoint, STPoint>> st_point_pairs,
      const std::shared_ptr<StGraphInput>& st_graph_input);

  static bool CheckLateralFarCutinAgentIsLonSafe(
      const agent::Agent& agent,
      const std::vector<std::pair<STPoint, STPoint>> st_point_pairs,
      const trajectory::TrajectoryPoint& planning_init_point,
      const std::shared_ptr<StGraphInput>& st_graph_input);

  static SecondOrderTimeOptimalTrajectory
  GenerateMaxDecelerationCurveByAgentVel(
      const double agent_vel, const trajectory::TrajectoryPoint& init_point);

  static bool IsBoundaryAboveRearTargetBoundary(const STBoundary& st_boundary,
                                                const STBoundary*);

  static planning_math::Box2d MakeEgoBox(
      const std::shared_ptr<planning_math::KDPath>& planned_kd_path,
      const double s);
};

}  // namespace speed
}  // namespace planning
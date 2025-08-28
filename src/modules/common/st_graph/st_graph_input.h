#pragma once

#include <utility>
#include <vector>

#include "agent/agent.h"
#include "agent/agent_manager.h"
#include "dynamic_world/dynamic_world.h"
#include "lat_lon_vehicle_motion_simulator.h"
#include "st_graph/path_border_querier.h"
#include "trajectory/trajectory_point.h"
#include "trajectory1d/second_order_time_optimal_trajectory.h"
#include "utils/kd_path.h"
#include "virtual_lane.h"
#include "virtual_lane_manager.h"

namespace planning {
namespace speed {

class StGraphInput {
 public:
  StGraphInput(const EgoPlanningConfigBuilder* config_builder,
               planning::framework::Session* session);

  StGraphInput() = default;

  ~StGraphInput() = default;

  void Update();

  void GetAgentOfTargetLane(
      const std::shared_ptr<planning_data::DynamicWorld>& dynamic_world,
      const bool is_lane_keeping);

  void MakeBuffer(const bool is_lane_keeping, const STGraphConfig& config);

  void MakePlanningInitPointBox();

  void FilterAgentsByDecisionType(
      const std::vector<std::shared_ptr<agent::Agent>>& origin_agents);

  void ExtendProcessedPath(
      const bool is_lane_keeping, const bool is_lane_borrow,
      const std::shared_ptr<planning_math::KDPath>& dp_path_coord,
      const std::shared_ptr<planning_math::KDPath>& lane_fusion_ego_center_lane,
      const std::shared_ptr<planning_math::KDPath>& planned_path);

  void ForwardExtendPlannedPath(
      const bool is_lane_keeping, const bool is_lane_borrow,
      const std::shared_ptr<planning_math::KDPath>& dp_path_coord,
      const std::shared_ptr<planning_math::KDPath>& lane_fusion_ego_center_lane,
      const std::shared_ptr<planning_math::KDPath>& planned_path,
      std::vector<planning_math::PathPoint>* const ptr_path_points);

  void ForwardExtendPlannedPathWithEgoLane(
      const std::shared_ptr<planning_math::KDPath>& lane_fusion_ego_center_lane,
      const std::shared_ptr<planning_math::KDPath>& planned_path,
      const double desired_path_length,
      std::vector<planning_math::PathPoint>* const ptr_path_points);

  std::shared_ptr<planning_math::KDPath> ForwardExtendPlannedPathWithDpPath(
      const std::shared_ptr<planning_math::KDPath>& dp_path_coord,
      const std::shared_ptr<planning_math::KDPath>& planned_path);

  void ForwardLinearlyExtendPlannedPath(
      const std::shared_ptr<planning_math::KDPath>& planned_path,
      const double desired_path_length,
      std::vector<planning_math::PathPoint>* const ptr_path_points);

  void BackwardExtendPoints(
      const std::shared_ptr<planning_math::KDPath>& planned_path,
      std::vector<planning_math::PathPoint>* const ptr_path_points);

  void MakePathBorderQuerier(
      const std::shared_ptr<planning_math::KDPath>& planned_path);

  void PlanningInitPointToTrajectoryPoint(const PlanningInitPoint& init_point);

  std::pair<int32_t, int32_t> MakeTargetLaneFrontRearAgents(
      framework::Session* session);

  const double lat_buffer() const;

  const std::string lane_change_request() const;

  const std::string lane_change_status() const;

  const double GetSuitableLateralBuffer(const agent::Agent& agent) const;

  const double GetSuitableLonBuffer(const agent::Agent& agent) const;

  const double start_absolute_time() const;

  const int32_t reserve_num() const;

  const std::vector<std::shared_ptr<agent::Agent>>& filtered_agents() const;

  std::shared_ptr<agent::AgentManager> mutable_agent_manager();

  const std::pair<double, double>& path_range() const;

  const std::pair<double, double>& time_range() const;

  const std::shared_ptr<planning_math::KDPath> processed_path() const;

  const PathBorderQuerier* path_border_querier() const;

  const trajectory::TrajectoryPoint& planning_init_point() const;

  const trajectory::TrajectoryPoint& time_aligned_ego_state() const;

  const agent::Agent* front_agent_of_target() const;

  const agent::Agent* rear_agent_of_target() const;

  double front_agent_lower_s_safety_buffer_for_lane_change() const;

  double large_agent_expand_param_for_consistency() const;

  double large_agent_small_expand_param_for_consistency() const;

  // const planning_data::DecisionOutput& decision_output() const;

  bool IsParallelToEgoLane(const int32_t lane_id) const;

  const std::shared_ptr<VirtualLaneManager> ptr_virtual_lane_manager() const;

  const std::shared_ptr<VirtualLane> ego_lane() const;

  bool is_lane_keeping() const;

  bool is_lane_change_cancle() const { return is_lane_change_cancle_; }

  bool is_in_lane_borrow_status() const { return is_in_lane_borrow_status_; }

  const SecondOrderTimeOptimalTrajectory* max_acceleration_curve() const;

  bool enable_backward_extend_st_boundary() const;

  double backward_extend_time_s() const;

  const planning_math::Box2d& planning_init_point_box() const;

  const bool is_rads_scene() const;

  std::shared_ptr<SecondOrderTimeOptimalTrajectory>
  GenerateMaxAccelerationCurve(
      const trajectory::TrajectoryPoint& planning_init_point,
      const std::shared_ptr<EgoStateManager>& ego_state_manager);

  const std::shared_ptr<
      simulator::LatLonVehicleMotionSimulator::SimulationResult>
  ego_motion_simulation_result() const {
    return ego_motion_simulation_result_ptr_;
  }

  void Reset();

 private:
  planning::framework::Session* session_ = nullptr;
  STGraphConfig config_;
  trajectory::TrajectoryPoint planning_init_point_;
  trajectory::TrajectoryPoint time_aligned_ego_state_;
  // PlanningInitPoint planning_init_point_;
  // PlanningInitPoint time_aligned_ego_state_;
  std::shared_ptr<VirtualLaneManager> virtual_lane_manager_;
  //   std::shared_ptr<VirtualLane> ego_lane_;
  VehicleParam vehicle_param_;
  bool is_lane_keeping_;
  bool is_lane_change_cancle_ = false;
  bool is_in_lane_borrow_status_ = false;
  std::shared_ptr<agent::AgentManager> mutable_agent_manager_;
  std::shared_ptr<SecondOrderTimeOptimalTrajectory> max_acceleration_curve_ =
      nullptr;
  std::pair<double, double> path_range_;
  std::pair<double, double> time_range_;
  std::shared_ptr<planning_math::KDPath> processed_path_ = nullptr;
  std::vector<std::shared_ptr<agent::Agent>> filtered_agents_;
  const agent::Agent* front_agent_of_target_ = nullptr;
  const agent::Agent* rear_agent_of_target_ = nullptr;
  // The path border that is expanded by veh_width
  std::shared_ptr<PathBorderQuerier> path_border_querier_ = nullptr;
  std::unordered_map<int32_t, bool> is_parallel_lane_map_;

  // lateral buffer for lane change and normal lane keeping
  // (except for large agent)
  double lat_buffer_ = 0.0;
  // lateral buffer for large agent when lane keeping
  double lat_buffer_for_large_agent_ = 0.0;
  double large_agent_expand_param_for_consistency_ = 0.0;
  double large_agent_small_expand_param_for_consistency_ = 0.0;
  double lat_buffer_for_cone_ = 0.0;
  double lon_buffer_for_large_heading_diff_ = 0.0;
  double person_lat_buffer_ = 0.0;
  double person_lon_buffer_ = 0.0;
  double bycicle_lat_buffer_ = 0.0;
  double bycicle_lon_buffer_ = 0.0;
  double tricycle_lat_buffer_ = 0.0;
  double tricycle_lon_buffer_ = 0.0;
  double reverse_vehicle_lat_buffer_m_ = 0.0;

  bool enable_backward_extend_st_boundary_ = false;
  double backward_extend_time_s_ = 0.0;
  planning_math::Box2d planning_init_point_box_;

  // ego motion preplanner simulation result
  std::shared_ptr<simulator::LatLonVehicleMotionSimulator::SimulationResult>
      ego_motion_simulation_result_ptr_;
};
}  // namespace speed
}  // namespace planning
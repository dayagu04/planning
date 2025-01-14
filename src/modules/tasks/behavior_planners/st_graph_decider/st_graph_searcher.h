#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "behavior_planners/closest_in_path_vehicle_decider/closest_in_path_vehicle_decider_output.h"
#include "common/st_graph/st_boundary.h"
#include "common/st_graph/st_graph_input.h"
#include "ego_planning_config.h"
#include "session.h"
#include "st_graph_searcher.pb.h"
#include "st_search_input.h"
#include "st_search_node.h"
#include "tasks/task.h"
#include "yield_front_vehicle_safe_function/yield_front_vehicle_safe_function.h"

namespace planning {

class StGraphSearcher : public Task {
  using CIPVInfo = ClosestInPathVehicleDeciderOutput;

 public:
  enum class AStarSearchStyle {
    ORDINARY = 0,
    RADICAL = 1,
  };

  struct AStarSearchConfig {
    double planning_time_horizon = 0.1;
    double max_accel_limit = 5.0;
    double min_accel_limit = -6.0;
    double max_jerk_limit = 10.0;
    double min_jerk_limit = -10.0;
    double accel_sample_num = 20.0;
    double s_step = 0.5;
    double t_step = 0.5;
    double vel_step = 0.4;
    double max_search_time = 0.1;
  };

 public:
  StGraphSearcher(const EgoPlanningConfigBuilder* config_builder,
                  framework::Session* session);
  virtual ~StGraphSearcher() = default;

  bool Execute() override;

 private:
  bool SearchStPath(std::vector<StSearchNode>* const searched_path,
                    AStarSearchStyle search_style);

  void SetSearchConfigBySearchStyle(AStarSearchStyle search_style);

  StSearchNode GenerateStartNode(
      const trajectory::TrajectoryPoint planning_init_point,
      const StSearchInput& input_info) const;

  bool IsReachGoal(const StSearchInput& input_info,
                   const StSearchNode& node) const;

  std::vector<StSearchNode> GenerateSuccessorNodes(
      const StSearchInput& input_info, const StSearchNode& current_node,
      const std::unordered_set<int64_t>& target_lane_agent_boundaries) const;

  std::unordered_set<int64_t> GetTargetLaneRearAgentStBoundaries() const;

  void SetSearchFailSafe() const;

  bool CheckYieldBackVehicle(
      const std::unordered_map<int64_t, speed::STBoundary::DecisionType>&
          decision_table) const;

  bool CheckOvertakeFrontVehicleOnTargetLane(
      const std::unordered_map<int64_t, speed::STBoundary::DecisionType>&
          decision_table) const;

  bool CheckIfFrontVehcileSafe();

  double GetAgentMinPredictionSpeed(const int64_t agent_id) const;

  void AddStGraphSearcherDataToProto(
      const std::vector<StSearchNode> st_search_path);

  // compute cost and h_cost
  void ComputeNodeCost(const StSearchInput& input_info,
                       const StSearchNode& current_node,
                       StSearchNode* const succ_node) const;

  double ComputeYieldCost(const StSearchInput& input_info,
                          const StSearchNode& node) const;

  double ComputeOvertakeCost(const StSearchInput& input_info,
                             const StSearchNode& node) const;

  double ComputeVelocityCost(const StSearchInput& input_info,
                             const StSearchNode& node) const;

  double ComputeAccelerationCost(const StSearchInput& input_info,
                                 const StSearchNode& current_node,
                                 const StSearchNode& node) const;

  double ComputeAccelerationSignCost(const StSearchInput& input_info,
                                     const StSearchNode& current_node,
                                     const StSearchNode& node) const;

  double ComputeJerkCost(const StSearchInput& input_info,
                         const StSearchNode& node) const;

  double ComputeLengthCost(const StSearchInput& input_info,
                           const StSearchNode& current_node,
                           const StSearchNode& node) const;

  double ComputeVirtualYieldCost(const StSearchInput& input_info,
                                 const StSearchNode& node) const;

  double ComputeHeuristicCost(const StSearchInput& input_info,
                              const StSearchNode& node) const;

  void SetStSearchFailSafeDecisionTable(
      const std::unordered_map<int64_t, std::unique_ptr<speed::STBoundary>>&
          boundary_id_st_boundaries_map,
      const std::unordered_map<int32_t, std::vector<int64_t>>&
          agent_id_st_boundaries_map,
      const speed::StGraphInput& st_graph_input, const CIPVInfo& cipv_info,
      std::unordered_map<int64_t, speed::STBoundary::DecisionType>*
          succ_decision_table) const;

  void AddAStarSearchCostDebugInfo(
      std::vector<StSearchNode>* const searched_path) const;

  StGraphSearcherConfig config_;
  planning::common::StGraphSearcher st_graph_searcher_pb_;
  std::shared_ptr<YieldFrontVehicleSafeFunction>
      yield_front_vehicle_safe_utils_;

 private:
  // search relevance
  StSearchNode farthest_node_;
  std::array<AStarSearchStyle, 2> search_style_context_;
  AStarSearchConfig search_config_;
};

}  // namespace planning
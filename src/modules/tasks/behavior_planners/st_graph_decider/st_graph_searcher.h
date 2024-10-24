#pragma once

#include "ego_planning_config.h"
#include "st_search_input.h"
#include "st_search_node.h"
#include "tasks/task.h"
#include "yield_front_vehicle_safe_function/yield_front_vehicle_safe_function.h"

namespace planning {

class StGraphSearcher : public Task {
 public:
  StGraphSearcher(const EgoPlanningConfigBuilder* config_builder,
                  framework::Session* session);
  virtual ~StGraphSearcher() = default;

  bool Execute() override;

 private:
  bool SearchStPath(std::vector<StSearchNode>* const searched_path);

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

  StGraphSearcherConfig config_;
  std::shared_ptr<YieldFrontVehicleSafeFunction>
      yield_front_vehicle_safe_utils_;
};

}  // namespace planning
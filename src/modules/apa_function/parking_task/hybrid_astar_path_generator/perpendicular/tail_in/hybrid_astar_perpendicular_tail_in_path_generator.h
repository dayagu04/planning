#pragma once
#include <memory>

#include "hybrid_astar_context.h"
#include "hybrid_astar_perpendicular_path_generator.h"

namespace planning {
namespace apa_planner {

class HybridAStarPerpendicularTailInPathGenerator
    : public HybridAStarPerpendicularPathGenerator {
 public:
  HybridAStarPerpendicularTailInPathGenerator() = default;
  HybridAStarPerpendicularTailInPathGenerator(
      const std::shared_ptr<CollisionDetectorInterface>&
          col_det_interface_ptr) {
    col_det_interface_ptr_ = col_det_interface_ptr;
  }

  virtual const bool Update() override;

  virtual const bool UpdateOnce(
      const PathColDetBuffer& path_col_det_buffer) override;

 private:
  SearchConfigSnapshot BuildSearchConfigSnapshot() const;
  SearchConfigSnapshot PrepareSearchPhases();
  void ModifyPreSearchConfig();
  PreSearchPhaseOutcome HandlePreSearchPhase();
  virtual const bool RunFormalSearch(
      const SearchConfigSnapshot& snapshot) override;
  void RestoreFormalSearchConfig(const SearchConfigSnapshot& snapshot);
  void InitInterestingArea();
  PathColDetBuffer BuildPreSearchPathColDetBuffer() const;
  void TryDecideCulDeSac(const PathColDetBuffer& pre_path_col_det_buffer);
  const bool RunPreSearch(const PathColDetBuffer& pre_path_col_det_buffer);
  void ConfigureSearchBudget();
  void ConfigureBaseAnalyticExpansionRequest(
      AnalyticExpansionRequest& analytic_expansion_request);
  void ConfigureAnalyticExpansionRequestForNewNode(
      const Node3d& new_node,
      AnalyticExpansionRequest& analytic_expansion_request) const;

  void UpdateCulDeSacLimitByNewNode(const Node3d& new_node);
  Node3d* FindOrAllocateSearchNode(size_t new_node_global_id,
                                   AstarNodeVisitedType& vis_type);
  void ActivateNodeInOpenSet(const Node3d& new_node, Node3d* node_in_pool);
  void InsertNewNodeIntoSearch(const Node3d& new_node, Node3d* node_in_pool);
  void RefreshNodeInOpenSet(const Node3d& new_node, Node3d* node_in_pool);
  void ReopenClosedNodeInSearch(const Node3d& new_node, Node3d* node_in_pool);

  virtual void UpdatePoseBoundary() override;

  virtual void CalcNodeGCost(Node3d* current_node, Node3d* next_node) override;

  virtual void CalcNodeHCost(
      Node3d* current_node, Node3d* next_node,
      const AnalyticExpansionRequest& analytic_expansion_request) override;

  virtual void ChooseBestCurveNode(
      const std::vector<CurveNode>& curve_node_to_goal_vec,
      const AnalyticExpansionType analytic_expansion_type,
      const bool consider_obs_dist, const PathColDetBuffer& safe_buffer,
      CurveNode& best_curve_node_to_goal) override;

  virtual const float CalcGearChangePoseCost(
      const common_math::PathPt<float>& gear_switch_pose, AstarPathGear gear,
      const float gear_switch_penalty, const float length_penalty) override;
};
}  // namespace apa_planner
}  // namespace planning

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
  link_pt_line::LinkPtLineInput<float> BuildDefaultLPLInput(
      const geometry_lib::PathPoint& end_pose) const;
  void ConfigureSearchBudget(size_t& find_success_curve_min_count,
                             double& find_success_curve_max_time);
  void PrepareLPLInputForCurrentNode(
      const Node3d* current_node,
      link_pt_line::LinkPtLineInput<float>& lpl_input) const;
  void PrepareLPLInputForNewNode(
      const Node3d& new_node,
      link_pt_line::LinkPtLineInput<float>& lpl_input) const;

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

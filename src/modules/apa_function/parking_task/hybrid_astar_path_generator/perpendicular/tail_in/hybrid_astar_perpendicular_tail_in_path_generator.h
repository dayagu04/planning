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

 private:
  PathColDetBuffer BuildPreSearchPathColDetBuffer() const;
  SearchConfigSnapshot BuildSearchConfigSnapshot() const;
  SearchConfigSnapshot PrepareSearchPhases();
  void ModifyPreSearchConfig();
  PreSearchPhaseOutcome HandlePreSearchPhase();
  void InitInterestingArea();
  void TryDecideCulDeSac(const PathColDetBuffer& pre_path_col_det_buffer);
  const bool RunPreSearch(const PathColDetBuffer& pre_path_col_det_buffer);
  void RestoreFormalSearchConfig(const SearchConfigSnapshot& snapshot);

  virtual const bool RunFormalSearch(
      const SearchConfigSnapshot& snapshot) override;

  virtual void ConfigureSearchBudget() override;
  virtual void CalcNodeGCost(Node3d* current_node, Node3d* next_node) override;

  virtual void ChooseBestCurveNode(
      std::vector<CurveNode>& curve_node_to_goal_vec,
      CurveNode& best_curve_node_to_goal) override;

  virtual const float CalcGearChangePoseCost(
      const common_math::PathPt<float>& gear_switch_pose, AstarPathGear gear,
      const float gear_switch_penalty, const float length_penalty) override;
};
}  // namespace apa_planner
}  // namespace planning

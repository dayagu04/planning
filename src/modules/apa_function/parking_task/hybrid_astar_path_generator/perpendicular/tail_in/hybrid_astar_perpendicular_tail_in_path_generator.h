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

  const bool UpdateOnce(const PathColDetBuffer& path_col_det_buffer);

 private:
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
#if USE_LINK_PT_LINE
      const common_math::PathPt<float>& gear_switch_pose,
#else
      const geometry_lib::PathPoint& gear_switch_pose,
#endif
      AstarPathGear gear, const float gear_switch_penalty,
      const float length_penalty) override;
};
}  // namespace apa_planner
}  // namespace planning
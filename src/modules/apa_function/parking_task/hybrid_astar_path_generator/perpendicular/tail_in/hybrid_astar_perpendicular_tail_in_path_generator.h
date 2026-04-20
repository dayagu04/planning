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
  const float CalcGearChangePoseCost(
      const common_math::PathPt<float>& gear_switch_pose,
      const float gear_switch_penalty, const float length_penalty);
  const float CalcGearSwitchPoseHeuDist(
      const common_math::PathPt<float>& gear_switch_pose,
      const float gear_switch_penalty);
  const float CalcGearSwitchPoseBaseCost(
      const common_math::PathPt<float>& gear_switch_pose, const float heu_dist,
      const float length_penalty) const;
  const float CalcGearSwitchPoseCollisionCost(
      const common_math::PathPt<float>& gear_switch_pose,
      const float length_penalty) const;
  const float CalcGearSwitchPoseRangeCost(
      const common_math::PathPt<float>& gear_switch_pose,
      const float gear_switch_penalty) const;
  const float CalcGearSwitchPosePreferXCost(
      const common_math::PathPt<float>& gear_switch_pose,
      const float gear_switch_penalty) const;

  virtual void PrepareFormalSearch(
      const SearchConfigSnapshot& snapshot) override;
  virtual void CalcNodeGCost(Node3d* current_node, Node3d* next_node) override;
  virtual CurveNodeScoreParam BuildCurveNodeScoreParam() const override;
  virtual void FillCurveNodeBaseCost(const CurveNode& curve_node,
                                     const CurveNodeScoreParam& score_param,
                                     PathCompareCost& cost) const override;
  virtual void FillCurveNodeObsDistCost(CurveNode& curve_node,
                                        const CurveNodeScoreParam& score_param,
                                        PathCompareCost& cost) override;
  virtual void FillCurveNodeGearSwitchCost(
      const CurveNode& curve_node, const CurveNodeScoreParam& score_param,
      PathCompareCost& cost) override;
};
}  // namespace apa_planner
}  // namespace planning

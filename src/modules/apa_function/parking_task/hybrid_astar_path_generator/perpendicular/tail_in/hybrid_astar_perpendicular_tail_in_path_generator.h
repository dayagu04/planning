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
  struct CurveNodeScoreParam {
    float gear_change_penalty;
    float length_penalty;
    float unsuitable_last_line_length_penalty;
    float kappa_change_penalty;
    float last_path_kappa_change_penalty;
    float lat_err_penalty;
    float heading_err_penalty;
  };

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

  CurveNodeScoreParam BuildCurveNodeScoreParam() const;
  void FillCurveNodeBaseCost(const CurveNode& curve_node,
                             const CurveNodeScoreParam& score_param,
                             PathCompareCost& cost) const;
  void FillCurveNodeObsDistCost(CurveNode& curve_node,
                                const CurveNodeScoreParam& score_param,
                                PathCompareCost& cost);
  void FillCurveNodeGearSwitchCost(const CurveNode& curve_node,
                                   const CurveNodeScoreParam& score_param,
                                   const float cur_theta, const float end_theta,
                                   const float cur_theta_err,
                                   PathCompareCost& cost);

  const float CalcGearSwitchPoseHeuDist(
      const common_math::PathPt<float>& gear_switch_pose, AstarPathGear gear,
      const float gear_switch_penalty);
  const float CalcGearSwitchPoseBaseCost(
      const common_math::PathPt<float>& gear_switch_pose, const float heu_dist,
      const float length_penalty) const;
  const float CalcGearSwitchPoseCollisionCost(
      const common_math::PathPt<float>& gear_switch_pose, AstarPathGear gear,
      const float length_penalty) const;
  const float CalcGearSwitchPoseRangeCost(
      const common_math::PathPt<float>& gear_switch_pose,
      const float gear_switch_penalty) const;
  const float CalcGearSwitchPosePreferXCost(
      const common_math::PathPt<float>& gear_switch_pose, AstarPathGear gear,
      const float gear_switch_penalty) const;

  virtual void ChooseBestCurveNode(
      std::vector<CurveNode>& curve_node_to_goal_vec,
      CurveNode& best_curve_node_to_goal) override;

  virtual const float CalcGearChangePoseCost(
      const common_math::PathPt<float>& gear_switch_pose, AstarPathGear gear,
      const float gear_switch_penalty, const float length_penalty) override;
};
}  // namespace apa_planner
}  // namespace planning

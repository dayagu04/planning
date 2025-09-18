#pragma once
#include "curve_node.h"
#include "hybrid_astar_context.h"
#include "parking_task.h"
namespace planning {
namespace apa_planner {

class PathCompareDecider final : public ParkingTask {
 public:
  PathCompareDecider() = default;
  ~PathCompareDecider() = default;

  const bool Compare(const HybridAStarRequest* request,
                     const CurveNode* best_node,
                     const CurveNode* challenging_node);

  const bool CompareForPerpendicularTailIn();

  const bool CompareForPerpendicularTailOut();

  static const float CalcHeuristicPointDistance(const Eigen::Vector2d& pos_a,
                                                const double heading_a,
                                                const Eigen::Vector2d& pos_b);

  static const float CalcHeuristicPointDistance(const Eigen::Vector2f& pos_a,
                                                const float heading_a,
                                                const Eigen::Vector2d& pos_b);

 private:
  const CurveNode* best_node_;
  const CurveNode* challenging_node_;
  const HybridAStarRequest* request_;
};
}  // namespace apa_planner
}  // namespace planning
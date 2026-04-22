#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "vec2d.h"

namespace planning {

struct UnifiedClusterConfig {
  bool   enable                 = false;
  double grid_resolution        = 0.25;   // m, grid cell size for dedup
  int    neighbor_range         = 1;      // neighbor search range for clustering (1=8-connected, 2=24-connected)
  double aspect_ratio_threshold = 3.0;    // (reserved)
  double rdp_tolerance          = 0.15;   // (reserved)
  double box_safety_buffer      = 0.1;    // (reserved)
  int    min_points             = 3;      // minimum points per cluster/piece
  double max_segment_length     = 3.0;    // m, max convex hull diagonal before bisection
  double compactness_threshold  = 0.80;   // convex_hull_area / AABB_area threshold
  double density_threshold      = 4.0;    // points per m² threshold for dense clusters
  double curvature_threshold    = 0.05;   // average curvature threshold for curved structures (1/radius)
  double bisect_stop_ratio      = 0.85;   // stop bisection if both halves > parent * ratio
};

struct ClusterObstacle {
  std::vector<planning_math::Vec2d> points;  // convex hull vertices, >= 3 pts
};

// ---------------------------------------------------------------------------
// UnifiedStaticCluster
//
// Pipeline:
//   1. Grid-Hash dedup (O(n))
//   2. N-connected Union-Find clustering (O(n·α(n)))
//   3. Recursive bisection with compactness check:
//      - compact cluster → convex hull (vehicle, small obstacle)
//      - non-compact cluster → bisect along PCA axis, recurse
// ---------------------------------------------------------------------------
class UnifiedStaticCluster {
 public:
  explicit UnifiedStaticCluster(const UnifiedClusterConfig &cfg);

  std::vector<ClusterObstacle> Process(
      const std::vector<planning_math::Vec2d> &gl_points,
      const std::vector<planning_math::Vec2d> &occ_points);

 private:
  struct Cell { int ix, iy; };
  struct DsuNode { int parent; int rank; };

  std::vector<Cell> BuildGrid(
      const std::vector<planning_math::Vec2d> &gl_points,
      const std::vector<planning_math::Vec2d> &occ_points);

  std::vector<std::vector<int>> ClusterCells(const std::vector<Cell> &cells);

  int  DsuFind(std::vector<DsuNode> &nodes, int i);
  void DsuUnion(std::vector<DsuNode> &nodes, int a, int b);

  // Recursive bisection: splits non-compact clusters along PCA axis
  std::vector<ClusterObstacle> RecursiveBisect(
      const std::vector<planning_math::Vec2d> &pts, int depth) const;

  double ComputePcaAngle(const std::vector<planning_math::Vec2d> &pts) const;
  double ComputeAverageCurvature(const std::vector<planning_math::Vec2d> &hull_points) const;
  double ObbArea(const std::vector<planning_math::Vec2d> &pts, double angle) const;
  double ObbAspectRatio(const std::vector<planning_math::Vec2d> &pts, double angle) const;
  double AabbDiagonal(const std::vector<planning_math::Vec2d> &pts) const;
  double AabbArea(const std::vector<planning_math::Vec2d> &pts) const;

  UnifiedClusterConfig cfg_;
  std::vector<planning_math::Vec2d> all_pts_;
};

}  // namespace planning

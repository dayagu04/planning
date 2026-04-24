#include "unified_static_cluster.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <unordered_map>

#include "log_glog.h"
#include "math/box2d.h"
#include "math/polygon2d.h"
#include "vec2d.h"

namespace planning {

using planning_math::Box2d;
using planning_math::Polygon2d;
using planning_math::Vec2d;

UnifiedStaticCluster::UnifiedStaticCluster(const UnifiedClusterConfig &cfg)
    : cfg_(cfg) {}

// ===========================================================================
// Process — main entry point
// ===========================================================================
std::vector<ClusterObstacle> UnifiedStaticCluster::Process(
    const std::vector<Vec2d> &gl_points,
    const std::vector<Vec2d> &occ_points) {
  std::vector<ClusterObstacle> result;
  if (!cfg_.enable) return result;

  // Step 1: grid-hash dedup
  std::vector<Vec2d> all_pts;
  const std::vector<Cell> cells = BuildGrid(gl_points, occ_points, all_pts);
  if (cells.empty()) return result;

  // Step 2: 8-connected Union-Find clustering
  std::vector<std::vector<int>> clusters = ClusterCells(cells);

  // Step 3: per-cluster shape processing
  for (const auto &member_indices : clusters) {
    if (static_cast<int>(member_indices.size()) < cfg_.min_points) continue;

    std::vector<Vec2d> pts;
    pts.reserve(member_indices.size());
    for (int idx : member_indices) pts.push_back(all_pts[idx]);

    // Recursive bisection: splits large clusters into convex-hull pieces
    // Small clusters (diagonal < max_segment_length) become a single convex hull
    auto pieces = RecursiveBisect(pts, 0);
    for (auto &obs : pieces) {
      if (static_cast<int>(obs.points.size()) >= cfg_.min_points) {
        // Calculate center for frame-to-frame tracking
        double cx = 0.0, cy = 0.0;
        for (const auto &p : obs.points) {
          cx += p.x();
          cy += p.y();
        }
        obs.center = Vec2d(cx / obs.points.size(), cy / obs.points.size());
        result.push_back(std::move(obs));
      }
    }
  }
  return result;
}

// ===========================================================================
// BuildGrid — O(n) grid-hash dedup
// ===========================================================================
std::vector<UnifiedStaticCluster::Cell> UnifiedStaticCluster::BuildGrid(
    const std::vector<Vec2d> &gl_points,
    const std::vector<Vec2d> &occ_points,
    std::vector<Vec2d> &all_pts) {
  all_pts.clear();
  const double inv_res = 1.0 / cfg_.grid_resolution;
  std::unordered_map<int64_t, int> grid;
  grid.reserve((gl_points.size() + occ_points.size()) * 2);
  std::vector<Cell> cells;
  cells.reserve(gl_points.size() + occ_points.size());

  auto insert_point = [&](const Vec2d &p) {
    if (p.x() == 0.0 && p.y() == 0.0) return;
    int ix = static_cast<int>(std::floor(p.x() * inv_res));
    int iy = static_cast<int>(std::floor(p.y() * inv_res));
    int64_t key = (static_cast<int64_t>(ix + 100000) << 32) |
                  static_cast<uint32_t>(iy + 100000);
    if (grid.find(key) == grid.end()) {
      grid[key] = static_cast<int>(all_pts.size());
      all_pts.push_back(p);
      cells.push_back({ix, iy});
    }
  };

  for (const auto &p : gl_points)  insert_point(p);
  for (const auto &p : occ_points) insert_point(p);
  return cells;
}

// ===========================================================================
// DSU helpers
// ===========================================================================
int UnifiedStaticCluster::DsuFind(std::vector<DsuNode> &nodes, int i) {
  while (nodes[i].parent != i) {
    nodes[i].parent = nodes[nodes[i].parent].parent;
    i = nodes[i].parent;
  }
  return i;
}

void UnifiedStaticCluster::DsuUnion(std::vector<DsuNode> &nodes, int a, int b) {
  a = DsuFind(nodes, a);
  b = DsuFind(nodes, b);
  if (a == b) return;
  if (nodes[a].rank < nodes[b].rank) std::swap(a, b);
  nodes[b].parent = a;
  if (nodes[a].rank == nodes[b].rank) nodes[a].rank++;
}

// ===========================================================================
// ClusterCells — N-connected Union-Find
// ===========================================================================
std::vector<std::vector<int>> UnifiedStaticCluster::ClusterCells(
    const std::vector<Cell> &cells) {
  const int n = static_cast<int>(cells.size());
  std::unordered_map<int64_t, int> pos_map;
  pos_map.reserve(n * 2);
  for (int i = 0; i < n; ++i) {
    int64_t key = (static_cast<int64_t>(cells[i].ix + 100000) << 32) |
                  static_cast<uint32_t>(cells[i].iy + 100000);
    pos_map[key] = i;
  }

  std::vector<DsuNode> nodes(n);
  for (int i = 0; i < n; ++i) { nodes[i].parent = i; nodes[i].rank = 0; }

  // N-connected: check all neighbors within range
  const int range = cfg_.neighbor_range;
  for (int i = 0; i < n; ++i) {
    for (int dx = -range; dx <= range; ++dx) {
      for (int dy = -range; dy <= range; ++dy) {
        if (dx == 0 && dy == 0) continue;
        int64_t nkey = (static_cast<int64_t>(cells[i].ix + dx + 100000) << 32) |
                       static_cast<uint32_t>(cells[i].iy + dy + 100000);
        auto it = pos_map.find(nkey);
        if (it != pos_map.end()) DsuUnion(nodes, i, it->second);
      }
    }
  }

  std::unordered_map<int, std::vector<int>> root_to_members;
  root_to_members.reserve(n);
  for (int i = 0; i < n; ++i) root_to_members[DsuFind(nodes, i)].push_back(i);

  std::vector<std::vector<int>> clusters;
  clusters.reserve(root_to_members.size());
  for (auto &kv : root_to_members) clusters.push_back(std::move(kv.second));
  return clusters;
}

// ===========================================================================
// RecursiveBisect — the core shape processor
//
// Strategy:
//   1. Compute convex hull of the point set
//   2. If the hull is "good enough" (small, or compact), return it directly
//   3. Otherwise, split along PCA principal axis and recurse on each half
//
// "Good enough" = diagonal < max_segment_length
//                 OR convex_hull_area / AABB_area > compactness_threshold
//
// This naturally handles:
//   - Vehicles (compact) → single convex hull, no split
//   - Straight walls (elongated) → bisected into short convex hull pieces
//   - Curved ramps (large, low compactness) → bisected into arc pieces
//   - L-shapes (low compactness) → bisected at the corner
//   - Irregular scattered points → bisected into smaller convex hulls
// ===========================================================================
std::vector<ClusterObstacle> UnifiedStaticCluster::RecursiveBisect(
    const std::vector<Vec2d> &pts, int depth) const {
  std::vector<ClusterObstacle> result;
  if (static_cast<int>(pts.size()) < cfg_.min_points) return result;

  // Compute convex hull
  Polygon2d hull;
  if (!Polygon2d::ComputeConvexHull(pts, &hull) || hull.points().size() < 3) {
    return result;
  }

  // Compute metrics
  double diag = AabbDiagonal(pts);
  double hull_area = hull.area();

  // Use OBB (PCA-aligned bounding box) for compactness instead of AABB
  // This makes compactness rotation-invariant: a 45° rotated rectangle
  // still gets compactness ~0.9 instead of ~0.5 with AABB
  double angle = ComputePcaAngle(pts);
  double obb_area = ObbArea(pts, angle);
  double compactness = (obb_area > 1e-6) ? (hull_area / obb_area) : 1.0;
  double point_density = (obb_area > 1e-6) ? (pts.size() / obb_area) : 0.0;

  // STRATEGY: Default to returning convex hull (minimize obstacle count)
  // Only split if the convex hull is CLEARLY unreasonable:
  //   - Large (diagonal > max_segment_length) AND
  //   - Non-compact (compactness < threshold) AND
  //   - Not dense (density < threshold)
  //
  // This ensures:
  //   - Vehicles stay as single convex hull (dense OR compact)
  //   - Small clusters stay as single convex hull (small diagonal)
  //   - Only large sparse non-compact clusters (curved ramps) get split

  bool is_large = diag > cfg_.max_segment_length;
  bool is_non_compact = compactness < cfg_.compactness_threshold;
  bool is_sparse = point_density < cfg_.density_threshold;

  // Condition 1: large + non-compact + sparse (triangles, straight walls)
  bool condition1 = is_large && is_non_compact && is_sparse;

  // Condition 2: elongated + curved (arcs, ramps)
  // OBB aspect ratio distinguishes elongated arcs from compact vehicles/triangles
  // Curvature distinguishes arcs from straight walls
  double avg_curvature = ComputeAverageCurvature(hull.points());
  double obb_aspect_ratio = ObbAspectRatio(pts, angle);
  bool is_elongated = obb_aspect_ratio > cfg_.aspect_ratio_threshold;
  bool is_curved = avg_curvature > cfg_.curvature_threshold;
  bool condition2 = is_large && is_elongated && is_curved;

  // Debug log for analysis
  // if (depth == 0 && pts.size() > 10) {
  //   std::cout << "[Cluster] pts=" << pts.size()
  //             << " diag=" << diag
  //             << " compact=" << compactness
  //             << " density=" << point_density
  //             << " curv=" << avg_curvature
  //             << " ar=" << obb_aspect_ratio
  //             << " | large=" << is_large
  //             << " non_compact=" << is_non_compact
  //             << " sparse=" << is_sparse
  //             << " elongated=" << is_elongated
  //             << " curved=" << is_curved
  //             << " | cond1=" << condition1
  //             << " cond2=" << condition2
  //             << " | split=" << ((condition1 || condition2) && depth < 8)
  //             << std::endl;
  // }

  // Return convex hull if neither condition is met, or depth limit reached
  if ((!condition1 && !condition2) || depth >= 8) {
    ClusterObstacle obs;
    obs.points = hull.points();
    result.push_back(std::move(obs));
    return result;
  }

  // Split along PCA principal axis at the median projection
  // Reuse angle computed above for OBB
  double cos_a = std::cos(angle);
  double sin_a = std::sin(angle);

  // Project all points
  std::vector<double> projections(pts.size());
  for (size_t i = 0; i < pts.size(); ++i) {
    projections[i] = pts[i].x() * cos_a + pts[i].y() * sin_a;
  }

  // Find median projection
  std::vector<double> sorted_proj = projections;
  std::nth_element(sorted_proj.begin(),
                   sorted_proj.begin() + sorted_proj.size() / 2,
                   sorted_proj.end());
  double median = sorted_proj[sorted_proj.size() / 2];

  // Split into two halves
  std::vector<Vec2d> left_pts, right_pts;
  left_pts.reserve(pts.size() / 2 + 1);
  right_pts.reserve(pts.size() / 2 + 1);
  for (size_t i = 0; i < pts.size(); ++i) {
    if (projections[i] <= median) {
      left_pts.push_back(pts[i]);
    } else {
      right_pts.push_back(pts[i]);
    }
  }

  // Guard: if split didn't separate points, or both halves still have
  // similar diagonal (points uniformly spread, not elongated), stop splitting
  if (left_pts.empty() || right_pts.empty()) {
    ClusterObstacle obs;
    obs.points = hull.points();
    result.push_back(std::move(obs));
    return result;
  }

  double left_diag = AabbDiagonal(left_pts);
  double right_diag = AabbDiagonal(right_pts);

  if (left_diag > diag * cfg_.bisect_stop_ratio &&
      right_diag > diag * cfg_.bisect_stop_ratio) {
    ClusterObstacle obs;
    obs.points = hull.points();
    result.push_back(std::move(obs));
    return result;
  }

  // Recurse
  auto left_result = RecursiveBisect(left_pts, depth + 1);
  auto right_result = RecursiveBisect(right_pts, depth + 1);
  result.insert(result.end(), left_result.begin(), left_result.end());
  result.insert(result.end(), right_result.begin(), right_result.end());
  return result;
}

// ===========================================================================
// Curvature computation — detects curved structures (arcs, ramps)
// ===========================================================================
double UnifiedStaticCluster::ComputeAverageCurvature(
    const std::vector<Vec2d> &hull_points) const {
  if (hull_points.size() < 3) return 0.0;

  double total_curvature = 0.0;
  int count = 0;

  // Use three-point method to compute curvature at each hull vertex
  for (size_t i = 0; i < hull_points.size(); ++i) {
    const Vec2d &p1 = hull_points[i];
    const Vec2d &p2 = hull_points[(i + 1) % hull_points.size()];
    const Vec2d &p3 = hull_points[(i + 2) % hull_points.size()];

    // Compute curvature = 1 / radius of circumcircle
    // Using Menger curvature formula: k = 4 * Area(triangle) / (a * b * c)
    double dx1 = p2.x() - p1.x(), dy1 = p2.y() - p1.y();
    double dx2 = p3.x() - p2.x(), dy2 = p3.y() - p2.y();
    double dx3 = p1.x() - p3.x(), dy3 = p1.y() - p3.y();

    double a = std::hypot(dx1, dy1);  // |p1-p2|
    double b = std::hypot(dx2, dy2);  // |p2-p3|
    double c = std::hypot(dx3, dy3);  // |p3-p1|

    // Triangle area using cross product
    double area = std::fabs(dx1 * dy2 - dy1 * dx2) * 0.5;

    if (a < 1e-6 || b < 1e-6 || c < 1e-6) continue;  // degenerate triangle

    double curvature = 4.0 * area / (a * b * c);
    total_curvature += curvature;
    count++;
  }

  return (count > 0) ? (total_curvature / count) : 0.0;
}

// ===========================================================================
// ComputePcaAngle
// ===========================================================================
double UnifiedStaticCluster::ComputePcaAngle(const std::vector<Vec2d> &pts) const {
  const int n = static_cast<int>(pts.size());
  double mx = 0.0, my = 0.0;
  for (const auto &p : pts) { mx += p.x(); my += p.y(); }
  mx /= n; my /= n;

  double cxx = 0.0, cyy = 0.0, cxy = 0.0;
  for (const auto &p : pts) {
    double dx = p.x() - mx, dy = p.y() - my;
    cxx += dx * dx; cyy += dy * dy; cxy += dx * dy;
  }

  double half_trace = (cxx + cyy) * 0.5;
  double disc = std::sqrt(((cxx - cyy) * 0.5) * ((cxx - cyy) * 0.5) + cxy * cxy);
  double lambda_max = half_trace + disc;

  double vx = cxy, vy = lambda_max - cxx;
  if (std::fabs(vx) < 1e-9 && std::fabs(vy) < 1e-9) return 0.0;
  return std::atan2(vy, vx);
}

// ===========================================================================
// OBB (Oriented Bounding Box) area — rotation-invariant compactness
// ===========================================================================
double UnifiedStaticCluster::ObbArea(const std::vector<Vec2d> &pts,
                                      double angle) const {
  double cos_a = std::cos(angle);
  double sin_a = std::sin(angle);

  // Project all points onto PCA principal axis and perpendicular axis
  double u_min = std::numeric_limits<double>::max();
  double u_max = std::numeric_limits<double>::lowest();
  double v_min = std::numeric_limits<double>::max();
  double v_max = std::numeric_limits<double>::lowest();

  for (const auto &p : pts) {
    double u = p.x() * cos_a + p.y() * sin_a;       // along principal axis
    double v = -p.x() * sin_a + p.y() * cos_a;      // perpendicular
    u_min = std::min(u_min, u);
    u_max = std::max(u_max, u);
    v_min = std::min(v_min, v);
    v_max = std::max(v_max, v);
  }

  return (u_max - u_min) * (v_max - v_min);
}

double UnifiedStaticCluster::ObbAspectRatio(const std::vector<Vec2d> &pts,
                                             double angle) const {
  double cos_a = std::cos(angle);
  double sin_a = std::sin(angle);

  double u_min = std::numeric_limits<double>::max();
  double u_max = std::numeric_limits<double>::lowest();
  double v_min = std::numeric_limits<double>::max();
  double v_max = std::numeric_limits<double>::lowest();

  for (const auto &p : pts) {
    double u = p.x() * cos_a + p.y() * sin_a;
    double v = -p.x() * sin_a + p.y() * cos_a;
    u_min = std::min(u_min, u);
    u_max = std::max(u_max, u);
    v_min = std::min(v_min, v);
    v_max = std::max(v_max, v);
  }

  double length = u_max - u_min;
  double width = v_max - v_min;
  if (width < 1e-6) return 1e6;
  return length / width;
}

// ===========================================================================
// AABB helpers
// ===========================================================================
double UnifiedStaticCluster::AabbDiagonal(const std::vector<Vec2d> &pts) const {
  double xmin = pts[0].x(), xmax = xmin, ymin = pts[0].y(), ymax = ymin;
  for (const auto &p : pts) {
    xmin = std::min(xmin, p.x()); xmax = std::max(xmax, p.x());
    ymin = std::min(ymin, p.y()); ymax = std::max(ymax, p.y());
  }
  return std::hypot(xmax - xmin, ymax - ymin);
}

double UnifiedStaticCluster::AabbArea(const std::vector<Vec2d> &pts) const {
  double xmin = pts[0].x(), xmax = xmin, ymin = pts[0].y(), ymax = ymin;
  for (const auto &p : pts) {
    xmin = std::min(xmin, p.x()); xmax = std::max(xmax, p.x());
    ymin = std::min(ymin, p.y()); ymax = std::max(ymax, p.y());
  }
  return (xmax - xmin) * (ymax - ymin);
}

}  // namespace planning

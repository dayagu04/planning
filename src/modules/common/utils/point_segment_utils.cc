#include "point_segment_utils.h"

#include <cmath>
#include <limits>

namespace planning {

std::vector<planning_math::Vec2d> DeduplicatePointsByDistance(
    const std::vector<planning_math::Vec2d> &points, double dedup_distance) {
  std::vector<planning_math::Vec2d> deduplicated_points;
  deduplicated_points.reserve(points.size());
  const double dedup_distance_sq = dedup_distance * dedup_distance;
  for (const auto &point : points) {
    bool exists = false;
    for (const auto &saved_point : deduplicated_points) {
      const double dx = point.x() - saved_point.x();
      const double dy = point.y() - saved_point.y();
      if (dx * dx + dy * dy <= dedup_distance_sq) {
        exists = true;
        break;
      }
    }
    if (!exists) {
      deduplicated_points.push_back(point);
    }
  }
  return deduplicated_points;
}

std::vector<std::vector<planning_math::Vec2d>> SplitPointsIntoSegmentsByGap(
    const std::vector<planning_math::Vec2d> &ordered_points,
    double split_gap_distance) {
  std::vector<std::vector<planning_math::Vec2d>> segments;
  if (ordered_points.empty()) {
    return segments;
  }
  std::vector<planning_math::Vec2d> current_segment;
  current_segment.push_back(ordered_points.front());
  for (size_t i = 1; i < ordered_points.size(); ++i) {
    const double dx = ordered_points[i].x() - ordered_points[i - 1].x();
    const double dy = ordered_points[i].y() - ordered_points[i - 1].y();
    const double dist = std::hypot(dx, dy);
    if (dist > split_gap_distance && !current_segment.empty()) {
      segments.push_back(current_segment);
      current_segment.clear();
    }
    current_segment.push_back(ordered_points[i]);
  }
  if (!current_segment.empty()) {
    segments.push_back(current_segment);
  }
  return segments;
}

std::vector<planning_math::Vec2d> SelectClosestSegmentToAnchorPoint(
    const std::vector<std::vector<planning_math::Vec2d>> &point_segments,
    const TrajectoryPoint &anchor_traj_point) {
  if (point_segments.empty()) {
    return {};
  }
  double best_dist_sq = std::numeric_limits<double>::max();
  size_t best_index = 0;
  for (size_t i = 0; i < point_segments.size(); ++i) {
    if (point_segments[i].empty()) {
      continue;
    }
    for (const auto &point : point_segments[i]) {
      const double dx = point.x() - anchor_traj_point.x;
      const double dy = point.y() - anchor_traj_point.y;
      const double dist_sq = dx * dx + dy * dy;
      if (dist_sq < best_dist_sq) {
        best_dist_sq = dist_sq;
        best_index = i;
      }
    }
  }
  return point_segments[best_index];
}

}  // namespace planning

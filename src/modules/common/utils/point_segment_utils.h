#pragma once

#include <vector>

#include "trajectory/trajectory_point.h"
#include "vec2d.h"

namespace planning {

std::vector<planning_math::Vec2d> DeduplicatePointsByDistance(
    const std::vector<planning_math::Vec2d> &points, double dedup_distance);

// Split ordered points into contiguous segments when adjacent points are
// farther apart than split_gap_distance. Input points are expected to preserve
// their original business order.
std::vector<std::vector<planning_math::Vec2d>> SplitPointsIntoSegmentsByGap(
    const std::vector<planning_math::Vec2d> &ordered_points,
    double split_gap_distance);

std::vector<planning_math::Vec2d> SelectClosestSegmentToAnchorPoint(
    const std::vector<std::vector<planning_math::Vec2d>> &point_segments,
    const TrajectoryPoint &anchor_traj_point);

}  // namespace planning

#pragma once

#include "./../convex_collision_detection/aabb2d.h"
#include "./../occupancy_grid_map/point_cloud_obstacle.h"
#include "pose2d.h"

namespace planning {

// for collision check performance, add a clear zone which is not contain any
// obstacle. So if your path in this zone, it is safe.
class ObstacleClearZone {
 public:
  ObstacleClearZone() = default;

  void GenerateBoundingBox(const Pose2f& start,
                           const ParkObstacleList* obstacles,
                           const bool enable_clear_zone);

  const bool IsContain(const cdl::AABB& box) const;

  const bool IsValidClearZone() const { return is_clear_zone_valid_; }

 private:
  bool IsCollisionForBox(const cdl::AABB& box,
                         const ParkObstacleList* obstacles);

  cdl::AABB box_;
  bool is_clear_zone_valid_;
};
}  // namespace planning
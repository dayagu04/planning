#pragma once

#include "./../collision_detection/aabb2d.h"
#include "./../occupancy_grid_map/point_cloud_obstacle.h"
#include "pose2d.h"

namespace planning {

// for collision check performance, add a clear zone which is not contain any
// obstacle. So if your path in this zone, it is safe.
class ObstacleClearZone {
 public:
  ObstacleClearZone() = default;

  bool GenerateBoundingBox(const Pose2D& start,
                           const ParkObstacleList* obstacles);

  const bool IsContain(const cdl::AABB& box);

 private:
  bool IsCollisionForBox(const cdl::AABB& box,
                         const ParkObstacleList* obstacles);

  cdl::AABB box_;
};
}  // namespace planning
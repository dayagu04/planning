#pragma once

#include <vector>

#include "pose2d.h"
#include "./../collision_detection/aabb2d.h"
#include "./../collision_detection/polygon_base.h"
#include "./../../modules/common/local_view.h"
#include "transform2d.h"

namespace planning {

enum class ParkObstacleType {
  none = 0,
  map_bound,
  slot_line,
  virtual_wall,
  ground_line,
  fusion_obj,
  uss_obj,
  max_num
};

// now, use convex hull collision detection API, so we use point to restore
// point cloud. But in the future, we will use occupancy grid
// map (ogm) to represent all obstacle in astar search.
struct PointCloudObstacle {
  std::vector<Position2D> points;

  Polygon2D envelop_polygon;
  cdl::AABB box;
  ParkObstacleType obs_type;
};

struct ParkObstacleList {
  std::vector<Position2D> virtual_obs;

  // fusion obj + ground line
  std::vector<PointCloudObstacle> point_cloud_list;

  void Clear() {
    virtual_obs.clear();
    point_cloud_list.clear();
    return;
  }

  const bool IsEmpty() const {
    if (point_cloud_list.size() > 0 || virtual_obs.size() > 0) {
      return false;
    }
    return true;
  }
};

class PointCloudObstacleTransform {
 public:
  PointCloudObstacleTransform() = default;

  const int GenerateLocalObstacle(
      ParkObstacleList& obs_list, const LocalView* local_view,
      const bool delete_obs_around_ego, const double slot_length,
      const double slot_width,
      const Pose2D& slot_base_pose, const Pose2D& ego_start,
      const Pose2D& ego_final_goal);

  void GenerateGlobalObstacle(ParkObstacleList& obs_list,
                              const LocalView* local_view);

 private:
};

}  // namespace planning
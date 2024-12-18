#pragma once

#include <vector>

#include "./../../modules/common/local_view.h"
#include "./../collision_detection/aabb2d.h"
#include "./../collision_detection/polygon_base.h"
#include "library/hybrid_astar_lib/hybrid_astar_common.h"
#include "pose2d.h"
#include "transform2d.h"
#include "src/modules/apa_function/apa_world/apa_obstacle.h"

namespace planning {

// now, use convex hull collision detection API, so we use point to restore
// point cloud. But in the future, we will use occupancy grid
// map (ogm) to represent all obstacle in astar search.
// todo: use heirachy collision checker by different height.
// e.g. lower obs < 0.3 meter. upper obs > 2.3 meter.
// todo: move to apa_function
struct PointCloudObstacle {
  std::vector<Position2D> points;

  Polygon2D envelop_polygon;
  cdl::AABB box;
  apa_planner::ApaObsAttributeType obs_type;
};

// todo: use ParkObstacle to replace PointCloudObstacle. And delete
// ParkObstacleList.
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

  const void GenerateLocalObstacle(ParkObstacleList& obs_list,
                                   const LocalView* local_view,
                                   const double slot_length,
                                   const double slot_width,
                                   const Pose2D& slot_base_pose,
                                   const Pose2D& ego_start,
                                   const bool enable_limiter_obs);

  void GenerateGlobalObstacle(ParkObstacleList& obs_list,
                              const LocalView* local_view,
                              const bool enable_limiter_obs);

 private:
  void SampleInLineSegment(const Eigen::Vector2d& start,
                           const Eigen::Vector2d& end,
                           std::vector<Position2D>* points);
};

}  // namespace planning
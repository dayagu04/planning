#pragma once

#include <vector>

#include "src/modules/common/local_view.h"
#include "src/library/convex_collision_detection/aabb2d.h"
#include "src/library/convex_collision_detection/polygon_base.h"
#include "pose2d.h"
#include "src/library/hybrid_astar_lib/hybrid_astar_common.h"
#include "src/modules/apa_function/apa_common/apa_obstacle_manager.h"
#include "transform2d.h"
#include "gjk2d_interface.h"

namespace planning {

// now, use convex hull collision detection API, so we use point to restore
// point cloud. But in the future, we will use occupancy grid
// map (ogm) to represent all obstacle in astar search.
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

  // fusion occ + ground line + od
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

  void GenerateLocalObstacleByLocalView(ParkObstacleList& obs_list,
                                        const LocalView* local_view,
                                        const double slot_length,
                                        const double slot_width,
                                        const Pose2D& slot_base_pose,
                                        const Pose2D& ego_start,
                                        const bool enable_limiter_obs);

  void GenerateLocalObstacle(
      std::shared_ptr<apa_planner::ApaObstacleManager> obs_manager,
      ParkObstacleList& obs_list, const Pose2D& ego_pose,
      const cdl::AABB& slot_box, const bool delete_slot_obs);

 private:
  void SampleInLineSegment(const Eigen::Vector2d& start,
                           const Eigen::Vector2d& end,
                           std::vector<Position2D>* points);

  void GetCompactCarPolygonByParam(Polygon2D* box, const double lat_buffer,
                                   const double lon_buffer);

 private:
  GJK2DInterface gjk_;
};

}  // namespace planning
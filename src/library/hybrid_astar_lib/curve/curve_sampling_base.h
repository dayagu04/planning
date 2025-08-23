#pragma once

#include "euler_distance_transform.h"
#include "hybrid_astar_common.h"
#include "hybrid_astar_request.h"
#include "node_collision_detect.h"
#include "src/library/hybrid_astar_lib/decider/obstacle_clear_zone.h"
#include "park_reference_line.h"
#include "point_cloud_obstacle.h"
#include "pose2d.h"

namespace planning {

class CurveSampling {
 public:
  CurveSampling() = default;
  explicit CurveSampling(const MapBound* XYbounds, const ParkObstacleList* obstacles,
                const AstarRequest* request, EulerDistanceTransform* edt,
                const ObstacleClearZone* clear_zone,
                ParkReferenceLine* ref_line,
                const PlannerOpenSpaceConfig* config, const float min_radius,
                std::shared_ptr<NodeCollisionDetect> collision_detect);

  virtual ~CurveSampling() = default;

  void SetSearchGoal(const Pose2f& search_goal) {
    search_goal_ = search_goal;
    // search_goal_.DebugString();
    return;
  }

 protected:
  const ParkObstacleList* obstacles_;
  // if search node in aabb, no need to check collision;
  const ObstacleClearZone* clear_zone_;

  EulerDistanceTransform* edt_;

  // xmin, xmax, ymin, ymax
  const MapBound *grid_map_bound_;

  const ParkReferenceLine* ref_line_;

  const AstarRequest *request_;

  const PlannerOpenSpaceConfig* config_;
  float min_radius_;
  std::shared_ptr<NodeCollisionDetect> collision_detect_;

  // target pose for astar search.
  Pose2f search_goal_;
};

}  // namespace planning
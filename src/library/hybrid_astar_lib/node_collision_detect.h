#pragma once

#include "euler_distance_transform.h"
#include "footprint_circle_model.h"
#include "gjk2d_interface.h"
#include "hybrid_astar_common.h"
#include "hybrid_astar_request.h"
#include "node3d.h"
#include "obstacle_clear_zone.h"
#include "point_cloud_obstacle.h"
#include "polygon_base.h"
#include "rs_path_interpolate.h"
#include "transform2d.h"

namespace planning {

class NodeCollisionDetect {
 public:
  NodeCollisionDetect() = default;

  explicit NodeCollisionDetect(const ParkObstacleList* obstacles,
                               EulerDistanceTransform* edt,
                               const ObstacleClearZone* clear_zone,
                               const MapBound* XYbounds,
                               const AstarRequest* request);

  ~NodeCollisionDetect() = default;

  void UpdateFootPrintBySafeBuffer(const float lat_buffer_outside,
                                   const float lat_buffer_inside,
                                   const float lon_buffer,
                                   const VehicleParam& vehicle_param,
                                   const PlannerOpenSpaceConfig& config);

  // check collision and validity
  bool IsValidByConvexHull(Node3d* node);

  // check collision and validity
  const bool IsValidByEDT(Node3d* node);

  // check Reeds Shepp path collision and validity
  bool RSPathCollisionCheck(const RSPath* reeds_shepp_to_end,
                            Node3d* rs_node_to_goal);

  bool IsRSPathSafeByConvexHull(const RSPath* reeds_shepp_path, Node3d* node);

  const bool IsRSPathSafeByEDT(const RSPath* reeds_shepp_path, Node3d* node);

  const bool IsPolynomialPathSafeByEDT(const std::vector<AStarPathPoint>& path,
                                       Node3d* node);

  int GetPathCollisionIndex(HybridAStarResult* result);

  int GetPathCollisionIDByEDT(HybridAStarResult* result);

  int GetPathCollisionIDByEDT(const std::vector<AStarPathPoint>& poly_path);

  const bool IsPolygonCollision(const Polygon2D* polygon);

  const bool IsFootPrintCollision(const Transform2d& tf);

  void DebugEDTCheck(HybridAStarResult* path);

  const bool IsPointOutOfGridMapBound(const float x, const float y) const;

  // debug
  FootPrintCircleModel* GetCircleFootPrint(const HierarchySafeBuffer buffer);

  const bool IsContainByRecommendBox(const Pose2f& global_pose);

 private:
  FootPrintCircleModel* GetCircleFootPrintModel(const Pose2f& pose,
                                                const bool is_circle_path);

  // radius: 50 meter
  inline const bool IsCirclePathByKappa(const float kappa) {
    if (kappa > 0.02f || kappa < -0.02f) {
      return true;
    }

    return false;
  }

  FootPrintCircleModel* GetCircleFootPrintModelForParkOut(
      const Pose2f& pose, const bool is_circle_path);

  FootPrintCircleModel* GetCircleFootPrintModelForParkIn(
      const Pose2f& pose, const bool is_circle_path);

 private:
  GJK2DInterface gjk_interface_;

  // convex hull for accurate car
  PolygonFootPrint cvx_hull_foot_print_;

  // Consider different vehicle postion or kappa, use different buffer.
  // If vehicle is in slot inside or slot outside, use different safe buffer;
  // If vehicle is large kappa, use different safe buffer;
  HierarchyBufferCircleFootPrint hierachy_circle_model_;

  // 用于区分库内库外
  // todo: add template for double/float
  cdl::AABB2f slot_box_;
  cdl::AABB2f recommend_route_box_;
  // If pose is in compact_route_box_, do not check it again.
  cdl::AABB2f compact_route_box_;
  std::array<Position2f, 4> veh_box_;

  const ParkObstacleList* obstacles_;
  // if search node in aabb, no need to check collision;
  const ObstacleClearZone* clear_zone_;

  EulerDistanceTransform* edt_;

  const MapBound* grid_map_bound_;
  const AstarRequest* request_;
  bool is_park_in_;
};
}  // namespace planning
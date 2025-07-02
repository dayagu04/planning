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

  void UpdateFootPrintBySafeBuffer(const double lat_buffer_outside,
                                   const double lat_buffer_inside,
                                   const double lon_buffer,
                                   const VehicleParam& vehicle_param,
                                   const PlannerOpenSpaceConfig& config);

  // check collision and validity
  bool ValidityCheckByConvex(Node3d* node);

  // check collision and validity
  const bool ValidityCheckByEDT(Node3d* node);

  // check Reeds Shepp path collision and validity
  bool RSPathCollisionCheck(const RSPath* reeds_shepp_to_end,
                            Node3d* rs_node_to_goal);

  bool IsRSPathSafeByConvexHull(const RSPath* reeds_shepp_path, Node3d* node);

  const bool IsRSPathSafeByEDT(const RSPath* reeds_shepp_path, Node3d* node);

  const bool IsPolynomialPathSafeByEDT(const std::vector<AStarPathPoint>& path,
                                       Node3d* node);

  size_t GetPathCollisionIndex(HybridAStarResult* result);

  size_t GetPathCollisionIDByEDT(HybridAStarResult* result);

  size_t GetPathCollisionIDByEDT(const std::vector<AStarPathPoint>& poly_path);

  const bool IsPolygonCollision(const Polygon2D* polygon);

  const bool IsFootPrintCollision(const Transform2d& tf);

  Polygon2D* GetVehPolygon(const AstarPathGear& gear);

  void DebugEDTCheck(HybridAStarResult* path);

  const bool IsPointBeyondBound(const double x, const double y) const;

  // debug
  FootPrintCircleModel* GetSlotOutsideCircleFootPrint();

 private:
  FootPrintCircleModel* GetCircleFootPrintModel(const Pose2D& pose,
                                                const bool is_circle_path);

  inline const bool IsCirclePathBySteeringWheel(const double front_wheel_angle) {
    if (front_wheel_angle > 0.2f || front_wheel_angle < -0.2f) {
      return true;
    }

    return false;
  }

  inline const bool IsCirclePathByKappa(const double kappa) {
    if (kappa > 0.067f || kappa < -0.067f) {
      return true;
    }

    return false;
  }

 private:
  GJK2DInterface gjk_interface_;

  // todo, width = vehicle width + mirror width + safe width, bounding box
  // to accelerate collision detection.
  Polygon2D veh_box_gear_none_;
  Polygon2D veh_box_gear_drive_;
  Polygon2D veh_box_gear_reverse_;

  // convex hull for accurate car
  PolygonFootPrint cvx_hull_foot_print_;

  // Consider different vehicle postion or kappa, use different buffer.
  // If vehicle is in slot inside or slot outside, use different safe buffer;
  // If vehicle is large kappa, use different safe buffer;
  HierarchyBufferCircleFootPrint hierachy_circle_model_;

  // 用于区分库内库外
  cdl::AABB slot_box_;

  const ParkObstacleList* obstacles_;
  // if search node in aabb, no need to check collision;
  const ObstacleClearZone* clear_zone_;

  EulerDistanceTransform* edt_;

  const MapBound *XYbounds_;
  const AstarRequest* request_;
};
}  // namespace planning
#pragma once

#include <cstddef>
#include "./../../modules/apa_function/src/apa_param_setting.h"
#include "./../collision_detection/gjk2d_interface.h"
#include "./../geometry_lib/include/geometry_math.h"
#include "point_cloud_obstacle.h"
#include "polygon_base.h"
#include "pose2d.h"

namespace planning {

// in system, you can use polygon foot_print or circle foot print.
struct PolygonFootPrint {
  Polygon2D body;
  Polygon2D mirror_left;
  Polygon2D mirror_right;
  Polygon2D max_polygon;
};

enum class VehCollisionPosition {
  NONE = 0,
  LEFT_MIRROR = 1,
  RIGHT_MIRROR,
  BODY,
  MAX_POLYGON_NO_COLLISION
};

// check path safe for polygon obs and ogm obstacle
class PathSafeChecker {
 public:
  PathSafeChecker() = default;

  void Excute(const std::vector<pnc::geometry_lib::PathPoint>& path,
              const pnc::geometry_lib::PathSegGear gear,
              const ParkObstacleList* obs, const Pose2D& ego_pose);

  const bool IsPathCollision() const { return is_path_collision_; }

  const bool IsEgoCollision() const { return is_ego_collision_; }

  void UpdatePathValidDist(
      const std::vector<pnc::geometry_lib::PathPoint>& path,
      const Pose2D& ego_pose);

  const double GetPathValidDist() const { return path_valid_dist_; }

  const size_t GetPathCollisionID() const { return path_collision_idx_; }

 private:
  void GenerateVehBox(const pnc::geometry_lib::PathSegGear gear,
                      const double lateral_safe_buffer,
                      const double lon_safe_buffer);

  void GenerateVehCompactPolygon(const pnc::geometry_lib::PathSegGear gear,
                                 const double lateral_safe_buffer,
                                 const double lon_safe_buffer);

  size_t GetNearestPathPoint(
      const std::vector<pnc::geometry_lib::PathPoint>& path,
      const Pose2D& pose);

  int GenerateMirrorPolygon(Polygon2D* box, const double x_length,
                            const double y_length, const Position2D& center);

  const bool IsPolygonCollision(const Polygon2D* car);

  int GetCompactCarPolygonByParam(Polygon2D* box, const double lat_buffer,
                                  const double lon_buffer);

  const bool IsFootPrintPolygonCollision(const Transform2d& tf,
                                         PolygonFootPrint* foot_print,
                                         VehCollisionPosition* collision_info);

  const ParkObstacleList* obs_;

  bool is_ego_collision_;
  bool is_path_collision_;
  size_t path_collision_idx_;
  GJK2DInterface gjk_interface_;

  size_t path_nearest_idx_;
  double path_valid_dist_;

  // Polygon2D veh_polygon_;
  // Polygon2D veh_end_polygon_;

  PolygonFootPrint polygon_foot_print_;
  PolygonFootPrint path_end_foot_print_;

  std::vector<double> hierarchy_lateral_safe_buffer_;
  double advised_lat_buffer_;
  double advised_lon_buffer_;
};

}  // namespace planning
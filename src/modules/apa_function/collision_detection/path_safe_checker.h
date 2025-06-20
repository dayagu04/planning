#pragma once

#include <cstddef>

#include "apa_obstacle_manager.h"
#include "geometry_math.h"
#include "point_cloud_obstacle.h"
#include "polygon_base.h"
#include "pose2d.h"
#include "src/library/convex_collision_detection/gjk2d_interface.h"
#include "src/modules/apa_function/apa_param_config.h"

namespace planning {

enum class VehCollisionPosition {
  NONE = 0,
  LEFT_MIRROR = 1,
  RIGHT_MIRROR,
  BODY,
  MAX_POLYGON_NO_COLLISION
};

enum class PathCheckRequest {
  NONE = 0,
  COLLISION_CHECK = 1,
  DISTANCE_CHECK = 2,
};

// check path safe for polygon obs and ogm obstacle by gjk for lon decision.
class PathSafeChecker {
 public:
  PathSafeChecker(
      const std::shared_ptr<apa_planner::ApaObstacleManager>& obs_manager) {
    SetObstacle(obs_manager);
  }
  ~PathSafeChecker() {}

  /**
   * [in]: ego_pose, requst, lat_buffer, lon_buffer, path
   */
  void Excute(const Pose2D& ego_pose, const PathCheckRequest requst,
              const double lat_buffer, const double lon_buffer,
              std::vector<pnc::geometry_lib::PathPoint>& path);

  const bool IsPathCollision() const { return is_path_collision_; }

  const bool IsEgoCollision() const { return is_ego_collision_; }

  void UpdatePathValidDist(
      const std::vector<pnc::geometry_lib::PathPoint>& path,
      const Pose2D& ego_pose);

  const double GetPathValidDist() const { return path_valid_dist_; }

  const size_t GetPathCollisionID() const { return path_collision_idx_; }

  bool CalcEgoCollision(const Pose2D& ego_pose, const double lat_buffer,
                        const double lon_buffer);

  const bool IsPolygonCollision(const Polygon2D* car);

  const double GetEgoPathProjectS() const { return ego_project_s_; }

 private:
  size_t GetNearestPathPoint(
      const std::vector<pnc::geometry_lib::PathPoint>& path,
      const Pose2D& pose);

  void GenerateMirrorPolygon(Polygon2D* box, const double x_length,
                             const double y_length, const Position2D& center);

  const bool IsVehicleCollision(const Transform2d& tf,
                                PolygonFootPrint* foot_print,
                                VehCollisionPosition* collision_info);

  // bounding box car body for coarse safe check.
  // bbox_lat_buffer: 车辆最外侧简化成bound box, 而不是非凸的多边形.
  void GenerateVehBox(const double lateral_safe_buffer,
                      const double lon_safe_buffer,
                      const double max_bbox_lat_buffer);

  void GetCompactCarPolygonByParam(Polygon2D* box, const double lat_buffer,
                                   const double lon_buffer);

  void DebugCollisionInfo(
      const size_t path_end_id, const VehCollisionPosition collision_component,
      const Pose2D& ego_pose,
      const std::vector<pnc::geometry_lib::PathPoint>& path) const;

  void ExcuteCollisionCheck(
      const std::vector<pnc::geometry_lib::PathPoint>& path,
      const Pose2D& ego_pose);

  // 做速度规划时，和障碍物距离超过0.5米的点，不必进行速度考虑，保持默认巡航速度即可.
  void ExcuteDistanceCheck(const Pose2D& ego_pose,
                           std::vector<pnc::geometry_lib::PathPoint>& path);

  const double GetVehicleDistance(const Transform2d& tf,
                                  PolygonFootPrint* foot_print,
                                  VehCollisionPosition* collision_info);

  const bool GetPolygonDistance(const Polygon2D* polygon, double* min_dist);

  void GenerateEgoS(const size_t nearest_id,
                    const std::vector<pnc::geometry_lib::PathPoint>& path,
                    const Pose2D& pose);

  void SetObstacle(
      const std::shared_ptr<apa_planner::ApaObstacleManager>& obs_manager) {
    obs_manager_ = obs_manager;
    return;
  }

 private:
  std::shared_ptr<apa_planner::ApaObstacleManager> obs_manager_;

  bool is_ego_collision_;
  bool is_path_collision_;
  size_t path_collision_idx_;
  GJK2DInterface gjk_interface_;

  size_t path_nearest_idx_;
  double path_valid_dist_;

  PolygonFootPrint polygon_foot_print_;

  double lat_buffer_;
  double lon_buffer_;

  double ego_project_s_;
};

}  // namespace planning
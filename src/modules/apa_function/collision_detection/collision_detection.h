#ifndef __COLLISION_DETECTION_H__
#define __COLLISION_DETECTION_H__

#include <cstdint>
#include <vector>

#include "Eigen/Core"
#include "apa_param_config.h"
#include "dubins_lib.h"
#include "euler_distance_transform.h"
#include "geometry_math.h"
#include "local_view.h"
#include "math_lib.h"
#include "path_safe_checker.h"
#include "planning_plan_c.h"
#include "transform_lib.h"

namespace planning {
namespace apa_planner {

const std::vector<Eigen::Vector2d> GetCarMaxPolygon(
    const pnc::geometry_lib::PathPoint &current_pose);

const pnc::geometry_lib::RectangleBound CalCarRectangleBound(
    const pnc::geometry_lib::PathPoint &current_pose);

class CollisionDetector {
 public:
  enum ObsType {
    NONE_OBS,
    CHANNEL_OBS,
    CURB_OBS,
    TLANE_OBS,
    TLANE_BOUNDARY_OBS,
    LINEARC_OBS,
    FUSION_OBS,
    GROUND_LINE_OBS,
    RECORD_OBS,
    VIRTUAL_OBS,
    USS_OBS,
    COUNT_OBS,
  };

  enum class ObsSlotType : uint8_t {
    OBS_INVALID,
    SLOT_INSIDE_OBS,
    SLOT_OUTSIDE_OBS,
    SLOT_IN_OBS,
    SLOT_DIRECTLY_BEHIND_OBS,
    SLOT_ENTRANCE_OBS,
    SLOT_OUT_OBS,
    OBS_COUNT,
  };

  struct CarMoveBound {
    double min_x;
    double min_y;
    double max_x;
    double max_y;
  };

 public:
  CollisionDetector() { Init(); }
  struct CollisionResult {
    bool collision_flag = false;
    double remain_dist = 25.0;
    double remain_car_dist = 25.0;
    double remain_obstacle_dist = 25.0;
    std::pair<double, pnc::geometry_lib::PathPoint> pt_closest2obs{
        26.8, pnc::geometry_lib::PathPoint()};
    Eigen::Vector2d col_pt_ego_global;
    Eigen::Vector2d col_pt_ego_local;
    Eigen::Vector2d col_pt_obs_global;
    int car_line_order = -1;
    std::vector<Eigen::Vector2d> traj_bound;
    ObsType obs_type = NONE_OBS;
  };

  struct Paramters {
    bool is_side_mirror_expand = true;
    double lat_inflation = apa_param.GetParam().car_lat_inflation_normal;
    double bound_expand = 0.5;
    bool use_bounding_box = false;
    Paramters() = default;
    Paramters(const double lat_inf, bool set_side_mirror_expand = true) {
      lat_inflation = lat_inf;
      is_side_mirror_expand = set_side_mirror_expand;
    }

    void Reset() {
      is_side_mirror_expand = true;
      lat_inflation = apa_param.GetParam().car_lat_inflation_normal;
      bound_expand = 0.5;
      use_bounding_box = false;
    }
  };

  void Init();
  void Reset();

  const CollisionResult Update(const pnc::geometry_lib::LineSegment &line_seg,
                               const double heading_start);

  const CollisionResult UpdateByObsMap(
      const std::vector<pnc::geometry_lib::PathPoint> &pt_vec,
      const double lat_buffer, const double lon_buffer);

  const CollisionResult UpdateByObsMapUss(
      const std::vector<pnc::geometry_lib::PathPoint> &pt_vec,
      const double lat_buffer, const double lon_buffer);

  const CollisionResult UpdateByObsMap(
      const pnc::geometry_lib::PathSegment &path_seg, const double lat_buffer,
      const double lon_buffer);

  const CollisionResult UpdateByObsMap(
      const pnc::geometry_lib::LineSegment &line_seg,
      const double heading_start);

  const CollisionResult Update(const pnc::geometry_lib::Arc &arc,
                               const double heading_start);

  const CollisionResult UpdateByObsMap(const pnc::geometry_lib::Arc &arc,
                                       const double heading_start);

  const CollisionResult Update(
      const pnc::geometry_lib::LineSegment &line_seg,
      const double heading_start,
      const std::vector<Eigen::Vector2d> &obs_pt_global_vec);

  const CollisionResult Update(
      const pnc::geometry_lib::Arc &arc, const double heading_start,
      const std::vector<Eigen::Vector2d> &obs_pt_global_vec);

  const CollisionResult UpdateByLineObs(
      const pnc::geometry_lib::LineSegment &line_seg,
      const double heading_start);

  const CollisionResult UpdateByLineObs(const pnc::geometry_lib::Arc &arc,
                                        const double heading_start);

  const CollisionResult UpdateByLineObs(
      const pnc::geometry_lib::LineSegment &line_seg,
      const double heading_start,
      const std::vector<pnc::geometry_lib::LineSegment> &obs_line_global_vec);

  const CollisionResult UpdateByLineObs(
      const pnc::geometry_lib::Arc &arc, const double heading_start,
      const std::vector<pnc::geometry_lib::LineSegment> &obs_line_global_vec);

  void SetObstacles(const std::vector<Eigen::Vector2d> &obs_pt_global_vec);
  void SetObstacles(const std::vector<Eigen::Vector2d> &obs_pt_global_vec,
                    const size_t obs_type);
  void AddObstacles(const std::vector<Eigen::Vector2d> &obs_pt_global_vec);
  void AddObstacles(const std::vector<Eigen::Vector2d> &obs_pt_global_vec,
                    const size_t obs_type);
  void AddObstacles(const Eigen::Vector2d &obs_pt_global);
  void AddObstacles(const Eigen::Vector2d &obs_pt_global,
                    const size_t obs_type);
  const std::vector<Eigen::Vector2d> &GetObstacles() const {
    return obs_pt_global_vec_;
  }

  const std::unordered_map<size_t, std::vector<Eigen::Vector2d>> &
  GetObstaclesMap() const {
    return obs_pt_global_map_;
  }

  void DeleteGivenTypeObstacles(const size_t obs_type) {
    obs_pt_global_map_.erase(obs_type);
  }

  void ClearObstacles();

  void DeleteObstacles(const size_t obs_type);

  void SetLineObstacles(
      const std::vector<pnc::geometry_lib::LineSegment> &obs_line_global_vec);
  void AddLineObstacles(
      const std::vector<pnc::geometry_lib::LineSegment> &obs_line_global_vec);
  void AddLineObstacles(const pnc::geometry_lib::LineSegment &obs_line_global);
  const std::vector<pnc::geometry_lib::LineSegment> GetLineObstacles() {
    return obs_line_global_vec_;
  }

  void SetParam(const Paramters &param);

  const Paramters GetParam() { return param_; }

  void SetLatInflation();

  const double CalMinDistObs2Car(const Eigen::Vector2d &obs,
                                 const pnc::geometry_lib::PathPoint &ego_pose);

  const bool IsObstacleInPath(
      const pnc::geometry_lib::PathSegment &temp_path_seg,
      const double long_safe_dist, const bool need_reverse);

  const bool IsObstacleInPath(
      const std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const double sample_ds, const bool need_reverse);

  const bool IsObstacleInCar(const pnc::geometry_lib::PathPoint &ego_pose);

  const bool IsObstacleInCar(const Eigen::Vector2d &obs_pos,
                             const pnc::geometry_lib::PathPoint &ego_pose);

  const bool IsObstacleInCar(const Eigen::Vector2d &obs_pos,
                             const pnc::geometry_lib::PathPoint &ego_pose,
                             double safe_dist);

  const double CalClosestDistFromObsToCar(
      const pnc::geometry_lib::PathPoint &ego_pose, const double lat_buffer,
      const bool safe_flag = true);

  const bool CalTrajBound(std::vector<Eigen::Vector2d> &traj_bound,
                          const pnc::geometry_lib::PathPoint &start_pose,
                          const pnc::geometry_lib::PathPoint &target_pose,
                          bool is_line);

  const std::vector<Eigen::Vector2d> CalTrajBound(
      const pnc::geometry_lib::PathPoint &start_pose,
      const pnc::geometry_lib::PathPoint &target_pose);

  const std::vector<Eigen::Vector2d> CalTrajBound(
      const pnc::geometry_lib::PathPoint &start_pose,
      const pnc::geometry_lib::PathPoint &target_pose,
      const pnc::geometry_lib::Arc &arc);

  const double GetCarMaxX(const pnc::geometry_lib::PathPoint &ego_pose);

  const std::vector<Eigen::Vector2d> CalPathSegBound(
      const pnc::geometry_lib::PathSegment &path_seg);

  static const ObsSlotType GetObsSlotType(
      const Eigen::Vector2d &obs,
      const std::pair<Eigen::Vector2d, Eigen::Vector2d> &slot_pt,
      const bool is_left_side, const bool is_replan,
      const bool is_vertical_slot = true);

  const bool IsObstacleInPolygon(
      const std::vector<Eigen::Vector2d> &vertex_vec);

 private:
  std::vector<pnc::geometry_lib::LineSegment> car_line_local_vec_;

  std::vector<Eigen::Vector2d> car_local_vertex_vec_;

  std::vector<Eigen::Vector2d> origin_car_local_vertex_vec_;

  std::vector<Eigen::Vector2d> obs_pt_global_vec_;

  std::unordered_map<size_t, std::vector<Eigen::Vector2d>> obs_pt_global_map_;

  std::vector<pnc::geometry_lib::LineSegment> obs_line_global_vec_;

  pnc::geometry_lib::RectangleBound path_seg_rectangle_bound_;
  std::vector<Eigen::Vector2d> origin_car_local_rectangle_vertex_vec_;

  Paramters param_;
};
}  // namespace apa_planner
}  // namespace planning
#endif
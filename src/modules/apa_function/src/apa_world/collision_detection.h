#ifndef __COLLISION_DETECTION_H__
#define __COLLISION_DETECTION_H__

#include <cstdint>
#include <vector>

#include "Eigen/Core"
#include "apa_param_setting.h"
#include "dubins_lib.h"
#include "geometry_math.h"
#include "local_view.h"
#include "math_lib.h"
#include "planning_plan_c.h"
#include "transform_lib.h"

namespace planning {

class CollisionDetector {
 public:
  enum ObsType {
    NONE_OBS,
    CHANNEL_OBS,
    TLANE_OBS,
    LINEARC_OBS,
    FUSION_OBS,
    RECORD_OBS,
    COUNT_OBS,
  };

  enum class ObsSlotType : uint8_t {
    OBS_INVALID,
    SLOT_INSIDE_OBS,
    SLOT_OUTSIDE_OBS,
    SLOT_IN_OBS,
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
    Eigen::Vector2d col_pt_ego_global;
    Eigen::Vector2d col_pt_ego_local;
    Eigen::Vector2d col_pt_obs_global;
    int car_line_order = -1;
    CarMoveBound car_move_bound;
    ObsType obs_type = NONE_OBS;
  };

  struct Paramters {
    double lat_inflation = apa_param.GetParam().car_lat_inflation_normal;
    Paramters() = default;
    Paramters(const double lat_inf) { lat_inflation = lat_inf; }

    void Reset() {
      lat_inflation = apa_param.GetParam().car_lat_inflation_normal;
    }
  };

  void Init();
  void Reset();

  const CollisionResult Update(const pnc::geometry_lib::LineSegment &line_seg,
                               const double heading_start);

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

  void SetParam(Paramters param);

  void SetLatInflation();

  const double CalMinDistObs2Car(const Eigen::Vector2d &obs,
                                 const pnc::geometry_lib::PathPoint &ego_pose);

  const bool IsObstacleInCar(const Eigen::Vector2d &obs_pos,
                             const pnc::geometry_lib::PathPoint &ego_pose);

  const bool IsObstacleInCar(const Eigen::Vector2d &obs_pos,
                             const pnc::geometry_lib::PathPoint &ego_pose,
                             double safe_dist);

  const bool CalCarMoveBound(CarMoveBound &car_move_bound,
                             const pnc::geometry_lib::PathPoint &start_pose,
                             const pnc::geometry_lib::PathPoint &target_pose);

  const ObsSlotType GetObsSlotType(
      const Eigen::Vector2d &obs,
      const std::pair<Eigen::Vector2d, Eigen::Vector2d> &slot_pt,
      const bool is_left_side, const bool is_vertical_slot = true);

 private:
  std::vector<pnc::geometry_lib::LineSegment> car_line_local_vec_;

  std::vector<Eigen::Vector2d> car_local_vertex_vec_;

  std::vector<Eigen::Vector2d> obs_pt_global_vec_;

  std::unordered_map<size_t, std::vector<Eigen::Vector2d>> obs_pt_global_map_;

  std::vector<pnc::geometry_lib::LineSegment> obs_line_global_vec_;

  Paramters param_;
};

}  // namespace planning
#endif
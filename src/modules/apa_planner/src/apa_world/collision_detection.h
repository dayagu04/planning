#ifndef __COLLISION_DETECTION_H__
#define __COLLISION_DETECTION_H__

#include <vector>

#include "Eigen/Core"
#include "apa_param_setting.h"
#include "dubins_lib.h"
#include "geometry_math.h"
#include "local_view.h"
#include "math_lib.h"
#include "planning_plan.pb.h"
#include "transform_lib.h"

namespace planning {

class CollisionDetector {
 public:
  CollisionDetector() { Init(); }
  struct CollisionResult {
    bool collision_flag = false;
    double remain_dist = 25.0;
    double remain_car_dist = 25.0;
    double remain_obstacle_dist = 25.0;
    Eigen::Vector2d collision_point;
    Eigen::Vector2d collision_point_global;
  };

  struct Paramters {
    double lat_inflation = apa_param.GetParam().lat_inflation;
    void Reset() { lat_inflation = apa_param.GetParam().lat_inflation; }
  };

  void Init();
  void Reset();

  const CollisionResult Update(const pnc::geometry_lib::LineSegment &line_seg,
                               const double heading_start);

  const CollisionResult Update(const pnc::geometry_lib::Arc &arc,
                               const double heading_start);

  const CollisionResult Update(
      const pnc::geometry_lib::LineSegment &line_seg,
      const double heading_start,
      const std::vector<Eigen::Vector2d> &obstacle_global_vec);

  const CollisionResult Update(
      const pnc::geometry_lib::Arc &arc, const double heading_start,
      const std::vector<Eigen::Vector2d> &obstacle_global_vec);

  void SetObstacles(const std::vector<Eigen::Vector2d> &obstacle_global_vec);
  void AddObstacles(const std::vector<Eigen::Vector2d> &obstacle_global_vec);
  void AddObstacles(const Eigen::Vector2d &obstacle_global);
  const std::vector<Eigen::Vector2d> GetObstacles() {
    return obstacle_global_vec_;
  }

  void SetParam(Paramters param);

  void SetLatInflation();

  const double CalMinDistObs2Car(const Eigen::Vector2d &obs,
                                 const pnc::geometry_lib::PathPoint &ego_pose);

  const bool IsObstacleInCar(const Eigen::Vector2d &obs_pos,
                             const pnc::geometry_lib::PathPoint &ego_pose);

 private:
  std::vector<pnc::geometry_lib::LineSegment> car_line_local_vec_;

  std::vector<Eigen::Vector2d> obstacle_global_vec_;

  Paramters param_;
};

}  // namespace planning
#endif
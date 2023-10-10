#ifndef __COLLISION_DETECTION_H__
#define __COLLISION_DETECTION_H__

#include <cstddef>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "dubins_lib/dubins_lib.h"
#include "dubins_lib/geometry_math.h"
#include "local_view.h"
#include "math_lib.h"
#include "planning_plan.pb.h"
#include "uss_obstacle_avoidance/uss_obstacle_avoidance.h"

namespace planning {

class CollisionDetector {
 public:
  void Init();
  void Reset();

  void GenObstacles();

  void GenCarCircles(
      std::vector<pnc::dubins_lib::DubinsLibrary::PathPoint> &path_point_vec);

  bool CollisionDetect();

  void SetUssOA(const UssObstacleAvoidance *uss_oa_ptr) {
    uss_oa_ptr_ = uss_oa_ptr;
  }

  void SetLocalView(const LocalView *local_view_ptr) {
    local_view_ptr_ = local_view_ptr;
  }

 private:
  // only updates when init
  std::vector<pnc::geometry_lib::Circle> car_circle_local_vec_;

  // updates every frame
  std::vector<pnc::geometry_lib::LineSegment> obstacle_global_vec_;

  std::vector<std::vector<pnc::geometry_lib::Circle>>
      car_circle_global_vec_path_vec_;

  const UssObstacleAvoidance *uss_oa_ptr_;
  const LocalView *local_view_ptr_;
};

};  // namespace planning

#endif

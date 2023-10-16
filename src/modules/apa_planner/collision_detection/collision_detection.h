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

  void GenObstaclesByUssOA();

  void GenCarCircles(
      const std::vector<pnc::dubins_lib::DubinsLibrary::PathPoint>
          &path_point_vec);

  bool CollisionDetect();

  void SetUssOA(const UssObstacleAvoidance *uss_oa_ptr) {
    uss_oa_ptr_ = uss_oa_ptr;
  }

  void SetObstacles(
      const std::vector<pnc::geometry_lib::LineSegment> &obstacle_global_vec) {
    obstacle_global_vec_ = obstacle_global_vec;
  }

  void AddObstacle(const pnc::geometry_lib::LineSegment &line_segment) {
    obstacle_global_vec_.emplace_back(line_segment);
  }

  void ClearObstacles() { obstacle_global_vec_.clear(); }

  void SetLocalView(const LocalView *local_view_ptr) {
    local_view_ptr_ = local_view_ptr;
  }

  void GenObstaclesSimulation(
      const std::vector<pnc::geometry_lib::LineSegment> &obstacle_line_vec);

  const std::vector<std::vector<pnc::geometry_lib::Circle>> GetCarCircle()
      const {
    return car_circle_global_vec_path_vec_;
  }

  const std::vector<pnc::geometry_lib::LineSegment> GetObstacles() const {
    return obstacle_global_vec_;
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

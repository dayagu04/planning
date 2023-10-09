#ifndef __COLLISION_DETECTION_H__
#define __COLLISION_DETECTION_H__

#include <cstddef>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/src/Core/Matrix.h"
#include "dubins_lib/geometry_math.h"
#include "local_view.h"
#include "planning_plan.pb.h"
#include "uss_obstacle_avoidance/uss_obstacle_avoidance.h"

namespace planning {

class CollisionDetector {
 public:
  struct PoseTf {
    Eigen::Vector2d pn = Eigen::Vector2d::Zero();
    double theta = 0.0;
    Eigen::Matrix2d rot_m = Eigen::Matrix2d::Identity();

    void SetEgoPose(Eigen::Vector2d &pn_ego, double theta_ego) {
      pn = pn_ego;
      theta = theta_ego;
      const double cos_theta = std::cos(theta_ego);
      const double sin_theta = std::sin(theta_ego);
      rot_m << cos_theta, -sin_theta, sin_theta, cos_theta;
    }

    const Eigen::Vector2d GetGlobalPos(const Eigen::Vector2d &pb) const {
      return rot_m * pb + pn;
    }
  };

  void Init();
  void Reset();

  void GenObstacles();
  void GenCarCircles();

  bool CollisionDetect(std::vector<Eigen::Vector2d> &pos_vec,
                       std::vector<double> &theta_vec);

  void SetUssOA(const UssObstacleAvoidance *uss_oa_ptr) {
    uss_oa_ptr_ = uss_oa_ptr;
  }

  void SetLocalView(const LocalView *local_view_ptr) {
    local_view_ptr_ = local_view_ptr;
  }

 private:
  PoseTf tf_;

  // only updates when init
  std::vector<pnc::geometry_lib::Circle> car_circle_local_vec_;

  // updates every frame
  std::vector<pnc::geometry_lib::LineSegment> obstacle_local_vec_;
  std::vector<pnc::geometry_lib::LineSegment> obstacle_global_vec_;

  std::vector<pnc::geometry_lib::Circle> car_circle_global_vec_;

  const UssObstacleAvoidance *uss_oa_ptr_;
  const LocalView *local_view_ptr_;
};

};  // namespace planning

#endif

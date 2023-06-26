#pragma once
#include "config/basic_type.h"
#include "ego_state_manager.h"
#include "math/math_utils.h"
#include "math/polygon2d.h"
#include "utils/frenet_coordinate_system.h"

namespace planning {

class FrenetEgoState {
 public:
  FrenetEgoState() = default;
  ~FrenetEgoState() = default;

  void update(const std::shared_ptr<FrenetCoordinateSystem> &frenet_coord,
              const planning::EgoStateManager &ego_state);

  double s() const { return s_; }
  double l() const { return l_; }
  double heading_angle() const { return heading_angle_; }
  double velocity() const { return velocity_; }
  double velocity_s() const { return velocity_ * std::cos(heading_angle_); }
  double velocity_l() const { return velocity_ * std::sin(heading_angle_); }
  double acc() const { return acc_; }
  const FrenetBoundary &boundary() const { return boundary_; }
  const FrenetBoundaryCorners &corners() const { return corners_; }
  const planning_math::Polygon2d &polygon() const { return polygon_; }
  bool planning_init_point_valid() const { return planning_init_point_valid_; }
  const PlanningInitPoint &planning_init_point() const {
    return planning_init_point_;
  }

 private:
  double s_;
  double l_;
  double velocity_;
  double acc_;
  double jerk_;
  double heading_angle_;
  FrenetBoundary boundary_;
  FrenetBoundaryCorners corners_;
  planning_math::Polygon2d polygon_;

  PlanningInitPoint planning_init_point_;
  bool planning_init_point_valid_ = false;
};

}  // namespace planning

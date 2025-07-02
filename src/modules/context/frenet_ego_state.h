#pragma once
#include "config/basic_type.h"
#include "ego_state_manager.h"
#include "math/math_utils.h"
#include "math/polygon2d.h"
#include "utils/agent_sl.h"
#include "utils/frenet_coordinate_system.h"
#include "utils/kd_path.h"
namespace planning {

class FrenetEgoState {
 public:
  FrenetEgoState() = default;
  ~FrenetEgoState() = default;

  void update(const std::shared_ptr<planning_math::KDPath> &frenet_coord,
              const planning::EgoStateManager &ego_state);

  double s() const { return s_; }
  double l() const { return l_; }
  double head_s() const { return head_s_; }
  double head_l() const { return head_l_; }
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
  const AgentSLInfo &ego_init_sl_info() const { return ego_init_sl_info_; }
  bool is_valid() const { return is_valid_; }
 private:
  double s_;
  double l_;
  double head_s_;
  double head_l_;
  double velocity_;
  double acc_;
  double jerk_;
  double heading_angle_;
  FrenetBoundary boundary_;
  FrenetBoundaryCorners corners_;
  planning_math::Polygon2d polygon_;

  PlanningInitPoint planning_init_point_;
  bool planning_init_point_valid_ = false;
  AgentSLInfo ego_init_sl_info_;
  bool is_valid_ = true;
};

}  // namespace planning

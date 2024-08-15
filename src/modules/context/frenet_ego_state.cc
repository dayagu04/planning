#include "frenet_ego_state.h"

#include <vector>

#include "session.h"
#include "utils/frenet_coordinate_system.h"
#include "vehicle_config_context.h"

namespace planning {

void FrenetEgoState::update(
    const std::shared_ptr<planning_math::KDPath> &frenet_coord,
    const planning::EgoStateManager &ego_state) {
  // Step 1) update location, velocity
  Point2D frenet_point, cart_point;
  cart_point.x = ego_state.ego_carte().x;
  cart_point.y = ego_state.ego_carte().y;
  if (frenet_coord->XYToSL(cart_point, frenet_point)) {
    s_ = frenet_point.x;
    l_ = frenet_point.y;
  } else {
    LOG_DEBUG("kd_path coordinate conversion failed");
  };
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  heading_angle_ = planning_math::NormalizeAngle(
      ego_state.heading_angle() - frenet_coord->GetPathCurveHeading(s_));
  velocity_ = ego_state.ego_v();
  acc_ = ego_state.ego_acc();
  jerk_ = ego_state.jerk();
  corners_.s_front_left =
      s_ + vehicle_param.front_edge_to_rear_axle * std::cos(heading_angle_) -
      vehicle_param.width / 2.0 * std::sin(heading_angle_);
  corners_.l_front_left =
      l_ + vehicle_param.front_edge_to_rear_axle * std::sin(heading_angle_) +
      vehicle_param.width / 2.0 * std::cos(heading_angle_);
  corners_.s_front_right =
      s_ + vehicle_param.front_edge_to_rear_axle * std::cos(heading_angle_) +
      vehicle_param.width / 2.0 * std::sin(heading_angle_);
  corners_.l_front_right =
      l_ + vehicle_param.front_edge_to_rear_axle * std::sin(heading_angle_) -
      vehicle_param.width / 2.0 * std::cos(heading_angle_);
  corners_.s_rear_left =
      s_ - vehicle_param.rear_edge_to_rear_axle * std::cos(heading_angle_) -
      vehicle_param.width / 2.0 * std::sin(heading_angle_);
  corners_.l_rear_left =
      l_ - vehicle_param.rear_edge_to_rear_axle * std::sin(heading_angle_) +
      vehicle_param.width / 2.0 * std::cos(heading_angle_);
  corners_.s_rear_right =
      s_ - vehicle_param.rear_edge_to_rear_axle * std::cos(heading_angle_) +
      vehicle_param.width / 2.0 * std::sin(heading_angle_);
  corners_.l_rear_right =
      l_ - vehicle_param.rear_edge_to_rear_axle * std::sin(heading_angle_) -
      vehicle_param.width / 2.0 * std::cos(heading_angle_);

  // Step 2) update polygon
  auto enu_polygon = ego_state.polygon();
  std::vector<planning_math::Vec2d> frenet_corners;
  for (auto &pt : enu_polygon.points()) {
    Point2D frenet_corner, cart_corner;
    cart_corner.x = pt.x();
    cart_corner.y = pt.y();
    (void)frenet_coord->XYToSL(cart_corner, frenet_corner);
    frenet_corners.emplace_back(
        planning_math::Vec2d(frenet_corner.x, frenet_corner.y));
  }
  polygon_ = planning_math::Polygon2d(frenet_corners);

  // Step 3) update boundary
  auto max_double = std::numeric_limits<double>::max();
  auto min_double = std::numeric_limits<double>::min();
  boundary_ = FrenetBoundary{max_double, min_double, max_double, min_double};
  for (auto &pt : frenet_corners) {
    boundary_.s_start = std::min(boundary_.s_start, pt.x());
    boundary_.s_end = std::max(boundary_.s_end, pt.x());
    boundary_.l_start = std::min(boundary_.l_start, pt.y());
    boundary_.l_end = std::max(boundary_.l_end, pt.y());
  }

  // Step 5) planning init point
  planning_init_point_ = ego_state.planning_init_point();

  CartesianState cstate;
  cstate.x = planning_init_point_.x;
  cstate.y = planning_init_point_.y;
  //   cstate.x = 10.;
  //   cstate.y = 0.;
  cstate.yaw = planning_init_point_.heading_angle;
  cstate.speed = std::max(planning_init_point_.v, 0.0);
  cstate.acceleration = planning_init_point_.a;
  cstate.curvature = planning_init_point_.curvature;
  auto ok = frenet_coord->CartStateToFrenetState(
      cstate, planning_init_point_.frenet_state);
  LOG_DEBUG(
      "planning_init_point: rel_t: %f, x: %f, y: %f, yaw: %f, v: %f, a: %f, s: %f, l: %f, dr/ds: %f, dds/drdr: %f | \
           ego_pose: x: %f, y: %F, s: %f, l: %f\n",
      planning_init_point_.relative_time, cstate.x, cstate.y, cstate.yaw,
      planning_init_point_.v, planning_init_point_.a,
      planning_init_point_.frenet_state.s, planning_init_point_.frenet_state.r,
      planning_init_point_.frenet_state.dr_ds,
      planning_init_point_.frenet_state.ddr_dsds, ego_state.ego_carte().x,
      ego_state.ego_carte().y, s_, l_);
  LOG_DEBUG("planning_init_point_valid: %d\n", ok);
  planning_init_point_valid_ = ok;

  planning_math::Vec2d center(
      planning_init_point_.x + std::cos(planning_init_point_.heading_angle) *
                                   vehicle_param.rear_axle_to_center,
      planning_init_point_.y + std::sin(planning_init_point_.heading_angle) *
                                   vehicle_param.rear_axle_to_center);
  planning_math::Box2d ego_init_box(center, planning_init_point_.heading_angle,
                                    vehicle_param.length, vehicle_param.width);

  GetLaneAgentSLInfo(frenet_coord, ego_init_box, &ego_init_sl_info_);
}

}  // namespace planning

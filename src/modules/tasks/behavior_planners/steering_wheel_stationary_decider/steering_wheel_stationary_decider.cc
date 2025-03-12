#include "steering_wheel_stationary_decider.h"
#include <cmath>
#include <cstddef>
#include <vector>

#include "debug_info_log.h"
#include "environmental_model.h"
#include "math/polygon2d.h"
#include "task_interface/lane_borrow_decider_output.h"

namespace planning {

SteeringWheelStationaryDecider::SteeringWheelStationaryDecider(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : Task(config_builder, session),
      config_(config_builder->cast<SteeringWheelStationaryDeciderConfig>()) {
  Init();
  name_ = "SteeringWheelStationaryDecider";
}

bool SteeringWheelStationaryDecider::Init() {
  Reset();
  // config
  safe_buffer_ = config_.safe_buffer;
  steer_angle_step_ = config_.steer_angle_step;
  min_steer_angle_ = config_.min_steer_angle / 57.3;  // rad
  steer_angle_thr_ = config_.steer_angle_thr;
  // vehicle param
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  ego_length_ = vehicle_param.length;
  ego_width_ = vehicle_param.max_width;
  front_edge_to_rear_axle_ = vehicle_param.front_edge_to_rear_axle;
  rear_edge_to_rear_axle_ = vehicle_param.rear_edge_to_rear_axle;
  rear_axle_to_center_ = vehicle_param.rear_axle_to_center;
  wheel_base_ = vehicle_param.wheel_base;
  steer_ratio_ = vehicle_param.steer_ratio;
  max_steer_angle_ = vehicle_param.max_steer_angle;
  max_front_wheel_angle_ = vehicle_param.max_front_wheel_angle;
  min_turn_radius_ = vehicle_param.min_turn_radius;
  // turning radius
  CalculateTurningRadius();
  // init output
  auto &steering_wheel_stationary_decider_output =
      session_->mutable_planning_context()
          ->mutable_steering_wheel_stationary_decider_output();
  steering_wheel_stationary_decider_output.Clear();
  return true;
}

void SteeringWheelStationaryDecider::CalculateTurningRadius() {
  // std::vector<double> turning_radius;
  // std::vector<double> outer_turning_radius;
  // double max_steer_angle = max_steer_angle_ * 57.3;  // deg
  // for (double steer_angle = max_steer_angle;
  //      steer_angle > min_steer_angle_;
  //      steer_angle -= steer_angle_step_) {
  //   double front_wheel_angle = steer_angle / steer_ratio_ / 57.3;  // rad
  //   double r = wheel_base_ / std::tan(front_wheel_angle);
  //   double triangle_side_length = (0.5 * ego_width_) + r;
  //   double R =
  //       std::sqrt(triangle_side_length * triangle_side_length +
  //                 front_edge_to_rear_axle_ * front_edge_to_rear_axle_);
  //   turning_radius.emplace_back(r);
  //   outer_turning_radius.emplace_back(R);
  // }
  // min_inner_radius_ = turning_radius.front();
  // max_inner_radius_ = turning_radius.back();
  // min_outer_radius_ = outer_turning_radius.front();
  // max_outer_radius_ = outer_turning_radius.back();
  // turning_radius_spline_.set_points(outer_turning_radius, turning_radius);
  double min_front_wheel_angle = min_steer_angle_ / steer_ratio_;
  double max_front_wheel_angle = max_steer_angle_ / steer_ratio_;
  max_inner_radius_ = wheel_base_ / std::tan(min_front_wheel_angle);
  min_inner_radius_ = wheel_base_ / std::tan(max_front_wheel_angle);
}

bool SteeringWheelStationaryDecider::Execute() {
  LOG_DEBUG("=======SteeringWheelStationaryDecider======= \n");
  if (!PreCheck()) {
    LOG_DEBUG("PreCheck failed\n");
    return false;
  }

  if (session_->is_hpp_scene()) {
    CalculateSteeringWheelAngleForHPP();  // static
  } else {
    CalculateSteeringWheelAngle();  // static
  }

  UpdateOutput();

  return true;
}

bool SteeringWheelStationaryDecider::CalculateSteeringWheelAngleForHPP() {
  const auto &current_state =
    session_->environmental_model().get_local_view().function_state_machine_info.current_state;
  if ((current_state != iflyauto::FunctionalState_HPP_CRUISE_ROUTING) &&
      (current_state != iflyauto::FunctionalState_HPP_CRUISE_SEARCHING)) {
    Reset();
    return false;
  }
  // session_->environmental_model().GetVehicleDbwStatus();
  // ego state
  const auto &ego_state_manager = session_->environmental_model().get_ego_state_manager();
  double ego_x = ego_state_manager->ego_pose().x;
  double ego_y = ego_state_manager->ego_pose().y;
  double ego_heading = ego_state_manager->ego_pose().theta;
  double ego_vel = ego_state_manager->ego_v();
  double ego_steer_angle = ego_state_manager->ego_steer_angle();  // rad
  // double init_vel = ego_state_manager->planning_init_point().v;
  double init_delta = ego_state_manager->planning_init_point().delta;
  double init_steer_angle = init_delta * steer_ratio_;
  // pass internal condition
  // bool find_pass_interval =
  //     session_->planning_context().lateral_obstacle_decider_output().find_pass_interval;
  // if (!find_pass_interval) {
  //   Reset();
  //   return false;
  // }
  // trajectory length condition
  const auto &traj_points = session_->planning_context().planning_result().traj_points;
  if (traj_points.back().s - traj_points.front().s >= 1.0) {
    Reset();
    return false;
  }
  // velocity condition
  if (ego_vel > 1e-3) {
    Reset();
    return false;
  }
  // steer angle condition
  // if (max_steer_angle_ - ego_steer_angle < 1e-6) {
  //   Reset();
  //   return false;
  // }
  // obstalce condition
  // GetTurningDirectionByPassInterval();  // wait pass interval
  GetTurningDirectionByRef();  // temp
  LOG_DEBUG("turning_direction:%f\n", turning_direction_);
  JSON_DEBUG_VALUE("turning_direction", turning_direction_)
  if (turning_direction_ > 0) {
    turning_direction_ = 1.0;
  } else if (turning_direction_ < 0) {
    turning_direction_ = -1.0;
  } else {
    Reset();
    return false;
  }
  const auto &target_obstacle = GetFrontNearestObstacle();
  if (target_obstacle == nullptr) {
    Reset();
    return false;
  }
  size_t last_target_target_obstacle_id = target_obstacle_id_;
  target_obstacle_id_ = target_obstacle->id();
  LOG_DEBUG("turning_target_obstacle_id:%zu\n", target_obstacle_id_);
  JSON_DEBUG_VALUE("turning_target_obstacle_id", target_obstacle_id_)

  double min_turning_radius = CalculateMinTurningRadius(target_obstacle);
  if (min_turning_radius < min_inner_radius_  ||
      min_turning_radius > max_inner_radius_) {
    Reset();
    return false;
  }
  // double triangle_side_length = (0.5 * ego_width_) + min_turning_radius;
  // double expected_outer_radius =
  //     std::sqrt(triangle_side_length * triangle_side_length +
  //               front_edge_to_rear_axle_ * front_edge_to_rear_axle_) -
  //     safe_buffer_;
  // if (expected_outer_radius < min_outer_radius_ ||
  //     expected_outer_radius > max_outer_radius_) {
  //   Reset();
  //   return false;
  // }
  double last_target_steering_angle = target_steering_angle_;
  double target_turning_radius = min_turning_radius;  // turning_radius_spline_(expected_outer_radius);
  double target_front_wheel_angle = std::atan(wheel_base_ / target_turning_radius);
  target_steering_angle_ = turning_direction_ * target_front_wheel_angle * steer_ratio_;  // rad
  if (is_need_steering_wheel_stationary_ &&
      last_target_target_obstacle_id == target_obstacle_id_) {
    target_obstacle_id_ = last_target_target_obstacle_id;
    target_steering_angle_ = last_target_steering_angle;
  }
  LOG_DEBUG("target_steering_angle:%f\n", (target_steering_angle_ * 57.3));
  JSON_DEBUG_VALUE("target_steering_angle", (target_steering_angle_ * 57.3))
  is_need_steering_wheel_stationary_ = true;
  if (is_need_steering_wheel_stationary_ &&
      std::fabs(target_steering_angle_ - ego_steer_angle) <= (3.0 / 57.3) &&
      std::fabs(ego_steer_angle - init_steer_angle) <= (3.0 / 57.3)) {
    Reset();
    return false;
  }
  return true;
}

bool SteeringWheelStationaryDecider::CalculateSteeringWheelAngle() {
  if (!session_->environmental_model().GetVehicleDbwStatus()) {
    Reset();
    return false;
  }
  // ego state
  const auto &ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  double ego_vel = ego_state_manager->ego_v();
  double ego_steer_angle = ego_state_manager->ego_steer_angle();  // rad
  // double init_vel = ego_state_manager->planning_init_point().v;
  double init_delta = ego_state_manager->planning_init_point().delta;
  double init_steer_angle = init_delta * steer_ratio_;
  // trajectory length condition
  const auto &traj_points = session_->planning_context().planning_result().traj_points;
  if (traj_points.back().s - traj_points.front().s >= 1.0) {
    Reset();
    return false;
  }
  // velocity condition
  if (ego_vel > 1e-3) {
    Reset();
    return false;
  }
  // steer angle condition
  // if (max_steer_angle_ - ego_steer_angle < 1e-6) {
  //   Reset();
  //   return false;
  // }
  // obstalce condition
  const auto &target_obstacle = GetFrontNearestObstacleByDecision();
  if (target_obstacle == nullptr) {
    Reset();
    return false;
  }
  size_t last_target_target_obstacle_id = target_obstacle_id_;
  target_obstacle_id_ = target_obstacle->id();
  LOG_DEBUG("turning_target_obstacle_id:%zu\n", target_obstacle_id_);
  JSON_DEBUG_VALUE("turning_target_obstacle_id", target_obstacle_id_)
  LOG_DEBUG("turning_direction:%f\n", turning_direction_);
  JSON_DEBUG_VALUE("turning_direction", turning_direction_)
  if (turning_direction_ > 0) {
    turning_direction_ = 1.0;
  } else if (turning_direction_ < 0) {
    turning_direction_ = -1.0;
  } else {
    Reset();
    return false;
  }
  if (!CheckLaneLineTypeByTurningDirection()){
    Reset();
    return false;
  }
  if (!CheckTurningSafety(target_obstacle)){
    Reset();
    return false;
  }

  double min_turning_radius = CalculateMinTurningRadius(target_obstacle);
  if (min_turning_radius < min_inner_radius_  ||
      min_turning_radius > max_inner_radius_) {
    Reset();
    return false;
  }
  // double triangle_side_length = (0.5 * ego_width_) + min_turning_radius;
  // double expected_outer_radius =
  //     std::sqrt(triangle_side_length * triangle_side_length +
  //               front_edge_to_rear_axle_ * front_edge_to_rear_axle_) -
  //     safe_buffer_;
  // if (expected_outer_radius < min_outer_radius_ ||
  //     expected_outer_radius > max_outer_radius_) {
  //   Reset();
  //   return false;
  // }
  double last_target_steering_angle = target_steering_angle_;
  double target_turning_radius = min_turning_radius;  // turning_radius_spline_(expected_outer_radius);
  double target_front_wheel_angle = std::atan(wheel_base_ / target_turning_radius);
  target_steering_angle_ = turning_direction_ * target_front_wheel_angle * steer_ratio_;  // rad
  if (is_need_steering_wheel_stationary_ &&
      last_target_target_obstacle_id == target_obstacle_id_ &&
      std::fabs(target_steering_angle_) <=
        std::fabs(last_target_steering_angle)) {
    target_obstacle_id_ = last_target_target_obstacle_id;
    target_steering_angle_ = last_target_steering_angle;
  }
  LOG_DEBUG("target_steering_angle:%f\n", (target_steering_angle_ * 57.3));
  JSON_DEBUG_VALUE("target_steering_angle", (target_steering_angle_ * 57.3))
  is_need_steering_wheel_stationary_ = true;
  if (is_need_steering_wheel_stationary_ &&
      std::fabs(target_steering_angle_ - ego_steer_angle) <= (3.0 / 57.3) &&
      std::fabs(ego_steer_angle - init_steer_angle) <= (3.0 / 57.3)) {
    Reset();
    return false;
  }
  return true;
}

double SteeringWheelStationaryDecider::CalculateMinTurningRadius(
    const std::shared_ptr<FrenetObstacle>& target_obstacle) {
  // ego state
  const auto &ego_state_manager = session_->environmental_model().get_ego_state_manager();
  double ego_x = ego_state_manager->ego_pose().x;
  double ego_y = ego_state_manager->ego_pose().y;
  double ego_heading = ego_state_manager->ego_pose().theta;
  // ref
  const auto &virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto &reference_path =
      virtual_lane_manager->get_current_lane()->get_reference_path();
  const auto &frenet_coord = reference_path->get_frenet_coord();
  double min_turning_radius = NL_NMAX;
  double target_point_x = 0;
  double target_point_y = 0;
  if (!target_obstacle->b_frenet_polygon_sequence_invalid() &&
      frenet_coord != nullptr) {
    const auto &frenet_polygon =
        target_obstacle->frenet_polygon_sequence()[0].second.points();
    for (const planning_math::Vec2d polygon_point : frenet_polygon) {
      Point2D cart_point;
      if (frenet_coord->SLToXY(
          Point2D(polygon_point.x() - safe_buffer_,
                               polygon_point.y() + turning_direction_ * safe_buffer_),
          cart_point)) {
        double dx = cart_point.x - ego_x;
        double dy = cart_point.y - ego_y;
        double numerator =
          dx * dx + dy * dy - front_edge_to_rear_axle_ * front_edge_to_rear_axle_ - 0.25 * ego_width_ * ego_width_;
        double denominator =
          ego_width_ - 2 * turning_direction_ * (dx * std::sin(ego_heading) - dy * std::cos(ego_heading));
        if (denominator == 0) {
          continue;
        }
        double turning_radius = numerator / denominator;
        if (turning_radius < 0) {
          continue;
        }
        if (turning_radius < min_turning_radius) {
          target_point_x = ego_x + dx;
          target_point_y = ego_y + dy;
        }
        min_turning_radius =
          std::min(min_turning_radius, turning_radius);
      }
    }
  } else {
    const std::vector<planning_math::Vec2d> &polygon_points =
        target_obstacle->obstacle()->perception_polygon().points();
    for (const planning_math::Vec2d polygon_point : polygon_points) {
      double dx = polygon_point.x() - ego_x - safe_buffer_;
      double dy = polygon_point.y() - ego_y + turning_direction_ * safe_buffer_;
      // double rel_theta = std::atan2(dy, dx);
      // dx -= safe_buffer_ * std::cos(rel_theta);
      // dy -= safe_buffer_ * std::sin(rel_theta);
      double numerator =
        dx * dx + dy * dy - front_edge_to_rear_axle_ * front_edge_to_rear_axle_ - 0.25 * ego_width_ * ego_width_;
      double denominator =
        ego_width_ - 2 * turning_direction_ * (dx * std::sin(ego_heading) - dy * std::cos(ego_heading));
      if (denominator == 0) {
        continue;
      }
      double turning_radius = numerator / denominator;
      if (turning_radius < 0) {
        continue;
      }
      if (turning_radius < min_turning_radius) {
        target_point_x = ego_x + dx;
        target_point_y = ego_y + dy;
      }
      min_turning_radius =
        std::min(min_turning_radius, turning_radius);
    }
  }
  LOG_DEBUG("(target_point_x, target_point_y):(%f, %f)\n", target_point_x, target_point_y);
  JSON_DEBUG_VALUE("target_turning_point_x", target_point_x)
  JSON_DEBUG_VALUE("target_turning_point_y", target_point_y)
  return min_turning_radius;
}

void SteeringWheelStationaryDecider::GetTurningDirectionByLane(
  const std::shared_ptr<planning::FrenetObstacle> &obstacle) {
  turning_direction_ = 0;
  double obs_s = obstacle->frenet_s();
  double obs_l = obstacle->frenet_l();
  double obs_width = obstacle->width();
  const auto& virtual_lane_manager =
    session_->environmental_model().get_virtual_lane_manager();
  const auto& current_lane = virtual_lane_manager->get_current_lane();
  double current_width = current_lane->width(obs_s);
  const auto& left_lane = virtual_lane_manager->get_left_lane();
  if (left_lane != nullptr) {
    double left_width = left_lane->width(obs_s);
    if ((left_width + current_width * 0.5) - (obs_l + 0.5 * obs_width) >
         ego_width_ + safe_buffer_) {
      turning_direction_ = 1;
      if (CheckLaneLineTypeByTurningDirection()) {
        return;
      }
    }
  }
  turning_direction_ = 0;
  const auto& right_lane = virtual_lane_manager->get_right_lane();
  if (right_lane != nullptr) {
    double right_width = right_lane->width(obs_s);
    if ((right_width + current_width * 0.5) + (obs_l - 0.5 * obs_width) >
         ego_width_ + safe_buffer_) {
      turning_direction_ = -1;
      if (CheckLaneLineTypeByTurningDirection()) {
        return;
      }
    }
  }
  turning_direction_ = 0;
}

void SteeringWheelStationaryDecider::GetTurningDirectionByRef() {
  const auto &virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto &reference_path =
      virtual_lane_manager->get_current_lane()->get_reference_path();
  double ego_l = reference_path->get_frenet_ego_state().l();
  const auto &frenet_coord = reference_path->get_frenet_coord();
  const auto &lateral_ref_path =
      session_->planning_context().general_lateral_decider_output().enu_ref_path;
  const auto &soft_bounds_frenet_point =
      session_->planning_context().general_lateral_decider_output().soft_bounds_frenet_point;
  turning_direction_ = 0;
  double max_rel_l = 0.0;
  for (size_t i = 0; i < lateral_ref_path.size(); ++i) {
    if (soft_bounds_frenet_point[i].first == soft_bounds_frenet_point[i].second) {
      Point2D frenet_point;
      if (frenet_coord->XYToSL(
          Point2D(lateral_ref_path[i].first, lateral_ref_path[i].second),
          frenet_point)) {
        if (std::fabs(frenet_point.y - ego_l) > max_rel_l) {
          max_rel_l = std::fabs(frenet_point.y - ego_l);
          turning_direction_ = frenet_point.y - ego_l;
        }
      }
    }
  }
}

void SteeringWheelStationaryDecider::GetTurningDirectionByPassInterval() {
  // bool find_pass_interval = session_->planning_context()
  //                             .lateral_obstacle_decider_output()
  //                             .find_pass_interval;
  // turning_direction_ = 0;
  // if (find_pass_interval) {
  // const auto &virtual_lane_manager =
  //     session_->environmental_model().get_virtual_lane_manager();
  // const auto &reference_path =
  //     virtual_lane_manager->get_current_lane()->get_reference_path();
  //   double ego_l = reference_path->get_frenet_ego_state().l();
  //   const auto &pass_interval = session_->planning_context()
  //                                 .lateral_obstacle_decider_output()
  //                                 .pass_interval;
  //   double pass_interval_mid = (pass_interval.first + pass_interval.second) * 0.5;
  //   if (pass_interval_mid > ego_l) {
  //     turning_direction_ = 1;
  //   } else if (pass_interval_mid < ego_l) {
  //     turning_direction_ = -1;
  //   }
  // }
}

bool SteeringWheelStationaryDecider::CheckLaneLineTypeByTurningDirection() {
  const auto &virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  if (turning_direction_ > 0 &&
      virtual_lane_manager->get_left_lane() == nullptr) {
    return false;
  } else if (turning_direction_ < 0 &&
             virtual_lane_manager->get_right_lane() == nullptr) {
    return false;
  }
  const auto &reference_path =
      virtual_lane_manager->get_current_lane()->get_reference_path();
  // const auto &reference_path = session_->planning_context()
  //                              .lane_change_decider_output()
  //                              .coarse_planning_info.reference_path;
  double ego_s = reference_path->get_frenet_ego_state().head_s();
  ReferencePathPoint path_point;
  if (reference_path->get_reference_point_by_lon(ego_s, path_point)) {
    if (turning_direction_ > 0) {
      if (path_point.left_lane_border_type ==
          iflyauto::LaneBoundaryType_MARKING_DASHED ||
          path_point.left_lane_border_type ==
          iflyauto::LaneBoundaryType_MARKING_SHORT_DASHED ||
          path_point.left_lane_border_type ==
          iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED ||
          path_point.left_lane_border_type ==
          iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
        return true;
      } else {
        return false;
      }
    } else if (turning_direction_ < 0) {
      if (path_point.right_lane_border_type ==
          iflyauto::LaneBoundaryType_MARKING_DASHED ||
          path_point.right_lane_border_type ==
          iflyauto::LaneBoundaryType_MARKING_SHORT_DASHED ||
          path_point.right_lane_border_type ==
          iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED ||
          path_point.right_lane_border_type ==
          iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
        return true;
      } else {
        return false;
      }
    }
  }
  return false;
}

bool SteeringWheelStationaryDecider::CheckTurningSafety(
    const std::shared_ptr<FrenetObstacle>& target_obstacle) {
  if (!target_obstacle->b_frenet_valid()) {
    return false;
  }
  // const auto &reference_path = session_->planning_context()
  //                              .lane_change_decider_output()
  //                              .coarse_planning_info.reference_path;
  const auto &virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto &reference_path =
      virtual_lane_manager->get_current_lane()->get_reference_path();
  const auto &obs_vec = reference_path->get_obstacles();
  double ego_s = reference_path->get_frenet_ego_state().s();
  double ego_l = reference_path->get_frenet_ego_state().l();
  double start_s = rear_axle_to_center_;
  double back_s = -rear_edge_to_rear_axle_ - 1.0;
  double end_s =
      target_obstacle->frenet_obstacle_boundary().s_end +
      1.0 - ego_s;
  double start_l = 0;
  double end_l = 0;
  if (turning_direction_ > 0) {
    start_l = target_obstacle->frenet_obstacle_boundary().l_end;
    end_l = start_l + ego_width_ + 0.5;
  } else if (turning_direction_ < 0) {
    end_l = target_obstacle->frenet_obstacle_boundary().l_start;
    start_l = end_l - ego_width_ - 0.5;
  } else {
    return false;
  }

  for (const auto &obs : obs_vec) {
    double obs_front_s = obs->frenet_obstacle_boundary().s_start - ego_s;
    double obs_bakc_s = obs->frenet_obstacle_boundary().s_end - ego_s;
    if (obs->id() == target_obstacle->id() ||
        !obs->b_frenet_valid() ||
        obs_front_s > end_s ||
        obs->frenet_obstacle_boundary().l_end < start_l ||
        obs->frenet_obstacle_boundary().l_start > end_l) {
      continue;
    }

    if (obs_bakc_s > start_s &&
        obs->frenet_obstacle_boundary().l_end >= start_l &&
        obs->frenet_obstacle_boundary().l_start <= end_l) {
      return false;
    }
    if (!obs->is_static()) {
      if (obs->frenet_obstacle_boundary().l_end >= start_l &&
          obs->frenet_obstacle_boundary().l_start <= end_l&&
          obs->frenet_velocity_s() > 1e-6) {
        double time =
            (std::fabs(obs_front_s) + start_s) /
             obs->frenet_velocity_s();
        if (time < 6.0) {
          return false;
        }
      }
    } else {
      if (obs_bakc_s > back_s &&
          ((turning_direction_ > 0 &&
           obs->l_relative_to_ego() < 0.5 * ego_width_ + 0.2) ||
          (turning_direction_ < 0 &&
           obs->l_relative_to_ego() > -0.5 * ego_width_ - 0.2))) {
        return false;
      }
    }
  }
  return true;
}

std::shared_ptr<planning::FrenetObstacle> SteeringWheelStationaryDecider::GetFrontNearestObstacle() {
  const auto &lat_obstacle_decision = session_->planning_context()
                                      .lateral_obstacle_decider_output()
                                      .lat_obstacle_decision;
  const auto &virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto &reference_path =
      virtual_lane_manager->get_current_lane()->get_reference_path();
  // const auto &reference_path = session_->planning_context()
  //                              .lane_change_decider_output()
  //                              .coarse_planning_info.reference_path;
  const auto &obs_vec = reference_path->get_obstacles();
  double min_s = reference_path->get_frenet_coord()->Length();
  double min_l = 100;
  double max_l = -100;
  int target_obs_id = -1;
  for (const auto &obs : obs_vec) {
    auto it = lat_obstacle_decision.find(obs->id());
    if (it == lat_obstacle_decision.end()) {
      continue;
    }
    LatObstacleDecisionType obstacle_decision = it->second;
    if (!obs->is_static() ||
        !obs->b_frenet_valid() ||
        obs->rel_s() < front_edge_to_rear_axle_ ||
        obs->rel_s() > 15.0 ||
        obstacle_decision == LatObstacleDecisionType::IGNORE ||
        // (turning_direction_ > 0 &&
        //  obstacle_decision == LatObstacleDecisionType::RIGHT) ||
        // (turning_direction_ < 0 &&
        //  obstacle_decision == LatObstacleDecisionType::LEFT) ||
        std::fabs(obs->l_relative_to_ego()) > ego_width_) {
      continue;
    }
    if (obs->rel_s() < min_s) {
      target_obs_id = obs->id();
      min_s = obs->rel_s();
      min_l = obs->frenet_l();
      max_l = obs->frenet_l();
    } else if (obs->rel_s() < min_s + 0.5 &&
               ((turning_direction_ > 0 &&
                obs->frenet_l() > max_l) ||
               (turning_direction_ < 0 &&
                obs->frenet_l() < min_l))) {
      target_obs_id = obs->id();
      min_s = obs->rel_s();
      min_l = obs->frenet_l();
      max_l = obs->frenet_l();
    }
  }
  const auto &obs_map = reference_path->get_obstacles_map();
  auto it = obs_map.find(target_obs_id);
  return it == obs_map.end() ? nullptr : it->second;
}

std::shared_ptr<planning::FrenetObstacle> SteeringWheelStationaryDecider::GetFrontNearestObstacleByDecision() {
  // const auto &reference_path = session_->planning_context()
  //                              .lane_change_decider_output()
  //                              .coarse_planning_info.reference_path;
  const auto &virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto &reference_path =
      virtual_lane_manager->get_current_lane()->get_reference_path();
  const auto &obs_map = reference_path->get_obstacles_map();
  int target_obs_id = GetFrontNearestObstacleInLaneChangeState();
  auto it = obs_map.find(target_obs_id);
  if (it != obs_map.end()) {
    return it->second;
  }
  target_obs_id = GetFrontNearestObstacleInLaneBorrowState();
  it = obs_map.find(target_obs_id);
  if (it != obs_map.end()) {
    return it->second;
  }
  const auto &ego_frenet_boundary = reference_path->get_ego_frenet_boundary();
  double left_start_l = ego_frenet_boundary.l_start;
  double left_end_l = ego_frenet_boundary.l_end + 1.5;
  double right_start_l = ego_frenet_boundary.l_start - 1.5;
  double right_end_l = ego_frenet_boundary.l_end;
  target_obs_id = -1;
  double min_s = reference_path->get_frenet_coord()->Length();
  turning_direction_ = 0;
  const auto &obs_vec = reference_path->get_obstacles();
  const auto &lat_obstacle_decision = session_->planning_context()
                                      .lateral_obstacle_decider_output()
                                      .lat_obstacle_decision;
  for (const auto &obs : obs_vec) {
    auto iter = lat_obstacle_decision.find(obs->id());
    if (iter == lat_obstacle_decision.end()) {
      continue;
    }
    LatObstacleDecisionType obstacle_decision = iter->second;
    if (!obs->is_static() ||
        !obs->b_frenet_valid() ||
        obs->rel_s() < front_edge_to_rear_axle_ ||
        obs->rel_s() > 15.0 ||
        obstacle_decision == LatObstacleDecisionType::IGNORE ||
        std::fabs(obs->l_relative_to_ego()) > ego_width_) {
      continue;
    }
    if (obs->rel_s() < min_s) {
      if (obstacle_decision == LatObstacleDecisionType::LEFT) {
        turning_direction_ = 1;
      } else if (obstacle_decision == LatObstacleDecisionType::RIGHT) {
        turning_direction_ = -1;
      }
      if (turning_direction_ > 0 &&
          obs->frenet_obstacle_boundary().l_start <= left_end_l&&
          obs->frenet_obstacle_boundary().l_end >= left_start_l) {
        target_obs_id = obs->id();
        min_s = obs->rel_s();
      } else if (turning_direction_ < 0 &&
                 obs->frenet_obstacle_boundary().l_start <= right_end_l&&
                 obs->frenet_obstacle_boundary().l_end >= right_start_l) {
        target_obs_id = obs->id();
        min_s = obs->rel_s();
      }
    }
  }
  it = obs_map.find(target_obs_id);
  return it == obs_map.end() ? nullptr : it->second;
}

int SteeringWheelStationaryDecider::GetFrontNearestObstacleInLaneChangeState() {
  const auto &virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto &reference_path =
      virtual_lane_manager->get_current_lane()->get_reference_path();
  // const auto &coarse_planning_info = session_->planning_context()
  //                                        .lane_change_decider_output()
  //                                        .coarse_planning_info;
  // const auto &reference_path = coarse_planning_info.reference_path;
  const auto &ego_frenet_boundary = reference_path->get_ego_frenet_boundary();
  double min_s = reference_path->get_frenet_coord()->Length();
  int target_obs_id = -1;
  // const auto &target_state = coarse_planning_info.target_state;
  // bool is_LC_CHANGE =
  //     target_state > kLaneKeeping;
  const auto &lc_request =
      session_->planning_context().lane_change_decider_output().lc_request;
  turning_direction_ = 0;
  double start_l = 0;
  double end_l = 0;
  if (lc_request > NO_CHANGE) {
    if (lc_request == LEFT_CHANGE) {
      turning_direction_ = 1;
      start_l = ego_frenet_boundary.l_start;
      end_l = ego_frenet_boundary.l_end + 1.5;
    } else if (lc_request == RIGHT_CHANGE) {
      turning_direction_ = -1;
      end_l = ego_frenet_boundary.l_end;
      start_l = ego_frenet_boundary.l_start - 1.5;
    }
    const auto &obs_vec = reference_path->get_obstacles();
    for (const auto &obs : obs_vec) {
      if (!obs->is_static() ||
          !obs->b_frenet_valid() ||
          obs->rel_s() < front_edge_to_rear_axle_ ||
          obs->rel_s() > 15.0 ||
          obs->frenet_obstacle_boundary().l_start > end_l ||
          obs->frenet_obstacle_boundary().l_end < start_l) {
        continue;
      }
      if (obs->rel_s() < min_s) {
        target_obs_id = obs->id();
        min_s = obs->rel_s();
      }
    }
  }
  return target_obs_id;
}

int SteeringWheelStationaryDecider::GetFrontNearestObstacleInLaneBorrowState() {
  // const auto &coarse_planning_info = session_->planning_context()
  //                                        .lane_change_decider_output()
  //                                        .coarse_planning_info;
  // const auto &reference_path = coarse_planning_info.reference_path;
  const auto &virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto &reference_path =
      virtual_lane_manager->get_current_lane()->get_reference_path();
  const auto &obs_map = reference_path->get_obstacles_map();
  double min_s = reference_path->get_frenet_coord()->Length();
  int target_obs_id = -1;
  turning_direction_ = 0;
  const auto &lane_borrow_decider_output =
      session_->planning_context().lane_borrow_decider_output();
  bool is_in_lane_borrow_status =
      lane_borrow_decider_output.is_in_lane_borrow_status;
  if (is_in_lane_borrow_status) {
    if (lane_borrow_decider_output.borrow_direction == LEFT_BORROW) {
      turning_direction_ = 1;
    } else if (lane_borrow_decider_output.borrow_direction == RIGHT_BORROW) {
      turning_direction_ = -1;
    }
    const std::vector<int> &blocked_obstacles =
        lane_borrow_decider_output.blocked_obs_id;
    for (int blocked_obs_id : blocked_obstacles) {
      auto blocked_obs = obs_map.find(blocked_obs_id);
      if (blocked_obs == obs_map.end() ) {
        continue;
      }
      if (blocked_obs->second->rel_s() >= front_edge_to_rear_axle_) {
        if (blocked_obs->second->rel_s() < min_s) {
          target_obs_id = blocked_obs->second->id();
          min_s = blocked_obs->second->rel_s();
        }
      }
    }
  }
  return target_obs_id;
}

bool SteeringWheelStationaryDecider::UpdateOutput() {
  auto &steering_wheel_stationary_decider_output =
      session_->mutable_planning_context()
          ->mutable_steering_wheel_stationary_decider_output();
  steering_wheel_stationary_decider_output.is_need_steering_wheel_stationary =
      is_need_steering_wheel_stationary_;
  steering_wheel_stationary_decider_output.target_steering_angle =
      target_steering_angle_;
  return true;
}

void SteeringWheelStationaryDecider::Reset() {
  is_need_steering_wheel_stationary_ = false;
  turning_direction_ = 0;
  target_steering_angle_ = 0.0;
  target_obstacle_id_ = 0;
}
}  // namespace planning

#include "lateral_request_decider.h"

#include "planning_context.h"
#include "planning_plan_c.h"
#include "environmental_model.h"

namespace planning {

constexpr double kMaxForwardDistance = 90.0;

LateralRequestDecider::LateralRequestDecider(framework::Session *session) {
  session_ = session;
  is_out_lane_abnormal_ = false;
}

bool LateralRequestDecider::Execute() {
  InitInfo();

  GenerateTakeoverRequest();

  return true;
}

void LateralRequestDecider::InitInfo() {
  auto& planning_output = session_->mutable_planning_context()->mutable_planning_output();
  planning_output.planning_request.take_over_req_level = iflyauto::REQUEST_LEVEL_NO_REQ;
  planning_output.planning_request.request_reason = iflyauto::REQUEST_REASON_NO_REASON;
}

bool LateralRequestDecider::GenerateTakeoverRequest() {
  if (!session_->environmental_model().GetVehicleDbwStatus()) {
    is_out_lane_abnormal_ = false;
    return true;
  }
  bool lat_acc_check_result = CheckLateralACC();
  // bool lk_pose_check_result = CheckPositionInLane();
  bool lat_collision_check_result = CheckLateralCollision();
  if (!lat_acc_check_result || !lat_collision_check_result) {
    auto& planning_output = session_->mutable_planning_context()->mutable_planning_output();
    planning_output.planning_request.take_over_req_level = iflyauto::REQUEST_LEVEL_WARRING;
    planning_output.planning_request.request_reason = iflyauto::REQUEST_REASON_LAT_COLLISION_RISK;
  }
  return true;
}

bool LateralRequestDecider::CheckLateralACC() {
  const auto& motion_planner_output =
      session_->planning_context().motion_planner_output();
  double curv_factor = motion_planner_output.curv_factor;
  const auto& lat_plan_vel =
      session_->planning_context().general_lateral_decider_output().v_cruise;
  if (lat_plan_vel < 1.0) {
    return true;
  }
  const auto& raw_traj_points =
      session_->planning_context().planning_result().raw_traj_points;
  double start_s = raw_traj_points.front().s;
  int lat_controllable_count = 0;
  const double kDangerousThr = 3.0;
  std::vector<double> lat_acc;
  lat_acc.reserve(raw_traj_points.size());
  const double tp_init_s = raw_traj_points.front().s;
  for (size_t i = 0; i < raw_traj_points.size(); i++) {
    if (raw_traj_points[i].t > 3.0 || raw_traj_points[i].s > (start_s + kMaxForwardDistance)) {
      break;
    }
    double tp_delta =
        motion_planner_output.delta_s_spline(raw_traj_points[i].s - tp_init_s);
    double tp_lat_acc =
        curv_factor * lat_plan_vel * lat_plan_vel * tp_delta;
        // curv_factor * raw_traj_points[i].v * raw_traj_points[i].v * tp_delta;
    lat_acc[i] = tp_lat_acc;
    if (tp_lat_acc > kDangerousThr) {
      lat_controllable_count++;
    }
  }
  if (lat_controllable_count > 1) {
    return false;
  }
  return true;
}

bool LateralRequestDecider::CheckPositionInLane() {
  const auto& lane_borrow_decider_output =
      session_->planning_context().lane_borrow_decider_output();
  const auto& spatio_temporal_union_plan_output =
      session_->planning_context().spatio_temporal_union_plan_output();
  const auto& coarse_planning_info = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info;
  bool is_LC_CHANGE = coarse_planning_info.target_state > kLaneKeeping;
  const auto& reference_path = coarse_planning_info.reference_path;
  if (reference_path == nullptr || is_LC_CHANGE ||
      lane_borrow_decider_output.is_in_lane_borrow_status ||
      spatio_temporal_union_plan_output.enable_using_st_plan) {
    is_out_lane_abnormal_ = true;
    return true;
  }
  const auto& frenet_coord = reference_path->get_frenet_coord();
  if (frenet_coord == nullptr) {
    return true;
  }
  // LK
  const auto& motion_planner_output = session_->planning_context()
                                        .motion_planner_output();
  const auto& concerned_index = motion_planner_output.concerned_index;
  const auto& raw_path_points = reference_path->get_points();
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto& raw_traj_points =
      session_->planning_context().planning_result().raw_traj_points;
  double start_s = raw_traj_points.front().s;
  double init_lane_width = raw_path_points.front().lane_width;
  int out_lane_count = 0;
  for (size_t i = 0; i < raw_traj_points.size(); i++) {
    if (i > concerned_index || raw_traj_points[i].t > 3.0 ||
        raw_traj_points[i].s > (start_s + kMaxForwardDistance)) {
      break;
    }
    double left_width = 0.5 * init_lane_width;
    double right_width = 0.5 * init_lane_width;
    ReferencePathPoint cur_path_point;
    if (reference_path->get_reference_point_by_lon(raw_traj_points[i].s, cur_path_point)) {
      left_width = cur_path_point.distance_to_left_lane_border;
      right_width = cur_path_point.distance_to_right_lane_border;
    }
    double center_x = raw_traj_points[i].x +
        std::cos(raw_traj_points[i].heading_angle) * vehicle_param.rear_axle_to_center;
    double center_y = raw_traj_points[i].y +
        std::sin(raw_traj_points[i].heading_angle) * vehicle_param.rear_axle_to_center;
    planning_math::Box2d ego_box({center_x, center_y}, raw_traj_points[i].heading_angle,
                                 vehicle_param.length, vehicle_param.max_width);
    std::vector<planning_math::Vec2d> frenet_corners;
    for (auto &pt : ego_box.GetAllCorners()) {
      Point2D frenet_corner, cart_corner;
      cart_corner.x = pt.x();
      cart_corner.y = pt.y();
      if (frenet_coord->XYToSL(cart_corner, frenet_corner)) {
        if (frenet_corner.y > left_width || frenet_corner.y < -right_width) {
          out_lane_count++;
          break;
        }
      }
    }
  }
  if (out_lane_count > 1) {
    if (is_out_lane_abnormal_) {
      return true;
    }
    return false;
  }
  is_out_lane_abnormal_ = false;
  return true;
}

bool LateralRequestDecider::CheckLateralCollision() {
  const auto& reference_path = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  const auto& motion_planner_output = session_->planning_context()
                                        .motion_planner_output();
  const auto& planned_kd_path = motion_planner_output.lateral_path_coord;
  if (reference_path == nullptr) {
    return true;
  }
  const auto& frenet_coord = reference_path->get_frenet_coord();
  if (frenet_coord == nullptr) {
    return true;
  }
  bool is_outline_valid = GenerateOutline();
  if (!is_outline_valid) {
    return true;
  }
  const auto& raw_traj_points =
      session_->planning_context().planning_result().raw_traj_points;
  double start_s = raw_traj_points.front().s;
  double end_s = std::min(raw_traj_points.back().s, start_s + kMaxForwardDistance);
  double concerned_end_s = start_s + kMaxForwardDistance;
  const auto& s_lat_vec = motion_planner_output.s_lat_vec;
  const size_t concerned_index = motion_planner_output.concerned_index;
  if (s_lat_vec.size() > concerned_index + 1) {
    concerned_end_s = start_s + s_lat_vec[concerned_index];
  }
  const double kLatCollisionThr = 0.15;
  int collision_count = 0;
  // obstacle
  const auto& obs_vec = reference_path->get_obstacles();
  for (const auto& obs : obs_vec) {
    if (obs->is_static()) {
      const auto& corner_points = obs->corner_points();
      for (const auto& corner_point : corner_points) {
        if (corner_point.x() >= start_s && corner_point.x() <= end_s) {
          double left_bound = lbound_s_spline_(corner_point.x());
          double right_bound = rbound_s_spline_(corner_point.x());
          if (corner_point.y() > left_bound &&
              corner_point.y() <= (left_bound + kLatCollisionThr)) {
            collision_count++;
          } else if (corner_point.y() < right_bound &&
                     corner_point.y() >= (right_bound - kLatCollisionThr)) {
            collision_count++;
          }
        }
      }
    }
  }
  if (collision_count > 1) {
    return false;
  }
  // road border
  collision_count = 0;
  end_s = std::min(concerned_end_s, end_s);
  const auto& general_lateral_decider_output =
      session_->planning_context().general_lateral_decider_output();
  double care_lon_area_road_border =
      std::min(start_s + general_lateral_decider_output.care_lon_area_road_border, end_s);
  const auto& road_boundaries = session_->environmental_model()
                                      .get_virtual_lane_manager()
                                      ->GetRoadboundary();
  for (const auto& road_boundary : road_boundaries) {
    for (const auto& road_segment : road_boundary) {
      Point2D cart_point(road_segment.second.x, road_segment.second.y);  // global
      Point2D frenet_point;
      if (frenet_coord->XYToSL(cart_point, frenet_point)) {
        if (frenet_point.x >= start_s && frenet_point.x <= care_lon_area_road_border) {
          double left_bound = lbound_s_spline_(frenet_point.x);
          double right_bound = rbound_s_spline_(frenet_point.x);
          if (frenet_point.y > left_bound &&
              frenet_point.y <= (left_bound + kLatCollisionThr)) {
            collision_count++;
          } else if (frenet_point.y < right_bound &&
                     frenet_point.y >= (right_bound - kLatCollisionThr)) {
            collision_count++;
          }
        }
      }
    }
  }
  if (collision_count > 1) {
    return false;
  }
  return true;
}

bool LateralRequestDecider::GenerateOutline() {
  const auto& reference_path = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  // if (reference_path == nullptr) {
  //   return false;
  // }
  const auto& frenet_coord = reference_path->get_frenet_coord();
  // if (frenet_coord == nullptr) {
  //   return false;
  // }
  const auto& raw_traj_points =
      session_->planning_context().planning_result().raw_traj_points;
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double half_ego_width = vehicle_param.max_width * 0.5;
  std::vector<double> left_outline_s;
  std::vector<double> left_outline_l;
  std::vector<double> right_outline_s;
  std::vector<double> right_outline_l;
  left_outline_s.reserve(raw_traj_points.size() + 1);
  left_outline_l.reserve(raw_traj_points.size() + 1);
  right_outline_s.reserve(raw_traj_points.size() + 1);
  right_outline_l.reserve(raw_traj_points.size() + 1);
  for (size_t i = 0; i < raw_traj_points.size(); ++i) {
    double traj_x = raw_traj_points[i].x;
    double traj_y = raw_traj_points[i].y;
    double traj_heading = raw_traj_points[i].heading_angle;
    double traj_s = raw_traj_points[i].s;
    double traj_l = raw_traj_points[i].l;
    double center_x =
        traj_x + std::cos(traj_heading) * vehicle_param.rear_axle_to_center;
    double center_y =
        traj_y + std::sin(traj_heading) * vehicle_param.rear_axle_to_center;
    planning_math::Box2d ego_box({center_x, center_y}, traj_heading,
                                 vehicle_param.length, vehicle_param.max_width);
    std::vector<planning_math::Vec2d> frenet_head_corners;
    std::vector<planning_math::Vec2d> frenet_back_corners;
    frenet_head_corners.reserve(2);
    frenet_back_corners.reserve(2);
    for (auto &pt : ego_box.GetAllCorners()) {
      Point2D frenet_corner, cart_corner;
      cart_corner.x = pt.x();
      cart_corner.y = pt.y();
      if (frenet_coord->XYToSL(cart_corner, frenet_corner)) {
        if (frenet_corner.x > traj_s) {
          frenet_head_corners.emplace_back(
              planning_math::Vec2d(frenet_corner.x, frenet_corner.y));
        } else {
          frenet_back_corners.emplace_back(
              planning_math::Vec2d(frenet_corner.x, frenet_corner.y));
        }
      }
    }
    if (i == 0) {
      std::pair<double, double> ego_back_lbound{traj_s, traj_l + half_ego_width};
      std::pair<double, double> ego_back_rbound{traj_s, traj_l - half_ego_width};
      if (frenet_back_corners.size() == 2) {
        ego_back_lbound.first = frenet_back_corners.front().x();
        ego_back_lbound.second = frenet_back_corners.front().y();
        ego_back_rbound.first = frenet_back_corners.back().x();
        ego_back_rbound.second = frenet_back_corners.back().y();
        if (ego_back_lbound.second < ego_back_rbound.second) {
          ego_back_lbound.swap(ego_back_rbound);
        }
      }
      left_outline_s.emplace_back(ego_back_lbound.first);
      left_outline_l.emplace_back(ego_back_lbound.second);
      right_outline_s.emplace_back(ego_back_rbound.first);
      right_outline_l.emplace_back(ego_back_rbound.second);
    }
    if (frenet_head_corners.size() != 2) {
      continue;
    }
    std::pair<double, double> ego_head_lbound{frenet_head_corners.front().x(), frenet_head_corners.front().y()};
    std::pair<double, double> ego_head_rbound{frenet_head_corners.back().x(), frenet_head_corners.back().y()};
    if (ego_head_lbound.second < ego_head_rbound.second) {
      ego_head_lbound.swap(ego_head_rbound);
    }
    if (!left_outline_s.empty()) {
      if (ego_head_lbound.first > left_outline_s.back()){
        left_outline_s.emplace_back(ego_head_lbound.first);
        left_outline_l.emplace_back(ego_head_lbound.second);
      }
    }
    if (!right_outline_s.empty()) {
      if (ego_head_rbound.first > right_outline_s.back()) {
        right_outline_s.emplace_back(ego_head_rbound.first);
        right_outline_l.emplace_back(ego_head_rbound.second);
      }
    }
  }
  // result
  if (left_outline_s.size() < 3 || right_outline_s.size() < 3) {
    return false;
  }
  lbound_s_spline_.set_points(left_outline_s, left_outline_l,
                              pnc::mathlib::spline::linear);
  rbound_s_spline_.set_points(right_outline_s, right_outline_l,
                              pnc::mathlib::spline::linear);
  return true;
}

}  // namespace planning
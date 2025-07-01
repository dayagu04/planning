#include "trajectory_cost.h"
#include <math.h>
#include <cmath>
#include "behavior_planners/dp_path_decider/dp_base.h"
#include "config/message_type.h"
#include "config/vehicle_param.h"
#include "define/geometry.h"
#include "ego_state_manager.h"
#include "ifly_time.h"
#include "math/box2d.h"
#include "math/curve1d/quintic_polynomial_curve1d.h"
#include "task_interface/lane_borrow_decider_output.h"
namespace planning {

ComparableCost TrajectoryCost::Calculate(const QuinticPolynomialCurve1d& curve,
                                         const double start_s,
                                         const double end_s, int current_level,
                                         int total_level,
                                         LaneBorrowStatus lane_borrow_status) {
  ComparableCost total_cost;
  // total_cost = CalculateStaticObsCost(curve, start_s, end_s);
  double time_stamp_1 = IflyTime::Now_ms();
  total_cost = CalObsCartCost(curve, start_s, end_s);
  double time_stamp_2 = IflyTime::Now_ms();
  total_cost += CalculatePathCost(curve, start_s, end_s, current_level,
                                  total_level, lane_borrow_status);
  double time_stamp_3 = IflyTime::Now_ms();
  total_cost += CalculateStitchCost(curve, start_s, end_s);
  double time_stamp_4 = IflyTime::Now_ms();

  safety_cost_time_ += (time_stamp_2 - time_stamp_1);
  path_cost_time_ += (time_stamp_3 - time_stamp_2);
  stitch_cost_time_ += (time_stamp_4 - time_stamp_3);

  return total_cost;
}

ComparableCost TrajectoryCost::CalculatePathCost(
    const QuinticPolynomialCurve1d& curve, const double start_s,
    const double end_s, int current_level, int total_level,
    LaneBorrowStatus lane_borrow_status) const {
  ComparableCost cost;
  double path_cost = 0.0;
  // lateral sample two lanes
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();

  for (double path_s = 0.0; path_s < (end_s - start_s);
       path_s += path_resolution_) {
    const double frenet_l = curve.Evaluate(0, path_s);
    // if (frenet_l < sample_right_boundary_ || frenet_l >
    // sample_left_boundary_) {
    //   cost.out_boundary_ = true;
    // }
    // lateral sample  lanes
    double sample_left_boundary = 0.0;
    double sample_right_boundary = 0.0;
    // avaliable lane boundary
    double ego_s = current_reference_path_ptr_->get_frenet_ego_state().s();
    double aheads = path_s + start_s - ego_s;
    double vehicle_width = vehicle_param.width;
    if (current_lane_ptr_ != nullptr) {
      sample_left_boundary =
          current_lane_ptr_->width_by_s(path_s + start_s) * 0.5 -
          vehicle_width * 0.5;
      sample_right_boundary =
          -current_lane_ptr_->width_by_s(path_s + start_s) * 0.5 +
          vehicle_width * 0.5;
    }
    if (left_lane_ptr_ != nullptr) {
      if (lane_borrow_status == kLaneBorrowCrossing || current_level == 1) {
        sample_left_boundary +=
            left_lane_ptr_->width_by_s(path_s + start_s) * 1.1;
      } else {
        sample_left_boundary +=
            left_lane_ptr_->width_by_s(path_s + start_s)
            * 0.6;  // larger than different from sample boundary
      }
    }
    if (right_lane_ptr_ != nullptr) {
      if (lane_borrow_status == kLaneBorrowCrossing || current_level == 1) {
        sample_right_boundary -=
            right_lane_ptr_->width_by_s(path_s + start_s) * 1.1;
      } else {
        sample_right_boundary -=
            right_lane_ptr_->width_by_s(path_s + start_s) * 0.6;
      }
    }
    // if (left_lane_ptr_ != nullptr) {
    //   sample_left_boundary +=
    //       left_lane_ptr_->width_by_s(path_s + start_s) *
    //       0.6;  // larger than different from sample boundary
    // }
    // if (right_lane_ptr_ != nullptr) {
    //   sample_right_boundary -=
    //       right_lane_ptr_->width_by_s(path_s + start_s) * 0.6;
    // }
    // 考虑道路边缘和物理隔离
    ReferencePathPoint refpath_pt{};
    double distance_to_left_road_border = 100;
    double distance_to_right_road_border = 100;
    if (current_reference_path_ptr_ != nullptr &&
        current_reference_path_ptr_->get_reference_point_by_lon(
            ego_frenet_state_.s(), refpath_pt)) {
      distance_to_left_road_border = refpath_pt.distance_to_left_road_border;
      distance_to_right_road_border = refpath_pt.distance_to_right_road_border;
    }
    sample_right_boundary =
        std::max(sample_right_boundary, -distance_to_right_road_border);
    sample_left_boundary =
        std::min(sample_left_boundary, distance_to_left_road_border);

    if (frenet_l < sample_right_boundary || frenet_l > sample_left_boundary) {
      cost.out_boundary_ = true;
    }

    std::function<double(const double)> quasi_softmax = [this](const double x) {
      const double l0 = 1.5;
      const double b = 0.4;
      const double k = 1.5;
      return (b + std::exp(-k * (x - l0))) / (1.0 + std::exp(-k * (x - l0)));
    };
    path_cost += frenet_l * frenet_l * coeff_l_cost_;
    const double frenet_dl = curve.Evaluate(1, path_s);
    path_cost += frenet_dl * frenet_dl * coeff_dl_cost_;
    const double frenet_ddl = curve.Evaluate(2, path_s);
    path_cost += frenet_ddl * frenet_ddl * coeff_ddl_cost_;
  }
  if (current_level == total_level) {
    const double end_l = curve.Evaluate(0, end_s - start_s);
    path_cost += end_l * end_l * coeff_end_l_cost_;
  }

  path_cost = path_cost * path_resolution_;
  cost.smooth_cost_ = path_cost;
  return cost;
}
// cost of each obs
ComparableCost TrajectoryCost::GetObsSLCost(
    const FrenetObstacleBoundary& frenet_obstacle_sl, const double curr_s,
    const double curr_l) const {
  ComparableCost obstacle_cost;
  // const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();

  const double ego_s_start = curr_s - vehicle_param.rear_edge_to_rear_axle;
  const double ego_s_end = curr_s + vehicle_param.front_edge_to_rear_axle;
  const double ego_l_start = curr_l - vehicle_param.max_width * 0.5;
  const double ego_l_end = curr_l + vehicle_param.max_width * 0.5;
  // const auto& obstacles = current_reference_path_ptr_->get_obstacles();

  // if (ego_l_end<frenet_obstacle_sl.l_start
  //   ||ego_l_start>frenet_obstacle_sl.l_end){
  //   return obstacle_cost;
  // }

  bool no_overlap = ego_s_end < frenet_obstacle_sl.s_start ||  // ego behind
                    ego_s_start > frenet_obstacle_sl.s_end ||  // ahead
                    ego_l_end < frenet_obstacle_sl.l_start ||  // left
                    ego_l_start > frenet_obstacle_sl.l_end;    // right
  if (!no_overlap) {
    obstacle_cost.has_collision_ = true;
  }
  // no overlap  but need consider
  std::function<double(const double, const double)> softmax =
      [](const double x, const double x0) {
        return std::exp(-(x - x0)) / (1.0 + std::exp(-(x - x0)));
      };
  const double obs_l =
      0.5 * (frenet_obstacle_sl.l_end + frenet_obstacle_sl.l_start);
  const double obs_s =
      0.5 * (frenet_obstacle_sl.s_start + frenet_obstacle_sl.s_end);

  const double ego_s = 0.5 * (ego_s_end + ego_s_start);
  const double ego_l = 0.5 * (ego_l_end + ego_l_start);

  const double delta_l = std::fabs(ego_l - obs_l);
  obstacle_cost.safety_cost_ +=
      coeff_collision_cost_ * softmax(delta_l, collision_distance_);

  const double delta_s = std::fabs(ego_s - obs_s);
  obstacle_cost.safety_cost_ +=
      coeff_collision_cost_ * softmax(delta_s, collision_distance_);
  return obstacle_cost;
}
ComparableCost TrajectoryCost::GetBoxCost(const Box2d& ego_box,
                                          const Box2d& obs_box) const {
  ComparableCost obstacle_cost;
  double distance = ego_box.DistanceTo(obs_box);
  if (distance < 0.01) {
    obstacle_cost.has_collision_ = true;
  }
  // no overlap  but need consider
  std::function<double(const double, const double)> softmax =
      [](const double x, const double x0) {
        return std::exp(-(x - x0)) / (1.0 + std::exp(-(x - x0)));
      };

  obstacle_cost.safety_cost_ +=
      coeff_collision_cost_ * softmax(distance, collision_distance_);
  return obstacle_cost;
}
bool TrajectoryCost::BoxHasOverlap(const Box2d& ego_box,
                                   const Box2d& obs_box) const {  // unused
  std::vector<planning_math::Vec2d> corners = obs_box.GetAllCorners();
  bool has_overlap = false;
  for (size_t i = 0; i < corners.size(); ++i) {
    if (ego_box.IsPointIn(corners[i])) {
      has_overlap = true;
      break;
    }
  }
  return has_overlap;
}
// ComparableCost TrajectoryCost::CalculateStaticObsCost(
//     const QuinticPolynomialCurve1d& curve, const double start_s,
//     const double end_s) const {
//   ComparableCost obstacles_cost;
//   const double path_resolution = path_resolution_;
//   for (double curr_s = start_s; curr_s <= end_s; curr_s += 2.0) {
//     const double curr_l = curve.Evaluate(0, curr_s - start_s);
//     for (const auto& obstacle:obstacles_info_){
//       FrenetObstacleBoundary obstacle_sl{obstacle.s_start, obstacle.s_end,
//       obstacle.l_start, obstacle.l_end}; obstacles_cost +=
//       GetObsSLCost(obstacle_sl, curr_s, curr_l);
//     }
//   }
//   obstacles_cost.safety_cost_ *= path_resolution;
//   return obstacles_cost;
// }
ComparableCost TrajectoryCost::CalculateStitchCost(
    const QuinticPolynomialCurve1d& curve, const double start_s,
    const double end_s) const {
  ComparableCost cost;
  double stitch_cost = 0;
  const auto& frenet_coord = current_reference_path_ptr_->get_frenet_coord();
  for (double path_s = 0.0; path_s < (end_s - start_s);
       path_s += path_resolution_) {
    if (ref_path_curve_.s_vec.size() == 0 ||
        path_s + start_s > ref_path_curve_.s_vec.back()) {
      break;  // cur curve too long or first path
    }
    const double frenet_l =
        curve.Evaluate(0, path_s);  // relative s in current curve
    const double frenet_s = path_s + start_s;
    const Point2D sl_point(frenet_s, frenet_l);
    Point2D cart_point;
    frenet_coord->SLToXY(sl_point, cart_point);
    Eigen::Vector2d point_to_proj(cart_point.x, cart_point.y);
    // calculate projection
    pnc::spline::Projection projection_point;
    projection_point.CalProjectionPoint(
        ref_path_curve_.x_s_spline, ref_path_curve_.y_s_spline,
        ref_path_curve_.s_vec.front(), ref_path_curve_.s_vec.back(),
        point_to_proj);
    // get projection GetOutput
    double dist = projection_point.GetOutput().dist_proj;
    stitch_cost += dist * coeff_stitch_cost_;
  }
  stitch_cost = stitch_cost * path_resolution_;
  cost.stitch_cost_ = stitch_cost;
  return cost;
}

void TrajectoryCost::BuildCurveBorder(const QuinticPolynomialCurve1d& curve,
                                      const double start_s,
                                      const double end_s) {
  const auto frenet_coord = current_reference_path_ptr_->get_frenet_coord();
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double front_edge_to_center = vehicle_param.front_edge_to_rear_axle;
  const double back_edge_to_center = vehicle_param.rear_edge_to_rear_axle;
  const double vehicle_length = vehicle_param.length;
  const double vehicle_width = vehicle_param.width;
  const double back_axle_to_center_dist =
      0.5 * (front_edge_to_center - back_edge_to_center);

  const int32_t start_idx = std::max(0, int32_t(start_s / path_resolution_));
  const int32_t end_idx = int32_t(end_s / path_resolution_);
  const double total_len = end_s - start_s;
  int32_t search_num = int32_t(total_len / path_resolution_) + 1;
  std::vector<speed::PathBorderSegment> path_border_segments;
  path_border_segments.reserve(search_num);

  for (int32_t i = start_idx; i < end_idx; ++i) {
    double desired_s = i * path_resolution_;
    double l = curve.Evaluate(0, desired_s);
    double tan_local_theta = curve.Evaluate(1, desired_s);

    Point2D frenet_point(desired_s, l);
    Point2D cart_point;
    frenet_coord->SLToXY(frenet_point, cart_point);
    Vec2d point(cart_point.x, cart_point.y);
    double theta = std::atan(tan_local_theta) +
                   frenet_coord->GetPathCurveHeading(desired_s);

    Vec2d center =
        point + Vec2d::CreateUnitVec2d(theta) * back_axle_to_center_dist;
    Box2d ego_box(center, theta, vehicle_length, vehicle_width);
    auto corners = ego_box.GetAllCorners();

    LineSegment2d left_segment(corners[2], corners[1]);
    LineSegment2d right_segment(corners[3], corners[0]);

    speed::PathBorderSegment
        path_border_segment(  // 边界包含 前后s 左右 segment
            i, desired_s, desired_s - back_edge_to_center,
            desired_s + front_edge_to_center, left_segment, right_segment);
    path_border_segments.emplace_back(path_border_segment);
  }
  curve_border_querier_ =
      std::make_shared<speed::PathBorderQuerier>(path_border_segments);
}
ComparableCost TrajectoryCost::CalObsCartCost(
    const QuinticPolynomialCurve1d& curve, const double start_s,
    const double end_s) const {
  ComparableCost cost;
  const auto frenet_coord = current_reference_path_ptr_->get_frenet_coord();
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double front_edge_to_center = vehicle_param.front_edge_to_rear_axle;
  const double back_edge_to_center = vehicle_param.rear_edge_to_rear_axle;
  const double vehicle_length = vehicle_param.length;
  const double vehicle_width = vehicle_param.width;
  const double back_axle_to_center_dist =
      0.5 * (front_edge_to_center - back_edge_to_center);
  const double path_resolution = path_resolution_;
  double lateral_buffer = 0.4;
  double longitudinal_buffer = 0.0;
  for (double curr_s = start_s; curr_s <= end_s; curr_s += 2.0) {
    const double curr_l = curve.Evaluate(0, curr_s - start_s);
    double tan_local_theta = curve.Evaluate(1, curr_s - start_s);
    Point2D frenet_point(curr_s, curr_l);
    Point2D cart_point;
    frenet_coord->SLToXY(frenet_point, cart_point);
    Vec2d point(cart_point.x, cart_point.y);
    double reference_heading = frenet_coord->GetPathCurveHeading(curr_s);
    double theta =
        std::atan(tan_local_theta) + frenet_coord->GetPathCurveHeading(curr_s);
    Vec2d center =
        point + Vec2d::CreateUnitVec2d(theta) * back_axle_to_center_dist;
    Box2d ego_box(center, theta, vehicle_length + longitudinal_buffer,
                  vehicle_width + lateral_buffer);
    for (const auto& obs_box : static_obstacles_box_) {
      // bool has_overlap = BoxHasOverlap(ego_box,obs_box);
      double dist_ignore = obs_box.half_length() + 13.0;
      if(obs_box.DistanceTo(center) > dist_ignore){
        continue;
      }
      cost += GetBoxCost(ego_box, obs_box);
    }
    // used all prediction time
    for (const auto& obs_box : flatted_dynamic_obstacles_box_) {
      double dist_ignore = obs_box.half_length() + 13.0;
      if(obs_box.DistanceTo(center) > dist_ignore){
        continue;
      }
      cost += GetBoxCost(ego_box, obs_box);
    }
  }
  // dynamic
  // longitudinal_buffer = 1.5;
  // unused time- time check
  // double ego_s = ego_frenet_state_.s();
  // double ego_v = ego_frenet_state_.velocity();
  // for(const auto& obs_boxes:dynamic_obstacles_box_){
  //     for(size_t i = 0; i < obs_boxes.size(); i++){
  //       // 0,0.6,1.2 ...
  //       const auto& agent_pred_box = obs_boxes[i];
  //       double ego_pred_s = 0.6 * i * ego_v + ego_s;
  //       if(ego_pred_s < start_s || ego_pred_s > end_s){
  //         break;
  //       }
  //       const double pred_l = curve.Evaluate(0, ego_pred_s - start_s);
  //       double tan_local_theta = curve.Evaluate(1, ego_pred_s- start_s);
  //       Point2D frenet_point(ego_pred_s ,pred_l);
  //       Point2D cart_point;
  //       frenet_coord->SLToXY(frenet_point, cart_point);
  //       Vec2d point(cart_point.x,cart_point.y);
  //       double theta = std::atan(tan_local_theta) +
  //       frenet_coord->GetPathCurveHeading(ego_pred_s); Vec2d center = point +
  //       Vec2d::CreateUnitVec2d(theta) * back_axle_to_center_dist; Box2d
  //       ego_pred_box(center, theta, vehicle_length + longitudinal_buffer ,
  //       vehicle_width + lateral_buffer); cost +=
  //       GetBoxCost(ego_pred_box,agent_pred_box);
  //       // const auto& ego_pred_box
  //     }
  //     // cost
  //   }

  cost.safety_cost_ *= path_resolution_;
  return cost;
}
ComparableCost TrajectoryCost::CalDynamicObsCartCost(
    const QuinticPolynomialCurve1d& curve, const double start_s,
    const double end_s) const {
  ComparableCost cost;
  // TODO
  return cost;
}
// namespace planning
}  // namespace planning

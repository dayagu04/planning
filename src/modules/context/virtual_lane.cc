#include "src/modules/context/virtual_lane.h"

namespace planning {

void VirtualLane::update_data(const FusionRoad::Lane& lane) {
  order_id_ = lane.order_id();
  virtual_id_ = lane.virtual_id();
  relative_id_ = lane.relative_id();
  ego_lateral_offset_ = lane.ego_lateral_offset();

  lane_type_ = lane.lane_type();
  lane_marks_ = lane.lane_marks();
  lane_source_ = lane.lane_source();
  lane_marks_ = lane.lane_marks();
  lane_reference_line_.CopyFrom(lane.lane_reference_line());
  lane_merge_split_point_.CopyFrom(lane.lane_merge_split_point());
  left_lane_boundary_.CopyFrom(lane.left_lane_boundary());
  right_lane_boundary_.CopyFrom(lane.right_lane_boundary());

  if (left_lane_boundary_.existence()) {
    if (right_lane_boundary_.existence()) {
      lane_status_ = BOTH_AVAILABLE;
    } else {
      lane_status_ = LEFT_AVAILABLE;
    }
  } else {
    if (right_lane_boundary_.existence()) {
      lane_status_ = RIGHT_AVAILABLE;
    } else {
      lane_status_ = BOTH_MISSING;
    }
  }

  center_line_points_track_id_.clear();
  for (auto& virtual_lane_refine_point : lane_reference_line_.virtual_lane_refline_points()) {
    // center_line_points_track_id_.emplace_back(virtual_lane_refine_point.track_id()); // todo
  }
}

bool VirtualLane::has_lines(LineDirection direction) const {
  assert(direction == LEFT || direction == RIGHT);

  if (direction == LEFT) {
    return (lane_status_ == LaneStatusEx::BOTH_AVAILABLE ||
            lane_status_ == LaneStatusEx::LEFT_AVAILABLE);
  } else {
    return (lane_status_ == LaneStatusEx::BOTH_AVAILABLE ||
            lane_status_ == LaneStatusEx::RIGHT_AVAILABLE);
  }

  return false;
}

// double VirtualLane::distance_to_line(double s, double l, LineDirection direction) {
//   assert(direction == LEFT || direction == RIGHT);

//   Point2D frenet_point(s,l);
//   Point2D cart_point;
//   reference_path_->get_frenet_coord()->FrenetCoord2CartCoord(frenet_point, cart_point);

//   double distance_ref, distance_line, distance_line_ref;
//   if (has_lines(direction)) {
//     std::vector<double> coefficient;
//     if (direction == LEFT) {
//       for (auto value : left_lane_boundary_.poly_coefficient()) {
//         coefficient.emplace_back(value);
//       }
//     } else {
//       for (auto value : right_lane_boundary_.poly_coefficient()) {
//         coefficient.emplace_back(value);
//       }
//     }
//     std::reverse(coefficient.begin(),coefficient.end());

//     distance_ref = l;
//     distance_line_ref = -get_dist(cart_point.x, cart_point.y, coefficient);
//     distance_line = distance_ref - distance_line_ref;
//   } else {
//     double distance_tr_ref = 0;
//     if (direction == LEFT) {
//       distance_line_ref = distance_tr_ref + 0.5 * DEFAULT_LANE_WIDTH;
//     } else {
//       distance_line_ref = distance_tr_ref - 0.5 * DEFAULT_LANE_WIDTH;
//     }

//     distance_ref = l;
//     distance_line = distance_ref - distance_line_ref;
//     return distance_line;
//   }
// }

// bool VirtualLane::is_obstacle_on(const Obstacle &tr) {
//   if (reference_path_ == nullptr) {
//     return false;
//   }
//   if (reference_path_->frenet_obstacles_map_.find(tr.id) == reference_path_->frenet_obstacles_map_.end()) {
//     return false;
//   }

//   const FrenetObstacle& frenet_obstacle = reference_path_->get_obstacles_map()[tr.id];
//   if (frenet_obstacle.rel_s() > 60 || frenet_obstacle.rel_s() < -40 || frenet_obstacle.s_min_l().x > 15 || frenet_obstacle.s_min_l().x < -15) {
//     return false;
//   }

//   double d_min_l = distance_to_line(frenet_obstacle.s_min_l().y, frenet_obstacle.s_min_l().x, LEFT);
//   double d_max_r = distance_to_line(frenet_obstacle.s_max_l().y, frenet_obstacle.s_max_l().x, RIGHT);

//   double check_offset = 0.0;

//   double lane_width = DEFAULT_LANE_WIDTH;
//   std::array<double, 3> xp_v{0., 3., 5.};
//   std::array<double, 3> fp_v{std::max(lane_width / 2 - 1.45, 0.0),
//                             std::max(lane_width / 2 - 1.65, 0.0), 0.0};
//   if (true) {
//     std::array<double, 5> xp_s{-30, -20, 20, 30, 60};
//     std::array<double, 5> fp_s{0.45, 0.2, 0.3, 0.45, 0.6};
//     if (check_offset == 0.0) {
//       check_offset =
//         interp(frenet_obstacle.rel_s(), xp_s, fp_s) + interp(frenet_obstacle.frenet_velocity_s(), xp_v, fp_v);
//     }

//     return (d_min_l < -check_offset && d_max_r > check_offset);
//   }
// }

uint VirtualLane::get_common_point_num(const std::shared_ptr<VirtualLane> &other) {
  if (other == nullptr) {
    return 0;
  }

  const std::vector<std::string> &other_point_ids = other->center_line_points_track_id();

  uint common_point_num = 0;
  for (auto &p : other_point_ids) { // todo
    // if (center_line_points_track_id_.find(p) != center_line_points_track_id_.end()) {
    //   common_point_num++;
    // }
  }

  return common_point_num;
}

bool VirtualLane::get_point_by_distance(double distance, FusionRoad::VirtualLanePoint *point) {
  double min_distance_diff = std::numeric_limits<double>::max();
  for (auto virtual_lane_refine_point : lane_reference_line_.virtual_lane_refline_points()) {
    double distance_diff = fabs(distance - virtual_lane_refine_point.car_point().x());
    if (distance_diff < min_distance_diff) {
      min_distance_diff = distance_diff;
      point->CopyFrom(virtual_lane_refine_point);
    }
  }

  if (min_distance_diff < std::numeric_limits<double>::max()) {
    return true;
  } else {
    return false;
  }
}
}
#include "virtual_lane.h"
#include <cassert>
#include "math/linear_interpolation.h"
#include "virtual_lane.h"
namespace planning {
VirtualLane::VirtualLane() {}

void VirtualLane::update_data(const FusionRoad::Lane &lane) {
  order_id_ = lane.order_id();
  // virtual_id_ = lane.virtual_id();
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

  for (auto poly_coefficient :
       lane.lane_reference_line().poly_coefficient_car()) {
    c_poly_.emplace_back(poly_coefficient);
  }
  assert(c_poly_.size() == 4);

  center_line_points_track_id_.clear();
  for (auto &virtual_lane_refine_point :
       lane_reference_line_.virtual_lane_refline_points()) {
    // center_line_points_track_id_.emplace_back(virtual_lane_refine_point.track_id());
    // // todo
  }
}

bool VirtualLane::calc_c_poly(std::vector<double> &output) {  // 该函数先保留着
  output.clear();

  if (lane_status_ == BOTH_MISSING) {
    output.assign(4, 0.0);
    return true;
  } else if (lane_status_ == LEFT_AVAILABLE) {
    if (left_lane_boundary_.poly_coefficient_size() < 4) {
      output.assign(4, 0.0);
      return true;
    }
    for (size_t i = 0; i < left_lane_boundary_.poly_coefficient_size(); i++) {
      output.push_back((left_lane_boundary_.poly_coefficient(i)));
      if (i == 0) {
        output[0] =
            left_lane_boundary_.poly_coefficient(0) -
            0.5 * width() *
                std::sqrt(1 +
                          std::pow(left_lane_boundary_.poly_coefficient(1), 2));
      }
    }
  } else if (lane_status_ == RIGHT_AVAILABLE) {
    if (right_lane_boundary_.poly_coefficient_size() < 4) {
      output.assign(4, 0.0);
      return true;
    }
    for (size_t i = 0; i < right_lane_boundary_.poly_coefficient_size(); i++) {
      output.push_back((right_lane_boundary_.poly_coefficient(i)));
      if (i == 0) {
        output[0] =
            right_lane_boundary_.poly_coefficient(0) +
            0.5 * width() *
                std::sqrt(
                    1 + std::pow(right_lane_boundary_.poly_coefficient(1), 2));
      }
    }
  } else {
    for (size_t i = 0; i < left_lane_boundary_.poly_coefficient_size() &&
                       i < right_lane_boundary_.poly_coefficient_size();
         i++) {
      output.push_back((left_lane_boundary_.poly_coefficient(i) +
                        right_lane_boundary_.poly_coefficient(i)) /
                       2.0);
    }
  }

  return true;
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

// double VirtualLane::distance_to_line(double s, double l, LineDirection
// direction) {
//   assert(direction == LEFT || direction == RIGHT);

//   Point2D frenet_point(s,l);
//   Point2D cart_point;
//   reference_path_->get_frenet_coord()->FrenetCoord2CartCoord(frenet_point,
//   cart_point);

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

uint VirtualLane::get_common_point_num(
    const std::shared_ptr<VirtualLane> &other) {
  if (other == nullptr) {
    return 0;
  }

  const std::vector<std::string> &other_point_ids =
      other->center_line_points_track_id();

  uint common_point_num = 0;
  for (auto &p : other_point_ids) {  // todo
    // if (center_line_points_track_id_.find(p) !=
    // center_line_points_track_id_.end()) {
    //   common_point_num++;
    // }
  }

  return common_point_num;
}

double VirtualLane::width() { return width(0); }

double VirtualLane::width(double x) {
  double width = 0;
  auto virtual_lane_refline_points =
      lane_reference_line_.virtual_lane_refline_points();
  if (virtual_lane_refline_points.size() < 1) {
  } else {
    auto comp = [](const FusionRoad::VirtualLanePoint &p, const double x) {
      return p.car_point().x() < x;
    };
    auto p_first_point =
        std::lower_bound(virtual_lane_refline_points.begin(),
                         virtual_lane_refline_points.end(), x, comp);
    if (p_first_point == virtual_lane_refline_points.begin()) {
      width = virtual_lane_refline_points.begin()->lane_width();
    } else if (p_first_point == virtual_lane_refline_points.end()) {
      width = std::prev(virtual_lane_refline_points.end())->lane_width();
    } else {
      width = planning_math::lerp((p_first_point - 1)->lane_width(),
                                  (p_first_point - 1)->car_point().x(),
                                  p_first_point->lane_width(),
                                  p_first_point->car_point().x(), x);
    }
  }
  return std::max(width, 2.8);
}
double VirtualLane::width_by_s(double s) {
  double width = 0;
  if (reference_path_ != nullptr) {
    auto &reference_path_points = reference_path_->get_points();
    if (reference_path_points.size() < 1) {
    } else {
      auto comp = [](const ReferencePathPoint &p, const double s) {
        return p.path_point.s < s;
      };
      auto p_first_point = std::lower_bound(
          reference_path_points.begin(), reference_path_points.end(), s, comp);
      if (p_first_point == reference_path_points.begin()) {
        width = reference_path_points.begin()->lane_width;
      } else if (p_first_point == reference_path_points.end()) {
        width = reference_path_points.end()->lane_width;
      } else {
        width = planning_math::lerp(
            (p_first_point - 1)->lane_width, (p_first_point - 1)->path_point.s,
            p_first_point->lane_width, p_first_point->path_point.s, s);
      }
    }
  }
  return std::max(width, 2.8);
}

bool VirtualLane::get_point_by_distance(double distance,
                                        FusionRoad::VirtualLanePoint *point) {
  double min_distance_diff = std::numeric_limits<double>::max();
  for (auto virtual_lane_refine_point :
       lane_reference_line_.virtual_lane_refline_points()) {
    double distance_diff =
        fabs(distance - virtual_lane_refine_point.car_point().x());
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

double VirtualLane::max_width() {
  switch (lane_type_) {
    case FusionRoad::LaneType::LANE_TYPE_UNKNOWN:;
    case FusionRoad::LaneType::LANE_TYPE_NORMAL:;
    case FusionRoad::LaneType::LANE_TYPE_VIRTUAL:;
    case FusionRoad::LaneType::LANE_TYPE_PARKING:;
    case FusionRoad::LaneType::LANE_TYPE_ACCELERATE:;
    case FusionRoad::LaneType::LANE_TYPE_DECELERATE:;
    case FusionRoad::LaneType::LANE_TYPE_BUS:;
    case FusionRoad::LaneType::LANE_TYPE_EMERGENCY:;
    case FusionRoad::LaneType::LANE_TYPE_ACCELERATE_DECELERATE:;
    case FusionRoad::LaneType::LANE_TYPE_LEFT_TURN_WAITTING_AREA:;
    case FusionRoad::LaneType::LANE_TYPE_NON_MOTOR:;
    case FusionRoad::LaneType::LANE_TYPE_RAMP:
      return 4.2;
    default:
      return DBL_MAX;
  }
}

bool VirtualLane::is_solid_line(int side) const {
  assert(side == 0 || side == 1);
  if (side == 0) {
    if (left_lane_boundary_.segment_size() > 0 &&
        left_lane_boundary_.segment(0).type() ==
            Common::LaneBoundaryType::MARKING_SOLID) {
      return true;
    }
  } else if (side == 1) {
    if (right_lane_boundary_.segment_size() > 0 &&
        right_lane_boundary_.segment(0).type() ==
            Common::LaneBoundaryType::MARKING_SOLID) {
      return true;
    }
  }
  return false;
}

double VirtualLane::min_width() {
  switch (lane_type_) {
    case FusionRoad::LaneType::LANE_TYPE_UNKNOWN:;
    case FusionRoad::LaneType::LANE_TYPE_NORMAL:;
    case FusionRoad::LaneType::LANE_TYPE_VIRTUAL:;
    case FusionRoad::LaneType::LANE_TYPE_PARKING:;
    case FusionRoad::LaneType::LANE_TYPE_ACCELERATE:;
    case FusionRoad::LaneType::LANE_TYPE_DECELERATE:;
    case FusionRoad::LaneType::LANE_TYPE_BUS:;
    case FusionRoad::LaneType::LANE_TYPE_EMERGENCY:;
    case FusionRoad::LaneType::LANE_TYPE_ACCELERATE_DECELERATE:;
    case FusionRoad::LaneType::LANE_TYPE_LEFT_TURN_WAITTING_AREA:;
    case FusionRoad::LaneType::LANE_TYPE_NON_MOTOR:;
    case FusionRoad::LaneType::LANE_TYPE_RAMP:
      return 2.8;
    default:
      return DBL_MAX;
  }
}

void VirtualLane::update_speed_limit(double ego_vel,
                                     double ego_v_cruise) {  // todo
  // update vision only v_cruise_
  if (get_lane_source() == FusionRoad::LaneSource::SOURCE_FUSION) {
    v_cruise_ = ego_v_cruise;
  }

  assert(reference_path_ != nullptr);
  auto &referece_path_points = reference_path_->get_points();
  if (referece_path_points.size() > 1) {
    double last_speed;
    bool find_last = false;
    bool find_change = false;
    double acc_brake_min = 100.0;
    for (size_t i = 1; i < referece_path_points.size(); ++i) {
      if (!find_last &&
          referece_path_points[i].path_point.s > 0.0) {  // hack: frenet_point
        find_last = true;
        last_speed = referece_path_points[i - 1].max_velocity;
        current_lane_speed_limit_ = last_speed;
        continue;
      }

      if (find_last && referece_path_points[i].max_velocity != last_speed) {
        double acc_brake = (std::pow(referece_path_points[i].max_velocity, 2) -
                            std::pow(ego_vel, 2)) /
                           std::max(1.0, referece_path_points[i].path_point.s);
        if (acc_brake < acc_brake_min) {
          acc_brake_min = acc_brake;
          find_change = true;
          speed_change_point_.x = referece_path_points[i].path_point.s;
          speed_change_point_.y = 0;
          speed_change_point_.speed = referece_path_points[i].max_velocity;
        }
      }
    }

    if (!find_change) {
      speed_change_point_.x = referece_path_points.back().path_point.s;
      speed_change_point_.y = 0;
      speed_change_point_.speed = referece_path_points.back().max_velocity;
    }
  }

  current_lane_speed_limit_ = std::min(current_lane_speed_limit_, ego_v_cruise);

  v_cruise_ = std::min(current_lane_speed_limit_, speed_change_point_.speed);
}

void VirtualLane::save_context(VirtualLaneContext &context) const {
  // todo: clren
}

void VirtualLane::restore_context(const VirtualLaneContext &context) {
  // todo: clren
}

}  // namespace planning
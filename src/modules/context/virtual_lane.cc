#include "virtual_lane.h"
#include "environmental_model.h"

#include <cassert>

#include "log.h"
#include "math/linear_interpolation.h"
#include "virtual_lane.h"

namespace {

constexpr double kMaxLaneWidth = 4.2;
constexpr double kMinLaneWidth = 2.8;

}  // namespace
namespace planning {
VirtualLane::VirtualLane() {}

void VirtualLane::update_data(const iflyauto::ReferenceLineMsg &lane) {
  order_id_ = lane.order_id;
  // virtual_id_ = lane.virtual_id();
  relative_id_ = lane.relative_id;
  // ego_lateral_offset_ = lane.ego_lateral_offset();

  lane_types_.reserve(lane.lane_types_size);
  for (int i = 0; i < lane.lane_types_size; i++) {
    lane_types_.emplace_back(lane.lane_types[i]);
  }
  lane_marks_.reserve(lane.lane_marks_size);
  for (int i = 0; i < lane.lane_marks_size; i++) {
    lane_marks_.emplace_back(lane.lane_marks[i]);
  }
  lane_sources_.reserve(lane.lane_sources_size);
  for (int i = 0; i < lane.lane_sources_size; i++) {
    lane_sources_.emplace_back(lane.lane_sources[i]);
  }
  lane_merge_split_point_ = lane.lane_merge_split_point;
  left_lane_boundary_ = lane.left_lane_boundary;
  right_lane_boundary_ = lane.right_lane_boundary;
  stop_line_ = lane.stop_line;

  if (left_lane_boundary_.existence) {
    if (right_lane_boundary_.existence) {
      lane_status_ = BOTH_AVAILABLE;
    } else {
      lane_status_ = LEFT_AVAILABLE;
    }
  } else {
    if (right_lane_boundary_.existence) {
      lane_status_ = RIGHT_AVAILABLE;
    } else {
      lane_status_ = BOTH_MISSING;
    }
  }
  const int num_of_reflane_point =
      lane.lane_reference_line.virtual_lane_refline_points_size;
  c_poly_.resize(NUM_OF_POLYNOMIAL);
  for (int i = 0; i < NUM_OF_POLYNOMIAL; i++) {
    c_poly_[i] = lane.lane_reference_line.poly_coefficient_car[i];
  }
  virtual_lane_refline_points_.resize(num_of_reflane_point);
  for (int i = 0; i < num_of_reflane_point; i++) {
    virtual_lane_refline_points_[i] =
        lane.lane_reference_line.virtual_lane_refline_points[i];
  }

  center_line_points_track_id_.clear();
  // for (auto &virtual_lane_refine_point :
  //      lane_reference_line_.virtual_lane_refline_points()) {
  // center_line_points_track_id_.emplace_back(virtual_lane_refine_point.track_id());
  // // todo
  // }

  width_ = width(0);
}

bool VirtualLane::calc_c_poly(std::vector<double> &output) {  // 该函数先保留着
  output.clear();

  if (lane_status_ == BOTH_MISSING) {
    output.assign(4, 0.0);
    return true;
  } else if (lane_status_ == LEFT_AVAILABLE) {
    for (size_t i = 0; i < NUM_OF_POLYNOMIAL; i++) {
      output.push_back((left_lane_boundary_.poly_coefficient[i]));
      if (i == 0) {
        output[0] =
            left_lane_boundary_.poly_coefficient[0] -
            0.5 * width() *
                std::sqrt(1 +
                          std::pow(left_lane_boundary_.poly_coefficient[1], 2));
      }
    }
  } else if (lane_status_ == RIGHT_AVAILABLE) {
    for (size_t i = 0; i < NUM_OF_POLYNOMIAL; i++) {
      output.push_back((right_lane_boundary_.poly_coefficient[i]));
      if (i == 0) {
        output[0] =
            right_lane_boundary_.poly_coefficient[0] +
            0.5 * width() *
                std::sqrt(
                    1 + std::pow(right_lane_boundary_.poly_coefficient[1], 2));
      }
    }
  } else {
    for (size_t i = 0; i < NUM_OF_POLYNOMIAL; i++) {
      output.push_back((left_lane_boundary_.poly_coefficient[i] +
                        right_lane_boundary_.poly_coefficient[i]) /
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

  // const std::vector<std::string> &other_point_ids =
  //     other->center_line_points_track_id();

  uint common_point_num = 0;
  // for (auto &p : other_point_ids) {  // todo
  // if (center_line_points_track_id_.find(p) !=
  // center_line_points_track_id_.end()) {
  //   common_point_num++;
  // }
  // }

  return common_point_num;
}

double VirtualLane::width(double x) {
  double width = 0;
  auto comp = [](const iflyauto::ReferencePoint &p, const double x) {
    return p.car_point.x < x;
  };
  auto p_first_point =
      std::lower_bound(virtual_lane_refline_points_.begin(),
                       virtual_lane_refline_points_.end(), x, comp);
  if (p_first_point == virtual_lane_refline_points_.begin()) {
    width = virtual_lane_refline_points_.begin()->lane_width;
  } else if (p_first_point == virtual_lane_refline_points_.end()) {
    width = std::prev(virtual_lane_refline_points_.end())->lane_width;
  } else {
    width = planning_math::lerp(
        (p_first_point - 1)->lane_width, (p_first_point - 1)->car_point.x,
        p_first_point->lane_width, p_first_point->car_point.x, x);
  }
  return std::min(std::max(width, kMinLaneWidth), kMaxLaneWidth);
}

double VirtualLane::width_by_s(double s) {
  double width = 0;
  if (reference_path_ != nullptr) {
    auto &reference_path_points = reference_path_->get_points();
    if (!reference_path_points.empty()) {
      auto comp = [](const ReferencePathPoint &p, const double s) {
        return p.path_point.s < s;
      };
      auto p_first_point = std::lower_bound(
          reference_path_points.begin(), reference_path_points.end(), s, comp);
      if (p_first_point == reference_path_points.begin()) {
        width = reference_path_points.front().lane_width;
      } else if (p_first_point == reference_path_points.end()) {
        width = reference_path_points.back().lane_width;
      } else {
        width = planning_math::lerp(
            (p_first_point - 1)->lane_width, (p_first_point - 1)->path_point.s,
            p_first_point->lane_width, p_first_point->path_point.s, s);
      }
    }
  }
  return std::min(std::max(width, min_width()), max_width());
}

bool VirtualLane::get_point_by_distance(double distance,
                                        iflyauto::ReferencePoint &point) {
  double min_distance_diff = std::numeric_limits<double>::max();
  for (auto &virtual_lane_refine_point : virtual_lane_refline_points_) {
    double distance_diff =
        fabs(distance - virtual_lane_refine_point.car_point.x);
    if (distance_diff < min_distance_diff) {
      min_distance_diff = distance_diff;
      point = virtual_lane_refine_point;
    }
  }

  if (min_distance_diff < std::numeric_limits<double>::max()) {
    return true;
  }
  return false;
}

double VirtualLane::max_width() const {
  if (lane_types_.size() > 0) {
    switch (lane_types_[0].type) {
      case iflyauto::LANETYPE_UNKNOWN:;
      case iflyauto::LANETYPE_NORMAL:;
      case iflyauto::LANETYPE_VIRTUAL:;
      case iflyauto::LANETYPE_PARKING:;
      case iflyauto::LANETYPE_ACCELERATE:;
      case iflyauto::LANETYPE_DECELERATE:;
      case iflyauto::LANETYPE_BUS:;
      case iflyauto::LANETYPE_EMERGENCY:;
      case iflyauto::LANETYPE_ACCELERATE_DECELERATE:;
      case iflyauto::LANETYPE_LEFT_TURN_WAITTING_AREA:;
      case iflyauto::LANETYPE_NON_MOTOR:;
        return kMaxLaneWidth;
      default:
        LOG_ERROR("Error Lane Type");
        return kMinLaneWidth;
    }
  } else {
    LOG_ERROR("Error Lane Type");
    return kMinLaneWidth;
  }
}

bool VirtualLane::is_solid_line(int side) const {
  assert(side == 0 || side == 1);
  if (side == 0) {
    if (left_lane_boundary_.type_segments_size > 0 &&
        left_lane_boundary_.type_segments[0].type ==
            iflyauto::LaneBoundaryType_MARKING_SOLID) {
      return true;
    }
  } else if (side == 1) {
    if (right_lane_boundary_.type_segments_size > 0 &&
        right_lane_boundary_.type_segments[0].type ==
            iflyauto::LaneBoundaryType_MARKING_SOLID) {
      return true;
    }
  }
  return false;
}

double VirtualLane::min_width() const {
  if (lane_types_.size() > 0) {
    switch (lane_types_[0].type) {
      case iflyauto::LANETYPE_UNKNOWN:;
      case iflyauto::LANETYPE_NORMAL:;
      case iflyauto::LANETYPE_VIRTUAL:;
      case iflyauto::LANETYPE_PARKING:;
      case iflyauto::LANETYPE_ACCELERATE:;
      case iflyauto::LANETYPE_DECELERATE:;
      case iflyauto::LANETYPE_BUS:;
      case iflyauto::LANETYPE_EMERGENCY:;
      case iflyauto::LANETYPE_ACCELERATE_DECELERATE:;
      case iflyauto::LANETYPE_LEFT_TURN_WAITTING_AREA:;
      case iflyauto::LANETYPE_NON_MOTOR:;
        return kMinLaneWidth;
      default:
        LOG_ERROR("Error Lane Type");
        return kMinLaneWidth;
    }
  } else {
    LOG_ERROR("Error Lane Type");
    return kMinLaneWidth;
  }
}

void VirtualLane::update_speed_limit(double ego_vel,
                                     double ego_v_cruise) {  // todo
  // update vision only v_cruise_
  if (get_lane_source() == iflyauto::LaneSource_SOURCE_CAMERA_HDMAP_FUSION) {
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

void VirtualLane::update_lane_tasks(const GeneralTaskMapInfo& general_task_map_info) {
  current_tasks_.clear();
  bool ego_on_ramp = general_task_map_info.is_on_ramp;
  if (ego_on_ramp) {
    ProcessEgoOnRampMLC(general_task_map_info);
  } else {
    ProcessEgoOnRoadMLC(general_task_map_info);
  }
}
void VirtualLane::ProcessEgoOnRoadMLC(const GeneralTaskMapInfo& general_task_map_info) {
  bool is_nearing_other_lane_merge_to_road_point = general_task_map_info.is_nearing_other_lane_merge_to_road_point;
  RampDirection first_merge_direction = general_task_map_info.first_merge_direction;
  RampDirection ramp_direction = general_task_map_info.ramp_direction;
  bool is_nearing_ramp = general_task_map_info.is_nearing_ramp;
  bool is_on_ramp = general_task_map_info.is_on_ramp;
  const int lane_num = general_task_map_info.lane_num_except_emergency;
  bool is_trigger_ego_not_on_side = general_task_map_info.is_leaving_ramp && !general_task_map_info.is_on_ramp;
  if (is_nearing_other_lane_merge_to_road_point) {//主路前方接近汇入区域的变道
    if (first_merge_direction == RAMP_ON_LEFT) {
      if (order_id_ + 1 == lane_num) {
        current_tasks_.emplace_back(-1);
        std::cout << "高速前方右侧有汇入车道，最右侧车道产生一个变道任务"
                  << std::endl;
      }
    } else if (first_merge_direction == RAMP_ON_RIGHT) {
      if (order_id_ == 0) {
        current_tasks_.emplace_back(1);
        std::cout << "高速前方左侧有汇入车道，最左侧车道产生一个变道任务"
                  << std::endl;
      }
    }
  } else if (is_nearing_ramp && !is_on_ramp) {//在主路上，前方接近ramp的变道
    if (ramp_direction == RAMP_ON_RIGHT) {
      for (int i = 0; i + order_id_ + 1 < lane_num; i++) {
        current_tasks_.emplace_back(1);
      }
    } else if (ramp_direction == RAMP_ON_LEFT) {
      for (int i = order_id_; i > 0; i--) {
        current_tasks_.emplace_back(-1);
      }
    }
  } else if (is_trigger_ego_not_on_side) {//在主路上，触发自车不在最右侧车道上的变道
    //TODO（fengwang31）：需要考虑上一次汇入的方向。目前默认匝道都是从右边汇入主路的
    if (order_id_ + 1 == lane_num) {
      current_tasks_.emplace_back(-1);
      std::cout << "在最右侧车道上时,向左产生一个变道任务" << std::endl;
    }
  }
}
void VirtualLane::ProcessEgoOnRampMLC(const GeneralTaskMapInfo& general_task_map_info) {
  const double dis_to_first_merge = general_task_map_info.distance_to_first_road_merge;
  const double dis_to_first_split = general_task_map_info.distance_to_first_road_split;
  const RampDirection first_merge_direction = general_task_map_info.first_merge_direction;
  const double trigger_mlc_distance_threshold_to_first_split_when_ego_on_ramp =
      266;
  const RampDirection first_split_direction = general_task_map_info.first_split_direction;
  const int lane_num = general_task_map_info.lane_num_except_emergency;
  const bool is_ramp_merge_to_road_on_expressway = general_task_map_info.is_ramp_merge_to_road_on_expressway;
  const bool is_ramp_merge_to_ramp_on_expressway = general_task_map_info.is_ramp_merge_to_ramp_on_expressway;
  const bool is_leaving_ramp = general_task_map_info.is_leaving_ramp;
  const double dis_to_second_merge = general_task_map_info.distance_to_second_road_merge;
  const double dis_to_second_split = general_task_map_info.distance_to_second_road_split;
  const double sum_dis_to_last_split_point_on_ramp = general_task_map_info.sum_dis_to_last_split_point_on_ramp;
  const RampDirection second_split_direction = general_task_map_info.second_split_direction;
  const RampDirection second_merge_direction = general_task_map_info.second_merge_direction;
  //在匝道上，经过split点50m后再开始触发变道任务，避免自车还在split的区域内
  const double dis_to_last_spli_threshold = 50;
  //在匝道汇入匝道时，距离merge的距离在200m范围内时，再生成地图变道任务，避免前面有1分2场景的不合理变道
  const double dis_to_first_merge_threshold = 100;
  //首先处理匝道上的分叉口
  if (dis_to_first_merge > dis_to_first_split &&
      dis_to_first_split <
        trigger_mlc_distance_threshold_to_first_split_when_ego_on_ramp) {
    if (first_split_direction == RAMP_ON_RIGHT) {
      for (int i = 0; i + order_id_ + 1 < lane_num; i++) {
        current_tasks_.emplace_back(1);
      }
    } else if (first_split_direction == RAMP_ON_LEFT) {
      for (int i = order_id_; i > 0; i--) {
        current_tasks_.emplace_back(-1);
      }
    }
  } else if (is_ramp_merge_to_road_on_expressway &&
             is_leaving_ramp) {//处理匝道汇入主路的场景
    if (first_merge_direction == RAMP_ON_RIGHT) {
      for (int i = order_id_; i > 0; i--) {
        current_tasks_.emplace_back(-1);
      }
    } else if (first_merge_direction == RAMP_ON_LEFT) {
      for (int i = 0; i + order_id_ + 1 < lane_num; i++) {
        current_tasks_.emplace_back(1);
      }
    }
  } else if (is_ramp_merge_to_ramp_on_expressway &&
            sum_dis_to_last_split_point_on_ramp > dis_to_last_spli_threshold &&
            dis_to_first_merge < dis_to_first_merge_threshold) {
    RampDirection next_lc_dir = RAMP_NONE;
    if (dis_to_second_merge < dis_to_first_split) {
      //下下一个场景还是merge场景
      next_lc_dir = second_merge_direction;
      if (first_merge_direction == RAMP_ON_RIGHT &&
          next_lc_dir == RAMP_ON_RIGHT) {
        for (int i = order_id_; i > 0; i--) {
          current_tasks_.emplace_back(-1);
        }
      } else if (first_merge_direction == RAMP_ON_LEFT &&
          next_lc_dir == RAMP_ON_LEFT) {
        for (int i = 0; i + order_id_ + 1 < lane_num; i++) {
          current_tasks_.emplace_back(1);
        }
      }
    } else if (dis_to_second_merge >= dis_to_first_split) {
      //下下一个是split的场景
      next_lc_dir = first_split_direction;
      if (first_merge_direction == RAMP_ON_RIGHT &&
          next_lc_dir == RAMP_ON_LEFT) {
        for (int i = order_id_; i > 0; i--) {
          current_tasks_.emplace_back(-1);
        }
      } else if (first_merge_direction == RAMP_ON_LEFT &&
          next_lc_dir == RAMP_ON_RIGHT) {
        for (int i = 0; i + order_id_ + 1 < lane_num; i++) {
          current_tasks_.emplace_back(1);
        }
      }
    }
  }
  //TODO（fengwang31）：匝道汇入匝道的scean
}
void VirtualLane::save_context(VirtualLaneContext &context) const {
  // todo: clren
}

void VirtualLane::restore_context(const VirtualLaneContext &context) {
  // todo: clren
}

}  // namespace planning
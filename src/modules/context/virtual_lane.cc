#include "virtual_lane.h"

#include <algorithm>
#include <cassert>

#include "config/basic_type.h"
#include "environmental_model.h"
#include "log.h"
#include "math/linear_interpolation.h"
#include "planning_context.h"
#include "virtual_lane.h"

namespace {

constexpr double kMaxLaneWidth = 4.2;
constexpr double kMinLaneWidth = 2.8;

}  // namespace
namespace planning {
VirtualLane::VirtualLane() {}

void VirtualLane::update_data(const iflyauto::ReferenceLineMsg &lane) {
  is_nearing_ramp_mlc_task_ = false;
  is_nearing_split_mlc_task_ = false;
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
  c_poly_.resize(FUSION_ROAD_LINE_POLYNOMIAL_NUM);
  for (int i = 0; i < FUSION_ROAD_LINE_POLYNOMIAL_NUM; i++) {
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
    for (size_t i = 0; i < FUSION_ROAD_LINE_POLYNOMIAL_NUM; i++) {
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
    for (size_t i = 0; i < FUSION_ROAD_LINE_POLYNOMIAL_NUM; i++) {
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
    for (size_t i = 0; i < FUSION_ROAD_LINE_POLYNOMIAL_NUM; i++) {
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
        return p.path_point.s() < s;
      };
      auto p_first_point = std::lower_bound(
          reference_path_points.begin(), reference_path_points.end(), s, comp);
      if (p_first_point == reference_path_points.begin()) {
        width = reference_path_points.front().lane_width;
      } else if (p_first_point == reference_path_points.end()) {
        width = reference_path_points.back().lane_width;
      } else {
        width = planning_math::lerp((p_first_point - 1)->lane_width,
                                    (p_first_point - 1)->path_point.s(),
                                    p_first_point->lane_width,
                                    p_first_point->path_point.s(), s);
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

bool VirtualLane::is_dash_line(
    const planning::framework::Session &session, const RequestType request_type,
    const std::shared_ptr<planning_math::KDPath> target_boundary_path) const {
  const auto &ego_state = session.environmental_model().get_ego_state_manager();
  double lane_line_length = 0.0;
  const auto &plannig_init_point = ego_state->planning_init_point();
  double ego_x = plannig_init_point.lat_init_state.x();
  double ego_y = plannig_init_point.lat_init_state.y();
  double ego_s = 0.0, ego_l = 0.0;
  iflyauto::LaneBoundary target_lane_boundary =
      request_type == LEFT_CHANGE ? left_lane_boundary_ : right_lane_boundary_;
  if (target_boundary_path != nullptr) {
    if (!target_boundary_path->XYToSL(ego_x, ego_y, &ego_s, &ego_l)) {
      return false;
    }
  } else {
    return false;
  }
  for (int i = 0; i < target_lane_boundary.type_segments_size; i++) {
    lane_line_length += target_lane_boundary.type_segments[i].length;
    if (lane_line_length > ego_s) {
      return target_lane_boundary.type_segments[i].type ==
                 iflyauto::LaneBoundaryType_MARKING_DASHED ||
             target_lane_boundary.type_segments[i].type ==
                 iflyauto::LaneBoundaryType_MARKING_VIRTUAL;
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
          referece_path_points[i].path_point.s() > 0.0) {  // hack: frenet_point
        find_last = true;
        last_speed = referece_path_points[i - 1].max_velocity;
        current_lane_speed_limit_ = last_speed;
        continue;
      }

      if (find_last && referece_path_points[i].max_velocity != last_speed) {
        double acc_brake =
            (std::pow(referece_path_points[i].max_velocity, 2) -
             std::pow(ego_vel, 2)) /
            std::max(1.0, referece_path_points[i].path_point.s());
        if (acc_brake < acc_brake_min) {
          acc_brake_min = acc_brake;
          find_change = true;
          speed_change_point_.x = referece_path_points[i].path_point.s();
          speed_change_point_.y = 0;
          speed_change_point_.speed = referece_path_points[i].max_velocity;
        }
      }
    }

    if (!find_change) {
      speed_change_point_.x = referece_path_points.back().path_point.s();
      speed_change_point_.y = 0;
      speed_change_point_.speed = referece_path_points.back().max_velocity;
    }
  }

  current_lane_speed_limit_ = std::min(current_lane_speed_limit_, ego_v_cruise);

  v_cruise_ = std::min(current_lane_speed_limit_, speed_change_point_.speed);
}

void VirtualLane::update_lane_tasks(const RouteInfoOutput &route_info_output) {
  current_tasks_.clear();
  bool ego_on_ramp = route_info_output.is_on_ramp;
  if (ego_on_ramp) {
    ProcessEgoOnRampMLC(route_info_output);
  } else {
    ProcessEgoOnRoadMLC(route_info_output);
  }
}
void VirtualLane::ProcessEgoOnRoadMLC(
    const RouteInfoOutput &route_info_output) {
  bool is_nearing_other_lane_merge_to_road_point =
      route_info_output.is_nearing_other_lane_merge_to_road_point;
  RampDirection first_merge_direction = route_info_output.first_merge_direction;
  RampDirection ramp_direction = route_info_output.ramp_direction;
  bool is_nearing_ramp = route_info_output.is_nearing_ramp;
  bool is_on_ramp = route_info_output.is_on_ramp;
  const int lane_num = route_info_output.lane_num_except_emergency;
  bool is_trigger_ego_not_on_side =
      route_info_output.is_leaving_ramp && !route_info_output.is_on_ramp;
  int lc_nums_for_split = route_info_output.lc_nums_for_split;
  bool is_ego_on_split_region = route_info_output.is_ego_on_split_region;
  int need_continue_lc_num_on_off_ramp_region =
      route_info_output.need_continue_lc_num_on_off_ramp_region;
  // 生成各个场景的变道任务
  if (need_continue_lc_num_on_off_ramp_region !=
      0) {  // 处理在下匝道的split区域继续生成1个下匝道的任务
    current_tasks_.emplace_back(need_continue_lc_num_on_off_ramp_region);
  } else if (lc_nums_for_split !=
             0) {  // 处理在接近split的区域生成1个选择split的任务
    current_tasks_.emplace_back(lc_nums_for_split);
    if (relative_id_ == 0) {
      //表示当前车道,输出给下游模块表示是在接近split的变道场景
      is_nearing_split_mlc_task_ = true;
    }
  } else if (
      is_nearing_other_lane_merge_to_road_point) {  // 主路前方接近汇入区域的变道
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
  } else if (is_nearing_ramp && !is_on_ramp &&
             !is_ego_on_split_region) {  // 在主路上，前方接近ramp的变道
    if (ramp_direction == RAMP_ON_RIGHT) {
      for (int i = 0; i + order_id_ + 1 < lane_num; i++) {
        current_tasks_.emplace_back(1);
        if (relative_id_ == 0) { //表示当前车道,输出给speed adjudst的标志位
          is_nearing_ramp_mlc_task_ = true;
        }
      }
    } else if (ramp_direction == RAMP_ON_LEFT) {
      for (int i = order_id_; i > 0; i--) {
        current_tasks_.emplace_back(-1);
        if (relative_id_ == 0) { //表示当前车道,输出给speed adjudst的标志位
          is_nearing_ramp_mlc_task_ = true;
        }
      }
    }
  } else if (
      is_trigger_ego_not_on_side) {  // 在主路上，触发自车不在最右侧车道上的变道
    // TODO（fengwang31）：需要考虑上一次汇入的方向。目前默认匝道都是从右边汇入主路的
    if (order_id_ + 1 == lane_num) {
      current_tasks_.emplace_back(-1);
      std::cout << "在最右侧车道上时,向左产生一个变道任务" << std::endl;
    }
  }
}
void VirtualLane::ProcessEgoOnRampMLC(
    const RouteInfoOutput &route_info_output) {
  const double dis_to_first_merge =
      route_info_output.distance_to_first_road_merge;
  const double dis_to_first_split =
      route_info_output.distance_to_first_road_split;
  const RampDirection first_merge_direction =
      route_info_output.first_merge_direction;
  const RampDirection first_split_direction =
      route_info_output.first_split_direction;
  const int lane_num = route_info_output.lane_num_except_emergency;
  const bool is_ramp_merge_to_road_on_expressway =
      route_info_output.is_ramp_merge_to_road_on_expressway;
  const bool is_ramp_merge_to_ramp_on_expressway =
      route_info_output.is_ramp_merge_to_ramp_on_expressway;
  const bool is_leaving_ramp = route_info_output.is_leaving_ramp;
  const double dis_to_second_merge =
      route_info_output.distance_to_second_road_merge;
  const double sum_dis_to_last_split_point_on_ramp =
      route_info_output.sum_dis_to_last_split_point_on_ramp;
  const RampDirection second_merge_direction =
      route_info_output.second_merge_direction;
  const bool is_ego_on_split_region = route_info_output.is_ego_on_split_region;
  const int merge_seg_forward_lane_nums =
      route_info_output.merge_seg_forward_lane_nums;
  const int merge_last_seg_forward_lane_nums =
      route_info_output.merge_last_seg_forward_lane_nums;
  // 在匝道汇入匝道时，距离merge的距离在100m范围内时，
  // 再生成地图变道任务，避免前面有1分2场景的不合理变道
  const double dis_to_first_merge_threshold = 100;
  int need_continue_lc_num_on_off_ramp_region =
      route_info_output.need_continue_lc_num_on_off_ramp_region;
  // 生成各个场景的变道任务
  if (need_continue_lc_num_on_off_ramp_region !=
      0) {  // 处理在下匝道的split区域继续生成1个下匝道的任务
    current_tasks_.emplace_back(need_continue_lc_num_on_off_ramp_region);
  } else if (dis_to_first_merge > dis_to_first_split &&
             !is_ego_on_split_region) {  // 首先处理匝道上的分叉口
    if (first_split_direction == RAMP_ON_RIGHT) {
      for (int i = 0; i + order_id_ + 1 < lane_num; i++) {
        current_tasks_.emplace_back(1);
      }
      if (relative_id_ == 0) {
        //表示当前车道,输出给下游模块表示是在接近split的变道场景
        is_nearing_split_mlc_task_ = true;
      }
    } else if (first_split_direction == RAMP_ON_LEFT) {
      for (int i = order_id_; i > 0; i--) {
        current_tasks_.emplace_back(-1);
      }
      if (relative_id_ == 0) {
        //表示当前车道,输出给下游模块表示是在接近split的变道场景
        is_nearing_split_mlc_task_ = true;
      }
    }
  } else if (is_ramp_merge_to_road_on_expressway &&
             is_leaving_ramp) {  // 处理匝道汇入主路的场景
    //在匝道汇入主路一般有两种：
    // 1、从右边汇入主路，一般是右边车道数较少，汇入主路车道数较多的场景，
    // 2、从左边汇入主路，目前出现的case中，左边汇入时，车道都是continue，没有左侧车道被汇入截断，因此在处理左侧汇入场景时，初步加一个车道数判断
    if (first_merge_direction == RAMP_ON_RIGHT) {
      // 右边汇入
      for (int i = order_id_; i > 0; i--) {
        current_tasks_.emplace_back(-1);
      }
    } else if (first_merge_direction == RAMP_ON_LEFT) {
      // 左边汇入
      if (merge_seg_forward_lane_nums > 1 &&
          merge_last_seg_forward_lane_nums == 1) {
        // 只有一条车道汇入时，正常往主路变道
        for (int i = 0; i + order_id_ + 1 < lane_num; i++) {
          current_tasks_.emplace_back(1);
        }
      } else if (merge_seg_forward_lane_nums >= 2 &&
                 merge_last_seg_forward_lane_nums >= 2) {
        // 有2条及以上车道往主路汇，且主路的车道数也大于等于2条时，说明当前车道大概率是continue的，那么自车不呆在最左侧车道即可。
        // 如果两条都不是continue，那么靠汇流变道了，或者后期加上前方收窄的信息后优化
        if (order_id_ == 0) {
          current_tasks_.emplace_back(1);
        }
      }
    }
  } else if (is_ramp_merge_to_ramp_on_expressway &&  // 匝道汇入匝道的scean
             !is_ego_on_split_region &&
             dis_to_first_merge < dis_to_first_merge_threshold) {
    RampDirection next_lc_dir = RAMP_NONE;
    if (dis_to_second_merge < dis_to_first_split) {
      // 下下一个场景还是merge场景
      next_lc_dir = second_merge_direction;
      if (first_merge_direction == RAMP_ON_RIGHT &&
          next_lc_dir == RAMP_ON_RIGHT) {
        for (int i = order_id_; i > 0; i--) {
          current_tasks_.emplace_back(-1);
        }
      }
      // fengwang31:由于目前大部分匝道汇入匝道的场景中，都是右边汇入的车道收窄，因此这里暂时不考虑左边的情况。
      //  } else if (first_merge_direction == RAMP_ON_LEFT &&
      //             next_lc_dir == RAMP_ON_LEFT) {
      //    for (int i = 0; i + order_id_ + 1 < lane_num; i++) {
      //      current_tasks_.emplace_back(1);
      //    }
      //  }
    } else if (dis_to_second_merge >= dis_to_first_split) {
      // 下下一个是split的场景
      next_lc_dir = first_split_direction;
      if (first_merge_direction == RAMP_ON_RIGHT &&
          next_lc_dir == RAMP_ON_LEFT) {
        for (int i = order_id_; i > 0; i--) {
          current_tasks_.emplace_back(-1);
        }
      }
      // fengwang31:由于目前大部分匝道汇入匝道的场景中，都是右边汇入的车道收窄，因此这里暂时不考虑左边的情况。
      //  else if (first_merge_direction == RAMP_ON_LEFT &&
      //             next_lc_dir == RAMP_ON_RIGHT) {
      //    for (int i = 0; i + order_id_ + 1 < lane_num; i++) {
      //      current_tasks_.emplace_back(1);
      //    }
      //  }
    }
  }
}
void VirtualLane::save_context(VirtualLaneContext &context) const {
  // todo: clren
}

void VirtualLane::restore_context(const VirtualLaneContext &context) {
  // todo: clren
}

}  // namespace planning
#include "cone_lane_change_request.h"

#include <glog/logging.h>
#include <math.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <complex>
#include <cstddef>
#include <limits>

#include "common.pb.h"
#include "common_c.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "define/geometry.h"
#include "ego_state_manager.h"
#include "ifly_time.h"
#include "lateral_obstacle.h"
#include "log.h"
#include "planning_context.h"
#include "tasks/behavior_planners/lane_change_decider/lane_change_requests/lane_change_request.h"
#include "virtual_lane_manager.h"

namespace planning {

namespace {
constexpr double kLongClusterCoeff = 5.0;
constexpr double kLatClusterThre = 0.3;
constexpr double kLatPassThre = 0.8;
constexpr double kLatPassThreBuffer = 0.35;
constexpr double kConeCrossingLaneLineBuffer = 0.15;
constexpr uint32_t kConeAlcCountThre = 4;
constexpr uint32_t kConeAlcMaxCountThre = 8;
constexpr uint32_t kConeAlcHystereticCount = 1;
constexpr int kConeAlcCountLowerThre = 0;
constexpr double kLongClusterTimeGap = 4.0;
constexpr double kDefaultLaneWidth = 4.5;
constexpr double kConeLaneChangelateralDistancethre = 2.25;
constexpr double kMinDefaultLaneWidth = 2.65;
constexpr uint32_t kConeDirecSize = 5;
constexpr double kConeDirecThre = 0.5;
constexpr double kConeSlopeThre = 1;
constexpr int kInvalidAgentId = -1;
}  // namespace
// class: ConeRequest
ConeRequest::ConeRequest(
    planning::framework::Session* session,
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
    std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr)
    : LaneChangeRequest(session, virtual_lane_mgr, lane_change_lane_mgr) {
  base_frenet_coord_ = std::make_shared<planning_math::KDPath>();
}

void ConeRequest::Update(int lc_status) {
  std::cout << "ConeRequest::Update::coming cone lane change request"
            << std::endl;
  lc_request_cancel_reason_ = IntCancelReasonType::NO_CANCEL;
  // trigger EA lane change when lane keep status.
  if (lc_status != kLaneKeeping && lc_status != kLaneChangePropose) {
    ILOG_DEBUG << "ConeRequest::Update: ego not in lane keeping!";
    return;
  }

  // intersection surpression
  if (EgoInIntersection()) {
    Reset();
    Finish();
    return;
  }

  lateral_obstacle_ = session_->environmental_model().get_lateral_obstacle();
  lane_tracks_manager_ =
      session_->environmental_model().get_lane_tracks_manager();

  const int current_lane_virtual_id =
      virtual_lane_mgr_->current_lane_virtual_id();

  if (lane_change_lane_mgr_->has_origin_lane()) {
    auto origin_lane = lane_change_lane_mgr_->olane();
    origin_lane_virtual_id_ = origin_lane->get_virtual_id();
  } else {
    origin_lane_virtual_id_ = current_lane_virtual_id;
  }
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  planning_init_point_ = ego_state->planning_init_point();
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  int fix_lane_virtual_id = lane_change_decider_output.fix_lane_virtual_id;
  int olane_virtual_id = lane_change_decider_output.origin_lane_virtual_id;
  int target_lane_virtual_id =
      lane_change_decider_output.target_lane_virtual_id;
  auto flane = virtual_lane_mgr_->get_lane_with_virtual_id(fix_lane_virtual_id);
  auto olane = virtual_lane_mgr_->get_lane_with_virtual_id(olane_virtual_id);
  auto tlane =
      virtual_lane_mgr_->get_lane_with_virtual_id(target_lane_virtual_id);

  UpdateConeSituation(lc_status);
  ILOG_DEBUG << "ConeRequest::Update: is_cone_lane_change_situation "
             << is_cone_lane_change_situation_;
  JSON_DEBUG_VALUE("is_cone_lane_change_situation_",
                   is_cone_lane_change_situation_);

  if (!is_cone_must_lane_change_situation_) {
    if (!is_cone_lane_change_situation_) {
      if (request_type_ != NO_CHANGE &&
          (lane_change_lane_mgr_->has_origin_lane() &&
           lane_change_lane_mgr_->is_ego_on(olane))) {
        Finish();
        Reset();
        set_target_lane_virtual_id(current_lane_virtual_id);
        ILOG_DEBUG
            << "[ConeRequest::update] " << __FUNCTION__ << " " << __LINE__
            << " finish request, !trigger_left_clc and !trigger_right_clc";
      }
      return;
    }
  }
  ConeDir();
  JSON_DEBUG_VALUE("cone_lane_change_direction_",
                   (int)cone_lane_change_direction_);

  setLaneChangeRequestByCone();
  if (trigger_lane_change_cancel_) {
    Finish();
    Reset();
    set_target_lane_virtual_id(current_lane_virtual_id);
    ILOG_DEBUG << "[ConeRequest::update] " << __FUNCTION__ << " " << __LINE__
               << " finish request, trigger_lane_change_cancel_ is true";
    lc_request_cancel_reason_ = IntCancelReasonType::MANUAL_CANCEL;
    return;
  }
  ILOG_DEBUG << "request_type_: [" << request_type_ << "] turn_signal_: [ "
             << turn_signal_ << "]";
}

void ConeRequest::UpdateConeSituation(int lc_status) {
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& function_info = session_->environmental_model().function_info();
  const auto& rlane = virtual_lane_mgr_->get_right_lane();
  const auto& llane = virtual_lane_mgr_->get_left_lane();
  const auto& ref_path_mgr = session_->environmental_model().get_reference_path_manager();
  const auto cur_reference_path =
      ref_path_mgr->get_reference_path_by_current_lane();
  double k_left_cone_occ_lane_line_buffer = kConeCrossingLaneLineBuffer;
  double k_right_cone_occ_lane_line_buffer = kConeCrossingLaneLineBuffer;
  double k_default_ego_pass_buffer = kLatPassThre;
  right_lane_nums_ = 0;
  left_lane_nums_ = 0;

  const auto base_lane =
      virtual_lane_mgr_->get_lane_with_virtual_id(origin_lane_virtual_id_);
  if (base_lane == nullptr) {
    ILOG_DEBUG << "base lane not exist";
    is_cone_lane_change_situation_ = false;
    return;
  }
  JSON_DEBUG_VALUE("cone_alc_trigger_counter_", cone_alc_trigger_counter_);
  std::shared_ptr<ReferencePath> origin_refline =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->get_reference_path_by_lane(origin_lane_virtual_id_, false);

  const auto& frenet_obstacles_map = origin_refline->get_obstacles_map();
  base_frenet_coord_ = origin_refline->get_frenet_coord();
  Point2D ego_frenet_point;
  Point2D ego_cart_point{planning_init_point_.lat_init_state.x(),
                         planning_init_point_.lat_init_state.y()};
  if (!base_frenet_coord_->XYToSL(ego_cart_point, ego_frenet_point)) {
    ILOG_DEBUG << "fail to get ego position on base lane";
    is_cone_lane_change_situation_ = false;
    return;
  }

  auto lane_nums_msg = base_lane->get_lane_nums();
  auto iter =
      std::find_if(lane_nums_msg.begin(), lane_nums_msg.end(),
                   [&ego_frenet_point](const iflyauto::LaneNumMsg& lane_num) {
                     return lane_num.begin <= ego_frenet_point.x &&
                            lane_num.end > ego_frenet_point.x;
                   });
  if (iter != lane_nums_msg.end()) {
    left_lane_nums_ = iter->left_lane_num;
    right_lane_nums_ = iter->right_lane_num;
  } else {
    left_lane_nums_ = llane ? 1 : 0;
    right_lane_nums_ = rlane ? 1 : 0;
  }

  if (llane == nullptr) {
    k_right_cone_occ_lane_line_buffer += 0.25;
  }
  if (rlane == nullptr) {
    k_left_cone_occ_lane_line_buffer += 0.25;
  }
  if (function_info.function_mode() == common::DrivingFunctionInfo::NOA) {
    k_default_ego_pass_buffer = kLatPassThre + kLatPassThreBuffer;
  }

  const double ego_rear_edge = vehicle_param.rear_edge_to_rear_axle;
  double eps_s = vehicle_param.length * kLongClusterCoeff;
  double eps_l = vehicle_param.width + kLatClusterThre;
  int cone_nums_of_front_objects = 0;
  int minPts = 1;
  cone_points_.clear();
  cone_cluster_attribute_set_.clear();

  const auto& tracks_map = lateral_obstacle_->tracks_map();
  const auto& front_obstacles_array = lateral_obstacle_->front_tracks();
  for (const auto front_obstacle : front_obstacles_array) {
    int obstacle_id = front_obstacle->id();
    auto front_vehicle_iter = tracks_map.find(obstacle_id);
    Point2D ego_cart_point{planning_init_point_.lat_init_state.x(),
                           planning_init_point_.lat_init_state.y()};
    if (front_vehicle_iter != tracks_map.end()) {
      if (obstacle_id == kInvalidAgentId) {
        continue;
      }
      if (front_vehicle_iter->second->type() ==
              iflyauto::OBJECT_TYPE_TRAFFIC_CONE ||
          (front_vehicle_iter->second->type() ==
               iflyauto::OBJECT_TYPE_CTASH_BARREL &&
           function_info.function_mode() == common::DrivingFunctionInfo::NOA)) {
        if (front_vehicle_iter->second->d_s_rel() < -ego_rear_edge ||
            front_vehicle_iter->second->d_s_rel() >
                base_frenet_coord_->Length() - ego_frenet_point.x) {
          continue;
        }
        cone_nums_of_front_objects++;
        Point2D obs_cart_point{0.0, 0.0};
        Point2D obs_frenet_point;
        obs_cart_point.x = front_vehicle_iter->second->obstacle()->x_center();
        obs_cart_point.y = front_vehicle_iter->second->obstacle()->y_center();
        // if (!base_frenet_coord_->XYToSL(obs_cart_point, obs_frenet_point)) {
        //   continue;
        // }

        if (frenet_obstacles_map.find(obstacle_id) ==
                frenet_obstacles_map.end() ||
            !frenet_obstacles_map.at(obstacle_id)->b_frenet_valid()) {
          continue;
        }
        double cone_s = front_vehicle_iter->second->frenet_s();
        double cone_l = front_vehicle_iter->second->frenet_l();
        double dist_to_left_boundary;
        if (!GetOriginLaneWidthByCone(base_lane, cone_s, cone_l, true,
                                      &dist_to_left_boundary)) {
          is_cone_lane_change_situation_ = false;
          return;
        }
        double dist_to_right_boundary;
        if (!GetOriginLaneWidthByCone(base_lane, cone_s, cone_l, false,
                                      &dist_to_right_boundary)) {
          is_cone_lane_change_situation_ = false;
          return;
        }
        auto point = ConePoint(front_vehicle_iter->first, obs_cart_point.x,
                               obs_cart_point.y, cone_s, cone_l,
                               dist_to_left_boundary, dist_to_right_boundary);
        cone_points_.push_back(point);
      }
    } else {
      continue;
    }
  }
  JSON_DEBUG_VALUE("cone_nums_of_front_objects", cone_nums_of_front_objects);

  if (cone_points_.empty()) {
    // if no cones found, counter--
    ILOG_DEBUG << "no cone found!!!";
    cone_alc_trigger_counter_ =
        std::max(cone_alc_trigger_counter_ - 1, kConeAlcCountLowerThre);
    is_cone_lane_change_situation_ = false;
    is_cone_must_lane_change_situation_ = false;
    return;
  } else {
    // 检测类型为cone的障碍物赋予cluster属性
    DbScan(cone_points_, eps_s, eps_l, minPts);
  }

  cone_cluster_size_.clear();
  cone_cluster_.clear();
  bool did_break = false;
  for (const auto& p : cone_points_) {
    // 构建相同cluster属性所包含cones的map
    cone_cluster_attribute_set_[p.cluster].push_back(p);
  }

  double lane_width =
      QueryLaneMinWidth(cone_points_, origin_lane_s_width_, ego_frenet_point.x);
  double pass_threshold_left = vehicle_param.width + k_default_ego_pass_buffer;
  double pass_threshold_right = vehicle_param.width + k_default_ego_pass_buffer;
  pass_threshold_left = std::max(pass_threshold_left,
                                 lane_width + k_left_cone_occ_lane_line_buffer);
  pass_threshold_right = std::max(
      pass_threshold_right, lane_width + k_right_cone_occ_lane_line_buffer);
  double all_cone_cluster_min_lateral_distance = std::numeric_limits<double>::max();
  for (const auto& cluster_attribute_iter : cone_cluster_attribute_set_) {
    int cluster = cluster_attribute_iter.first;
    const std::vector<ConePoint>& points = cluster_attribute_iter.second;
    double min_left_l, min_right_l;
    double total_l = 0.0;
    min_left_l = CalcClusterToBoundaryDist(points, LEFT_CHANGE);
    min_right_l = CalcClusterToBoundaryDist(points, RIGHT_CHANGE);
    // double max_l = std::max(min_left_l, min_right_l);
    // if (max_l < all_cone_cluster_min_distance) {
    //   all_cone_cluster_min_distance = max_l;
    // }
    // 过滤横向远离车道中心线的锥桶簇
    double min_l_to_center_line = 10.0;
    if (points.size() <= 0) {
      continue;
    }
    for (const auto &p : points) {
      min_l_to_center_line = std::min(std::abs(p.l), min_l_to_center_line);
      total_l += p.l;
    }
    if (min_l_to_center_line > kConeLaneChangelateralDistancethre) {
      continue;
    }
    if (min_l_to_center_line < all_cone_cluster_min_lateral_distance) {
      all_cone_cluster_min_lateral_distance = min_l_to_center_line;
    }
    double average_l = total_l / points.size();
    // if (left_lane_nums_ == 0 && right_lane_nums_ == 0) {
    //   pass_threshold_left =
    //       vehicle_param.width + kLatPassThre + kLatPassThreBuffer;
    //   pass_threshold_right =
    //       vehicle_param.width + kLatPassThre + kLatPassThreBuffer;
    // } else if (left_lane_nums_ == 0) {
    //   pass_threshold_left =
    //       vehicle_param.width + kLatPassThre + kLatPassThreBuffer;
    //   pass_threshold_right = vehicle_param.width + kLatPassThre;
    // } else if (right_lane_nums_ == 0) {
    //   pass_threshold_left = vehicle_param.width + kLatPassThre;
    //   pass_threshold_right =
    //       vehicle_param.width + kLatPassThre + kLatPassThreBuffer;
    // } else {
    //   pass_threshold_left = vehicle_param.width + kLatPassThre;
    //   pass_threshold_right = vehicle_param.width + kLatPassThre;
    // }

    ILOG_DEBUG << "min_left_l is:" << min_left_l
               << ", min_right_l is: is:" << min_right_l
               << ", pass_threshold_left is:" << pass_threshold_left
               << ", pass_threshold_right is:" << pass_threshold_right;

    // judge if to trigger cone lc
    if ((min_left_l < pass_threshold_left &&
         min_right_l < pass_threshold_right) ||
        (!llane && min_right_l < pass_threshold_right && points.size() >= 5 && average_l > 0.0) ||
        (!rlane && min_left_l < pass_threshold_left && points.size() >= 5 && average_l < 0.0)) {
      cone_alc_trigger_counter_++;
      ILOG_DEBUG << "trigger_counter is " << cone_alc_trigger_counter_
                 << ", cluster is " << cluster;
      if (cone_alc_trigger_counter_ >= kConeAlcMaxCountThre) {
        is_cone_must_lane_change_situation_ = true;
        return;
      }
      if (cone_alc_trigger_counter_ >= kConeAlcCountThre) {
        is_cone_lane_change_situation_ = true;
        return;
      }
      did_break = true;
      break;
    }
  }
  if (all_cone_cluster_min_lateral_distance > kConeLaneChangelateralDistancethre) {
    is_cone_must_lane_change_situation_ = false;
  }
  // if all clusters is far away from cernter line, counter--
  if (!did_break) {
    cone_alc_trigger_counter_ =
        std::max(cone_alc_trigger_counter_ - 1, kConeAlcCountLowerThre);
    ILOG_DEBUG << "trigger_counter is " << cone_alc_trigger_counter_;
    if (cone_alc_trigger_counter_ < kConeAlcHystereticCount) {
      is_cone_lane_change_situation_ = false;
    }
  }
  // if all cone l is larger than threshold, then no need to lane change
  return;
}

void ConeRequest::setLaneChangeRequestByCone() {
  const int current_lane_virtual_id =
      virtual_lane_mgr_->current_lane_virtual_id();
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  int olane_virtual_id = lane_change_decider_output.origin_lane_virtual_id;
  const auto& llane = virtual_lane_mgr_->get_left_lane();
  const auto& rlane = virtual_lane_mgr_->get_right_lane();
  std::shared_ptr<ReferencePathManager> reference_path_mgr =
      session_->mutable_environmental_model()->get_reference_path_manager();
  auto olane = virtual_lane_mgr_->get_lane_with_virtual_id(olane_virtual_id);

  if (lane_change_lane_mgr_->has_origin_lane()) {
    auto origin_lane = lane_change_lane_mgr_->olane();
    origin_lane_virtual_id_ = origin_lane->get_virtual_id();
  } else {
    origin_lane_virtual_id_ = current_lane_virtual_id;
  }
  int target_lane_virtual_id_tmp{origin_lane_virtual_id_};

  if (llane != nullptr) {
    left_reference_path_ = reference_path_mgr->get_reference_path_by_lane(
        llane->get_virtual_id(), false);
    ILOG_DEBUG << "ConeRequest::Update: for left_lane: update "
               << llane->get_virtual_id();
  } else {
    left_reference_path_ = nullptr;
  }

  if (rlane != nullptr) {
    right_reference_path_ = reference_path_mgr->get_reference_path_by_lane(
        rlane->get_virtual_id(), false);
    ILOG_DEBUG << "ConeRequest::Update: for right_lane: update "
               << rlane->get_virtual_id();
  } else {
    right_reference_path_ = nullptr;
  }
  bool enable_left = llane && left_reference_path_ &&
                     llane->get_lane_type() != iflyauto::LANETYPE_OPPOSITE;
  bool enable_right = rlane && right_reference_path_ &&
                      rlane->get_lane_type() != iflyauto::LANETYPE_OPPOSITE;
  const bool is_left_lane_change_safe =
      enable_left && ComputeLcValid(LEFT_CHANGE);
  const bool is_right_lane_change_safe =
      enable_right && ComputeLcValid(RIGHT_CHANGE);

  if (cone_lane_change_direction_ == LEFT_CHANGE) {
    if (request_type_ != LEFT_CHANGE && enable_left &&
        !IsRoadBorderSurpressDuringLaneChange(
            LEFT_CHANGE, origin_lane_virtual_id_, llane->get_virtual_id())) {
      target_lane_virtual_id_tmp = origin_lane_virtual_id_ - 1;
      GenerateRequest(LEFT_CHANGE);
      set_target_lane_virtual_id(target_lane_virtual_id_tmp);
      ILOG_DEBUG << "[ConeRequest::update] Ask for cone changing lane to left";
    }
  } else if (cone_lane_change_direction_ == RIGHT_CHANGE) {
    if (request_type_ != RIGHT_CHANGE && enable_right &&
        !IsRoadBorderSurpressDuringLaneChange(
            RIGHT_CHANGE, origin_lane_virtual_id_, rlane->get_virtual_id())) {
      target_lane_virtual_id_tmp = origin_lane_virtual_id_ + 1;
      GenerateRequest(RIGHT_CHANGE);
      set_target_lane_virtual_id(target_lane_virtual_id_tmp);
      ILOG_DEBUG << "[ConeRequest::update] Ask for cone changing lane to left";
    }
  } else if (cone_lane_change_direction_ == NO_CHANGE &&
             request_type_ != NO_CHANGE &&
             (lane_change_lane_mgr_->has_origin_lane() &&
              lane_change_lane_mgr_->is_ego_on(olane))) {
    Finish();
    set_target_lane_virtual_id(target_lane_virtual_id_tmp);
    ILOG_DEBUG << "[ConeRequest::update] " << __FUNCTION__ << ":" << __LINE__
               << "finish request, cone_lane_change_direction == NO_CHANGE";
  } else if (cone_lane_change_direction_ == NO_CHANGE) {
    // do nothing
    return;
  }
}

void ConeRequest::GetTargetLaneWidthByCone(
    const std::vector<std::pair<double, double>> lane_s_width,
    const std::shared_ptr<VirtualLane> base_lane, const double cone_s,
    const double cone_l, bool is_left, double* dist) {
  double target_lane_width = kDefaultLaneWidth;

  if (!lane_s_width.empty()) {
    target_lane_width = QueryLaneWidth(cone_s, lane_s_width);
  }

  if (is_left) {
    double left_width = 0.5 * target_lane_width;
    if (cone_l > left_width) {
      *dist = kDefaultLaneWidth;
    } else {
      *dist = left_width - cone_l;
    }
  } else {
    double right_width = 0.5 * target_lane_width;
    if (cone_l < -right_width) {
      *dist = kDefaultLaneWidth;
    } else {
      *dist = right_width + cone_l;
    }
  }
}

bool ConeRequest::GetOriginLaneWidthByCone(
    const std::shared_ptr<VirtualLane> base_lane, const double cone_s,
    const double cone_l, bool is_left, double* dist) {
  if (base_lane == nullptr) {
    return false;
  }

  std::shared_ptr<ReferencePath> origin_refline =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->get_reference_path_by_lane(base_lane->get_virtual_id(), false);

  double origin_lane_width = kDefaultLaneWidth;
  if (origin_refline != nullptr) {
    origin_lane_s_width_.clear();
    origin_lane_s_width_.reserve(origin_refline->get_points().size());
    for (auto i = 0; i < origin_refline->get_points().size(); i++) {
      const ReferencePathPoint& ref_path_point =
          origin_refline->get_points()[i];
      origin_lane_s_width_.emplace_back(std::make_pair(
          ref_path_point.path_point.s(), ref_path_point.lane_width));
    }
    origin_lane_width = QueryLaneWidth(cone_s, origin_lane_s_width_);
  }

  if (is_left) {
    double left_width = 0.5 * origin_lane_width;
    if (cone_l > left_width) {
      *dist = kDefaultLaneWidth;
    } else {
      *dist = left_width - cone_l;
    }
  } else {
    double right_width = 0.5 * origin_lane_width;
    if (cone_l < -right_width) {
      *dist = kDefaultLaneWidth;
    } else {
      *dist = right_width + cone_l;
    }
  }
  return true;
}

void ConeRequest::ConeDir() {
  const auto& route_info_output =
      session_->environmental_model().get_route_info()->get_route_info_output();
  const auto& rlane = virtual_lane_mgr_->get_right_lane();
  const auto& llane = virtual_lane_mgr_->get_left_lane();
  const auto& function_info = session_->environmental_model().function_info();
  const auto& merge_point_info = route_info_output.merge_point_info;
  double distance_to_first_road_split = NL_NMAX;
  double dis_to_first_merge = NL_NMAX;
  double dis_to_merge_point = NL_NMAX;
  if (function_info.function_mode() == common::DrivingFunctionInfo::NOA) {
    dis_to_merge_point = merge_point_info.dis_to_merge_fp;
    const auto& split_region_info_list =
        route_info_output.split_region_info_list;
    const auto& merge_region_info_list =
        route_info_output.merge_region_info_list;
    if (!split_region_info_list.empty()) {
      if (split_region_info_list[0].is_valid) {
        distance_to_first_road_split =
            split_region_info_list[0].distance_to_split_point;
      }
    }
    if (!merge_region_info_list.empty()) {
      if (merge_region_info_list[0].is_valid) {
        dis_to_first_merge = merge_region_info_list[0].distance_to_split_point;
      }
    }
  }
  const auto base_lane =
      virtual_lane_mgr_->get_lane_with_virtual_id(origin_lane_virtual_id_);
  const std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes =
      virtual_lane_mgr_->get_virtual_lanes();
  cone_lane_change_direction_ = NO_CHANGE;
  int current_left_boundary_type = 0;
  int current_right_boundary_type = 0;
  const auto& feasible_lane_sequence =
      route_info_output.mlc_decider_route_info.feasible_lane_sequence;
  const double dis_ego_to_last_split_point = route_info_output.accumulate_dis_ego_to_last_split_point;
  bool left_lane_is_on_navigation_route = true;
  bool right_lane_is_on_navigation_route = true;
  if (distance_to_first_road_split < 300.0 || dis_to_merge_point < 200.0 || dis_ego_to_last_split_point < 100.0) {
    if (feasible_lane_sequence.size() > 0) {
      int current_lane_order_num = left_lane_nums_ + 1;
      int target_lane_order_num = current_lane_order_num - 1;
      if (std::find(feasible_lane_sequence.begin(),
                    feasible_lane_sequence.end(),
                    target_lane_order_num) == feasible_lane_sequence.end()) {
        left_lane_is_on_navigation_route = false;
      }
    }

    if (feasible_lane_sequence.size() > 0) {
      int current_lane_order_num = left_lane_nums_ + 1;
      int target_lane_order_num = current_lane_order_num + 1;
      if (std::find(feasible_lane_sequence.begin(),
                    feasible_lane_sequence.end(),
                    target_lane_order_num) == feasible_lane_sequence.end()) {
        right_lane_is_on_navigation_route = false;
      }
    }
  }
  // 获取左车道线型
  if (base_lane != nullptr) {
    auto left_boundary_type =
        MakesureCurrentBoundaryType(LEFT_CHANGE, origin_lane_virtual_id_);
    current_left_boundary_type = left_boundary_type;
    // 获取右车道线型
    auto right_boundary_type =
        MakesureCurrentBoundaryType(RIGHT_CHANGE, origin_lane_virtual_id_);
    current_right_boundary_type = right_boundary_type;
  } else {
    return;
  }

  bool left_change_available = false;
  bool right_change_available = false;

  RequestType cone_dir;
  bool cone_distribution_enable_left = true;
  bool cone_distribution_enable_right = true;

  // right seach
  if (CheckEgoLaneAvailable(false)) {
    if (rlane != nullptr && rlane->width() > kMinDefaultLaneWidth) {
      std::shared_ptr<ReferencePath> right_refline =
          session_->mutable_environmental_model()
              ->get_reference_path_manager()
              ->get_reference_path_by_lane(rlane->get_virtual_id(), false);
      right_lane_s_width_.clear();
      if (right_refline != nullptr) {
        right_lane_s_width_.reserve(right_refline->get_points().size());
        for (auto i = 0; i < right_refline->get_points().size(); i++) {
          const ReferencePathPoint& ref_path_point =
              right_refline->get_points()[i];
          right_lane_s_width_.emplace_back(std::make_pair(
              ref_path_point.path_point.s(), ref_path_point.lane_width));
        }
        cone_distribution_enable_right = EnableTargetLane(false, rlane);
      }

      if (CheckTargetLaneAvailable(false, rlane) &&
          cone_distribution_enable_right) {
        right_change_available = true;
        ILOG_DEBUG << "right_change_available:" << right_change_available;
      }
    }
  }
  // left seach
  if (CheckEgoLaneAvailable(true)) {
    if (llane != nullptr && llane->width() > kMinDefaultLaneWidth) {
      std::shared_ptr<ReferencePath> left_refline =
          session_->mutable_environmental_model()
              ->get_reference_path_manager()
              ->get_reference_path_by_lane(llane->get_virtual_id(), false);
      left_lane_s_width_.clear();
      if (left_refline != nullptr) {
        left_lane_s_width_.reserve(left_refline->get_points().size());
        for (auto i = 0; i < left_refline->get_points().size(); i++) {
          const ReferencePathPoint& ref_path_point =
              left_refline->get_points()[i];
          left_lane_s_width_.emplace_back(std::make_pair(
              ref_path_point.path_point.s(), ref_path_point.lane_width));
        }
        cone_distribution_enable_left = EnableTargetLane(true, llane);
      }

      if (CheckTargetLaneAvailable(true, llane) &&
          cone_distribution_enable_left) {
        left_change_available = true;
        ILOG_DEBUG << "left_change_available: " << left_change_available;
      }
    }
  }

  // scc优先利用锥桶分布判断变道方向
  if (function_info.function_mode() == common::DrivingFunctionInfo::SCC &&
      ConesDirection(cone_dir)) {
    if (cone_dir == LEFT_CHANGE && llane && cone_distribution_enable_left) {
      cone_lane_change_direction_ = LEFT_CHANGE;
      return;
    }
    if (cone_dir == RIGHT_CHANGE && rlane && cone_distribution_enable_right) {
      cone_lane_change_direction_ = RIGHT_CHANGE;
      return;
    }
  }

  if (left_change_available && left_lane_is_on_navigation_route) {
    ILOG_DEBUG << "cone alc left!!!";
    cone_lane_change_direction_ = LEFT_CHANGE;
    return;
  } else if (right_change_available && right_lane_is_on_navigation_route) {
    ILOG_DEBUG << "cone alc right!!!";
    cone_lane_change_direction_ = RIGHT_CHANGE;
    return;
  } else {
    // default
    cone_lane_change_direction_ = NO_CHANGE;
    return;
  }

  // if (left_change_available && right_change_available) {
  //   ILOG_DEBUG << "cone alc right!!!";
  //   cone_lane_change_direction_ = RIGHT_CHANGE;
  //   return;
  // } else if (left_change_available) {
  //   ILOG_DEBUG << "cone alc left!!!";
  //   cone_lane_change_direction_ = LEFT_CHANGE;
  //   return;
  // } else if (right_change_available) {
  //   ILOG_DEBUG << "cone alc right!!!";
  //   cone_lane_change_direction_ = RIGHT_CHANGE;
  //   return;
  // } else {
  //   // default
  //   cone_lane_change_direction_ = NO_CHANGE;
  //   return;
  // }
}

bool ConeRequest::ConesDirection(RequestType& direction) {
  direction = NO_CHANGE;
  for (const auto& cluster_attribute_iter : cone_cluster_attribute_set_) {
    std::vector<ConePoint> points = cluster_attribute_iter.second;
    if (points.size() < kConeDirecSize) {
      continue;
    }
    double rank_correlation = ConeSpearmanRankCorrelation(points);
    double cones_slope = ConeComputeSlope(points);
    ILOG_DEBUG << "[ConesDirection] rank_correlation is " << rank_correlation
               << ", [ConesDirection] cones_slope is " << cones_slope;

    // rank_correlation接近1，表示数据之间有很强的正相关
    // 结合线性回归求得的斜率，判断导流方向
    if (rank_correlation > kConeDirecThre &&
        std::abs(cones_slope) < kConeSlopeThre) {
      direction = LEFT_CHANGE;
      return true;
    } else if (rank_correlation < -kConeDirecThre &&
               std::abs(cones_slope) < kConeSlopeThre) {
      direction = RIGHT_CHANGE;
      return true;
    } else {
      // do nothing
    }
  }
  // if all clusters have no direction
  return false;
}

bool ConeRequest::CheckEgoLaneAvailable(bool is_left) {
  const auto& ego_lane =
      virtual_lane_mgr_->get_lane_with_virtual_id(origin_lane_virtual_id_);
  if (ego_lane == nullptr) {
    ILOG_DEBUG << "seach fail: ego lane is nullptr";
    return false;
  }

  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& ego_vel = ego_state->ego_v();

  double left_width, right_width;
  Point2D ego_frenet_point;
  Point2D ego_cart_point{planning_init_point_.lat_init_state.x(),
                         planning_init_point_.lat_init_state.y()};
  if (!base_frenet_coord_->XYToSL(ego_cart_point, ego_frenet_point)) {
    ILOG_DEBUG
        << "[CheckEgoLaneAvailable] fail to get ego position on base lane";
    return false;
  }

  // left_width = 0.5 * ego_lane->width_by_s(ego_frenet_point.x);
  // right_width = 0.5 * ego_lane->width_by_s(ego_frenet_point.x);

  if (is_left) {
    for (auto& p : cone_points_) {
      left_width = QueryLaneWidth(p.s, origin_lane_s_width_) * 0.5;
      double ego_distance_to_cone = p.s - ego_frenet_point.x;
      if (ego_distance_to_cone < ego_vel * kLongClusterTimeGap / 2 && p.l > 0 &&
          p.l < left_width) {
        ILOG_DEBUG << "left front cone is dangerous for lane change";
        // return false;
      }
    }
  } else {
    for (auto& p : cone_points_) {
      right_width = QueryLaneWidth(p.s, origin_lane_s_width_) * 0.5;
      double ego_distance_to_cone = p.s - ego_frenet_point.x;
      if (ego_distance_to_cone < ego_vel * kLongClusterTimeGap / 2 && p.l < 0 &&
          p.l > -right_width) {
        ILOG_DEBUG << "right front cone is dangerous for lane change";
        // return false;
      }
    }
  }
  return true;
}

bool ConeRequest::CheckTargetLaneAvailable(
    bool is_left, const std::shared_ptr<VirtualLane> seach_lane) {
  if (seach_lane == nullptr) {
    ILOG_DEBUG << "seach fail: seach lane is nullptr";
    return false;
  }
  std::vector<std::pair<double, double>> lane_s_width;
  if (is_left) {
    lane_s_width = left_lane_s_width_;
  } else {
    lane_s_width = right_lane_s_width_;
  }
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  double pass_thre = vehicle_param.width + kLatPassThre;

  std::shared_ptr<ReferencePath> target_refline =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->get_reference_path_by_lane(seach_lane->get_virtual_id(), false);

  std::shared_ptr<planning_math::KDPath> target_lane_frenet_coord =
      target_refline->get_frenet_coord();

  std::vector<ConePoint> serach_cone_points = cone_points_;
  for (auto& p : serach_cone_points) {
    if (!target_lane_frenet_coord->XYToSL(p.x, p.y, &p.s, &p.l)) {
      ILOG_DEBUG << "[CheckTargetLaneAvailable]: XYToSL fail";
      continue;
    } else {
      GetTargetLaneWidthByCone(lane_s_width, seach_lane, p.s, p.l, true,
                               &p.left_dist);
      GetTargetLaneWidthByCone(lane_s_width, seach_lane, p.s, p.l, false,
                               &p.right_dist);
    }
  }

  double max_l = CalcClusterToBoundaryDist(serach_cone_points, NO_CHANGE);
  // std::cout << "max_l: " << max_l << " pass_thre: " << pass_thre <<
  // std::endl; judge if to trigger cone lc
  if (max_l <= pass_thre) {
    ILOG_DEBUG << " target lane is blocked";
  }

  return max_l > pass_thre;
}

bool ConeRequest::EnableTargetLane(
    bool is_left, const std::shared_ptr<VirtualLane> seach_lane) {
  const auto& ego_lane_road_right_decider_output =
      session_->planning_context().ego_lane_road_right_decider_output();
  const bool is_merge_region =
      ego_lane_road_right_decider_output.is_merge_region;
  const double lane_occ_proportion = 0.65;
  if (seach_lane == nullptr) {
    ILOG_DEBUG << "seach fail: seach lane is nullptr";
    return false;
  }
  if (cone_cluster_attribute_set_.empty()) {
    return false;
  }
  std::vector<std::pair<double, double>> lane_s_width;
  if (is_left) {
    lane_s_width = left_lane_s_width_;
  } else {
    lane_s_width = right_lane_s_width_;
  }

  if (lane_s_width.empty()) {
    return false;
  }
  std::shared_ptr<ReferencePath> target_refline =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->get_reference_path_by_lane(seach_lane->get_virtual_id(), false);

  std::shared_ptr<planning_math::KDPath> target_lane_frenet_coord =
      target_refline->get_frenet_coord();
  for (const auto& cluster_attribute_iter : cone_cluster_attribute_set_) {
    int cluster = cluster_attribute_iter.first;
    std::vector<ConePoint> serach_cone_points = cluster_attribute_iter.second;
    double total_lane_width = 0.0;
    double total_cone_l = 0.0;
    double average_l = 0.0;
    int cone_num = 0;
    double total_cone_l_origin = 0.0;
    double average_l_origin = 0.0;
    for (auto& p : serach_cone_points) {
      Point2D point(p.x, p.y);
      Point2D frenet_point;
      if (!target_lane_frenet_coord->XYToSL(point, frenet_point)) {
        continue;
      } else {
        total_lane_width += QueryLaneWidth(frenet_point.x, lane_s_width);
        total_cone_l += std::fabs(frenet_point.y);
        total_cone_l_origin += std::fabs(p.l);
        cone_num++;
      }
    }
    double average_lane_width = total_lane_width / std::max(cone_num, 1);
    double average_cone_l = total_cone_l / std::max(cone_num, 1);
    average_l_origin = total_cone_l_origin / std::max(cone_num, 1);
    if ((average_cone_l < average_lane_width * lane_occ_proportion ||
         std::fabs(seach_lane->get_ego_lateral_offset()) > average_cone_l ||
         (average_cone_l - average_l_origin < average_lane_width * 0.5 &&
          !is_merge_region)) &&
        cone_num >= 5) {
      return false;
    }
  }

  return true;
}

// 用于获取元素的秩次
std::vector<double> ConeRequest::ConeRankify(std::vector<double>& arr) {
  int n = arr.size();
  std::vector<double> ranks(n);
  std::iota(ranks.begin(), ranks.end(), 0);

  // 根据arr的值给ranks排序
  std::sort(ranks.begin(), ranks.end(),
            [&](int i, int j) { return arr[i] < arr[j]; });

  for (int i = 0; i < n; ++i) {
    arr[ranks[i]] = i;
  }

  return arr;
}

double ConeRequest::ConeSpearmanRankCorrelation(
    const std::vector<ConePoint> points) {
  int n = points.size();

  std::vector<double> s(n);
  std::vector<double> l(n);
  for (int i = 0; i < n; ++i) {
    s[i] = points[i].s;
    l[i] = points[i].l;
  }

  // 计算元素的秩次向量
  std::vector<double> s_rank = ConeRankify(s);
  std::vector<double> l_rank = ConeRankify(l);

  // 计算秩次之差的平方和
  double d_square_sum = 0.0;
  for (int i = 0; i < n; ++i) {
    d_square_sum += std::pow(s_rank[i] - l_rank[i], 2);
  }

  // 计算斯皮尔曼秩相关系数
  return 1 - (6 * d_square_sum) / (n * (std::pow(n, 2) - 1));
}

double ConeRequest::ConeComputeSlope(std::vector<ConePoint> points) {
  double s_mean, l_mean;
  double s_stddev, l_stddev;
  // 计算cone的s、l的平均值
  if (ConeMean(points, s_mean, l_mean)) {
    // 计算cone的s、l的标准差
    if (ConeStddev(points, s_mean, l_mean, s_stddev, l_stddev)) {
      if (s_stddev == 0) {
        return std::numeric_limits<double>::max();
      } else if (l_stddev == 0) {
        return 0.0;
      }
    }
  }

  // 数据标准化
  if (ConeStandardize(points)) {
    // std::cout << "s_mean" << s_mean << "l_mean" << l_mean << std::endl;
    double numerator = 0.0, denominator = 0.0;
    s_mean = 0.0;
    l_mean = 0.0;
    ConeMean(points, s_mean, l_mean);

    for (const auto& point : points) {
      numerator += (point.s - s_mean) * (point.l - l_mean);
      denominator += std::pow(point.s - s_mean, 2);
      // std::cout << "numerator" << numerator << "denominator" << denominator
      // << std::endl;
    }
    denominator = std::max(denominator, 0.001);
    // 求协方差矩阵的特征值 （反映数据集s、l之间的关系强度）
    return numerator / denominator;
  } else {
    return std::numeric_limits<double>::max();
  }
}

bool ConeRequest::ConeStandardize(std::vector<ConePoint>& points) {
  double s_mean, l_mean;
  // 计算锥桶障碍物的s、l均值
  if (ConeMean(points, s_mean, l_mean)) {
    double s_stddev, l_stddev;
    // 计算s、l的标准差
    if (ConeStddev(points, s_mean, l_mean, s_stddev, l_stddev)) {
      if (s_stddev != 0 && l_stddev != 0) {
        for (auto& point : points) {
          point.s = (point.s - s_mean) / s_stddev;
          point.l = (point.l - l_mean) / l_stddev;
        }
        return true;
      }
    }
  }
  return false;
}

bool ConeRequest::ConeMean(const std::vector<ConePoint>& points, double& s_mean,
                           double& l_mean) {
  if (points.empty()) {
    return false;
  }
  for (const auto& point : points) {
    s_mean += point.s;
    l_mean += point.l;
  }
  s_mean /= points.size();
  l_mean /= points.size();
  return true;
}

bool ConeRequest::ConeStddev(const std::vector<ConePoint>& points,
                             double s_mean, double l_mean, double& s_stddev,
                             double& l_stddev) {
  if (points.empty()) {
    return false;
  }
  double sum_s = 0.0;
  double sum_l = 0.0;
  for (const auto& point : points) {
    sum_s += (point.s - s_mean) * (point.s - s_mean);
    sum_l += (point.l - l_mean) * (point.l - l_mean);
  }

  s_stddev = std::sqrt(sum_s / (points.size() - 1));
  l_stddev = std::sqrt(sum_l / (points.size() - 1));
  return true;
}

void ConeRequest::Reset() {
  cone_alc_trigger_counter_ = 0;
  is_cone_lane_change_situation_ = false;
  is_cone_must_lane_change_situation_ = false;
  cone_lane_change_direction_ = NO_CHANGE;
  cone_cluster_size_.clear();
  cone_cluster_.clear();
  cone_points_.clear();
  cone_cluster_attribute_set_.clear();
  right_lane_nums_ = 0;
  left_lane_nums_ = 0;
}

}  // namespace planning
#include "cone_lane_change_request.h"

#include <glog/logging.h>
#include <math.h>

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
#include "log.h"
#include "planning_context.h"
#include "tasks/behavior_planners/lane_change_decider/lane_change_requests/lane_change_request.h"
#include "tasks/behavior_planners/lane_change_decider/lateral_behavior_object_selector.h"
#include "tracked_object.h"
#include "virtual_lane_manager.h"

namespace planning {

namespace {
constexpr double kLongClusterCoeff = 2.6;
constexpr double kLatClusterThre = 0.6;
constexpr double kLatPassThre = 0.5;
constexpr double kLatPassThreBuffer = 0.2;
constexpr uint32_t kConeAlcCountThre = 3;
constexpr int kConeAlcCountLowerThre = 0;
constexpr double kLongClusterTimeGap = 5.0;
constexpr double kDefaultLaneWidth = 3.75;
constexpr double kMinDefaultLaneWidth = 3.0;
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
  base_frenet_coord_ = std::make_shared<KDPath>();
}

void ConeRequest::Update(int lc_status) {
  std::cout << "ConeRequest::Update::coming cone lane change request"
            << std::endl;

  // trigger EA lane change when lane keep status.
  if (lc_status != kLaneKeeping && lc_status != kLaneChangePropose) {
    LOG_DEBUG("ConeRequest::Update: ego not in lane keeping!");
    return;
  }
  lateral_obstacle_ = session_->environmental_model().get_lateral_obstacle();
  lane_tracks_manager_ =
      session_->environmental_model().get_lane_tracks_manager();
  const auto& tracks = lateral_obstacle_->all_tracks();
  tracks_map_.clear();
  for (auto track : tracks) {
    tracks_map_[track.track_id] = track;
  }

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
  is_cone_lane_change_situation_ = false;

  UpdateConeSituation(lc_status);
  LOG_DEBUG("ConeRequest::Update: is_cone_lane_change_situation %d",
            is_cone_lane_change_situation_);
  JSON_DEBUG_VALUE("is_cone_lane_change_situation_",
                   is_cone_lane_change_situation_);

  if (!is_cone_lane_change_situation_) {
    if (request_type_ != NO_CHANGE &&
        (lane_change_lane_mgr_->has_origin_lane() &&
         lane_change_lane_mgr_->is_ego_on(olane))) {
      Finish();
      Reset();
      set_target_lane_virtual_id(current_lane_virtual_id);
      LOG_DEBUG(
          "[ConeRequest::update] %s:%d finish request, "
          "!trigger_left_clc and !trigger_right_clc\n",
          __FUNCTION__, __LINE__);
    }
    return;
  }
  ConeDir();
  JSON_DEBUG_VALUE("cone_lane_change_direction_",
                   (int)cone_lane_change_direction_);

  setLaneChangeRequestByCone();
  LOG_DEBUG("request_type_: [%d] turn_signal_: [%d]\n", request_type_,
            turn_signal_);
}

void ConeRequest::UpdateConeSituation(int lc_status) {
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  double ego_fx = std::cos(ego_state->ego_pose_raw().theta);
  double ego_fy = std::sin(ego_state->ego_pose_raw().theta);

  const auto base_lane =
      virtual_lane_mgr_->get_lane_with_virtual_id(origin_lane_virtual_id_);
  if (base_lane == nullptr) {
    LOG_DEBUG("base lane not exist");
    is_cone_lane_change_situation_ = false;
    return;
  }
  JSON_DEBUG_VALUE("cone_alc_trigger_counter_", cone_alc_trigger_counter_);
  std::shared_ptr<ReferencePath> origin_refline =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->get_reference_path_by_lane(origin_lane_virtual_id_, false);

  base_frenet_coord_ = origin_refline->get_frenet_coord();
  Point2D ego_frenet_point;
  Point2D ego_cart_point{planning_init_point_.lat_init_state.x(),
                         planning_init_point_.lat_init_state.y()};
  if (!base_frenet_coord_->XYToSL(ego_cart_point, ego_frenet_point)) {
    LOG_DEBUG("fail to get ego position on base lane");
    is_cone_lane_change_situation_ = false;
    return;
  }
  const double ego_front_edge = vehicle_param.front_edge_to_rear_axle;
  const double ego_rear_edge = vehicle_param.rear_edge_to_rear_axle;
  double eps_s = vehicle_param.length * kLongClusterCoeff;
  double eps_l = vehicle_param.width + kLatClusterThre;
  const double ego_vel = ego_state->ego_v();
  int cone_nums_of_front_objects = 0;
  int minPts = 1;
  cone_points_.clear();
  cone_cluster_attribute_set_.clear();
  is_cone_lane_change_situation_ = false;

  const std::vector<TrackedObject>& front_obstacles_array =
      lateral_obstacle_->front_tracks();
  for (const auto& front_obstacle : front_obstacles_array) {
    auto front_vehicle_iter = tracks_map_.find(front_obstacle.track_id);
    if (front_vehicle_iter != tracks_map_.end()) {
      if (front_vehicle_iter->second.track_id == kInvalidAgentId) {
        continue;
      }
      if (front_vehicle_iter->second.type ==
          Common::ObjectType::OBJECT_TYPE_TRAFFIC_CONE) {
        if (front_vehicle_iter->second.d_rel < -ego_rear_edge ||
            front_vehicle_iter->second.d_rel >
                base_frenet_coord_->Length() - ego_frenet_point.x) {
          continue;
        }
        cone_nums_of_front_objects++;

        Point2D obs_cart_point{0.0, 0.0};
        Point2D obs_frenet_point;
        obs_cart_point.x = ego_cart_point.x +
                           front_vehicle_iter->second.center_x * ego_fx -
                           front_vehicle_iter->second.center_y * ego_fy;
        obs_cart_point.y = ego_cart_point.y +
                           front_vehicle_iter->second.center_x * ego_fy +
                           front_vehicle_iter->second.center_y * ego_fx;
        if (!base_frenet_coord_->XYToSL(obs_cart_point, obs_frenet_point)) {
          continue;
        }
        double cone_s = obs_frenet_point.x;
        double cone_l = obs_frenet_point.y;
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
    LOG_DEBUG("no cone found!!!\n");
    cone_alc_trigger_counter_ =
        std::max(cone_alc_trigger_counter_ - 1, kConeAlcCountLowerThre);
    is_cone_lane_change_situation_ = false;
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

  const std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes =
      virtual_lane_mgr_->get_virtual_lanes();
  int lane_nums = relative_id_lanes.size();
  int lane_index = base_lane->get_order_id();
  int right_lane_nums = std::max((int)lane_nums - lane_index - 1, 0);
  int left_lane_nums = lane_index;

  for (const auto& cluster_attribute_iter : cone_cluster_attribute_set_) {
    int cluster = cluster_attribute_iter.first;
    const std::vector<ConePoint>& points = cluster_attribute_iter.second;
    double min_left_l, min_right_l, pass_threshold_left, pass_threshold_right;
    min_left_l = CalcClusterToBoundaryDist(points, LEFT_CHANGE);
    min_right_l = CalcClusterToBoundaryDist(points, RIGHT_CHANGE);

    if (left_lane_nums == 0 && right_lane_nums == 0) {
      pass_threshold_left =
          vehicle_param.width + kLatPassThre + kLatPassThreBuffer;
      pass_threshold_right =
          vehicle_param.width + kLatPassThre + kLatPassThreBuffer;
    } else if (left_lane_nums == 0) {
      pass_threshold_left =
          vehicle_param.width + kLatPassThre + kLatPassThreBuffer;
      pass_threshold_right = vehicle_param.width + kLatPassThre;
    } else if (right_lane_nums == 0) {
      pass_threshold_left = vehicle_param.width + kLatPassThre;
      pass_threshold_right =
          vehicle_param.width + kLatPassThre + kLatPassThreBuffer;
    } else {
      pass_threshold_left = vehicle_param.width + kLatPassThre;
      pass_threshold_right = vehicle_param.width + kLatPassThre;
    }

    LOG_DEBUG(
        "min_left_l is: %f, min_right_l is: %f "
        "pass_threshold_left is: %f, pass_threshold_right is: %f "
        "\n",
        min_left_l, min_right_l, pass_threshold_left, pass_threshold_right);

    // judge if to trigger cone lc
    if (min_left_l < pass_threshold_left &&
        min_right_l < pass_threshold_right) {
      cone_alc_trigger_counter_++;
      LOG_DEBUG("trigger_counter is %d, cluster is %d \n",
                cone_alc_trigger_counter_, cluster);
      if (cone_alc_trigger_counter_ >= kConeAlcCountThre) {
        is_cone_lane_change_situation_ = true;
        return;
      }
      did_break = true;
      break;
    }
  }
  // if all clusters is far away from cernter line, counter--
  if (!did_break) {
    cone_alc_trigger_counter_ =
        std::max(cone_alc_trigger_counter_ - 1, kConeAlcCountLowerThre);
    LOG_DEBUG("trigger_counter is %d \n", cone_alc_trigger_counter_);
  }
  // if all cone l is larger than threshold, then no need to lane change
  return;
}

void ConeRequest::setLaneChangeRequestByCone() {
  const int current_lane_virtual_id =
      virtual_lane_mgr_->current_lane_virtual_id();
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  int olane_virtual_id = lane_change_decider_output.origin_lane_virtual_id;
  const auto& clane = virtual_lane_mgr_->get_current_lane();
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
    LOG_DEBUG("ConeRequest::Update: for left_lane: update %d\n",
              llane->get_virtual_id());
  } else {
    left_reference_path_ = nullptr;
  }

  if (rlane != nullptr) {
    right_reference_path_ = reference_path_mgr->get_reference_path_by_lane(
        rlane->get_virtual_id(), false);
    LOG_DEBUG("ConeRequest::Update: for right_lane: update %d\n",
              rlane->get_virtual_id());
  } else {
    right_reference_path_ = nullptr;
  }
  bool enable_left = llane && left_reference_path_;
  bool enable_right = rlane && right_reference_path_;
  const bool is_left_lane_change_safe =
      enable_left && compute_lc_valid_info(LEFT_CHANGE);
  const bool is_right_lane_change_safe =
      enable_right && compute_lc_valid_info(RIGHT_CHANGE);

  if (cone_lane_change_direction_ == LEFT_CHANGE) {
    if (request_type_ != LEFT_CHANGE && is_left_lane_change_safe) {
      target_lane_virtual_id_tmp = origin_lane_virtual_id_ - 1;
      GenerateRequest(LEFT_CHANGE);
      set_target_lane_virtual_id(target_lane_virtual_id_tmp);
      LOG_DEBUG(
          "[ConeRequest::update] Ask for cone changing lane to left "
          "\n");
    }
  } else if (cone_lane_change_direction_ == RIGHT_CHANGE) {
    if (request_type_ != RIGHT_CHANGE && is_right_lane_change_safe) {
      target_lane_virtual_id_tmp = origin_lane_virtual_id_ + 1;
      GenerateRequest(RIGHT_CHANGE);
      set_target_lane_virtual_id(target_lane_virtual_id_tmp);
      LOG_DEBUG(
          "[ConeRequest::update] Ask for cone changing lane to left "
          "\n");
    }
  } else if (cone_lane_change_direction_ == NO_CHANGE &&
             request_type_ != NO_CHANGE &&
             (lane_change_lane_mgr_->has_origin_lane() &&
              lane_change_lane_mgr_->is_ego_on(olane))) {
    Finish();
    set_target_lane_virtual_id(target_lane_virtual_id_tmp);
    LOG_DEBUG(
        "[ConeRequest::update] %s:%d finish request, "
        "cone_lane_change_direction == NO_CHANGE\n",
        __FUNCTION__, __LINE__);
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
          ref_path_point.path_point.s, ref_path_point.lane_width));
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

void ConeRequest::DbScan(std::vector<ConePoint>& cone_points, double eps_s,
                         double eps_l, int minPts) {
  int c = 0;  // cluster index

  for (size_t index = 0; index < cone_points.size(); ++index) {
    if (!cone_points[index].visited) {
      // 根据锥桶间的距离聚类
      ExpandCluster(cone_points, index, c, eps_s, eps_l, minPts);
      c++;
    }
  }
}

bool ConeRequest::ConeDistance(const ConePoint& a, const ConePoint& b,
                               double eps_s, double eps_l) {
  return std::abs(a.s - b.s) < eps_s && std::abs(a.l - b.l) < eps_l;
}

void ConeRequest::ExpandCluster(std::vector<ConePoint>& cone_points, int index,
                                int c, double eps_s, double eps_l, int minPts) {
  std::vector<int> neighborPts;

  for (size_t i = 0; i < cone_points.size(); ++i) {
    if (ConeDistance(cone_points[index], cone_points[i], eps_s, eps_l)) {
      neighborPts.push_back(i);
    }
  }

  if (neighborPts.size() < minPts) {
    // The point is noise
    cone_points[index].cluster = -1;
    return;
  }

  // Assign the cluster to initial point
  cone_points[index].visited = true;
  cone_points[index].cluster = c;

  // Check all neighbours for being part of the cluster
  for (auto& neighborPt : neighborPts) {
    ConePoint& p_neighbor = cone_points[neighborPt];
    if (!p_neighbor.visited) {
      // Recursively expand the cluster
      ExpandCluster(cone_points, neighborPt, c, eps_s, eps_l, minPts);
    }
  }
}

double ConeRequest::CalcClusterToBoundaryDist(
    const std::vector<ConePoint>& points, RequestType direction) {
  double left_l = std::abs(points[0].left_dist);
  double right_l = std::abs(points[0].right_dist);
  for (const auto& p : points) {
    left_l = std::min(std::abs(p.left_dist), left_l);
    right_l = std::min(std::abs(p.right_dist), right_l);
  }
  if (direction == LEFT_CHANGE) {
    return left_l;
  } else if (direction == RIGHT_CHANGE) {
    return right_l;
  } else {
    return std::max(left_l, right_l);
  }
}

void ConeRequest::ConeDir() {
  const auto base_lane =
      virtual_lane_mgr_->get_lane_with_virtual_id(origin_lane_virtual_id_);
  const std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes =
      virtual_lane_mgr_->get_virtual_lanes();
  int lane_nums = relative_id_lanes.size();
  int lane_index = base_lane->get_order_id();
  int right_lane_nums = std::max((int)lane_nums - lane_index - 1, 0);
  int left_lane_nums = lane_index;
  cone_lane_change_direction_ = NO_CHANGE;
  int current_left_boundary_type = 0;
  int current_right_boundary_type = 0;

  // 获取左车道线型
  if (base_lane != nullptr) {
    auto left_lane_boundary = base_lane->get_left_lane_boundary();
    auto left_boundary_type =
        MakesureCurrentBoundaryType(LEFT_CHANGE, origin_lane_virtual_id_);
    current_left_boundary_type = left_boundary_type;
    // 获取右车道线型
    auto right_lane_boundary = base_lane->get_right_lane_boundary();
    auto right_boundary_type =
        MakesureCurrentBoundaryType(RIGHT_CHANGE, origin_lane_virtual_id_);
    current_right_boundary_type = right_boundary_type;
  } else {
    return;
  }

  if (left_lane_nums == 0 && right_lane_nums == 0) {
    return;
  }
  bool left_change_available = false;
  bool right_change_available = false;

  RequestType cone_dir;
  if (ConesDirection(cone_dir)) {
    if (cone_dir == LEFT_CHANGE && left_lane_nums) {
      cone_lane_change_direction_ = LEFT_CHANGE;
      return;
    }
    if (cone_dir == RIGHT_CHANGE && right_lane_nums) {
      cone_lane_change_direction_ = RIGHT_CHANGE;
      return;
    }
  } else {
    // right seach
    if (CheckEgoLaneAvailable(false)) {
      if (right_lane_nums > 0) {
        const auto& rlane = virtual_lane_mgr_->get_right_lane();
        if (rlane != nullptr && rlane->width() > kMinDefaultLaneWidth) {
          std::shared_ptr<ReferencePath> target_refline =
              session_->mutable_environmental_model()
                  ->get_reference_path_manager()
                  ->get_reference_path_by_lane(rlane->get_virtual_id(), false);
          right_lane_s_width_.clear();
          if (target_refline != nullptr) {
            right_lane_s_width_.reserve(target_refline->get_points().size());
            for (auto i = 0; i < target_refline->get_points().size(); i++) {
              const ReferencePathPoint& ref_path_point =
                  target_refline->get_points()[i];
              right_lane_s_width_.emplace_back(std::make_pair(
                  ref_path_point.path_point.s, ref_path_point.lane_width));
            }
          }
          if (CheckTargetLaneAvailable(false, rlane)) {
            right_change_available = true;
            LOG_DEBUG("right_change_available: %d \n", right_change_available);
          }
        }
      }
    }
    // left seach
    if (CheckEgoLaneAvailable(true)) {
      if (left_lane_nums > 0) {
        const auto& llane = virtual_lane_mgr_->get_left_lane();
        if (llane != nullptr && llane->width() > kMinDefaultLaneWidth) {
          std::shared_ptr<ReferencePath> target_refline =
              session_->mutable_environmental_model()
                  ->get_reference_path_manager()
                  ->get_reference_path_by_lane(llane->get_virtual_id(), false);
          left_lane_s_width_.clear();
          if (target_refline != nullptr) {
            left_lane_s_width_.reserve(target_refline->get_points().size());
            for (auto i = 0; i < target_refline->get_points().size(); i++) {
              const ReferencePathPoint& ref_path_point =
                  target_refline->get_points()[i];
              left_lane_s_width_.emplace_back(std::make_pair(
                  ref_path_point.path_point.s, ref_path_point.lane_width));
            }
          }
          if (CheckTargetLaneAvailable(true, llane)) {
            left_change_available = true;
            LOG_DEBUG("left_change_available: %d \n", left_change_available);
          }
        }
      }
    }
  }

  if (left_change_available && right_change_available) {
    if (current_right_boundary_type !=
        iflyauto::LaneBoundaryType_MARKING_SOLID) {
      LOG_DEBUG("cone alc right!!!\n");
      cone_lane_change_direction_ = RIGHT_CHANGE;
      return;
    }
    if (current_left_boundary_type !=
        iflyauto::LaneBoundaryType_MARKING_SOLID) {
      LOG_DEBUG("cone alc left!!!\n");
      cone_lane_change_direction_ = LEFT_CHANGE;
      return;
    }
    LOG_DEBUG("cone alc right!!!\n");
    cone_lane_change_direction_ = RIGHT_CHANGE;
    return;
  } else if (left_change_available) {
    LOG_DEBUG("cone alc left!!!\n");
    cone_lane_change_direction_ = LEFT_CHANGE;
    return;
  } else if (right_change_available) {
    LOG_DEBUG("cone alc right!!!\n");
    cone_lane_change_direction_ = RIGHT_CHANGE;
    return;
  } else {
    // default
    cone_lane_change_direction_ = NO_CHANGE;
    return;
  }
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
    LOG_DEBUG(
        "[ConesDirection] rank_correlation is %f, [ConesDirection] cones_slope "
        "is %f \n",
        rank_correlation, cones_slope);

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
  ;
  if (ego_lane == nullptr) {
    LOG_DEBUG("seach fail: ego lane is nullptr \n");
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
    LOG_DEBUG("[CheckEgoLaneAvailable] fail to get ego position on base lane");
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
        LOG_DEBUG("left front cone is dangerous for lane change\n");
        return false;
      }
    }
  } else {
    for (auto& p : cone_points_) {
      right_width = QueryLaneWidth(p.s, origin_lane_s_width_) * 0.5;
      double ego_distance_to_cone = p.s - ego_frenet_point.x;
      if (ego_distance_to_cone < ego_vel * kLongClusterTimeGap / 2 && p.l < 0 &&
          p.l > -right_width) {
        LOG_DEBUG("right front cone is dangerous for lane change\n");
        return false;
      }
    }
  }
  return true;
}

bool ConeRequest::CheckTargetLaneAvailable(
    bool is_left, const std::shared_ptr<VirtualLane> seach_lane) {
  if (seach_lane == nullptr) {
    LOG_DEBUG("seach fail: seach lane is nullptr \n");
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

  std::shared_ptr<KDPath> target_lane_frenet_coord =
      target_refline->get_frenet_coord();

  std::vector<ConePoint> serach_cone_points = cone_points_;
  for (auto& p : serach_cone_points) {
    if (!target_lane_frenet_coord->XYToSL(p.x, p.y, &p.s, &p.l)) {
      LOG_DEBUG("[CheckTargetLaneAvailable]: XYToSL fail \n");
      return false;
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
    LOG_DEBUG(" target lane is blocked\n");
  }

  return max_l > pass_thre;
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

double ConeRequest::QueryLaneWidth(
    const double s0,
    const std::vector<std::pair<double, double>>& lane_s_width) {
  auto comp = [](const std::pair<double, double>& s_width, const double s) {
    return s_width.first < s;
  };
  double lane_width;
  const auto& first_pair_on_lane =
      std::lower_bound(lane_s_width.begin(), lane_s_width.end(), s0, comp);

  if (first_pair_on_lane == lane_s_width.begin()) {
    lane_width = lane_s_width.front().second;
  } else if (first_pair_on_lane == lane_s_width.end()) {
    lane_width = lane_s_width.back().second;
  } else {
    lane_width = planning_math::lerp(
        (first_pair_on_lane - 1)->second, (first_pair_on_lane - 1)->first,
        first_pair_on_lane->second, first_pair_on_lane->first, s0);
  }
  return std::fmax(lane_width, 2.8);
}

void ConeRequest::Reset() {
  cone_alc_trigger_counter_ = 0;
  is_cone_lane_change_situation_ = false;
  cone_cluster_size_.clear();
  cone_cluster_.clear();
  cone_points_.clear();
  cone_cluster_attribute_set_.clear();
}

}  // namespace planning
#include "lane_reference_path.h"

#include <openssl/evp.h>
#include <sys/param.h>

#include <cmath>
#include <complex>

#include "environmental_model.h"
#include "ifly_time.h"
#include "modules/context/static_analysis_storage/static_analysis_utils.h"
#include "obstacle_manager.h"
#include "session.h"
#include "virtual_lane_manager.h"

namespace planning {

constexpr double kDefaultLaneBorderDis = 20.0;

using namespace planning_math;

LaneReferencePath::LaneReferencePath(int target_lane_virtual_id)
    : ReferencePath() {
  lane_virtual_id_ = target_lane_virtual_id;
  // update();
  ILOG_DEBUG << "construct lane_reference_path: target_lane_virtual_id:"
             << target_lane_virtual_id;
  static_analysis_storage_ = std::make_shared<StaticAnalysisStorage>();
}

void LaneReferencePath::update(planning::framework::Session *session) {
  ILOG_DEBUG << "update LaneReferencePath";
  session_ = session;
  // Step 1) import reference_path pointer to virtual_lane
  auto virtual_lane = session->mutable_environmental_model()
                          ->mutable_virtual_lane_manager()
                          ->mutable_lane_with_virtual_id(lane_virtual_id_);
  if (virtual_lane == nullptr) {
    std::cout << "virtual_lane == nullptr!!!:" << lane_virtual_id_ << std::endl;
    return;
  }
  std::cout << "get id " << lane_virtual_id_ << std::endl;
  virtual_lane->update_reference_path(shared_from_this());

  // Step 2) get reference_points
  ReferencePathPoints raw_reference_path_points;

  bool ok = false;
  if (session_->is_hpp_scene()) {
    ok = get_ref_points_hpp(raw_reference_path_points);
  } else if (session_->is_rads_scene()) {
    ok = get_ref_points_rads(raw_reference_path_points);
  } else {
    ok = get_ref_points(raw_reference_path_points);
  }

  // Step 3) update
  if (ok) {
    // Step 3-1) update ref path
    auto current_time = IflyTime::Now_ms();
    if (session_->is_hpp_scene()) {
      update_refpath_points_in_hpp(ego_projection_length_in_reference_path_,
                                   raw_reference_path_points);
    } else {
      bool is_need_smooth = false;
      int current_lane_virtual_id = session_->environmental_model()
                                        .get_virtual_lane_manager()
                                        ->current_lane_virtual_id();
      if (current_lane_virtual_id == lane_virtual_id_) {
        is_need_smooth = true;
      }
      update_refpath_points(raw_reference_path_points, is_need_smooth);
    }
    valid_ = refined_ref_path_points_.size() >= 3;
    if (!valid_) {
      return;
    }
    auto end_time = IflyTime::Now_ms();
    ILOG_INFO << "update_refpath_points time:" << end_time - current_time;
    // Step 3-2) update frenet ego state
    frenet_ego_state_.update(
        frenet_coord_,
        *session_->mutable_environmental_model()->get_ego_state_manager());
    // Step 3-3) update obstacles
    update_obstacles();
    // Step 3-4) update virtual_lane speed_limit
    virtual_lane->update_speed_limit(
        session->environmental_model().get_ego_state_manager()->ego_v(),
        session->environmental_model().get_ego_state_manager()->ego_v_cruise());
  } else {
    ILOG_ERROR << "LaneReferencePath::update failed";
  }

  if (session_->is_hpp_scene()) {
    // Step 4) update road_wideh base obs for hpp
    RoadBorderWidthCalByObs(refined_ref_path_points_, frenet_obstacles_);
  }

  // Step 5) update road_type for hpp
  if (session_->is_hpp_scene()) {
    StaticAnalysisUtils::RoadTypeAnalysis(
        refined_ref_path_points_, frenet_coord_, static_analysis_storage_);

    StaticAnalysisUtils::PassageTypeAnalysis(
        refined_ref_path_points_, frenet_coord_, static_analysis_storage_);

    StaticAnalysisUtils::ElemTypeAnalysis(
        std::static_pointer_cast<const ReferencePath>(shared_from_this()),
        refined_ref_path_points_, frenet_coord_, static_analysis_storage_);

    auto &planning_debug_info =
        DebugInfoManager::GetInstance().GetDebugInfoPb();
    static_analysis_storage_->SerializeToDebugInfo(
        frenet_coord_, *planning_debug_info->mutable_static_analysis_result());
  }
}

bool LaneReferencePath::RoadBorderWidthCalByObs(
    ReferencePathPoints &refer_path_points,
    const std::vector<std::shared_ptr<FrenetObstacle>> &frenet_obstacles) {
  if (refer_path_points.size() <= 2) {
    ILOG_ERROR << "RoadBorderWidthCalByObs: refer_path_points size <= 2";
    return false;
  }

  constexpr double kDistanceBorderDefault = 3.5;
  constexpr double kLonBuffer = 0.5;
  constexpr double kThresholdBak = 0.2;
  constexpr double kRightFarThreshold = -kDistanceBorderDefault + kThresholdBak;
  constexpr double kLeftFarThreshold = kDistanceBorderDefault - kThresholdBak;

  size_t obs_idx = 0;  // 双指针：障碍物指针，随 s 单调前进
  for (size_t i = 0; i < refer_path_points.size(); ++i) {
    if (std::isnan(refer_path_points[i].path_point.x()) ||
        std::isnan(refer_path_points[i].path_point.y())) {
      ILOG_ERROR << "RoadBorderWidthCalByObs: skip NaN point at index " << i;
      continue;
    }

    double left_border = kDistanceBorderDefault;
    double right_border = -kDistanceBorderDefault;

    const double s_start =
        std::max(refer_path_points[i].path_point.s() - kLonBuffer, 0.0);
    const double s_end = (i + 1 < refer_path_points.size())
                             ? refer_path_points[i + 1].path_point.s()
                             : s_start + 3.0;

    // 推进 obs_idx，跳过 s_end < s_start 的障碍物（已完全落在当前段之前）
    while (obs_idx < frenet_obstacles.size()) {
      const auto &obs = frenet_obstacles[obs_idx];
      if (!obs) {
        ++obs_idx;
        continue;
      }
      const auto &bnd = obs->frenet_obstacle_boundary();
      if (bnd.s_end >= s_start) break;
      ++obs_idx;
    }

    // 从 obs_idx 起遍历与 [s_start, s_end] 重叠的障碍物，遇 s_start > s_end
    // 即停
    for (size_t j = obs_idx; j < frenet_obstacles.size(); ++j) {
      const auto &obs = frenet_obstacles[j];
      if (!obs) continue;

      const auto &bnd = obs->frenet_obstacle_boundary();
      if (bnd.s_start > s_end) break;
      if (bnd.l_start * bnd.l_end < 0.0) continue;  // 横跨车道中线，忽略

      if (bnd.l_start > 0.0) {
        if (bnd.l_end >= kLeftFarThreshold &&
            bnd.l_start <= kDistanceBorderDefault) {
          left_border = std::min(left_border, bnd.l_start);
        }
      } else {
        if (bnd.l_start <= kRightFarThreshold &&
            bnd.l_end >= -kDistanceBorderDefault) {
          right_border = std::max(right_border, bnd.l_end);
        }
      }
    }
    refer_path_points[i].distance_to_left_road_border = left_border;
    refer_path_points[i].distance_to_right_road_border =
        std::fabs(right_border);
    refer_path_points[i].lane_width = left_border + std::fabs(right_border);
  }
  return true;
}

void LaneReferencePath::Update(planning::framework::Session *session,
                               ReferencePathPoints &raw_reference_path_points) {
  ILOG_DEBUG << "update LaneReferencePath";
  session_ = session;
  // Step 1) import reference_path pointer to virtual_lane
  auto virtual_lane = session->mutable_environmental_model()
                          ->mutable_virtual_lane_manager()
                          ->mutable_lane_with_virtual_id(lane_virtual_id_);
  if (virtual_lane == nullptr) {
    std::cout << "virtual_lane == nullptr!!!:" << lane_virtual_id_ << std::endl;
    return;
  }
  std::cout << "get id " << lane_virtual_id_ << std::endl;
  virtual_lane->update_reference_path(shared_from_this());

  // Step 2) get reference_points
  bool ok = false;
  ok = ExtendConstructionRefPathPoints(raw_reference_path_points);

  // Step 3) update
  if (ok) {
    // Step 3-1) update ref path
    auto current_time = IflyTime::Now_ms();
    bool is_need_smooth = false;
    int current_lane_virtual_id = session_->environmental_model()
                                      .get_virtual_lane_manager()
                                      ->current_lane_virtual_id();
    if (current_lane_virtual_id == lane_virtual_id_) {
      is_need_smooth = true;
    }
    update_refpath_points(raw_reference_path_points, is_need_smooth);
    valid_ = refined_ref_path_points_.size() >= 3;
    if (!valid_) {
      return;
    }
    auto end_time = IflyTime::Now_ms();
    ILOG_INFO << "update_refpath_points time:" << end_time - current_time;
    // Step 3-2) update frenet ego state
    frenet_ego_state_.update(
        frenet_coord_,
        *session_->mutable_environmental_model()->get_ego_state_manager());
    // Step 3-3) update obstacles
    update_obstacles();
    // Step 3-4) update virtual_lane speed_limit
    virtual_lane->update_speed_limit(
        session->environmental_model().get_ego_state_manager()->ego_v(),
        session->environmental_model().get_ego_state_manager()->ego_v_cruise());
    is_construction_scene_ref_path_ = true;
    ref_path_source_ = ReferencePathSource::CONSTRUCTION_SCENE;
  } else {
    ILOG_ERROR << "LaneReferencePath::update failed";
  }
};

// TODO:clren   这个函数还没有使用   判断逻辑可以更改
bool LaneReferencePath::is_obstacle_ignorable(
    const std::shared_ptr<FrenetObstacle> obstacle) {
  bool res{false};
  bool is_obstacle_right_behind_ego{false};
  if (obstacle->frenet_obstacle_boundary().s_end <
      frenet_ego_state_.boundary().s_start) {
    if (obstacle->frenet_l() > frenet_ego_state_.boundary().l_start &&
        obstacle->frenet_l() < frenet_ego_state_.boundary().l_end) {
      is_obstacle_right_behind_ego = true;
    }
  }

  if (!is_obstacle_right_behind_ego) {
    return res;
  }

  bool is_obstacle_current_on_lane{false};
  ReferencePathPoint obstacle_matched_lane_point;
  if (get_reference_point_by_lon(obstacle->frenet_s(),
                                 obstacle_matched_lane_point)) {
    if (obstacle->frenet_obstacle_boundary().l_start >
            -obstacle_matched_lane_point.distance_to_right_lane_border &&
        obstacle->frenet_obstacle_boundary().l_end <
            obstacle_matched_lane_point.distance_to_left_lane_border) {
      is_obstacle_current_on_lane = true;
    }
  }

  if (is_obstacle_current_on_lane) {
    res = true;
  }

  return res;
}

void LaneReferencePath::update_obstacles() {
  auto obstacle_manager =
      session_->mutable_environmental_model()->get_obstacle_manager();
  if(obstacle_manager == nullptr) {
    return;
  }
  if (session_->is_hpp_scene()) {
    frenet_obstacles_.clear();
    speed_bump_frenet_obstacles_.clear();
    turnstile_frenet_obstacles_.clear();
    semantic_sign_frenet_obstacles_.clear();
    generate_frenet_obstacles(obstacle_manager->get_obstacles(),
                              frenet_obstacles_, frenet_obstacles_map_);
    generate_frenet_obstacles(obstacle_manager->get_groundline_obstacles(),
                              frenet_obstacles_, frenet_obstacles_map_);
    generate_frenet_obstacles(obstacle_manager->get_occupancy_obstacles(),
                              frenet_obstacles_, frenet_obstacles_map_);
    generate_frenet_obstacles(obstacle_manager->get_map_static_obstacles(),
                              frenet_obstacles_, frenet_obstacles_map_);
    generate_frenet_obstacles(obstacle_manager->get_speed_bump_obstacles(),
                              speed_bump_frenet_obstacles_,
                              speed_bump_frenet_obstacles_map_);
    generate_frenet_obstacles(obstacle_manager->get_turnstile_obstacles(),
                              turnstile_frenet_obstacles_,
                              turnstile_frenet_obstacles_map_);

    generate_frenet_obstacles(obstacle_manager->get_semantic_sign_obstacles(),
                              semantic_sign_frenet_obstacles_,
                              semantic_sign_frenet_obstacles_map_);

    // sort obs by s
    std::sort(frenet_obstacles_.begin(), frenet_obstacles_.end(),
              [](const auto &a, const auto &b) {
                if (!a) return false;
                if (!b) return true;
                return a->frenet_obstacle_boundary().s_end <
                       b->frenet_obstacle_boundary().s_end;
              });
  } else {
    obstacle_manager->generate_frenet_obstacles(*this);
  }

  assign_obstacles_to_lane();
  parking_spaces_ = obstacle_manager->get_parking_space().Items();
  free_space_ground_lines_ =
      obstacle_manager->get_groundline_obstacles().Items();
  road_edges_ = obstacle_manager->get_road_edge_obstacles().Items();
}

void LaneReferencePath::generate_frenet_obstacles(
    const IndexedList<int, Obstacle> &obstacles,
    std::vector<std::shared_ptr<FrenetObstacle>> &frenet_obstacles,
    std::unordered_map<int, std::shared_ptr<FrenetObstacle>>
        &frenet_obstacles_map) {
  frenet_obstacles_.reserve(frenet_obstacles_.size() +
                            obstacles.Items().size());
  for (const Obstacle *obstacle_ptr : obstacles.Items()) {
    Point2D frenet_point, cart_point;
    cart_point.x = obstacle_ptr->x_center();
    cart_point.y = obstacle_ptr->y_center();

    if (!frenet_coord_->XYToSL(cart_point, frenet_point) ||
        std::isnan(frenet_point.x) || std::isnan(frenet_point.y)) {
      ILOG_DEBUG << "cart_point to frenet_point failed, obstacle_id: "
                 << obstacle_ptr->id();
      continue;
    }

    // construct frenet_obstacle
    std::shared_ptr<FrenetObstacle> frenet_obstacle =
        std::make_shared<FrenetObstacle>(
            obstacle_ptr, *this,
            session_->environmental_model().get_ego_state_manager(), true);
    frenet_obstacles.emplace_back(frenet_obstacle);
    frenet_obstacles_map[obstacle_ptr->id()] = frenet_obstacle;
  }
}

bool LaneReferencePath::get_ref_points(ReferencePathPoints &ref_path_points) {
  // get lane
  auto virtual_lane_manager =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  auto virtual_lane =
      virtual_lane_manager->get_lane_with_virtual_id(lane_virtual_id_);
  auto &reference_path_manager =
      session_->environmental_model().get_reference_path_manager();
  // get raw ref line
  auto &lane_points = virtual_lane->lane_points();
  std::cout << "lane_points.size(): " << lane_points.size() << std::endl;
  const double width = virtual_lane->width();
  ref_path_points.clear();
  ref_path_points.reserve(lane_points.size());
  for (auto &refline_pt : lane_points) {
    ReferencePathPoint ref_path_pt;
    ref_path_pt.path_point.set_x(refline_pt.local_point.x);
    ref_path_pt.path_point.set_y(refline_pt.local_point.y);
    ref_path_pt.path_point.set_z(refline_pt.local_point.z);
    ref_path_pt.path_point.set_theta(refline_pt.enu_heading);
    ref_path_pt.path_point.set_kappa(refline_pt.curvature);
    // ref_path_pt.distance_to_left_lane_border = std::fmin(
    //     refline_pt.distance_to_left_lane_border, kDefaultLaneBorderDis);
    // ref_path_pt.distance_to_right_lane_border = std::fmin(
    //     refline_pt.distance_to_right_lane_border, kDefaultLaneBorderDis);
    ref_path_pt.distance_to_left_road_border = std::fmin(
        refline_pt.distance_to_left_road_border, kDefaultLaneBorderDis);
    ref_path_pt.distance_to_right_road_border = std::fmin(
        refline_pt.distance_to_right_road_border, kDefaultLaneBorderDis);
    ref_path_pt.distance_to_left_lane_border = width * 0.5;
    ref_path_pt.distance_to_right_lane_border = width * 0.5;
    ref_path_pt.left_road_border_type = refline_pt.left_road_border_type;
    ref_path_pt.right_road_border_type = refline_pt.right_road_border_type;
    ref_path_pt.left_lane_border_type = refline_pt.left_lane_border_type;
    ref_path_pt.right_lane_border_type = refline_pt.right_lane_border_type;
    ref_path_pt.lane_width = width;
    ref_path_pt.max_velocity = refline_pt.speed_limit_max;
    ref_path_pt.min_velocity = refline_pt.speed_limit_min;
    ref_path_pt.type = ReferencePathPointType::MAP;
    ref_path_pt.is_in_intersection = refline_pt.is_in_intersection;

    // check direction
    if (not ref_path_points.empty()) {
      const auto &pre_pt = ref_path_points.back();
      Vec2d delta{ref_path_pt.path_point.x() - pre_pt.path_point.x(),
                  ref_path_pt.path_point.y() - pre_pt.path_point.y()};
      Vec2d cur_direction =
          Vec2d::CreateUnitVec2d(ref_path_pt.path_point.theta());
      if (cur_direction.InnerProd(delta) < 0) {
        // temporaly skip direction check since input data is bad @clren
        // continue;
      }
    }
    ref_path_points.emplace_back(std::move(ref_path_pt));
  }
  if (ref_path_points.empty()) {
    return false;
  }
  raw_start_point_ = ref_path_points.front();
  raw_end_point_ = ref_path_points.back();
  // 判断参考线的长度是否大于自车5s的行驶距离，如果不够的话，则延长参考线
  //  calculate reference path origin total length
  double origin_reference_path_total_length = 0;
  for (int i = 1; i < ref_path_points.size(); i++) {
    const auto &cur_point = ref_path_points[i].path_point;
    const auto &pre_point = ref_path_points[i - 1].path_point;
    origin_reference_path_total_length += std::hypotf(
        pre_point.x() - cur_point.x(), pre_point.y() - cur_point.y());
  }
  origin_reference_path_length_ = origin_reference_path_total_length;
  const bool is_highway =
      session_->get_scene_type() == planning::common::SceneType::HIGHWAY;
  if (ref_path_points.size() >= 2 && is_highway) {
    const std::shared_ptr<EgoStateManager> ego_state_mgr =
        session_->mutable_environmental_model()->get_ego_state_manager();
    const double ego_v = ego_state_mgr->ego_v();
    const double cruise_v = ego_state_mgr->ego_v_cruise();
    const double preview_dis = std::fmax(ego_v, cruise_v) * 6.0;
    const double extend_buff = 5;
    const double ego_projection_length_in_reference_path =
        CalculateEgoProjectionDistanceInReferencePath(ref_path_points);
    // if need to extend reference path length
    if (preview_dis + ego_projection_length_in_reference_path + extend_buff >
        origin_reference_path_total_length) {
      const double extend_length =
          preview_dis + ego_projection_length_in_reference_path -
          origin_reference_path_total_length + extend_buff;
      ReferencePathPoint extend_point;
      const int point_nums = ref_path_points.size();
      extend_point = CalculateExtendedReferencePathPoint(
          ref_path_points[point_nums - 2], ref_path_points[point_nums - 1],
          extend_length);
      ref_path_points.emplace_back(std::move(extend_point));
    }
  }
  return ref_path_points.size() >= 3;
}

bool LaneReferencePath::get_ref_points_hpp(
    ReferencePathPoints &ref_path_points) {
  auto virtual_lane_manager =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  auto virtual_lane =
      virtual_lane_manager->get_lane_with_virtual_id(lane_virtual_id_);
  auto &lane_points = virtual_lane->lane_points();
  std::cout << "lane_points.size(): " << lane_points.size() << std::endl;
  const double width = virtual_lane->width();
  ref_path_points.clear();
  ref_path_points.reserve(lane_points.size());
  auto is_enu_valid = session_->environmental_model().location_valid();
  for (auto &refline_pt : lane_points) {
    ReferencePathPoint ref_path_pt;
    if (is_enu_valid) {
      ref_path_pt.path_point.set_x(refline_pt.local_point.x);
      ref_path_pt.path_point.set_y(refline_pt.local_point.y);
      ref_path_pt.path_point.set_z(refline_pt.local_point.z);
      ref_path_pt.path_point.set_theta(refline_pt.enu_heading);

    } else {
      ref_path_pt.path_point.set_x(refline_pt.car_point.x);
      ref_path_pt.path_point.set_y(refline_pt.car_point.y);
      ref_path_pt.path_point.set_z(0.0);
      ref_path_pt.path_point.set_theta(refline_pt.car_heading);
    }
    ref_path_pt.path_point.set_kappa(refline_pt.curvature);
    ref_path_pt.distance_to_left_road_border = 5.0;
    ref_path_pt.distance_to_right_road_border = 5.0;
    ref_path_pt.distance_to_left_lane_border = 5.0;
    ref_path_pt.distance_to_right_lane_border = 5.0;
    ref_path_pt.left_road_border_type = refline_pt.left_road_border_type;
    ref_path_pt.right_road_border_type = refline_pt.right_road_border_type;
    ref_path_pt.left_lane_border_type = refline_pt.left_lane_border_type;
    ref_path_pt.right_lane_border_type = refline_pt.right_lane_border_type;
    ref_path_pt.lane_width = width;
    ref_path_pt.max_velocity = refline_pt.speed_limit_max;
    ref_path_pt.min_velocity = refline_pt.speed_limit_min;
    ref_path_pt.type = ReferencePathPointType::MAP;
    ref_path_pt.is_in_intersection = refline_pt.is_in_intersection;
    ref_path_pt.is_ramp =
        (refline_pt.lane_type == iflyauto::LaneType::LANETYPE_RAMP);
    ref_path_pt.ramp_slope = refline_pt.slope;

    // check direction
    // if (not ref_path_points.empty()) {
    //   const auto &pre_pt = ref_path_points.back();
    //   Vec2d delta{ref_path_pt.path_point.x - pre_pt.path_point.x,
    //               ref_path_pt.path_point.y - pre_pt.path_point.y};
    //   Vec2d cur_direction =
    //       Vec2d::CreateUnitVec2d(ref_path_pt.path_point.theta());
    //   if (cur_direction.InnerProd(delta) < 0) {
    // temporaly skip direction check since input data is bad @clren
    // continue;
    //   }
    // }
    ref_path_points.emplace_back(std::move(ref_path_pt));
  }
  if (ref_path_points.empty()) {
    return false;
  }
  raw_start_point_ = ref_path_points.front();
  raw_end_point_ = ref_path_points.back();
  // 判断参考线的长度是否大于自车5s的行驶距离，如果不够的话，则延长参考线
  //  calculate reference path origin total length
  double origin_reference_path_total_length = 0;
  for (int i = 1; i < ref_path_points.size(); i++) {
    const auto &cur_point = ref_path_points[i].path_point;
    const auto &pre_point = ref_path_points[i - 1].path_point;
    origin_reference_path_total_length += std::hypotf(
        pre_point.x() - cur_point.x(), pre_point.y() - cur_point.y());
  }
  origin_reference_path_length_ = origin_reference_path_total_length;
  ego_projection_length_in_reference_path_ =
      CalculateEgoProjectionDistanceInReferencePath(ref_path_points);
  if (ref_path_points.size() >= 2) {
    const double extended_ref_path_length =
        CalculateExtendedReferencePathLength(
            origin_reference_path_total_length,
            ego_projection_length_in_reference_path_, ref_path_points);
    if (extended_ref_path_length > origin_reference_path_total_length) {
      const double extended_length =
          extended_ref_path_length - origin_reference_path_total_length;
      ReferencePathPoint extend_point;
      const int point_nums = ref_path_points.size();
      extend_point = CalculateExtendedReferencePathPoint(
          ref_path_points[point_nums - 2], ref_path_points[point_nums - 1],
          extended_length);
      ref_path_points.emplace_back(std::move(extend_point));
      extended_reference_path_length_ = extended_ref_path_length;
    } else {
      extended_reference_path_length_ = origin_reference_path_total_length;
    }
  }
  return ref_path_points.size() >= 3;
}

bool LaneReferencePath::get_ref_points_rads(
    ReferencePathPoints &ref_path_points) {
  // get lane
  auto virtual_lane_manager =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  auto virtual_lane =
      virtual_lane_manager->get_lane_with_virtual_id(lane_virtual_id_);
  auto &reference_path_manager =
      session_->environmental_model().get_reference_path_manager();
  // get raw ref line
  auto &lane_points = virtual_lane->lane_points();
  if (lane_points.size() < 2) {
    return false;
  }
  // std::cout << "lane_points.size(): " << lane_points.size() << std::endl;
  const double width = virtual_lane->width();
  const double length = lane_points.back().s;
  const double backward_extend_buff = 5.0;
  const double forward_extend_buff = 16.0;
  const auto &ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  double start_s = std::min(ego_state_manager->ego_drive_distance(), length) -
                   backward_extend_buff;
  double end_s = ego_state_manager->ego_drive_distance() + forward_extend_buff;
  ref_path_points.clear();
  ref_path_points.reserve(lane_points.size());
  for (auto &refline_pt : lane_points) {
    if (refline_pt.s < start_s) {
      continue;
    }
    if (refline_pt.s > end_s) {
      break;
    }
    ReferencePathPoint ref_path_pt;
    ref_path_pt.path_point.set_x(refline_pt.local_point.x);
    ref_path_pt.path_point.set_y(refline_pt.local_point.y);
    ref_path_pt.path_point.set_z(refline_pt.local_point.z);
    ref_path_pt.path_point.set_theta(refline_pt.enu_heading);
    ref_path_pt.path_point.set_kappa(refline_pt.curvature);
    ref_path_pt.path_point.set_s(refline_pt.s);
    // ref_path_pt.distance_to_left_lane_border = std::fmin(
    //     refline_pt.distance_to_left_lane_border, kDefaultLaneBorderDis);
    // ref_path_pt.distance_to_right_lane_border = std::fmin(
    //     refline_pt.distance_to_right_lane_border, kDefaultLaneBorderDis);
    ref_path_pt.distance_to_left_road_border = std::fmin(
        refline_pt.distance_to_left_road_border, kDefaultLaneBorderDis);
    ref_path_pt.distance_to_right_road_border = std::fmin(
        refline_pt.distance_to_right_road_border, kDefaultLaneBorderDis);
    ref_path_pt.distance_to_left_lane_border = width * 0.5;
    ref_path_pt.distance_to_right_lane_border = width * 0.5;
    ref_path_pt.left_road_border_type = refline_pt.left_road_border_type;
    ref_path_pt.right_road_border_type = refline_pt.right_road_border_type;
    ref_path_pt.left_lane_border_type = refline_pt.left_lane_border_type;
    ref_path_pt.right_lane_border_type = refline_pt.right_lane_border_type;
    ref_path_pt.lane_width = width;
    ref_path_pt.max_velocity = refline_pt.speed_limit_max;
    ref_path_pt.min_velocity = refline_pt.speed_limit_min;
    ref_path_pt.type = ReferencePathPointType::MAP;
    ref_path_pt.is_in_intersection = refline_pt.is_in_intersection;

    // check direction
    if (not ref_path_points.empty()) {
      const auto &pre_pt = ref_path_points.back();
      Vec2d delta{ref_path_pt.path_point.x() - pre_pt.path_point.x(),
                  ref_path_pt.path_point.y() - pre_pt.path_point.y()};
      Vec2d cur_direction =
          Vec2d::CreateUnitVec2d(ref_path_pt.path_point.theta());
      if (cur_direction.InnerProd(delta) < 0) {
        // temporaly skip direction check since input data is bad @clren
        // continue;
      }
    }
    ref_path_points.emplace_back(std::move(ref_path_pt));
  }
  if (ref_path_points.empty()) {
    return false;
  }
  raw_start_point_ = ref_path_points.front();
  raw_end_point_ = ref_path_points.back();
  // calculate reference path origin total length
  double origin_reference_path_total_length = 0;
  for (int i = 1; i < ref_path_points.size(); i++) {
    const auto &cur_point = ref_path_points[i].path_point;
    const auto &pre_point = ref_path_points[i - 1].path_point;
    origin_reference_path_total_length += std::hypotf(
        pre_point.x() - cur_point.x(), pre_point.y() - cur_point.y());
  }
  origin_reference_path_length_ = origin_reference_path_total_length;
  if (ref_path_points.size() >= 2) {
    // raw theta invalid -> projection invalid
    const int ego_nearest_path_point_index =
        CalculateNearestDistancePathPoint(ref_path_points);
    double cur_path_point_s =
        ref_path_points[ego_nearest_path_point_index].path_point.s();
    // backward
    double backward_extend_length = std::max(
        0.0, backward_extend_buff -
                 (cur_path_point_s - ref_path_points.front().path_point.s()));
    if (backward_extend_length > 1e-2) {
      ReferencePathPoint backward_extend_point;
      backward_extend_point = CalculateExtendedReferencePathPoint(
          ref_path_points[1], ref_path_points[0], backward_extend_length);
      backward_extend_point.path_point.set_s(
          ref_path_points.front().path_point.s() - backward_extend_length);
      ref_path_points.emplace(ref_path_points.begin(),
                              std::move(backward_extend_point));
    }
    // forward
    double forward_extend_length = std::max(
        0.0, forward_extend_buff -
                 (ref_path_points.back().path_point.s() - cur_path_point_s));
    if (forward_extend_length > 1e-2) {
      ReferencePathPoint forward_extend_point;
      const int point_nums = ref_path_points.size();
      forward_extend_point = CalculateExtendedReferencePathPoint(
          ref_path_points[point_nums - 2], ref_path_points[point_nums - 1],
          forward_extend_length);
      forward_extend_point.path_point.set_s(
          ref_path_points.back().path_point.s() + forward_extend_length);
      ref_path_points.emplace_back(std::move(forward_extend_point));
    }
  }
  return ref_path_points.size() >= 3;
}

bool LaneReferencePath::ExtendConstructionRefPathPoints(
    ReferencePathPoints &ref_path_points) {
  // 判断参考线的长度是否大于自车5s的行驶距离，如果不够的话，则延长参考线
  //  calculate reference path origin total length
  double origin_reference_path_total_length = 0;
  for (int i = 1; i < ref_path_points.size(); i++) {
    const auto &cur_point = ref_path_points[i].path_point;
    const auto &pre_point = ref_path_points[i - 1].path_point;
    origin_reference_path_total_length += std::hypotf(
        pre_point.x() - cur_point.x(), pre_point.y() - cur_point.y());
  }
  origin_reference_path_length_ = origin_reference_path_total_length;
  const bool is_highway =
      session_->get_scene_type() == planning::common::SceneType::HIGHWAY;
  if (ref_path_points.size() >= 2 && is_highway) {
    const std::shared_ptr<EgoStateManager> ego_state_mgr =
        session_->mutable_environmental_model()->get_ego_state_manager();
    const double ego_v = ego_state_mgr->ego_v();
    const double cruise_v = ego_state_mgr->ego_v_cruise();
    const double preview_dis = std::fmax(ego_v, cruise_v) * 6.0;
    const double extend_buff = 5;
    const double ego_projection_length_in_reference_path =
        CalculateEgoProjectionDistanceInReferencePath(ref_path_points);
    // if need to extend reference path length
    if (preview_dis + ego_projection_length_in_reference_path + extend_buff >
        origin_reference_path_total_length) {
      const double extend_length =
          preview_dis + ego_projection_length_in_reference_path -
          origin_reference_path_total_length + extend_buff;
      ReferencePathPoint extend_point;
      const int point_nums = ref_path_points.size();
      extend_point = CalculateExtendedReferencePathPoint(
          ref_path_points[point_nums - 2], ref_path_points[point_nums - 1],
          extend_length);
      ref_path_points.emplace_back(std::move(extend_point));
    }
  }
  return ref_path_points.size() >= 3;
}

void LaneReferencePath::assign_obstacles_to_lane() {
  lane_obstacles_id_.clear();
  lane_leadone_obstacle_ = -1;  // 需要注意id是否为负数 todo
  lane_leadtwo_obstacle_ = -1;

  std::vector<std::shared_ptr<FrenetObstacle>> sorted_obstacles;
  for (auto &frenet_obstacle : frenet_obstacles_) {
    if (IsObstacleOn(frenet_obstacle)) {
      sorted_obstacles.push_back(frenet_obstacle);
    }
  }
  std::sort(sorted_obstacles.begin(), sorted_obstacles.end(),
            compare_obstacle_s_ascend);

  int leadone{-1};
  int leadtwo{-1};
  double s_ego = get_frenet_ego_state().s();
  for (auto &frenet_obstacle : sorted_obstacles) {
    lane_obstacles_id_.emplace_back(frenet_obstacle->id());
    auto fusion_source = frenet_obstacle->obstacle()->fusion_source();
    if (fusion_source & OBSTACLE_SOURCE_CAMERA) {
      if (frenet_obstacle->frenet_s() > s_ego && leadone == -1) {
        leadone = frenet_obstacle->id();
        continue;
      }
      if (frenet_obstacle->frenet_s() > s_ego && leadone != -1 &&
          leadtwo == -1) {
        leadtwo = frenet_obstacle->id();
        break;
      }
    }
  }
  lane_leadone_obstacle_ = leadone;
  lane_leadtwo_obstacle_ = leadtwo;
}

bool LaneReferencePath::IsObstacleOn(
    std::shared_ptr<FrenetObstacle> frenet_obstacle) {
  if (frenet_obstacle->rel_s() > 180 || frenet_obstacle->rel_s() < -50 ||
      frenet_obstacle->s_min_l().x > 15 || frenet_obstacle->s_max_l().x < -15 ||
      !frenet_obstacle->b_frenet_valid()) {
    return false;
  }
  double check_offset = 0;
  double lane_width = 3.8;
  auto virtual_lane_manager =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  auto virtual_lane =
      virtual_lane_manager->get_lane_with_virtual_id(lane_virtual_id_);
  lane_width = virtual_lane->width_by_s(frenet_obstacle->frenet_s());
  double distance_to_left_line =
      frenet_obstacle->s_min_l().x - lane_width / 2.0;
  double distance_to_right_line =
      frenet_obstacle->s_max_l().x + lane_width / 2.0;
  // 扩张自车道上的锥桶，使之能被绑定在相邻车道上
  if (frenet_obstacle->type() ==
          iflyauto::ObjectType::OBJECT_TYPE_TRAFFIC_CONE &&
      std::abs(frenet_obstacle->l_relative_to_ego()) < lane_width) {
    distance_to_left_line = distance_to_left_line - 1.0;
    distance_to_right_line = distance_to_right_line + 1.0;
  }
  std::array<double, 3> xp_vs{0., 3., 5.};
  std::array<double, 3> fp_l1{std::max(lane_width / 2 - 1.45, 0.0),
                              std::max(lane_width / 2 - 1.65, 0.0), 0.0};

  // 可以根据不同的fusion_source障碍物的横向速度的准确度选择不同的逻辑
  if (1) {
    std::array<double, 5> xp_rel_s{-30, -20, 20, 30, 60};
    std::array<double, 5> fp_l2{0.45, 0.2, 0.3, 0.45, 0.6};
    if (frenet_obstacle->type() ==
        iflyauto::ObjectType::OBJECT_TYPE_TRAFFIC_CONE) {
      check_offset = interp(frenet_obstacle->rel_s(), xp_rel_s, fp_l2);
    } else {
      check_offset = interp(frenet_obstacle->rel_s(), xp_rel_s, fp_l2) +
                     interp(frenet_obstacle->frenet_velocity_s(), xp_vs, fp_l1);
    }
    return (distance_to_left_line < -check_offset &&
            distance_to_right_line > check_offset);
  } else {
    double prediction_time = 1.5;
    double check_offset =
        (frenet_obstacle->rel_s() > -10)
            ? 0.2
            : 0.4 + interp(frenet_obstacle->frenet_velocity_s(), xp_vs, fp_l1);

    double predict_distance_min_l =
        distance_to_left_line +
        frenet_obstacle->frenet_velocity_l() * prediction_time;
    double predict_distance_max_r =
        distance_to_right_line +
        frenet_obstacle->frenet_velocity_l() * prediction_time;

    return (predict_distance_min_l < -check_offset &&
            predict_distance_max_r > check_offset);
  }
}

void LaneReferencePath::cal_current_leadone_leadtwo_to_ego() {
  int current_leadone_id{-1};
  int current_leadtwo_id{-1};

  double s_ego = get_frenet_ego_state().s();
  auto lane_obstacles = get_lane_obstacles_ids();
  std::vector<std::shared_ptr<FrenetObstacle>> sorted_obstacles;
  for (auto &frenet_obstacle : frenet_obstacles_) {
    if (std::find(lane_obstacles.begin(), lane_obstacles.end(),
                  frenet_obstacle->id()) == lane_obstacles.end() ||
        frenet_obstacle->frenet_s() > s_ego) {
      continue;
    }
    if (is_potential_current_leadone_leadtwo_to_ego(frenet_obstacle)) {
      sorted_obstacles.emplace_back(frenet_obstacle);
    }
  }

  std::sort(sorted_obstacles.begin(), sorted_obstacles.end(),
            compare_obstacle_s_ascend);

  for (auto &frenet_obstacle : sorted_obstacles) {
    if (current_leadone_id == -1) {
      current_leadone_id = frenet_obstacle->id();
      continue;
    }
    if (current_leadone_id != -1 && current_leadtwo_id == -1) {
      current_leadtwo_id = frenet_obstacle->id();
      break;
    }
  }
  current_leadone_obstacle_to_ego_ = current_leadone_id;
  current_leadtwo_obstacle_to_ego_ = current_leadtwo_id;
}

bool LaneReferencePath::is_potential_current_leadone_leadtwo_to_ego(
    const std::shared_ptr<FrenetObstacle> frenet_obstacle) {
  double l_relative_to_ego = frenet_obstacle->l_relative_to_ego();
  if (fabs(l_relative_to_ego) < 1.6) {
    return true;
  } else {
    return false;
  }
}
ReferencePathPoint LaneReferencePath::CalculateExtendedReferencePathPoint(
    const ReferencePathPoint &p1, const ReferencePathPoint &p2,
    const double length) const {
  // 计算直线方向向量
  double dx = p2.path_point.x() - p1.path_point.x();
  double dy = p2.path_point.y() - p1.path_point.y();
  // 计算直线长度
  double line_length = sqrt(dx * dx + dy * dy);
  // 将方向向量归一化
  dx /= line_length;
  dy /= line_length;
  // 计算延长后的点坐标
  ReferencePathPoint extend_point;
  extend_point.path_point.set_x(p2.path_point.x() + dx * length);
  extend_point.path_point.set_y(p2.path_point.y() + dy * length);

  const auto &last_point = p2;
  extend_point.path_point.set_z(last_point.path_point.z());
  extend_point.path_point.set_theta(last_point.path_point.theta());
  extend_point.path_point.set_kappa(1e-6);

  extend_point.distance_to_left_lane_border =
      last_point.distance_to_left_lane_border;
  extend_point.distance_to_left_road_border =
      last_point.distance_to_left_road_border;
  extend_point.distance_to_right_lane_border =
      last_point.distance_to_right_lane_border;
  extend_point.distance_to_right_road_border =
      last_point.distance_to_right_road_border;

  extend_point.left_road_border_type = last_point.left_road_border_type;
  extend_point.right_road_border_type = last_point.right_road_border_type;
  extend_point.left_lane_border_type = last_point.left_lane_border_type;
  extend_point.right_lane_border_type = last_point.right_lane_border_type;
  extend_point.lane_width = last_point.lane_width;
  extend_point.max_velocity = last_point.max_velocity;
  extend_point.min_velocity = last_point.min_velocity;
  extend_point.type = ReferencePathPointType::MAP;
  extend_point.is_in_intersection = last_point.is_in_intersection;

  return extend_point;
}

double LaneReferencePath::CalculateEgoProjectionDistanceInReferencePath(
    const ReferencePathPoints &ref_path_points) const {
  const std::shared_ptr<EgoStateManager> ego_state_mgr =
      session_->mutable_environmental_model()->get_ego_state_manager();
  // const auto &ego_pose = ego_state_mgr->ego_pose();
  // double dx = ego_pose.x - ref_path_points[0].path_point.x;
  // double dy = ego_pose.y - ref_path_points[0].path_point.y;
  const auto &lat_init_state =
      ego_state_mgr->planning_init_point().lat_init_state;
  double dx = lat_init_state.x() - ref_path_points[0].path_point.x();
  double dy = lat_init_state.y() - ref_path_points[0].path_point.y();
  const int point_nums = ref_path_points.size();
  int nearest_point_index = 0;
  double accumulate_distance_for_nearest_point = 0;
  double accumulate_distance_reference_path = 0;
  double min_distance_square_to_ego_point = dx * dx + dy * dy;
  // find nearest point
  for (int i = 1; i < point_nums; i++) {
    const auto &cur_point = ref_path_points[i].path_point;
    const auto &pre_point = ref_path_points[i - 1].path_point;
    accumulate_distance_reference_path += std::hypotf(
        pre_point.x() - cur_point.x(), pre_point.y() - cur_point.y());
    dx = lat_init_state.x() - cur_point.x();
    dy = lat_init_state.y() - cur_point.y();
    double temp_min_distance_square_to_ego_point = dx * dx + dy * dy;
    if (temp_min_distance_square_to_ego_point <
        min_distance_square_to_ego_point) {
      nearest_point_index = i;
      accumulate_distance_for_nearest_point =
          accumulate_distance_reference_path;
      min_distance_square_to_ego_point = temp_min_distance_square_to_ego_point;
    }
  }
  // calculate ego projection distance in reference path
  const auto &nearest_point = ref_path_points[nearest_point_index].path_point;
  dx = lat_init_state.x() - nearest_point.x();
  dy = lat_init_state.y() - nearest_point.y();
  const double projection_length = dx * std::cos(nearest_point.theta()) +
                                   dy * std::sin(nearest_point.theta());
  const double ego_projection_distance_in_reference_path =
      projection_length + accumulate_distance_for_nearest_point;
  return ego_projection_distance_in_reference_path;
}

double LaneReferencePath::CalculatePointProjectionDistanceInReferencePath(
    const planning_math::Vec2d &point,
    const ReferencePathPoints &ref_path_points) const {
  double point_x = point.x();
  double point_y = point.y();
  double dx = point_x - ref_path_points[0].path_point.x();
  double dy = point_y - ref_path_points[0].path_point.y();
  const int point_nums = ref_path_points.size();
  int nearest_point_index = 0;
  double accumulate_distance_for_nearest_point = 0;
  double accumulate_distance_reference_path = 0;
  double min_distance_square_to_point = dx * dx + dy * dy;
  // find nearest point
  for (int i = 1; i < point_nums; i++) {
    const auto &cur_point = ref_path_points[i].path_point;
    const auto &pre_point = ref_path_points[i - 1].path_point;
    accumulate_distance_reference_path += std::hypotf(
        pre_point.x() - cur_point.x(), pre_point.y() - cur_point.y());
    dx = point_x - cur_point.x();
    dy = point_y - cur_point.y();
    double temp_min_distance_square_to_point = dx * dx + dy * dy;
    if (temp_min_distance_square_to_point < min_distance_square_to_point) {
      nearest_point_index = i;
      accumulate_distance_for_nearest_point =
          accumulate_distance_reference_path;
      min_distance_square_to_point = temp_min_distance_square_to_point;
    }
  }
  // calculate projection distance in reference path
  const auto &nearest_point = ref_path_points[nearest_point_index].path_point;
  dx = point_x - nearest_point.x();
  dy = point_y - nearest_point.y();
  const double projection_length = dx * std::cos(nearest_point.theta()) +
                                   dy * std::sin(nearest_point.theta());
  const double projection_distance_in_reference_path =
      projection_length + accumulate_distance_for_nearest_point;
  return projection_distance_in_reference_path;
}

int LaneReferencePath::CalculateNearestDistancePathPoint(
    const ReferencePathPoints &ref_path_points) const {
  const std::shared_ptr<EgoStateManager> ego_state_mgr =
      session_->mutable_environmental_model()->get_ego_state_manager();
  const auto &ego_pose = ego_state_mgr->ego_pose();
  double dx = ego_pose.x - ref_path_points[0].path_point.x();
  double dy = ego_pose.y - ref_path_points[0].path_point.y();
  // const auto& lat_init_state =
  //     ego_state_mgr->planning_init_point().lat_init_state;
  // double dx = lat_init_state.x() - ref_path_points[0].path_point.x();
  // double dy = lat_init_state.y() - ref_path_points[0].path_point.y();
  const int point_nums = ref_path_points.size();
  int nearest_point_index = 0;
  double accumulate_distance_for_nearest_point = 0;
  double accumulate_distance_reference_path = ref_path_points[0].path_point.s();
  double min_distance_square_to_ego_point = dx * dx + dy * dy;
  double ego_drive_distance = ego_state_mgr->ego_drive_distance();
  // find nearest point
  for (int i = 1; i < point_nums; i++) {
    const auto &cur_point = ref_path_points[i].path_point;
    const auto &pre_point = ref_path_points[i - 1].path_point;
    accumulate_distance_reference_path += std::hypotf(
        pre_point.x() - cur_point.x(), pre_point.y() - cur_point.y());
    if (std::fabs(accumulate_distance_reference_path - ego_drive_distance) >
        12.0) {
      continue;
    }
    dx = ego_pose.x - cur_point.x();
    dy = ego_pose.y - cur_point.y();
    double temp_min_distance_square_to_ego_point = dx * dx + dy * dy;
    if (temp_min_distance_square_to_ego_point <
        min_distance_square_to_ego_point) {
      nearest_point_index = i;
      accumulate_distance_for_nearest_point =
          accumulate_distance_reference_path;
      min_distance_square_to_ego_point = temp_min_distance_square_to_ego_point;
    }
  }
  ego_state_mgr->set_ego_drive_distance(
      ref_path_points[nearest_point_index].path_point.s());
  // calculate ego projection distance in reference path
  // const auto& nearest_point =
  // ref_path_points[nearest_point_index].path_point; dx = ego_pose.x() -
  // nearest_point.x(); dy = ego_pose.y() - nearest_point.y(); const double
  // projection_length = dx * std::cos(nearest_point.theta()) +
  //                                  dy * std::sin(nearest_point.theta());
  // const double ego_projection_distance_in_reference_path =
  //     projection_length + accumulate_distance_for_nearest_point;
  return nearest_point_index;
}

double LaneReferencePath::CalculateExtendedReferencePathLength(
    const double curr_ref_path_length, const double curr_ego_proj_length,
    const ReferencePathPoints &curr_ref_path_points) {
  constexpr double kTargetSlotExtendedBuffer = 5.0;
  // TODO(taolu):
  // 待在感知到目标车位之前可以从地图拿到目标车位的位置，可以再把这个参数调回 30
  constexpr double kDefaultExtendedReferencePathLength = 0.0;

  double res_ref_path_length = curr_ref_path_length;
  const auto &parking_slot_manager =
      session_->environmental_model().get_parking_slot_manager();
  if (parking_slot_manager->IsExistTargetSlot()) {
    const auto target_slot_point = parking_slot_manager->GetTargetSlotCenter();
    const auto target_slot_proj_s =
        CalculatePointProjectionDistanceInReferencePath(target_slot_point,
                                                        curr_ref_path_points);
    res_ref_path_length = std::max(
        curr_ref_path_length, target_slot_proj_s + kTargetSlotExtendedBuffer);
  } else {
    res_ref_path_length =
        std::max(curr_ref_path_length,
                 curr_ego_proj_length + kDefaultExtendedReferencePathLength);
  }
  return res_ref_path_length;
}
}  // namespace planning

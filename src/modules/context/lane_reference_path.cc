#include "lane_reference_path.h"
#include <openssl/evp.h>
#include <sys/param.h>
#include <cmath>

#include "ifly_time.h"
#include "obstacle_manager.h"
#include "session.h"
#include "virtual_lane_manager.h"

namespace planning {

using namespace planning_math;

LaneReferencePath::LaneReferencePath(int target_lane_virtual_id)
    : ReferencePath() {
  lane_virtual_id_ = target_lane_virtual_id;
  // update();
  LOG_DEBUG("construct lane_reference_path: target_lane_virtual_id: %d\n",
            target_lane_virtual_id);
}

void LaneReferencePath::update(planning::framework::Session *session) {
  LOG_DEBUG("update LaneReferencePath\n");
  session_ = session;
  // Step 1) import reference_path pointer to virtual_lane
  auto virtual_lane = session->mutable_environmental_model()
                          ->mutable_virtual_lane_manager()
                          ->mutable_lane_with_virtual_id(lane_virtual_id_);
  std::cout << "get id " << lane_virtual_id_ << std::endl;
  virtual_lane->update_reference_path(shared_from_this());

  // Step 2) get reference_points
  ReferencePathPoints raw_reference_path_points;
  bool ok = get_ref_points(raw_reference_path_points);

  // Step 3) update
  if (ok) {
    auto current_time = IflyTime::Now_ms();
    update_refpath_points(raw_reference_path_points);
    valid_ = refined_ref_path_points_.size() >= 3;
    if (!valid_) {
      return;
    }
    auto end_time = IflyTime::Now_ms();
    LOG_DEBUG("update_refpath_points time:%f\n", end_time - current_time);
    frenet_ego_state_.update(
        frenet_coord_,
        *session_->mutable_environmental_model()->get_ego_state_manager());
    update_obstacles();

    // Step 4) update virtual_lane speed_limit
    virtual_lane->update_speed_limit(
        session->environmental_model().get_ego_state_manager()->ego_v(),
        session->environmental_model().get_ego_state_manager()->ego_v_cruise());
  } else {
    LOG_DEBUG("LaneReferencePath::update failed");
  }
}

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
  obstacle_manager->generate_frenet_obstacles(*this);
  assign_obstacles_to_lane();
  parking_spaces_ = obstacle_manager->get_parking_space().Items();
  free_space_ground_lines_ =
      obstacle_manager->get_groundline_obstacles().Items();
  road_edges_ = obstacle_manager->get_road_edge_obstacles().Items();
}

bool LaneReferencePath::get_ref_points(ReferencePathPoints &ref_path_points) {
  auto virtual_lane_manager =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  auto virtual_lane =
      virtual_lane_manager->get_lane_with_virtual_id(lane_virtual_id_);
  auto &lane_points = virtual_lane->lane_points();
  std::cout << "lane_points.size(): " << lane_points.size() << std::endl;
  const double width = virtual_lane->width();
  ref_path_points.clear();
  auto is_enu_valid = session_->environmental_model().location_valid();
  for (auto &refline_pt : lane_points) {
    constexpr double kDefaultLaneBorderDis = 20.0;
    ReferencePathPoint ref_path_pt;
    if (is_enu_valid) {
      ref_path_pt.path_point.x = refline_pt.local_point.x;
      ref_path_pt.path_point.y = refline_pt.local_point.y;
      ref_path_pt.path_point.z = refline_pt.local_point.z;
      ref_path_pt.path_point.theta = refline_pt.enu_heading;

    } else {
      ref_path_pt.path_point.x = refline_pt.car_point.x;
      ref_path_pt.path_point.y = refline_pt.car_point.y;
      ref_path_pt.path_point.z = 0;
      ref_path_pt.path_point.theta = refline_pt.car_heading;
    }
    ref_path_pt.path_point.kappa = refline_pt.curvature;
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
      Vec2d delta{ref_path_pt.path_point.x - pre_pt.path_point.x,
                  ref_path_pt.path_point.y - pre_pt.path_point.y};
      Vec2d cur_direction =
          Vec2d::CreateUnitVec2d(ref_path_pt.path_point.theta);
      if (cur_direction.InnerProd(delta) < 0) {
        // temporaly skip direction check since input data is bad @clren
        // continue;
      }
    }
    ref_path_points.emplace_back(std::move(ref_path_pt));
  }
  //判断参考线的长度是否大于自车5s的行驶距离，如果不够的话，则延长参考线
  // calculate reference path origin total length
  double origin_reference_path_total_length = 0;
  for (int i = 1; i < ref_path_points.size(); i++) {
    const auto &cur_point = ref_path_points[i].path_point;
    const auto &pre_point = ref_path_points[i - 1].path_point;
    origin_reference_path_total_length +=
        std::hypotf(pre_point.x - cur_point.x, pre_point.y - cur_point.y);
  }
  origin_reference_path_length_ = origin_reference_path_total_length;
  const bool is_highway =
      session_->get_scene_type() == planning::common::SceneType::HIGHWAY;
  if (ref_path_points.size() >= 2 && is_highway) {
    const std::shared_ptr<EgoStateManager> ego_state_mgr =
        session_->mutable_environmental_model()->get_ego_state_manager();
    const double ego_v = ego_state_mgr->ego_v();
    const double cruise_v = ego_state_mgr->ego_v_cruise();
    const double preview_dis = std::fmax(ego_v, cruise_v) * 5;
    const double extend_buff = 5;
    const double ego_projection_length_in_reference_path =
        CalculateEgoProjectionDistanceInReferencePath(ref_path_points);
    // if need to extend reference path length
    if (preview_dis + ego_projection_length_in_reference_path >
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
  lane_leadone_obstacle_ = -1;  //需要注意id是否为负数 todo
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
  if (frenet_obstacle->rel_s() > 120 || frenet_obstacle->rel_s() < -50 ||
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
  std::array<double, 3> xp_vs{0., 3., 5.};
  std::array<double, 3> fp_l1{std::max(lane_width / 2 - 1.45, 0.0),
                              std::max(lane_width / 2 - 1.65, 0.0), 0.0};

  // 可以根据不同的fusion_source障碍物的横向速度的准确度选择不同的逻辑
  if (1) {
    std::array<double, 5> xp_rel_s{-30, -20, 20, 30, 60};
    std::array<double, 5> fp_l2{0.45, 0.2, 0.3, 0.45, 0.6};
    check_offset = interp(frenet_obstacle->rel_s(), xp_rel_s, fp_l2) +
                   interp(frenet_obstacle->frenet_velocity_s(), xp_vs, fp_l1);

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
  double dx = p2.path_point.x - p1.path_point.x;
  double dy = p2.path_point.y - p1.path_point.y;
  // 计算直线长度
  double line_length = sqrt(dx * dx + dy * dy);
  // 将方向向量归一化
  dx /= line_length;
  dy /= line_length;
  // 计算延长后的点坐标
  ReferencePathPoint extend_point;
  extend_point.path_point.x = p2.path_point.x + dx * length;
  extend_point.path_point.y = p2.path_point.y + dy * length;

  const auto &last_point = p2;
  extend_point.path_point.set_z(last_point.path_point.z);
  extend_point.path_point.set_theta(last_point.path_point.theta);
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
  const auto &lat_init_state = ego_state_mgr->planning_init_point().lat_init_state;
  double dx = lat_init_state.x() - ref_path_points[0].path_point.x;
  double dy = lat_init_state.y() - ref_path_points[0].path_point.y;
  const int point_nums = ref_path_points.size();
  int nearest_point_index = 0;
  double accumulate_distance_for_nearest_point = 0;
  double accumulate_distance_reference_path = 0;
  double min_distance_square_to_ego_point = dx * dx + dy * dy;
  // find nearest point
  for (int i = 1; i < point_nums; i++) {
    const auto &cur_point = ref_path_points[i].path_point;
    const auto &pre_point = ref_path_points[i - 1].path_point;
    accumulate_distance_reference_path +=
        std::hypotf(pre_point.x - cur_point.x, pre_point.y - cur_point.y);
    dx = lat_init_state.x() - cur_point.x;
    dy = lat_init_state.y() - cur_point.y;
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
  dx = lat_init_state.x() - nearest_point.x;
  dy = lat_init_state.y() - nearest_point.y;
  const double projection_length =
      dx * std::cos(nearest_point.theta) + dy * std::sin(nearest_point.theta);
  const double ego_projection_distance_in_reference_path =
      projection_length + accumulate_distance_for_nearest_point;
  return ego_projection_distance_in_reference_path;
}

}  // namespace planning

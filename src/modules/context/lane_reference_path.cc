#include "lane_reference_path.h"

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
  bool ok = get_points_by_lane_id(lane_virtual_id_, raw_reference_path_points);

  // Step 3) update
  if (ok) {
    auto current_time = IflyTime::Now_ms();
    update_refpath_points(raw_reference_path_points, false);
    auto end_time = IflyTime::Now_ms();
    LOG_DEBUG("update_refpath_points time:%f\n", end_time - current_time);
    frenet_ego_state_.update(
        frenet_coord_,
        *session_->mutable_environmental_model()->get_ego_state_manager());
    update_obstacles();
    valid_ = true;

    // Step 4) update virtual_lane speed_limit
    virtual_lane->update_speed_limit(
        session->environmental_model().get_ego_state_manager()->ego_v(),
        session->environmental_model().get_ego_state_manager()->ego_v_cruise());
  } else {
    LOG_DEBUG("LaneReferencePath::update failed");
  }
}

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

bool LaneReferencePath::get_points_by_lane_id(
    int target_lane_virtual_id, ReferencePathPoints &ref_path_points) {
  auto virtual_lane_manager =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  auto virtual_lane =
      virtual_lane_manager->get_lane_with_virtual_id(target_lane_virtual_id);
  if (virtual_lane == nullptr) {
    return false;
  }
  auto &lane_points = virtual_lane->lane_points();
  std::cout << "lane_points.size(): " << lane_points.size() << std::endl;

  ref_path_points.clear();
  auto is_enu_valid = session_->environmental_model().location_valid();
  for (auto &refline_pt : lane_points) {
    constexpr double kDefaultLaneBorderDis = 20.0;
    ReferencePathPoint ref_path_pt;
    if (is_enu_valid) {
      ref_path_pt.path_point.x = refline_pt.local_point().x();
      ref_path_pt.path_point.y = refline_pt.local_point().y();
      ref_path_pt.path_point.z = refline_pt.local_point().z();
      ref_path_pt.path_point.theta = refline_pt.enu_heading();
      ref_path_pt.path_point.kappa = refline_pt.enu_curvature();
    } else {
      ref_path_pt.path_point.x = refline_pt.car_point().x();
      ref_path_pt.path_point.y = refline_pt.car_point().y();
      ref_path_pt.path_point.theta = refline_pt.heading();
      ref_path_pt.path_point.kappa = refline_pt.curvature();
      ref_path_pt.path_point.z = 0;
    }

    ref_path_pt.distance_to_left_lane_border = std::fmin(
        refline_pt.distance_to_left_lane_border(), kDefaultLaneBorderDis);
    ref_path_pt.distance_to_right_lane_border = std::fmin(
        refline_pt.distance_to_right_lane_border(), kDefaultLaneBorderDis);
    ref_path_pt.distance_to_left_road_border = std::fmin(
        refline_pt.distance_to_left_road_border(), kDefaultLaneBorderDis);
    ref_path_pt.distance_to_right_road_border = std::fmin(
        refline_pt.distance_to_right_road_border(), kDefaultLaneBorderDis);
    ref_path_pt.left_road_border_type = refline_pt.left_road_border_type();
    ref_path_pt.right_road_border_type = refline_pt.right_road_border_type();
    ref_path_pt.left_lane_border_type = refline_pt.left_lane_border_type();
    ref_path_pt.right_lane_border_type = refline_pt.right_lane_border_type();
    ref_path_pt.lane_width = refline_pt.lane_width();
    ref_path_pt.max_velocity = refline_pt.speed_limit();
    ref_path_pt.type = ReferencePathPointType::MAP;
    ref_path_pt.is_in_intersection = refline_pt.is_in_intersection();

    // check direction
    if (not ref_path_points.empty()) {
      const auto &pre_pt = ref_path_points.back();
      Vec2d delta{ref_path_pt.path_point.x - pre_pt.path_point.x,
                  ref_path_pt.path_point.y - pre_pt.path_point.y};
      Vec2d cur_direction =
          Vec2d::CreateUnitVec2d(ref_path_pt.path_point.theta);
      if (cur_direction.InnerProd(delta) < 0) {
        continue;
      }
    }
    ref_path_points.emplace_back(std::move(ref_path_pt));
  }

  return ref_path_points.size() >= 3;
}

void LaneReferencePath::assign_obstacles_to_lane() {
  const double HALF_LANE_WIDTH = 1.8;
  lane_obstacles_id_.clear();
  lane_leadone_obstacle_ = -1;  //需要注意id是否为负数 todo
  lane_leadtwo_obstacle_ = -1;

  std::vector<std::shared_ptr<FrenetObstacle>> sorted_obstacles;
  for (auto &frenet_obstacle : frenet_obstacles_) {
    ReferencePathPoint point;
    // Get the width of the lane at the obstacle's position
    bool valid = get_reference_point_by_lon(frenet_obstacle->frenet_s(), point);
    auto half_lane_width =
        valid == true ? (point.lane_width / 2) : HALF_LANE_WIDTH;
    if (std::fabs(frenet_obstacle->frenet_l()) < half_lane_width) {
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
    if (frenet_obstacle->frenet_s() > s_ego && leadone == -1) {
      leadone = frenet_obstacle->id();
      continue;
    }
    if (frenet_obstacle->frenet_s() > s_ego && leadone != -1 && leadtwo == -1) {
      leadtwo = frenet_obstacle->id();
      break;
    }
  }
  lane_leadone_obstacle_ = leadone;
  lane_leadtwo_obstacle_ = leadtwo;
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

}  // namespace planning

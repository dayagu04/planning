#include "modules/context/lane_reference_path.h"

#include "framework/session.h"
#include "modules/context/obstacle_manager.h"
#include "modules/context/virtual_lane_manager.h"

namespace planning {


using namespace planning_math;

LaneReferencePath::LaneReferencePath(int target_lane_virtual_id) : ReferencePath() {
  target_lane_virtual_id_ = target_lane_virtual_id;
  // update();
  LOG_DEBUG("lane_reference_path: target_lane_virtual_id: %d", target_lane_virtual_id);
}

void LaneReferencePath::update(planning::framework::Session *session) {
  session_ = session;
  // Step 1) get reference_points
  ReferencePathPoints points;
  bool ok = get_points_by_lane_id(target_lane_virtual_id_, points);

  // Step 2) update
  if (ok) {
    update_refpath_points(points);
    frenet_ego_state_.update(
        frenet_coord_,
        *session_->mutable_environmental_model()->ego_state_manager());
    update_obstacles();
    valid_ = true;
  } else {
    LOG_DEBUG("LaneReferencePath::update failed");
  }
}

bool LaneReferencePath::is_obstacle_ignorable(const FrenetObstacle &obstacle) {
  bool res{false};
  bool is_obstacle_right_behind_ego{false};
  if (obstacle.frenet_obstacle_boundary().s_end <
      frenet_ego_state_.boundary().s_start) {
    if (obstacle.frenet_l() > frenet_ego_state_.boundary().l_start &&
        obstacle.frenet_l() < frenet_ego_state_.boundary().l_end) {
      is_obstacle_right_behind_ego = true;
    }
  }

  if (!is_obstacle_right_behind_ego) {
    return res;
  }

  bool is_obstacle_current_on_lane{false};
  ReferencePathPoint obstacle_matched_lane_point;
  if (get_reference_point_by_lon(obstacle.frenet_s(),
                                 obstacle_matched_lane_point)) {
    if (obstacle.frenet_obstacle_boundary().l_start >
            -obstacle_matched_lane_point.distance_to_right_lane_border &&
        obstacle.frenet_obstacle_boundary().l_end <
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
      session_->mutable_environmental_model()->obstacle_manager();
  frenet_obstacles_ = obstacle_manager->get_reference_path_obstacles(*this);
  frenet_obstacles_map_ =
      obstacle_manager->get_reference_path_obstacles_map(*this);

  parking_spaces_ = obstacle_manager->get_parking_space().Items();
  free_space_ground_lines_ =
      obstacle_manager->get_groundline_obstacles().Items();
  road_edges_ = obstacle_manager->get_road_edge_obstacles().Items();
}

bool LaneReferencePath::get_points_by_lane_id(
    int target_lane_virtual_id, ReferencePathPoints &ref_path_points) {
  auto virtual_lane_manager =
      session_->mutable_environmental_model()->virtual_lane_manager();
  auto virtual_lane =
      virtual_lane_manager->get_lane_with_virtual_id(target_lane_virtual_id);
  if (virtual_lane == nullptr) {
    return false;
  }
  auto &lane_points = virtual_lane->lane_points();

  ref_path_points.clear();
  for (auto &refline_pt : lane_points) {
    constexpr double kDefaultLaneBorderDis = 20.0;
    ReferencePathPoint ref_path_pt;
    ref_path_pt.enu_point = Point3D{
        refline_pt.enu_point.x, refline_pt.enu_point.y, refline_pt.enu_point.z};
    ref_path_pt.curvature = refline_pt.curvature;
    ref_path_pt.yaw = refline_pt.yaw;
    ref_path_pt.distance_to_left_lane_border = std::fmin(
        refline_pt.distance_to_left_lane_border, kDefaultLaneBorderDis);
    ref_path_pt.distance_to_right_lane_border = std::fmin(
        refline_pt.distance_to_right_lane_border, kDefaultLaneBorderDis);
    ref_path_pt.distance_to_left_road_border = std::fmin(
        refline_pt.distance_to_left_road_border, kDefaultLaneBorderDis);
    ref_path_pt.distance_to_right_road_border = std::fmin(
        refline_pt.distance_to_right_road_border, kDefaultLaneBorderDis);
    ref_path_pt.left_road_border_type = refline_pt.left_road_border_type;
    ref_path_pt.right_road_border_type = refline_pt.right_road_border_type;
    ref_path_pt.left_lane_border_type = refline_pt.left_lane_border_type;
    ref_path_pt.right_lane_border_type = refline_pt.right_lane_border_type;
    ref_path_pt.type = ReferencePathPointType::MAP;
    ref_path_pt.is_in_intersection = refline_pt.is_in_intersection;

    // check direction
    if (not ref_path_points.empty()) {
      const auto &pre_pt = ref_path_points.back();
      Vec2d delta{ref_path_pt.enu_point.x - pre_pt.enu_point.x,
                  ref_path_pt.enu_point.y - pre_pt.enu_point.y};
      Vec2d cur_direction = Vec2d::CreateUnitVec2d(ref_path_pt.yaw);
      if (cur_direction.InnerProd(delta) < 0) {
        continue;
      }
    }
    ref_path_points.emplace_back(std::move(ref_path_pt));
  }

  return ref_path_points.size() >= 3;
}


}  // namespace planning

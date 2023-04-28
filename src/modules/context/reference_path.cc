#include "modules/context/reference_path.h"
#include <cmath>
#include "framework/session.h"
#include "modules/context/ego_state_manager.h"
#include "modules/context/obstacle_manager.h"
#include "modules/common/math/math_utils.h"
#include "common/ifly_time.h"

namespace planning {

ReferencePath::ReferencePath() { init(); }

void ReferencePath::init() {
  valid_ = false;

  // init frenet parameters
  frenet_parameters_.zero_speed_threshold = 0.1;
  frenet_parameters_.coord_transform_precision = 0.01;
  frenet_parameters_.step_s = 0.3;
  frenet_parameters_.coarse_step_s = 1.0;
  frenet_parameters_.optimization_gamma = 0.5;
  frenet_parameters_.max_iter = 15;
}

void ReferencePath::update(planning::framework::Session *session) {
  session_ = session;
  // Step 1) update ego state
  frenet_ego_state_.update(
      frenet_coord_, *session_->mutable_environmental_model()->get_ego_state_manager());

  // Step 2) update obstacles
  update_obstacles();
}

void ReferencePath::update_obstacles() {
  auto obstacle_manager =
      session_->mutable_environmental_model()->get_obstacle_manager();
  obstacle_manager->generate_frenet_obstacles(*this);
}

void ReferencePath::update_refpath_points(const ReferencePathPoints &points) {
  if (points.size() <= 2) {
    LOG_ERROR("update_refpath_points: points size < 2");
    return;
  }

  // Step 1) reset coord system
  std::vector<Point2D> coord_points;
  for (auto &ref_path_pt : points) {
    if (std::isnan(ref_path_pt.enu_point.x) ||
        std::isnan(ref_path_pt.enu_point.y)) {
      LOG_ERROR("update_refpath_points: skip NaN point");
      continue;
    }

    auto pt = Point2D(ref_path_pt.enu_point.x, ref_path_pt.enu_point.y);
    if (not coord_points.empty()) {
      auto &last_pt = coord_points.back();
      if (planning_math::Vec2d(last_pt.x - pt.x, last_pt.y - pt.y).Length() <
          1e-2) {
        continue;
      }
    }

    coord_points.emplace_back(pt);
  }
  frenet_coord_ = std::make_shared<FrenetCoordinateSystem>(coord_points,
                                                           frenet_parameters_);

  // Step 2) update frenet points
  points_.clear();
  for (auto pt : points) {
    if (std::isnan(pt.enu_point.x) || std::isnan(pt.enu_point.y)) {
      continue;
    }
    if (frenet_coord_->CartCoord2FrenetCoord(
            Point2D(pt.enu_point.x, pt.enu_point.y), pt.frenet_point) ==
        TRANSFORM_SUCCESS) {
      if (not points_.empty() and
          pt.frenet_point.x < points_.back().frenet_point.x) {
        continue;
      }
      pt.curvature = frenet_coord_->GetRefCurveCurvature(pt.frenet_point.x);
      pt.yaw = frenet_coord_->GetRefCurveHeading(pt.frenet_point.x);
      points_.push_back(pt);
    }
  }
}

bool ReferencePath::is_obstacle_ignorable(const std::shared_ptr<FrenetObstacle> obstacle) {
  bool res{false};
  if (obstacle->frenet_obstacle_boundary().s_end <
      frenet_ego_state_.boundary().s_start) {
    if (obstacle->frenet_l() > frenet_ego_state_.boundary().l_start &&
        obstacle->frenet_l() < frenet_ego_state_.boundary().l_end) {
      res = true;
    }
  }
  return res;
}

bool ReferencePath::get_reference_point_by_lon(
    double s, ReferencePathPoint &reference_path_point) const {
  if (std::isnan(s) || s < points_.front().frenet_point.x ||
      s > points_.back().frenet_point.x) {
    return false;
  }

  size_t pos_idx = 0;
  while (pos_idx < points_.size() - 1) {
    if (points_[pos_idx].frenet_point.x <= s and
        s <= points_[pos_idx + 1].frenet_point.x) {
      break;
    }
    pos_idx++;
  }

  auto &pre_reference_point = points_[pos_idx];
  auto &next_reference_point = points_[pos_idx + 1];

  auto interpolate_ratio = (next_reference_point.frenet_point.x - s) /
                           (next_reference_point.frenet_point.x -
                            pre_reference_point.frenet_point.x);

  reference_path_point.distance_to_left_lane_border = planning_math::interpolate(
      pre_reference_point.distance_to_left_lane_border,
      next_reference_point.distance_to_left_lane_border, interpolate_ratio);
  reference_path_point.distance_to_right_lane_border = planning_math::interpolate(
      pre_reference_point.distance_to_right_lane_border,
      next_reference_point.distance_to_right_lane_border, interpolate_ratio);
  reference_path_point.distance_to_left_road_border = planning_math::interpolate(
      pre_reference_point.distance_to_left_road_border,
      next_reference_point.distance_to_left_road_border, interpolate_ratio);
  reference_path_point.distance_to_right_road_border = planning_math::interpolate(
      pre_reference_point.distance_to_right_road_border,
      next_reference_point.distance_to_right_road_border, interpolate_ratio);
  reference_path_point.enu_point.x =
      planning_math::interpolate(pre_reference_point.enu_point.x,
                  next_reference_point.enu_point.x, interpolate_ratio);
  reference_path_point.enu_point.y =
      planning_math::interpolate(pre_reference_point.enu_point.y,
                  next_reference_point.enu_point.y, interpolate_ratio);
  reference_path_point.enu_point.z =
      planning_math::interpolate(pre_reference_point.enu_point.z,
                  next_reference_point.enu_point.z, interpolate_ratio);
  reference_path_point.curvature =
      planning_math::interpolate(pre_reference_point.curvature, next_reference_point.curvature,
                  interpolate_ratio);
  reference_path_point.yaw = planning_math::InterpolateAngle(
      pre_reference_point.yaw, next_reference_point.yaw, interpolate_ratio);

  reference_path_point.left_road_border_type =
      next_reference_point.left_road_border_type;
  reference_path_point.right_road_border_type =
      next_reference_point.right_road_border_type;
  reference_path_point.left_lane_border_type =
      next_reference_point.left_lane_border_type;
  reference_path_point.right_lane_border_type =
      next_reference_point.right_lane_border_type;

  reference_path_point.frenet_point = {s, 0};
  reference_path_point.type = ReferencePathPointType::INTERPOLATE;

  return true;
}

bool ReferencePath::transform_trajectory_points(
    TrajectoryPoints &trajectory_points) const {
  bool ok = true;
  for (auto &traj_pt : trajectory_points) {
    auto success = transform_trajectory_point(traj_pt);
    if (not success) {
      ok = false;
    }
  }
  return ok;
}

bool ReferencePath::transform_trajectory_point(TrajectoryPoint &traj_pt) const {
  if (std::isnan(traj_pt.x) || std::isnan(traj_pt.y)) {
    traj_pt.frenet_valid = false;
    return false;
  }
  Point2D frenet_point;
  auto success =
      frenet_coord_->CartCoord2FrenetCoord(Point2D{traj_pt.x, traj_pt.y},
                                           frenet_point) == TRANSFORM_SUCCESS;
  if (success) {
    traj_pt.s = frenet_point.x;
    traj_pt.l = frenet_point.y;
    traj_pt.frenet_valid = true;
  }
  return success;
}

}  // namespace planning

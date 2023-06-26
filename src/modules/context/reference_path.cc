#include "reference_path.h"
#include <cmath>
#include "ego_state_manager.h"
#include "ifly_time.h"
#include "math/math_utils.h"
#include "obstacle_manager.h"
#include "session.h"
#include "vec2d.h"
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
      frenet_coord_,
      *session_->mutable_environmental_model()->get_ego_state_manager());

  // Step 2) update obstacles
  update_obstacles();
}

void ReferencePath::update_obstacles() {
  auto obstacle_manager =
      session_->mutable_environmental_model()->get_obstacle_manager();
  obstacle_manager->generate_frenet_obstacles(*this);
}

void ReferencePath::update_refpath_points(
    ReferencePathPoints &raw_ref_path_points, bool is_need_density) {
  if (raw_ref_path_points.size() <= 2) {
    LOG_ERROR("update_refpath_points: points size < 2");
    return;
  }
  const double interp_gap_s = 2.0;
  // Step 1) update refined path points
  refined_ref_path_points_ = raw_ref_path_points;
  if (is_need_density) {
    densifying_refined_path_points(refined_ref_path_points_);
  }

  // Step 2) reset coord system from refined_ref_path_points_
  std::vector<Point2D> coord_points;
  for (auto iter = refined_ref_path_points_.begin();
       iter != refined_ref_path_points_.end();) {
    if (std::isnan(iter->path_point.x) || std::isnan(iter->path_point.y)) {
      LOG_ERROR("update_refpath_points: skip NaN point");
      refined_ref_path_points_.erase(iter);
      continue;
    }
    auto pt = Point2D(iter->path_point.x, iter->path_point.y);
    if (not coord_points.empty()) {
      auto &last_pt = coord_points.back();
      if (planning_math::Vec2d(last_pt.x - pt.x, last_pt.y - pt.y).Length() <
          1e-2) {
        iter++;
        continue;
      }
    }
    coord_points.emplace_back(pt);
    iter++;
  }
  // 需要检查coord_points数量是否满足要求，  frenet_coord_是否构建成功
  frenet_coord_ = std::make_shared<FrenetCoordinateSystem>(coord_points,
                                                           frenet_parameters_);

  // Step 3) 1. update raw_ref_path_points' frenet points by frenet_coord_
  //         2. if s > interp_gap_s, it need interpolate
  for (auto iter = raw_ref_path_points.begin();
       iter != raw_ref_path_points.end();) {
    if (std::isnan(iter->path_point.x) || std::isnan(iter->path_point.y)) {
      LOG_ERROR("raw_ref_path_points: skip NaN point");
      raw_ref_path_points.erase(iter);
      continue;
    }
    Point2D frenet_point;
    if (frenet_coord_->CartCoord2FrenetCoord(
            Point2D(iter->path_point.x, iter->path_point.y), frenet_point) ==
        TRANSFORM_SUCCESS) {
      if (iter != raw_ref_path_points.begin()) {
        iter->path_point.s = frenet_point.x;
        iter->path_point.kappa =
            frenet_coord_->GetRefCurveCurvature(frenet_point.x);
        iter->path_point.theta =
            frenet_coord_->GetRefCurveHeading(frenet_point.x);
        double dist = iter->path_point.s - (iter - 1)->path_point.s;
        if (dist < 0) {
          LOG_ERROR("raw_ref_path_points frenet x is smaller");
          raw_ref_path_points.erase(iter);
          continue;
        } else if (
            dist >
            interp_gap_s) {  //检验原始点的密度，如果点距离大于interp_gap_s，需要插值加密
          int num_point = ceil(dist / interp_gap_s - 1);
          double interpolate_s = dist / (num_point + 1);
          ReferencePathPoints ref_path_pts;
          for (double dis = (iter - 1)->path_point.s;
               dis < iter->path_point.s;) {
            ReferencePathPoint ref_path_pt;
            dis += interpolate_s;
            get_reference_point_by_lon_from_raw_ref_path_points(
                dis, raw_ref_path_points, ref_path_pt);
            ref_path_pts.emplace_back(std::move(ref_path_pt));
          }
          iter = raw_ref_path_points.insert(iter, ref_path_pts.begin(),
                                            ref_path_pts.end());
          iter += ref_path_pts.size();
        }
      }
    } else {
      LOG_ERROR("trasform raw_ref_path_points frenet Failed");
      raw_ref_path_points.erase(iter);
      continue;
    }
    iter++;
  }

  // Step 4) update refined_ref_path_points_ again
  if (is_need_density) {
    for (auto iter = refined_ref_path_points_.begin();
         iter != refined_ref_path_points_.end();) {
      if (std::isnan(iter->path_point.x) || std::isnan(iter->path_point.y)) {
        refined_ref_path_points_.erase(iter);
        continue;
      }
      if (iter != refined_ref_path_points_.begin() &&
          iter->path_point.s > (iter - 1)->path_point.s) {
        refined_ref_path_points_.erase(iter);
        continue;
      }
      get_reference_point_by_lon_from_raw_ref_path_points(
          iter->path_point.s, raw_ref_path_points, *iter);
      iter->path_point.kappa =
          frenet_coord_->GetRefCurveCurvature(iter->path_point.s);
      iter->path_point.theta =
          frenet_coord_->GetRefCurveHeading(iter->path_point.s);
      // check direction
      if (iter != refined_ref_path_points_.begin()) {
        planning_math::Vec2d delta{
            iter->path_point.x - (iter - 1)->path_point.x,
            iter->path_point.y - (iter - 1)->path_point.y};
        planning_math::Vec2d cur_direction =
            planning_math::Vec2d::CreateUnitVec2d(iter->path_point.theta);
        if (cur_direction.InnerProd(delta) < 0) {
          refined_ref_path_points_.erase(iter);
          continue;
        }
      }
      ++iter;
    }
  } else {
    refined_ref_path_points_ = raw_ref_path_points;
  }
}

void ReferencePath::densifying_refined_path_points(
    ReferencePathPoints &refined_ref_path_points) {
  if (refined_ref_path_points.size() < 2) {
    return;
  }

  //对原始点进行加密
  double interp_gap_s = 0.5;
  std::vector<double> u_points{0};
  std::vector<double> x_points, y_points;
  for (auto &ref_path_point : refined_ref_path_points) {
    x_points.push_back(ref_path_point.path_point.x);
    y_points.push_back(ref_path_point.path_point.y);
  }
  for (size_t i = 0; i + 1 < x_points.size(); i++) {
    double dist = std::sqrt(std::pow(x_points[i + 1] - x_points[i], 2) +
                            std::pow(y_points[i + 1] - y_points[i], 2));
    u_points.push_back(u_points[i] + dist);
  }

  planning::planning_math::spline x_spline, y_spline;
  x_spline.set_points(u_points, x_points);
  y_spline.set_points(u_points, y_points);

  std::vector<double> us;
  discrete(u_points[0], u_points.back(), interp_gap_s, us);

  refined_ref_path_points.clear();
  size_t index = 0;
  for (size_t i = 0; i < us.size(); i++) {
    ReferencePathPoint ref_path_pt;
    auto &pp = ref_path_pt.path_point;
    pp.x = x_spline(us[i]);
    pp.y = y_spline(us[i]);

    if (i == 0) {
      pp.s = 0;
    } else {
      double dist = std::sqrt(
          std::pow(pp.x - refined_ref_path_points[i - 1].path_point.x, 2) +
          std::pow(pp.y - refined_ref_path_points[i - 1].path_point.y, 2));

      pp.s = refined_ref_path_points[i - 1].path_point.s + dist;
    }

    // something strange
    for (size_t j = 0; j + 1 < u_points.size(); j++) {
      if (j < index) {
        continue;
      }

      if (us[i] >= u_points[j] && us[i] < u_points[j + 1]) {
        index = j;
        break;
      }
      index++;
    }
    refined_ref_path_points.push_back(std::move(ref_path_pt));
  }
}

bool ReferencePath::is_obstacle_ignorable(
    const std::shared_ptr<FrenetObstacle> obstacle) {
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
  if (std::isnan(s) || s < refined_ref_path_points_.front().path_point.s ||
      s > refined_ref_path_points_.back().path_point.s) {
    return false;
  }

  size_t pos_idx = 0;
  while (pos_idx < refined_ref_path_points_.size() - 1) {
    if (refined_ref_path_points_[pos_idx].path_point.s <= s and
        s <= refined_ref_path_points_[pos_idx + 1].path_point.s) {
      break;
    }
    pos_idx++;
  }

  auto &pre_reference_point = refined_ref_path_points_[pos_idx];
  auto &next_reference_point = refined_ref_path_points_[pos_idx + 1];

  auto interpolate_ratio =
      (next_reference_point.path_point.s - s) /
      (next_reference_point.path_point.s - pre_reference_point.path_point.s);

  reference_path_point.distance_to_left_lane_border =
      planning_math::Interpolate(
          pre_reference_point.distance_to_left_lane_border,
          next_reference_point.distance_to_left_lane_border, interpolate_ratio);
  reference_path_point.distance_to_right_lane_border =
      planning_math::Interpolate(
          pre_reference_point.distance_to_right_lane_border,
          next_reference_point.distance_to_right_lane_border,
          interpolate_ratio);
  reference_path_point.distance_to_left_road_border =
      planning_math::Interpolate(
          pre_reference_point.distance_to_left_road_border,
          next_reference_point.distance_to_left_road_border, interpolate_ratio);
  reference_path_point.distance_to_right_road_border =
      planning_math::Interpolate(
          pre_reference_point.distance_to_right_road_border,
          next_reference_point.distance_to_right_road_border,
          interpolate_ratio);
  reference_path_point.path_point.x = planning_math::Interpolate(
      pre_reference_point.path_point.x, next_reference_point.path_point.x,
      interpolate_ratio);
  reference_path_point.path_point.y = planning_math::Interpolate(
      pre_reference_point.path_point.y, next_reference_point.path_point.y,
      interpolate_ratio);
  reference_path_point.path_point.z = planning_math::Interpolate(
      pre_reference_point.path_point.z, next_reference_point.path_point.z,
      interpolate_ratio);
  reference_path_point.path_point.kappa = planning_math::Interpolate(
      pre_reference_point.path_point.kappa,
      next_reference_point.path_point.kappa, interpolate_ratio);
  reference_path_point.path_point.theta = planning_math::InterpolateAngle(
      pre_reference_point.path_point.theta,
      next_reference_point.path_point.theta, interpolate_ratio);

  reference_path_point.lane_width = planning_math::Interpolate(
      pre_reference_point.lane_width, next_reference_point.lane_width,
      interpolate_ratio);

  reference_path_point.max_velocity = pre_reference_point.max_velocity;

  reference_path_point.left_road_border_type =
      next_reference_point.left_road_border_type;
  reference_path_point.right_road_border_type =
      next_reference_point.right_road_border_type;
  reference_path_point.left_lane_border_type =
      next_reference_point.left_lane_border_type;
  reference_path_point.right_lane_border_type =
      next_reference_point.right_lane_border_type;

  reference_path_point.path_point.s = s;
  reference_path_point.type = ReferencePathPointType::INTERPOLATE;

  return true;
}

bool ReferencePath::get_reference_point_by_lon_from_raw_ref_path_points(
    double s, const ReferencePathPoints &raw_reference_path_point,
    ReferencePathPoint &reference_path_point) {
  if (std::isnan(s) || s < raw_reference_path_point.front().path_point.s ||
      s > raw_reference_path_point.back().path_point.s) {
    return false;
  }

  size_t pos_idx = 0;
  while (pos_idx < raw_reference_path_point.size() - 1) {
    if (raw_reference_path_point[pos_idx].path_point.s <= s and
        s <= raw_reference_path_point[pos_idx + 1].path_point.s) {
      break;
    }
    pos_idx++;
  }

  auto &pre_reference_point = raw_reference_path_point[pos_idx];
  auto &next_reference_point = raw_reference_path_point[pos_idx + 1];

  auto interpolate_ratio =
      (next_reference_point.path_point.s - s) /
      (next_reference_point.path_point.s - pre_reference_point.path_point.s);

  reference_path_point.distance_to_left_lane_border =
      planning_math::Interpolate(
          pre_reference_point.distance_to_left_lane_border,
          next_reference_point.distance_to_left_lane_border, interpolate_ratio);
  reference_path_point.distance_to_right_lane_border =
      planning_math::Interpolate(
          pre_reference_point.distance_to_right_lane_border,
          next_reference_point.distance_to_right_lane_border,
          interpolate_ratio);
  reference_path_point.distance_to_left_road_border =
      planning_math::Interpolate(
          pre_reference_point.distance_to_left_road_border,
          next_reference_point.distance_to_left_road_border, interpolate_ratio);
  reference_path_point.distance_to_right_road_border =
      planning_math::Interpolate(
          pre_reference_point.distance_to_right_road_border,
          next_reference_point.distance_to_right_road_border,
          interpolate_ratio);
  reference_path_point.path_point.x = planning_math::Interpolate(
      pre_reference_point.path_point.x, next_reference_point.path_point.x,
      interpolate_ratio);
  reference_path_point.path_point.y = planning_math::Interpolate(
      pre_reference_point.path_point.y, next_reference_point.path_point.y,
      interpolate_ratio);
  reference_path_point.path_point.z = planning_math::Interpolate(
      pre_reference_point.path_point.z, next_reference_point.path_point.z,
      interpolate_ratio);
  reference_path_point.path_point.kappa = planning_math::Interpolate(
      pre_reference_point.path_point.kappa,
      next_reference_point.path_point.kappa, interpolate_ratio);
  reference_path_point.path_point.theta = planning_math::InterpolateAngle(
      pre_reference_point.path_point.theta,
      next_reference_point.path_point.theta, interpolate_ratio);

  reference_path_point.lane_width = planning_math::Interpolate(
      pre_reference_point.lane_width, next_reference_point.lane_width,
      interpolate_ratio);

  reference_path_point.max_velocity = pre_reference_point.max_velocity;

  reference_path_point.left_road_border_type =
      next_reference_point.left_road_border_type;
  reference_path_point.right_road_border_type =
      next_reference_point.right_road_border_type;
  reference_path_point.left_lane_border_type =
      next_reference_point.left_lane_border_type;
  reference_path_point.right_lane_border_type =
      next_reference_point.right_lane_border_type;

  reference_path_point.path_point.s = s;
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

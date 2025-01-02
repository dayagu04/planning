#include "reference_path.h"

#include <cmath>
#include <cstddef>

#include "ego_state_manager.h"
#include "ifly_time.h"
#include "log.h"
#include "math/math_utils.h"
#include "obstacle_manager.h"
#include "session.h"
#include "utils/kd_path.h"
#include "utils/path_point.h"
#include "vec2d.h"
namespace planning {

ReferencePath::ReferencePath() { init(); }

void ReferencePath::init() {
  valid_ = false;

  // init frenet parameters
  // frenet_parameters_.zero_speed_threshold = 0.1;
  // frenet_parameters_.coord_transform_precision = 0.01;
  // frenet_parameters_.step_s = 0.3;
  // frenet_parameters_.coarse_step_s = 2.0;
  // frenet_parameters_.optimization_gamma = 0.5;
  // frenet_parameters_.max_iter = 15;
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
    const ReferencePathPoints &raw_ref_path_points) {
  if (raw_ref_path_points.size() <= 2) {
    LOG_ERROR("update_refpath_points: points size < 2");
    return;
  }

  // Step 1) reset coord system from refined_ref_path_points_
  std::vector<planning_math::PathPoint> coord_path_points;
  coord_path_points.reserve(raw_ref_path_points.size());
  for (const auto &point : raw_ref_path_points) {
    if (std::isnan(point.path_point.x()) || std::isnan(point.path_point.y())) {
      LOG_ERROR("update_refpath_points: skip NaN point");
      continue;
    }
    auto pt =
        planning_math::PathPoint(point.path_point.x(), point.path_point.y());
    // std ::cout << "path_point: " << pt.x() << "," << pt.y() <<std::endl;
    if (not coord_path_points.empty()) {
      auto &last_pt = coord_path_points.back();
      if (planning_math::Vec2d(last_pt.x() - pt.x(), last_pt.y() - pt.y())
              .Length() < 1e-2) {
        continue;
      }
    }
    coord_path_points.emplace_back(pt);
  }
  // 需要检查coord_points数量是否满足要求，  frenet_coord_是否构建成功
  frenet_coord_ =
      std::make_shared<planning_math::KDPath>(std::move(coord_path_points));

  // Step 2) 1. update refined_ref_path_points_' frenet points by frenet_coord_
  refined_ref_path_points_.clear();
  refined_ref_path_points_.reserve(raw_ref_path_points.size());
  for (auto pt : raw_ref_path_points) {
    if (std::isnan(pt.path_point.x()) || std::isnan(pt.path_point.y())) {
      LOG_ERROR("raw_ref_path_points: skip NaN point");
      continue;
    }
    Point2D frenet_point;
    if (frenet_coord_->XYToSL(pt.path_point.x(), pt.path_point.y(),
                              &frenet_point.x, &frenet_point.y)) {
      pt.path_point.set_s(frenet_point.x);
      if (!refined_ref_path_points_.empty() &&
          pt.path_point.s() < refined_ref_path_points_.back().path_point.s()) {
        continue;
      }
      auto kd_path_point = frenet_coord_->GetPathPointByS(frenet_point.x);
      pt.path_point.set_kappa(kd_path_point.kappa());
      pt.path_point.set_theta(kd_path_point.theta());

      refined_ref_path_points_.emplace_back(pt);
    }
  }
}

void ReferencePath::update_refpath_points_in_hpp(
    const double ego_projection_length_in_reference_path,
    const ReferencePathPoints &raw_ref_path_points) {
  if (raw_ref_path_points.size() <= 2) {
    LOG_ERROR("update_refpath_points: points size < 2");
    return;
  }

  //
  const auto &lat_init_state = session_->mutable_environmental_model()
                                   ->get_ego_state_manager()
                                   ->planning_init_point()
                                   .lat_init_state;
  const auto &v_ref_cruise = std::fmax(
      session_->environmental_model().get_ego_state_manager()->ego_v_cruise(),
      2.0);

  const float kPreviewTime = 10.0;
  const double kMinPreviewLength = 25.0;
  const double kMaxRearDistance = 25.0;
  double drop_length =
      std::max(ego_projection_length_in_reference_path - kMaxRearDistance, 0.0);
  double init_length = ego_projection_length_in_reference_path;
  double preview_length =
      std::max(v_ref_cruise * kPreviewTime, kMinPreviewLength);
  // Step 1) reset coord system from refined_ref_path_points_
  std::vector<planning_math::PathPoint> coord_path_points;
  coord_path_points.reserve(raw_ref_path_points.size());
  double ref_length = 0.0;
  size_t start_index = 0;
  size_t end_index = raw_ref_path_points.size();
  for (size_t i = 0; i < raw_ref_path_points.size(); ++i) {
    if (std::isnan(raw_ref_path_points[i].path_point.x()) ||
        std::isnan(raw_ref_path_points[i].path_point.y())) {
      LOG_ERROR("update_refpath_points: skip NaN point");
      continue;
    }
    auto pt = planning_math::PathPoint(raw_ref_path_points[i].path_point.x(),
                                       raw_ref_path_points[i].path_point.y());
    // std ::cout << "path_point: " << pt.x() << "," << pt.y() <<std::endl;
    if (i > 1) {
      const auto &last_pt = raw_ref_path_points[i - 1].path_point;
      double diff_s =
          planning_math::Vec2d(last_pt.x() - pt.x(), last_pt.y() - pt.y())
              .Length();
      ref_length += diff_s;
      if (diff_s < 1e-2) {
        continue;
      }
      // check direction
      Vec2d last_direction = Vec2d::CreateUnitVec2d(last_pt.theta());
      Vec2d cur_direction =
          Vec2d::CreateUnitVec2d(raw_ref_path_points[i].path_point.theta());
      if (cur_direction.InnerProd(last_direction) < 0) {
        if (ref_length > init_length) {
          end_index = i;
          break;
        } else {
          coord_path_points.clear();
          start_index = i;
          drop_length = std::max(drop_length, ref_length);
        }
        LOG_DEBUG("ref path direction check error since input data is bad! \n");
      }
    }
    if (ref_length >= drop_length) {
      if (ref_length <= (init_length + preview_length)) {
        end_index = i + 1;
        coord_path_points.emplace_back(pt);
      } else {
        break;
      }
    } else {
      start_index = i + 1;
    }
  }
  // 需要检查coord_points数量是否满足要求，  frenet_coord_是否构建成功
  if (coord_path_points.size() < 2) {
    LOG_DEBUG("drop_length = %f, init_length = %f\n", drop_length, init_length);
    LOG_ERROR("update_refpath_points_in_hpp: coord points size < 2");
    return;
  }
  frenet_coord_ = std::make_shared<KDPath>(std::move(coord_path_points));

  // Step 2) 1. update refined_ref_path_points_' frenet points by frenet_coord_
  refined_ref_path_points_.clear();
  refined_ref_path_points_.reserve(raw_ref_path_points.size());
  for (size_t i = start_index; i < end_index; ++i) {
    auto pt = raw_ref_path_points[i];
    if (std::isnan(pt.path_point.x()) || std::isnan(pt.path_point.y())) {
      LOG_ERROR("raw_ref_path_points: skip NaN point");
      continue;
    }
    Point2D frenet_point;
    if (frenet_coord_->XYToSL(pt.path_point.x(), pt.path_point.y(),
                              &frenet_point.x, &frenet_point.y)) {
      pt.path_point.set_s(frenet_point.x);
      if (!refined_ref_path_points_.empty() &&
          pt.path_point.s() < refined_ref_path_points_.back().path_point.s()) {
        continue;
      }
      auto kd_path_point = frenet_coord_->GetPathPointByS(frenet_point.x);
      pt.path_point.set_kappa(kd_path_point.kappa());
      pt.path_point.set_theta(kd_path_point.theta());

      refined_ref_path_points_.emplace_back(pt);
    }
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
  if (std::isnan(s) || s < refined_ref_path_points_.front().path_point.s() ||
      s > refined_ref_path_points_.back().path_point.s()) {
    return false;
  }

  /*@cailiu: use binary search*/
  size_t low = 0;
  size_t high = refined_ref_path_points_.size() - 1;
  size_t pos_idx;
  while (low <= high) {
    pos_idx = low + (high - low) / 2;
    if (refined_ref_path_points_[pos_idx].path_point.s() <= s and
        (pos_idx == refined_ref_path_points_.size() - 1 or
         s <= refined_ref_path_points_[pos_idx + 1].path_point.s())) {
      break;
    } else if (refined_ref_path_points_[pos_idx].path_point.s() < s) {
      low = pos_idx + 1;
    } else {
      high = pos_idx - 1;
    }
  }

  auto &pre_reference_point = refined_ref_path_points_[pos_idx];
  auto &next_reference_point = refined_ref_path_points_[pos_idx + 1];

  auto interpolate_ratio = (next_reference_point.path_point.s() - s) /
                           (next_reference_point.path_point.s() -
                            pre_reference_point.path_point.s());

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
  reference_path_point.path_point.set_x(planning_math::Interpolate(
      pre_reference_point.path_point.x(), next_reference_point.path_point.x(),
      interpolate_ratio));
  reference_path_point.path_point.set_y(planning_math::Interpolate(
      pre_reference_point.path_point.y(), next_reference_point.path_point.y(),
      interpolate_ratio));
  reference_path_point.path_point.set_z(planning_math::Interpolate(
      pre_reference_point.path_point.z(), next_reference_point.path_point.z(),
      interpolate_ratio));
  reference_path_point.path_point.set_kappa(planning_math::Interpolate(
      pre_reference_point.path_point.kappa(),
      next_reference_point.path_point.kappa(), interpolate_ratio));
  reference_path_point.path_point.set_theta(planning_math::InterpolateAngle(
      pre_reference_point.path_point.theta(),
      next_reference_point.path_point.theta(), interpolate_ratio));

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

  reference_path_point.path_point.set_s(s);
  reference_path_point.type = ReferencePathPointType::INTERPOLATE;

  return true;
}

bool ReferencePath::get_reference_point_by_lon_from_raw_ref_path_points(
    double s, const ReferencePathPoints &raw_reference_path_point,
    ReferencePathPoint &reference_path_point) {
  if (std::isnan(s) || s < raw_reference_path_point.front().path_point.s()) {
    return false;
  }

  size_t pos_idx = 0;
  while (pos_idx < raw_reference_path_point.size() - 1) {
    if (raw_reference_path_point[pos_idx].path_point.s() <= s and
        s <= raw_reference_path_point[pos_idx + 1].path_point.s()) {
      break;
    }
    pos_idx++;
  }

  auto &pre_reference_point = raw_reference_path_point[pos_idx];
  auto &next_reference_point = raw_reference_path_point[pos_idx + 1];

  if (pre_reference_point.path_point.s() > s ||
      next_reference_point.path_point.s() <
          s) {  // 防止未搜索到满足条件的pos_idx
    return false;
  }

  auto interpolate_ratio = (next_reference_point.path_point.s() - s) /
                           (next_reference_point.path_point.s() -
                            pre_reference_point.path_point.s());

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

  reference_path_point.path_point.set_x(planning_math::Interpolate(
      pre_reference_point.path_point.x(), next_reference_point.path_point.x(),
      interpolate_ratio));

  reference_path_point.path_point.set_y(planning_math::Interpolate(
      pre_reference_point.path_point.y(), next_reference_point.path_point.y(),
      interpolate_ratio));

  reference_path_point.path_point.set_z(planning_math::Interpolate(
      pre_reference_point.path_point.z(), next_reference_point.path_point.z(),
      interpolate_ratio));

  reference_path_point.path_point.set_kappa(planning_math::Interpolate(
      pre_reference_point.path_point.kappa(),
      next_reference_point.path_point.kappa(), interpolate_ratio));

  reference_path_point.path_point.set_theta(planning_math::InterpolateAngle(
      pre_reference_point.path_point.theta(),
      next_reference_point.path_point.theta(), interpolate_ratio));

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

  reference_path_point.path_point.set_s(s);
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
      frenet_coord_->XYToSL(Point2D{traj_pt.x, traj_pt.y}, frenet_point);
  if (success) {
    traj_pt.s = frenet_point.x;
    traj_pt.l = frenet_point.y;
    traj_pt.frenet_valid = true;
  }
  return success;
}

}  // namespace planning

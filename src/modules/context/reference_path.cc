#include "reference_path.h"

#include <cmath>
#include <cstddef>
#include "ego_state_manager.h"
#include "ifly_time.h"
#include "math/math_utils.h"
#include "obstacle_manager.h"
#include "session.h"
#include "spline_projection.h"
#include "utils/kd_path.h"
#include "utils/path_point.h"
#include "vec2d.h"
namespace planning {

const static double kLaneDropLength = 20.0;
const static double kEgoBehindLength = -10.0;
const static double kEgoAheadLength = 150.0;
const static double kMinEgoAheadLength = 90.0;
const static double kForwardSamplingGap = 5.0;
const static double kForwardSamplingStep = 2.0;
const static double kReverseEgoBehindLength = -10.0;
const static double kReverseEgoAheadLength = 20.0;
const static double kReverseSamplingGap = 1.5;
const static double kReverseSamplingStep = 0.5;
const static double kMinRefVel = 2.0;

ReferencePath::ReferencePath() { init(); }

void ReferencePath::init() {
  valid_ = false;
  frenet_coord_ = nullptr;
  // init frenet parameters
  // frenet_parameters_.zero_speed_threshold = 0.1;
  // frenet_parameters_.coord_transform_precision = 0.01;
  // frenet_parameters_.step_s = 0.3;
  // frenet_parameters_.coarse_step_s = 2.0;
  // frenet_parameters_.optimization_gamma = 0.5;
  // frenet_parameters_.max_iter = 15;
  // smooth
  is_smoothed_ = false;
  is_enable_raw_line_extend_ = false;
  is_enable_clothoid_extend_ = false;
  valid_lane_line_length_ = 0.;
  min_ahead_length_ = kMinEgoAheadLength;
  sampling_behind_length_ = kEgoBehindLength;
  sampling_ahead_length_ = kEgoAheadLength;
  sampling_gap_ = kForwardSamplingGap;
  sampling_step_ = kForwardSamplingStep;
  ref_path_curve_info_.Clear();
  smooth_input_.Clear();
  smooth_output_.Clear();
  ref_path_smoother_info_.Clear();
}

void ReferencePath::update(planning::framework::Session *session) {
  session_ = session;
  obstacles_frenet_infos_.clear();
  obstacles_frenet_infos_recurrence_.clear();
  // Step 1) update ego state
  frenet_ego_state_.update(
      frenet_coord_,
      *session_->mutable_environmental_model()->get_ego_state_manager());

  // Step 2) update obstacles
  update_obstacles();
}

void ReferencePath::update_obstacles() {
  auto obstacle_manager_ =
      session_->mutable_environmental_model()->get_obstacle_manager();
  obstacle_manager_->generate_frenet_obstacles(*this);
}

void ReferencePath::update_refpath_points(
    const ReferencePathPoints &raw_ref_path_points,
    const bool is_need_smooth) {
  if (raw_ref_path_points.size() <= 2) {
    ILOG_ERROR << "update_refpath_points: points size < 2";
    return;
  }

  // Step 1) reset coord system from refined_ref_path_points_
  std::vector<planning_math::PathPoint> coord_path_points;
  coord_path_points.reserve(raw_ref_path_points.size());
  for (const auto &point : raw_ref_path_points) {
    if (std::isnan(point.path_point.x()) || std::isnan(point.path_point.y())) {
      ILOG_ERROR << "update_refpath_points: skip NaN point";
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
    coord_path_points.emplace_back(std::move(pt));
  }
  // 需要检查coord_points数量是否满足要求，  frenet_coord_是否构建成功
  if (coord_path_points.size() < KDPath::kKDPathMinPathPointSize) {
    ILOG_ERROR << "update_refpath_points: coord points size < 2";
    return;
  }
  frenet_coord_ =
      std::make_shared<planning_math::KDPath>(std::move(coord_path_points));

  // Step 2) 1. update refined_ref_path_points_' frenet points by frenet_coord_
  refined_ref_path_points_.clear();
  refined_ref_path_points_.reserve(raw_ref_path_points.size());
  for (auto pt : raw_ref_path_points) {
    if (std::isnan(pt.path_point.x()) || std::isnan(pt.path_point.y())) {
      ILOG_ERROR << "raw_ref_path_points: skip NaN point";
      continue;
    }
    Point2D frenet_point;
    if (frenet_coord_->XYToSL(pt.path_point.x(), pt.path_point.y(),
                              &frenet_point.x, &frenet_point.y)) {
      pt.path_point.set_s(frenet_point.x);
      if (!refined_ref_path_points_.empty() &&
          pt.path_point.s() < refined_ref_path_points_.back().path_point.s() + 1e-3) {
        continue;
      }
      auto kd_path_point = frenet_coord_->GetPathPointByS(frenet_point.x);
      pt.path_point.set_kappa(kd_path_point.kappa());
      pt.path_point.set_theta(kd_path_point.theta());

      refined_ref_path_points_.emplace_back(pt);
    }
  }
  Point2D frenet_start_point;
  if (frenet_coord_->XYToSL(raw_start_point_.path_point.x(), raw_start_point_.path_point.y(),
                            &frenet_start_point.x, &frenet_start_point.y)) {
    raw_start_point_.path_point = frenet_coord_->GetPathPointByS(frenet_start_point.x);
  }
  Point2D frenet_end_point;
  if (frenet_coord_->XYToSL(raw_end_point_.path_point.x(), raw_end_point_.path_point.y(),
                            &frenet_end_point.x, &frenet_end_point.y)) {
    raw_end_point_.path_point = frenet_coord_->GetPathPointByS(frenet_end_point.x);
  }
  UpdateReferencePath(is_need_smooth);
}

void ReferencePath::update_refpath_points_in_hpp(
    const double ego_projection_length_in_reference_path,
    const ReferencePathPoints &raw_ref_path_points) {
  if (raw_ref_path_points.size() <= 2) {
    ILOG_ERROR << "update_refpath_points: points size < 2";
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
  const double kMinPreviewLength = 30.0;
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
      ILOG_ERROR << "update_refpath_points: skip NaN point";
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
      planning_math::Vec2d last_direction =
          planning_math::Vec2d::CreateUnitVec2d(last_pt.theta());
      planning_math::Vec2d cur_direction =
          planning_math::Vec2d::CreateUnitVec2d(
              raw_ref_path_points[i].path_point.theta());
      if (cur_direction.InnerProd(last_direction) < 0) {
        if (ref_length > init_length) {
          end_index = i;
          break;
        } else {
          coord_path_points.clear();
          start_index = i;
          drop_length = std::max(drop_length, ref_length);
        }
        ILOG_WARN << "ref path direction check error since input data is bad!";
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
  if (coord_path_points.size() < KDPath::kKDPathMinPathPointSize) {
    ILOG_DEBUG << "drop_length = " << drop_length
               << ", init_length = " << init_length;
    ILOG_ERROR << "update_refpath_points_in_hpp: coord points size < 2";
    return;
  }
  frenet_coord_ =
      std::make_shared<planning_math::KDPath>(std::move(coord_path_points));

  // Step 2) 1. update refined_ref_path_points_' frenet points by frenet_coord_
  refined_ref_path_points_.clear();
  refined_ref_path_points_.reserve(raw_ref_path_points.size());
  for (size_t i = start_index; i < end_index; ++i) {
    auto pt = raw_ref_path_points[i];
    if (std::isnan(pt.path_point.x()) || std::isnan(pt.path_point.y())) {
      ILOG_ERROR << "raw_ref_path_points: skip NaN point";
      continue;
    }
    Point2D frenet_point;
    if (frenet_coord_->XYToSL(pt.path_point.x(), pt.path_point.y(),
                              &frenet_point.x, &frenet_point.y)) {
      pt.path_point.set_s(frenet_point.x);
      if (!refined_ref_path_points_.empty() &&
          pt.path_point.s() < refined_ref_path_points_.back().path_point.s() + 1e-3) {
        continue;
      }
      auto kd_path_point = frenet_coord_->GetPathPointByS(frenet_point.x);
      pt.path_point.set_kappa(kd_path_point.kappa());
      pt.path_point.set_theta(kd_path_point.theta());

      refined_ref_path_points_.emplace_back(pt);
    }
  }
  Point2D frenet_start_point;
  if (frenet_coord_->XYToSL(raw_start_point_.path_point.x(), raw_start_point_.path_point.y(),
                            &frenet_start_point.x, &frenet_start_point.y)) {
    raw_start_point_.path_point = frenet_coord_->GetPathPointByS(frenet_start_point.x);
  }
  Point2D frenet_end_point;
  if (frenet_coord_->XYToSL(raw_end_point_.path_point.x(), raw_end_point_.path_point.y(),
                            &frenet_end_point.x, &frenet_end_point.y)) {
    raw_end_point_.path_point = frenet_coord_->GetPathPointByS(frenet_end_point.x);
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

bool ReferencePath::get_polygon_at_time(
    const int id, bool is_use_recurrence, const int relative_time,
    planning_math::Polygon2d &obstacle_polygon) {
  auto obstacle_manager =
      session_->mutable_environmental_model()->get_obstacle_manager();
  const auto obstacle_ptr = obstacle_manager->find_obstacle(id);
  if (obstacle_ptr == nullptr) {
    return false;
  }
  if (is_use_recurrence) {
    if (obstacles_frenet_infos_recurrence_.find(id) !=
        obstacles_frenet_infos_recurrence_.end()) {
      if (obstacles_frenet_infos_recurrence_[id].find(relative_time) !=
          obstacles_frenet_infos_recurrence_[id].end()) {
        obstacle_polygon =
            obstacles_frenet_infos_recurrence_[id][relative_time];
        return true;
      }
    }
    Point2D frenet_point, carte_point;
    double frenet_s;
    carte_point.x = obstacle_ptr->x_center();
    carte_point.y = obstacle_ptr->y_center();
    if (!frenet_coord_->XYToSL(carte_point, frenet_point) ||
        std::isnan(frenet_point.x) || std::isnan(frenet_point.y)) {
      return false;
    } else {
      frenet_s = frenet_point.x;
    }
    double curve_heading = frenet_coord_->GetPathCurveHeading(frenet_s);
    double frenet_relative_velocity_angle = planning_math::NormalizeAngle(
        obstacle_ptr->velocity_angle() - curve_heading);
    double frenet_velocity_s =
        obstacle_ptr->velocity() * std::cos(frenet_relative_velocity_angle);
    double prediction_frenet_s = frenet_velocity_s * relative_time * 0.1;
    auto enu_polygon =
        obstacle_ptr->get_polygon_at_point(obstacle_ptr->get_point_at_time(0));
    std::vector<planning_math::Vec2d> frenet_points;
    frenet_points.reserve(enu_polygon.points().size());
    for (auto &pt : enu_polygon.points()) {
      Point2D frenet_point, carte_point;
      carte_point.x = pt.x();
      carte_point.y = pt.y();
      if (!frenet_coord_->XYToSL(carte_point, frenet_point)) {
        ILOG_INFO << "obstacle id " << obstacle_ptr->id()
                  << " Frenet_coord failed!!";
        continue;
      }
      frenet_points.emplace_back(std::move(planning_math::Vec2d(
          frenet_point.x + prediction_frenet_s, frenet_point.y)));
    }
    bool ok = planning_math::Polygon2d::ComputeConvexHull(frenet_points,
                                                          &obstacle_polygon);
    if (ok) {
      obstacles_frenet_infos_recurrence_[id][relative_time] = obstacle_polygon;
    }
    return ok;
  } else {
    if (obstacles_frenet_infos_.find(id) != obstacles_frenet_infos_.end()) {
      if (obstacles_frenet_infos_[id].find(relative_time) !=
          obstacles_frenet_infos_[id].end()) {
        obstacle_polygon = obstacles_frenet_infos_[id][relative_time];
        return true;
      }
    }
    const auto &enu_polygon = obstacle_ptr->get_bounding_box(
        obstacle_ptr->get_point_at_time(relative_time * 0.1));

    const auto &corners = enu_polygon.GetAllCorners();

    std::vector<planning_math::Vec2d> frenet_points;
    frenet_points.reserve(corners.size());
    for (auto &pt : corners) {
      Point2D frenet_point, carte_point;
      carte_point.x = pt.x();
      carte_point.y = pt.y();
      if (!frenet_coord_->XYToSL(carte_point, frenet_point)) {
        ILOG_INFO << "obstacle id " << obstacle_ptr->id()
                  << " Frenet_coord failed!!!!";
        continue;
      }
      frenet_points.emplace_back(std::move(
          planning_math::Vec2d(frenet_point.x, frenet_point.y)));
    }
    bool ok = planning_math::Polygon2d::ComputeConvexHull(frenet_points,
                                                          &obstacle_polygon);
    if (ok) {
      obstacles_frenet_infos_[id][relative_time] = obstacle_polygon;
    }
    return ok;
  }
}

bool ReferencePath::UpdateReferencePath(const bool is_need_smooth) {
  // Step 1) Check points size
  if (refined_ref_path_points_.size() < 3) {
    ILOG_ERROR << "UpdateReferencePath: points size < 3";
    return false;
  }
  // Step 2) Calculate valid lane line length
  CalculateValidLaneLineLength();
  // Step 3) Calculate road curve
  std::vector<double> raw_x_vec;
  std::vector<double> raw_y_vec;
  std::vector<double> raw_s_vec;
  CalculateRoadCurvature(raw_x_vec, raw_y_vec, raw_s_vec);
  pnc::mathlib::spline x_s_spline;
  pnc::mathlib::spline y_s_spline;
  x_s_spline.set_points(raw_s_vec, raw_x_vec);
  y_s_spline.set_points(raw_s_vec, raw_y_vec);
  const auto &planning_init_point =
      session_->environmental_model()
              .get_ego_state_manager()
              ->planning_init_point();
  Eigen::Vector2d init_point(planning_init_point.lat_init_state.x(),
                             planning_init_point.lat_init_state.y());
  pnc::spline::Projection projection_spline;
  projection_spline.CalProjectionPoint(
      x_s_spline, y_s_spline, raw_s_vec.front(), raw_s_vec.back(), init_point);
  double init_s = projection_spline.GetOutput().s_proj;
  // Step 4) Update road curve info
  HandleRoadCurvature(init_s);
  // Step 5) Smooth ref path
  if (is_need_smooth) {
    InitReferencePathSmoother();
    std::vector<planning_math::PathPoint> smoothed_path_points;
    auto start_smooth_time = IflyTime::Now_us();
    is_smoothed_ = SmoothReferencePath(init_s, x_s_spline, y_s_spline, smoothed_path_points);
    auto end_smooth_time = IflyTime::Now_us();
    JSON_DEBUG_VALUE("smooth_refpath_points_cost", end_smooth_time - start_smooth_time)
    SaveSmootherDebugInfo();
    if (is_smoothed_) {
      // Step 6) Update path info
      UpdateReferencePathInfo(smoothed_path_points);
      return true;
    }
  }
  return false;
}

void ReferencePath::CalculateValidLaneLineLength() {
  const auto &virtual_lane_manager =
      session_->environmental_model()
              .get_virtual_lane_manager();
  const auto &virtual_lane =
      virtual_lane_manager->get_current_lane();
  const auto& left_lane_boundary = virtual_lane->get_left_lane_boundary();
  const auto& right_lane_boundary = virtual_lane->get_right_lane_boundary();
  double valid_lane_line_length = refined_ref_path_points_.back().path_point.s();
  if (frenet_coord_ != nullptr) {
    size_t left_points_size =
      left_lane_boundary.enu_points_size;
    size_t right_points_size =
      right_lane_boundary.enu_points_size;
    const auto& left_end_point =
      left_lane_boundary.enu_points[left_points_size - 1];
    const auto& right_end_point =
      right_lane_boundary.enu_points[right_points_size - 1];
    Point2D frenet_left_end_point, frenet_right_end_point;
    if (frenet_coord_->XYToSL(
        Point2D(left_end_point.x, left_end_point.y),
        frenet_left_end_point)) {
      valid_lane_line_length = std::min(frenet_left_end_point.x, valid_lane_line_length);
    }
    if (frenet_coord_->XYToSL(
        Point2D(right_end_point.x, right_end_point.y),
        frenet_right_end_point)) {
      valid_lane_line_length = std::min(frenet_right_end_point.x, valid_lane_line_length);
    }
  }
  valid_lane_line_length_ = valid_lane_line_length;
}

bool ReferencePath::CalculateRoadCurvature(
    std::vector<double> &x_vec,
    std::vector<double> &y_vec,
    std::vector<double> &s_vec) {
  std::vector<double> curve_vec;
  x_vec.resize(refined_ref_path_points_.size());
  y_vec.resize(refined_ref_path_points_.size());
  s_vec.resize(refined_ref_path_points_.size());
  curve_vec.resize(refined_ref_path_points_.size());
  for (size_t i = 0; i < refined_ref_path_points_.size(); ++i) {
    x_vec[i] = refined_ref_path_points_.at(i).path_point.x();
    y_vec[i] = refined_ref_path_points_.at(i).path_point.y();
    s_vec[i] =
        i > 0 ? s_vec[i - 1] +
                    std::hypot(refined_ref_path_points_.at(i).path_point.x() -
                               refined_ref_path_points_.at(i - 1).path_point.x(),
                               refined_ref_path_points_.at(i).path_point.y() -
                               refined_ref_path_points_.at(i - 1).path_point.y())
              : 0.;
    if (refined_ref_path_points_.size() > 10 &&
        i > 4 && i < refined_ref_path_points_.size() - 5) {
      // double side_a =
      //     std::hypot(path_points.at(i + 5).x() -
      //                path_points.at(i).x(),
      //                path_points.at(i + 5).y() -
      //                path_points.at(i).y());
      // double side_b =
      //     std::hypot(path_points.at(i + 5).x() -
      //                path_points.at(i - 5).x(),
      //                path_points.at(i + 5).y() -
      //                path_points.at(i - 5).y());
      // double side_c =
      //     std::hypot(path_points.at(i).x() -
      //                path_points.at(i - 5).x(),
      //                path_points.at(i).y() -
      //                path_points.at(i - 5).y());
      // if (side_a == 0 || side_b == 0 || side_c == 0) {
      //   curve_vec[i] = curve_vec[i - 1];
      // } else {
      //   double cos_B = (side_a * side_a + side_c * side_c - side_b * side_b)
      //   / (2 * side_a * side_c); double sin_B = std::sqrt(std::max((1 - cos_B
      //   * cos_B), 1e-6)); curve_vec[i] = 2.0 * sin_B / side_b;
      // }
      // 计算向量
      double dx1 =
          refined_ref_path_points_.at(i).path_point.x() - refined_ref_path_points_.at(i - 5).path_point.x();
      double dy1 =
          refined_ref_path_points_.at(i).path_point.y() - refined_ref_path_points_.at(i - 5).path_point.y();
      double dx2 =
          refined_ref_path_points_.at(i + 5).path_point.x() - refined_ref_path_points_.at(i).path_point.x();
      double dy2 =
          refined_ref_path_points_.at(i + 5).path_point.y() - refined_ref_path_points_.at(i).path_point.y();
      // 计算弧长
      double ds1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
      double ds2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
      if (ds1 < 1e-6 || ds2 < 1e-6) {
        curve_vec[i] = curve_vec[i - 1];
      } else {
        // 平均弧长
        double avg_ds = (ds1 + ds2) / 2.0;
        // 计算一阶导数（单位切向量）
        double dx_ds = dx1 / ds1;
        double dy_ds = dy1 / ds1;
        // 计算二阶导数（曲率向量）
        double d2x = ((dx2 / ds2) - (dx1 / ds1)) / avg_ds;
        double d2y = ((dy2 / ds2) - (dy1 / ds1)) / avg_ds;
        // 计算曲率: k = (x'y'' - y'x'') / (x'^2 + y'^2)^(3/2)
        double numerator = dx_ds * d2y - dy_ds * d2x;
        double denominator = std::pow(dx_ds * dx_ds + dy_ds * dy_ds, 1.5);
        if (std::fabs(denominator) < 1e-6) {
          curve_vec[i] = curve_vec[i - 1];
        } else {
          curve_vec[i] = numerator / denominator;
          // 处理数值异常
          if (std::isnan(curve_vec[i]) ||
              std::isinf(curve_vec[i])) {
            curve_vec[i] = curve_vec[i - 1];
          }
        }
      }
    } else {
      curve_vec[i] = refined_ref_path_points_.at(i).path_point.kappa();
    }
    // curve_radius_vec[i] = std::min(
    //     std::max(1.0 / (curve_vec[i] + 1e-6), -10000.0), 10000.0);
  }
  if (s_vec.size() < 3 ||
      s_vec.size() != curve_vec.size()) {
    return false;
  }
  raw_k_s_spline_.set_points(s_vec, curve_vec);
  return true;
}

bool ReferencePath::IsExistValidVirtualLaneAheadEgo(
    double preview_range, double min_virtual_length,
    double& virtual_length, double& dist_to_virtual_start) {
  static const double kScanVirtualStep = 5.0;  // 虚拟区域向前扫描步长(m)
  const auto& planning_init_point =
      session_->environmental_model().get_ego_state_manager()->planning_init_point();
  Point2D init_frenet_pt;
  bool is_in_virtual_area = false;
  virtual_length = 0.;
  dist_to_virtual_start = 100.;
  if (frenet_coord_->XYToSL(planning_init_point.x, planning_init_point.y,
                            &init_frenet_pt.x, &init_frenet_pt.y)) {
    bool is_find_valid_virtual_area = false;
    double virtual_area_start_s = 100.;
    double virtual_area_end_s = 0.;
    for (double ref_pt_s = init_frenet_pt.x; ref_pt_s < init_frenet_pt.x + preview_range; ref_pt_s += kScanVirtualStep) {
      ReferencePathPoint ego_ahead_ref_pt;
      if (get_reference_point_by_lon(ref_pt_s, ego_ahead_ref_pt)) {
        if (ego_ahead_ref_pt.left_lane_border_type == iflyauto::LaneBoundaryType_MARKING_VIRTUAL &&
            ego_ahead_ref_pt.right_lane_border_type == iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
          if (!is_find_valid_virtual_area) {
            virtual_area_start_s = ref_pt_s;
            is_find_valid_virtual_area = true;
          }
        } else {
          if (is_find_valid_virtual_area) {
            virtual_area_end_s = ref_pt_s;
            virtual_length = virtual_area_end_s - virtual_area_start_s;
            if (virtual_length >= min_virtual_length) {
              is_in_virtual_area = true;
              dist_to_virtual_start = virtual_area_start_s - init_frenet_pt.x;
              break;
            } else {
              is_find_valid_virtual_area = false;
            }
          }
        }
      }
    }
    if (is_find_valid_virtual_area && !is_in_virtual_area) {
      virtual_area_end_s = init_frenet_pt.x + preview_range;
      virtual_length = virtual_area_end_s - virtual_area_start_s;
      if (virtual_length >= min_virtual_length) {
        is_in_virtual_area = true;
        dist_to_virtual_start = virtual_area_start_s - init_frenet_pt.x;
      } else {
        is_find_valid_virtual_area = false;
      }
    }
  }
  return is_in_virtual_area;
}

bool ReferencePath::HandleRoadCurvature(
    const double init_s) {
  // 1. 曲率半径阈值
  static const double kStraightRadiusDefault = 2000.0;      // 默认直道判断半径阈值(m)
  static const double kSharpRadiusThreshold = 300.0;        // 急弯场景的曲率半径阈值(m)
  static const double kIntersectionRadiusThreshold = 50.0;  // 路口场景的曲率半径阈值(m)
  // 2. 采样与窗口参数
  static const size_t kMaxSamplingSize = 20;     // 最大采样点数
  static const double kSamplingStep = 2.0;       // 滑动窗口内的采样步长(m)
  static const double kSamplingGap = 5.0;        // 相邻采样点的间隔(m)
  static const double kMaxSamplingRange = 81.0;  // 最大采样范围(自车位置向前，m)
  static const int kCurvatureWindowSize = 5;     // 曲率平滑窗口大小(±5步，共11个点)
  // 3. 道路预览与范围参数
  static const double kPreviewRangeMin = 31.0;      // 最小预览范围(m)
  static const double kPreviewRangeVelCoeff = 5.0;  // 预览范围与车速的系数(5.0*v)
  static const double kPreviewRangeOffset = 20.0;   // 预览范围偏移量(m)
  // 4. 弯道长度判断阈值
  static const double kMinContinuousCurveLength = 21.0;      // 持续弯道判断的最小长度(m)
  static const double kAverageContinuousCurveLength = 40.0;  // 持续弯道判断的平均长度(m)
  static const double kMinCurveGapLength = 31.0;                 // 最小连续弯道间隔长度(m)
  static const double kMinCurveLength = 6.0;                 // 最小有效弯道长度(m)
  static const double kMinMidCurveLength = 11.0;             // 中等弯道的最小长度(m)
  static const double kMinBigCurveContinuousLength = 30.0;   // 大弯道的最小持续长度(m)
  static const double kMinConnectionLength = 21.0;           // 连续弯道段的最大间隔(m)
  // 5. 近/远距范围参数
  static const double kNearRangeStartOffset = -1.0;  // 近距范围起始偏移(自车位置向后，m)
  static const double kNearRangeEndOffset = 11.0;    // 近距范围结束偏移(自车位置向前，m)
  static const double kFarRangeMin = 31.0;           // 远距范围最小距离(m)
  static const double kFarRangeMax = 61.0;           // 远距范围最大距离(m)
  static const double kFarRangeVelCoeff = 4.0;       // 远距范围与车速的系数(4.0*v)
  static const double kFarRangeStartOffset = -11.0;  // 远距范围起始偏移(m，[far-11, far+1])
  static const double kFarRangeEndOffset = 1.0;      // 远距范围结束偏移(m，[far-11, far+1])
  // 6. 弯道类型判断辅助参数
  static const double kCurveMaxStartDistance = 51.0;  // 有效弯道的最大起始距离(自车向前，m)
  static const double kMinRadiusChangeDiff = 200.0;   // 急弯判断的半径变化阈值(m)
  // 7. 虚拟区域判断阈值
  static const double kVirtualRangeVelCoeff = 5.0;           // 虚拟范围与车速的系数(5.0*v)
  static const double kVirtualRangeOffset = 5.0;             // 虚拟范围偏移量(m)
  static const double kMinIntersectionVirtualLength = 20.0;  // 路口虚拟区域最小长度(m)
  static const double kMinVirtualLength = 40.0;              // 非路口虚拟区域最小长度(m)
  static const double kScanVirtualRange = 70.0;              // 虚拟区域向前扫描范围(m)
  static const double kScanVirtualStep = 5.0;               // 虚拟区域向前扫描步长(m)

  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto& planning_init_point =
      session_->environmental_model()
              .get_ego_state_manager()
              ->planning_init_point();
  double init_v =
      std::max(planning_init_point.v, kMinRefVel);
  const auto& virtual_lane_manager =
      session_->environmental_model()
              .get_virtual_lane_manager();
  // const auto& current_lane =
  //     virtual_lane_manager->get_current_lane();
  // double max_virtual_seg_ahead_x =
  //     current_lane->get_max_virtual_seg_ahead_x();
  // double max_virtual_seg_ahead_length =
  //     current_lane->get_max_virtual_seg_ahead_length();
  double virtual_length = 0.;
  double dist_to_virtual_start = 100.;
  bool is_in_virtual_area = IsExistValidVirtualLaneAheadEgo(kScanVirtualRange, kMinIntersectionVirtualLength, virtual_length, dist_to_virtual_start);
  // 1.average curvature filter. sliding window
  std::vector<double> xp_vel{4.167, 8.333, 10.0};          // 20.0
  std::vector<double> fp_radius_thr{200.0, 400.0, 600.0};  // 1000.0
  double curve_radius_thr = interp(init_v, xp_vel, fp_radius_thr);
  double straight_radius_thr = kStraightRadiusDefault;
  if ((virtual_lane_manager
           ->GetIntersectionState() >= common::APPROACH_INTERSECTION &&
       virtual_lane_manager
           ->GetIntersectionState() < common::OFF_INTERSECTION) ||
      (is_in_virtual_area &&
       dist_to_virtual_start < (init_v * kVirtualRangeVelCoeff + kVirtualRangeOffset) &&
       virtual_length >= std::max(0.5 * init_v * kVirtualRangeVelCoeff, kMinVirtualLength))) {
    straight_radius_thr = curve_radius_thr;
    curve_radius_thr = kIntersectionRadiusThreshold;
  }
  double big_curve_thr = 1.0 / curve_radius_thr;
  double straight_curv_thr = 1.0 / straight_radius_thr;
  ref_path_curve_info_.curve_vec.reserve(kMaxSamplingSize);
  ref_path_curve_info_.s_vec.reserve(kMaxSamplingSize);
  std::vector<std::pair<double, double>> large_curvature_segments;
  large_curvature_segments.reserve(kMaxSamplingSize);
  double valid_range = std::max(valid_lane_line_length_ - init_s, 0.0);
  double min_valid_continuous_length = std::max(valid_range, kMinCurveLength);
  double sampling_range = init_s + std::min(kMaxSamplingRange, valid_range);
  double preview_range =
      std::min(std::max(init_s + init_v * kPreviewRangeVelCoeff + kPreviewRangeOffset, init_s + kPreviewRangeMin), sampling_range);
  double segment_start = init_s;
  double left_curve_length = 0.0;
  double right_curve_length = 0.0;
  double sum_near_curve = 0.0;
  double sum_far_curve = 0.0;
  double near_range_start = init_s + kNearRangeStartOffset;
  double near_range_end = init_s + kNearRangeEndOffset;
  double far_range =
      std::min(std::max(init_s + init_v * kFarRangeVelCoeff, init_s + kFarRangeMin), kFarRangeMax);
  double far_range_start = far_range + kFarRangeStartOffset;
  double far_range_end = far_range + kFarRangeEndOffset;
  int near_count = 0;
  int far_count = 0;
  bool is_big_curve = false;
  std::pair<double, double> left_max_curve_s{1e-4, 0.0};
  std::pair<double, double> right_max_curve_s{1e-4, 0.0};
  for (double sampling_s = init_s - kSamplingGap; sampling_s <= sampling_range; sampling_s += kSamplingGap) {
    std::vector<double> curv_window_vec;
    for (int j = -kCurvatureWindowSize; j <= kCurvatureWindowSize; ++j) {
      double curv = 1e-4;
      if (raw_k_s_spline_.get_x().size() > 0) {
        curv = raw_k_s_spline_(sampling_s + j * kSamplingStep);
      } else {
        break;
      }
      curv_window_vec.emplace_back(curv);
    }
    if (curv_window_vec.empty()) {
      continue;
    }
    double curv_sum = 0.0;
    for (int ind = 0; ind < curv_window_vec.size(); ++ind) {
      curv_sum += curv_window_vec[ind];
    }
    double avg_curv = curv_sum / curv_window_vec.size();
    if (sampling_s < preview_range) {
      if (std::fabs(avg_curv) > straight_curv_thr) {
        // sign
        if (avg_curv > 1e-6) {
          left_curve_length += kSamplingGap;
          right_curve_length = 0.0;
          if (std::fabs(avg_curv) > left_max_curve_s.first) {
            left_max_curve_s.first = std::fabs(avg_curv);
            left_max_curve_s.second = sampling_s;
          }
          right_max_curve_s.first = 1e-4;
          right_max_curve_s.second = 0.0;
        } else {
          left_curve_length = 0.0;
          right_curve_length += kSamplingGap;
          left_max_curve_s.first = 1e-4;
          left_max_curve_s.second = 0.0;
          if (std::fabs(avg_curv) > right_max_curve_s.first) {
            right_max_curve_s.first = std::fabs(avg_curv);
            right_max_curve_s.second = sampling_s;
          }
        }
        if (left_curve_length > kMinContinuousCurveLength) {
          ref_path_curve_info_.is_left = true;
          double last_left_length =
              ref_path_curve_info_.left_s_range.second - ref_path_curve_info_.left_s_range.first;
          if (left_curve_length > last_left_length) {
            ref_path_curve_info_.left_s_range.first = sampling_s - left_curve_length;
            ref_path_curve_info_.left_s_range.second = sampling_s;
            ref_path_curve_info_.left_max_curve = left_max_curve_s;
          }
        } else if (right_curve_length > kMinContinuousCurveLength) {
          ref_path_curve_info_.is_right = true;
          double last_right_length =
              ref_path_curve_info_.right_s_range.second - ref_path_curve_info_.right_s_range.first;
          if (right_curve_length > last_right_length) {
            ref_path_curve_info_.right_s_range.first = sampling_s - right_curve_length;
            ref_path_curve_info_.right_s_range.second = sampling_s;
            ref_path_curve_info_.right_max_curve = right_max_curve_s;
          }
        }
        // big
        if (std::fabs(avg_curv) > big_curve_thr) {
          if (!is_big_curve) {
            segment_start = sampling_s;
            if (!ref_path_curve_info_.curve_vec.empty()) {
              if (std::fabs(ref_path_curve_info_.curve_vec.back()) > straight_curv_thr) {
                segment_start -= (kSamplingGap * 0.5);
              }
            }
            is_big_curve = true;
          }
        } else {
          if (is_big_curve) {
            double curve_seg_length = sampling_s - segment_start - (kSamplingGap * 0.5);
            if (curve_seg_length > kMinMidCurveLength ||
                (curve_seg_length > kMinCurveLength &&
                segment_start - init_s < kMinCurveLength)) {
              large_curvature_segments.emplace_back(segment_start, sampling_s - kSamplingGap);
            }
            is_big_curve = false;
          }
        }
      } else {
        left_curve_length = 0.0;
        right_curve_length = 0.0;
        left_max_curve_s.first = 1e-4;
        left_max_curve_s.second = 0.0;
        right_max_curve_s.first = 1e-4;
        right_max_curve_s.second = 0.0;
        if (is_big_curve) {
          double curve_seg_length = sampling_s - segment_start - (kSamplingGap * 0.5);
          if (curve_seg_length > kMinMidCurveLength ||
              (curve_seg_length > kMinCurveLength &&
                segment_start - init_s < kMinCurveLength)) {
            large_curvature_segments.emplace_back(segment_start, sampling_s - kSamplingGap);
          }
          is_big_curve = false;
        }
      }
    } else {
      left_curve_length = 0.0;
      right_curve_length = 0.0;
      left_max_curve_s.first = 1e-4;
      left_max_curve_s.second = 0.0;
      right_max_curve_s.first = 1e-4;
      right_max_curve_s.second = 0.0;
      if (is_big_curve) {
        double curve_seg_length = sampling_s - segment_start - (kSamplingGap * 0.5);
        if (curve_seg_length > kMinMidCurveLength ||
            (curve_seg_length > kMinCurveLength &&
              segment_start - init_s < kMinCurveLength)) {
          large_curvature_segments.emplace_back(segment_start, sampling_s - kSamplingGap);
        }
        is_big_curve = false;
      }
    }
    if (sampling_s > near_range_start &&
        sampling_s < near_range_end) {
      sum_near_curve += avg_curv;
      near_count++;
    }
    if (sampling_s > far_range_start &&
        sampling_s < far_range_end) {
      sum_far_curve += avg_curv;
      far_count++;
    }
    ref_path_curve_info_.curve_vec.emplace_back(avg_curv);
    ref_path_curve_info_.s_vec.emplace_back(sampling_s);
    if (std::fabs(avg_curv) > ref_path_curve_info_.max_curve) {
      ref_path_curve_info_.max_curve = std::fabs(avg_curv);
      ref_path_curve_info_.max_curve_s = (sampling_s);
    }
  }
  double aver_near_radius = 1e4;
  if (near_count > 0) {
    aver_near_radius = std::fabs(1.0 / (sum_near_curve / near_count));
  }
  double aver_far_radius = 1e4;
  if (far_count > 0) {
    aver_far_radius = std::fabs(1.0 / (sum_far_curve / far_count));
  }
  ref_path_curve_info_.min_radius = 1.0 / ref_path_curve_info_.max_curve;
  if (is_big_curve && (sampling_range - segment_start > kMinMidCurveLength)) {
    large_curvature_segments.emplace_back(segment_start, sampling_range + kSamplingGap);
  }
  is_big_curve = false;
  // 2.scene recognition
  if (large_curvature_segments.empty()) {
    if (ref_path_curve_info_.is_left && ref_path_curve_info_.is_right) {
      ref_path_curve_info_.curve_type = ReferencePathCurveInfo::CurveType::S_CURVE;
    } else if ((ref_path_curve_info_.is_left && !ref_path_curve_info_.is_right &&
                ref_path_curve_info_.left_s_range.second - ref_path_curve_info_.left_s_range.first >=
                std::min(kAverageContinuousCurveLength, min_valid_continuous_length)) ||
               (!ref_path_curve_info_.is_left && ref_path_curve_info_.is_right &&
                ref_path_curve_info_.right_s_range.second - ref_path_curve_info_.right_s_range.first >=
                std::min(kAverageContinuousCurveLength, min_valid_continuous_length))) {
      ref_path_curve_info_.curve_type = ReferencePathCurveInfo::CurveType::NORMAL_CURVE;
    } else {
      ref_path_curve_info_.curve_type = ReferencePathCurveInfo::CurveType::STRAIGHT;
    }
    return true;
  }
  size_t connection_seg_count = 0;
  double seg_start_s = large_curvature_segments.front().first;
  double seg_end_s = large_curvature_segments.front().second;
  if (large_curvature_segments.size() > 1) {
    for (size_t i = 1; i < large_curvature_segments.size(); ++i) {
      double start_s = large_curvature_segments[i].first;
      double end_s = large_curvature_segments[i].second;
      // if (end_s - start_s < kMinCurveLength) {
      //   if ((i + 1) >= large_curvature_segments.size()) {
      //     break;
      //   }
      //   seg_start_s = large_curvature_segments[i + 1].first;
      //   seg_end_s = large_curvature_segments[i + 1].second;
      //   continue;
      // }
      if (start_s - seg_end_s < kMinConnectionLength) {
        connection_seg_count++;
        seg_end_s = end_s;
        continue;
      } else {
        break;
      }
    }
  }
  double init_dist_to_seg = seg_start_s - init_s;
  double seg_length = seg_end_s - seg_start_s;
  double cureve_max_start_s = init_s + kCurveMaxStartDistance;
  if ((init_dist_to_seg > kMinCurveLength &&
       (seg_length < kMinCurveLength ||
        (seg_length < kMinMidCurveLength &&
         ref_path_curve_info_.min_radius > curve_radius_thr))) ||
      seg_start_s > cureve_max_start_s) {
    if (ref_path_curve_info_.is_left && ref_path_curve_info_.is_right) {
      ref_path_curve_info_.curve_type = ReferencePathCurveInfo::CurveType::S_CURVE;
    } else if ((ref_path_curve_info_.is_left && !ref_path_curve_info_.is_right &&
                ref_path_curve_info_.left_s_range.second - ref_path_curve_info_.left_s_range.first >=
                std::min(kAverageContinuousCurveLength, min_valid_continuous_length)) ||
               (!ref_path_curve_info_.is_left && ref_path_curve_info_.is_right &&
                ref_path_curve_info_.right_s_range.second - ref_path_curve_info_.right_s_range.first >=
                std::min(kAverageContinuousCurveLength, min_valid_continuous_length))) {
      ref_path_curve_info_.curve_type = ReferencePathCurveInfo::CurveType::NORMAL_CURVE;
    } else {
      ref_path_curve_info_.curve_type = ReferencePathCurveInfo::CurveType::STRAIGHT;
    }
    return true;
  }
  ref_path_curve_info_.start_s = seg_start_s;
  ref_path_curve_info_.end_s = seg_end_s;
  // double max_curve_change_length = 36.0;
  // if (large_curvature_segments.size() - connection_seg_count > 1 &&
  //     seg_length < max_curve_change_length &&
  //     (init_dist_to_seg < kMinMidCurveLength ||
  //      seg_length > kMinMidCurveLength) &&
  //     ref_path_curve_info_.min_radius < curve_radius_thr) {
  if (ref_path_curve_info_.min_radius < kSharpRadiusThreshold &&
      (aver_near_radius - ref_path_curve_info_.min_radius > kMinRadiusChangeDiff ||
       aver_far_radius - ref_path_curve_info_.min_radius > kMinRadiusChangeDiff) &&
      (init_dist_to_seg < kMinMidCurveLength ||
       seg_length > kMinMidCurveLength)) {
    ref_path_curve_info_.curve_type = ReferencePathCurveInfo::CurveType::SHARP_CURVE;
  } else if (ref_path_curve_info_.is_left &&
             ref_path_curve_info_.is_right &&
             seg_length > kMinMidCurveLength) {
    if ((ref_path_curve_info_.left_max_curve.first > 1.0 / kSharpRadiusThreshold ||
         ref_path_curve_info_.right_max_curve.first > 1.0 / kSharpRadiusThreshold) &&
        (std::fabs(ref_path_curve_info_.left_max_curve.second -
                   ref_path_curve_info_.right_max_curve.second) < kMinCurveGapLength)) {
      ref_path_curve_info_.curve_type = ReferencePathCurveInfo::CurveType::SHARP_CURVE;
    } else {
      ref_path_curve_info_.curve_type = ReferencePathCurveInfo::CurveType::S_CURVE;
    }
  } else {
    if ((init_dist_to_seg < kMinMidCurveLength ||
         seg_length > std::min(kMinConnectionLength, min_valid_continuous_length)) &&
        (std::fabs(init_dist_to_seg) + seg_length) >=
         std::min(kMinBigCurveContinuousLength, min_valid_continuous_length)) {
      ref_path_curve_info_.curve_type = ReferencePathCurveInfo::CurveType::BIG_CURVE;
    } else {
      if (ref_path_curve_info_.is_left && ref_path_curve_info_.is_right) {
        if ((ref_path_curve_info_.left_max_curve.first > 1.0 / kSharpRadiusThreshold ||
            ref_path_curve_info_.right_max_curve.first > 1.0 / kSharpRadiusThreshold) &&
            (std::fabs(ref_path_curve_info_.left_max_curve.second -
                      ref_path_curve_info_.right_max_curve.second) < kMinCurveGapLength)) {
          ref_path_curve_info_.curve_type = ReferencePathCurveInfo::CurveType::SHARP_CURVE;
        } else {
          ref_path_curve_info_.curve_type = ReferencePathCurveInfo::CurveType::S_CURVE;
        }
      } else if (((ref_path_curve_info_.is_left && !ref_path_curve_info_.is_right &&
                   ref_path_curve_info_.left_s_range.second - ref_path_curve_info_.left_s_range.first >=
                   std::min(kAverageContinuousCurveLength, min_valid_continuous_length)) ||
                  (!ref_path_curve_info_.is_left && ref_path_curve_info_.is_right &&
                   ref_path_curve_info_.right_s_range.second - ref_path_curve_info_.right_s_range.first >=
                   std::min(kAverageContinuousCurveLength, min_valid_continuous_length)))) {
        ref_path_curve_info_.curve_type = ReferencePathCurveInfo::CurveType::NORMAL_CURVE;
      } else {
        ref_path_curve_info_.curve_type = ReferencePathCurveInfo::CurveType::STRAIGHT;
      }
    }
  }
  return true;
}

void ReferencePath::InitReferencePathSmoother() {
  planning::FemPosDeviationSmootherConfig smoother_param;
  smoother_param.set_apply_curvature_constraint(false);
  smoother_param.set_use_sqp(false);
  smoother_param.set_verbose(false);
  smoother_param.set_scaled_termination(true);
  smoother_param.set_warm_start(false);
  smoother_param.set_sqp_pen_max_iter(10);
  smoother_param.set_sqp_sub_max_iter(100);
  smoother_param.set_max_iter(100);
  smoother_param.set_weight_fem_pos_deviation(10000.0);
  smoother_param.set_weight_ref_deviation(10.0);
  smoother_param.set_weight_path_length(5.0);
  smoother_param.set_weight_curvature_constraint_slack_var(100.0);
  smoother_param.set_curvature_constraint(0.2);
  smoother_param.set_sqp_ftol(0.0001);
  smoother_param.set_sqp_ctol(0.0001);
  smoother_param.set_time_limit(0.01);
  // ref_path_smoother_.SetFemPosDeviationSmootherConfig(smoother_param);
  auto &smoother_config =
      ref_path_smoother_.MutableFemPosDeviationSmootherConfig();
  smoother_config = std::move(smoother_param);
  if (session_->is_rads_scene()) {
    is_enable_raw_line_extend_ = true;
    min_ahead_length_ = kReverseEgoAheadLength;
    sampling_behind_length_ = kReverseEgoBehindLength;
    sampling_ahead_length_ = kReverseEgoAheadLength;
    sampling_gap_ = kReverseSamplingGap;
    sampling_step_ = kReverseSamplingStep;
  }
}

bool ReferencePath::SmoothReferencePath(
    const double init_s,
    const pnc::mathlib::spline &x_s_spline,
    const pnc::mathlib::spline &y_s_spline,
    std::vector<planning_math::PathPoint> &smoothed_path_points) {
  // 1.Preprocessing: sampling ref points
  double behind_partition_length = 0.0;
  double ahead_partition_length = 0.0;
  std::vector<double> refined_x_vec;
  std::vector<double> refined_y_vec;
  // if (!PartionedRefPoints(
  //     init_s, x_s_spline, y_s_spline,
  //     behind_partition_length, ahead_partition_length,
  //     refined_x_vec, refined_y_vec)) {
  //   return false;
  // }
  if (!SamplingRefPoints(init_s, x_s_spline, y_s_spline,
                         behind_partition_length,
                         ahead_partition_length,
                         refined_x_vec, refined_y_vec)) {
    return false;
  }
  // 2.Preprocessing: prepare the input
  std::vector<std::pair<double, double>> raw_points_vec;
  std::vector<double> bounds;
  std::pair<double, double> init_point{
      refined_x_vec.front(), refined_y_vec.front()};
  if (!HandleInputData(refined_x_vec, refined_y_vec, init_point, bounds, raw_points_vec)) {
    return false;
  }
  // 3.Smooth start
  bool is_smooth_success =
      ref_path_smoother_.Solve(raw_points_vec, bounds, &smooth_output_.points_x, &smooth_output_.points_y);
  ref_path_smoother_info_.set_is_smooth_success(is_smooth_success);
  ref_path_smoother_info_.mutable_smoother_config()
                         ->CopyFrom(ref_path_smoother_.GetFemPosDeviationSmootherConfig());
  // 4.Postprocessing: prepare the output
  if (is_smooth_success &&
      smooth_output_.points_x.size() == smooth_output_.points_y.size() &&
      smooth_output_.points_x.size() >= 5 &&
      smooth_output_.points_x.size() == raw_points_vec.size()) {
    HandleOutputData(init_point);
    // 5.Postprocessing: dense smoothed path points
    GenerateDenseRefPathPoints(
        behind_partition_length, ahead_partition_length, x_s_spline, y_s_spline, smoothed_path_points);
    return true;
  }
  smooth_output_.Clear();
  return false;
}

bool ReferencePath::PartionedRefPoints(
    const double init_s,
    const pnc::mathlib::spline &x_s_spline,
    const pnc::mathlib::spline &y_s_spline,
    double &behind_partition_length,
    double &ahead_partition_length,
    std::vector<double> &refined_x_vec,
    std::vector<double> &refined_y_vec) {
  // check points size
  const auto &raw_points_x =
      x_s_spline.get_y();
  const auto &raw_points_y =
      y_s_spline.get_y();
  const auto &raw_points_s =
      y_s_spline.get_x();
  if (raw_points_x.empty() ||
      raw_points_y.empty() ||
      raw_points_s.empty() ||
      raw_points_x.size() != raw_points_y.size() ||
      raw_points_x.size() != raw_points_s.size() ||
      raw_points_x.size() < 5) {
    return false;
  }
  // s range
  const std::pair<double, double> s_range(
      init_s + sampling_behind_length_, init_s + sampling_ahead_length_);
  // start
  auto start_iter = std::min_element(raw_points_s.begin(), raw_points_s.end(),
      [&s_range](const double a, const double b) {
        return std::fabs(a - s_range.first) <
               std::fabs(b - s_range.first);
      });
  const auto ego_behind_idx = std::distance(raw_points_s.begin(), start_iter);
  const int split_start_idx =
      std::max(std::min(static_cast<int>(ego_behind_idx), static_cast<int>(raw_points_s.size() - 2)), 0);
  auto split_start_x_iter =
      raw_points_x.begin() + split_start_idx;
  auto split_start_y_iter =
      raw_points_y.begin() + split_start_idx;
  auto split_start_s_iter =
      raw_points_s.begin() + split_start_idx;
  behind_partition_length = *split_start_s_iter;
  // end
  auto end_iter = std::min_element(raw_points_s.begin(), raw_points_s.end(),
      [&s_range](const double a, const double b) {
        return std::fabs(a - s_range.second) <
               std::fabs(b - s_range.second);
      });
  const auto ego_ahead_idx = std::distance(raw_points_s.begin(), end_iter);
  const int split_end_idx =
      std::max(std::min(static_cast<int>(ego_ahead_idx), static_cast<int>(raw_points_s.size() - 1)), split_start_idx);
  auto split_end_x_iter =
      raw_points_x.begin() + split_end_idx;
  auto split_end_y_iter =
      raw_points_y.begin() + split_end_idx;
  auto split_end_s_iter =
      raw_points_s.begin() + split_end_idx;
  ahead_partition_length = raw_points_s.back() - *split_end_s_iter;
  // refined points
  refined_x_vec.assign(split_start_x_iter, split_end_x_iter);
  refined_y_vec.assign(split_start_y_iter, split_end_y_iter);
  return true;
}

bool ReferencePath::SamplingRefPoints(
    const double init_s,
    const pnc::mathlib::spline &x_s_spline,
    const pnc::mathlib::spline &y_s_spline,
    double &behind_partition_length,
    double &ahead_partition_length,
    std::vector<double> &refined_x_vec,
    std::vector<double> &refined_y_vec) {
  // check points size
  const auto &raw_points_x =
      x_s_spline.get_y();
  const auto &raw_points_y =
      y_s_spline.get_y();
  const auto &raw_points_s =
      y_s_spline.get_x();
  if (raw_points_x.empty() ||
      raw_points_y.empty() ||
      raw_points_s.empty() ||
      raw_points_x.size() != raw_points_y.size() ||
      raw_points_x.size() != raw_points_s.size() ||
      raw_points_x.size() < 5) {
    return false;
  }
  // sampling
  double start_s = std::max(init_s + sampling_behind_length_, 0.0);
  double end_s =
      std::min(std::max(std::min(init_s + sampling_ahead_length_,
                                 valid_lane_line_length_ - kLaneDropLength),
                        init_s + min_ahead_length_),
               raw_points_s.back());
  behind_partition_length = start_s;
  ahead_partition_length = raw_points_s.back() - end_s;
  refined_x_vec.reserve(raw_points_x.size());
  refined_y_vec.reserve(raw_points_y.size());
  double sampling_end_s = 0.0;
  for (double pt_s = start_s; pt_s < end_s + 1e-3; pt_s += sampling_gap_) {
    double pt_x = x_s_spline(pt_s);
    double pt_y = y_s_spline(pt_s);
    refined_x_vec.emplace_back(pt_x);
    refined_y_vec.emplace_back(pt_y);
    sampling_end_s = pt_s;
  }
  ahead_partition_length = std::max(raw_points_s.back() - sampling_end_s, ahead_partition_length);
  if (refined_x_vec.empty() || refined_y_vec.empty()) {
    return false;
  }
  return true;
}

bool ReferencePath::HandleInputData(
    const std::vector<double> &refined_x_vec,
    const std::vector<double> &refined_y_vec,
    const std::pair<double, double> &init_point,
    std::vector<double> &bounds,
    std::vector<std::pair<double, double>> &raw_points_vec) {
  if (refined_x_vec.size() < 5 ||
      refined_x_vec.size() != refined_y_vec.size()) {
    return false;
  }
  auto raw_x_vec = ref_path_smoother_info_.mutable_raw_x_vec();
  auto raw_y_vec = ref_path_smoother_info_.mutable_raw_y_vec();
  raw_x_vec->Reserve(refined_x_vec.size());
  raw_y_vec->Reserve(refined_y_vec.size());
  raw_points_vec.reserve(refined_x_vec.size());
  // set bound
  if (session_->is_rads_scene()) {
    double bound_val = map_bound_val_[0];
    ref_path_smoother_info_.set_bound_val(bound_val);
    bounds.resize(refined_x_vec.size(), bound_val);
    SetSmoothBounds(bound_val, bounds);
  } else {
    auto &reference_path_manager =
        session_->mutable_environmental_model()->get_reference_path_manager();
    auto &smooth_bound_filter =
        reference_path_manager->MutableSmoothBoundFilter();
    std::vector<double> xp_road_radius{400.0, 1500.0, 3000.0};
    double bound_val =
        planning::interp(ref_path_curve_info_.min_radius, xp_road_radius, map_bound_val_);
    // double weight_fem_pos_deviation =
    //     planning::interp(ref_path_curve_info_.min_radius, xp_road_radius, map_weight_fem_pos_deviation_);
    // if (ref_path_curve_info_.curve_type == ReferencePathCurveInfo::CurveType::NORMAL_CURVE) {
    //   bound_val = map_bound_val_[1];
    //   weight_fem_pos_deviation = map_weight_fem_pos_deviation_[1];
    // }
    if (ref_path_curve_info_.curve_type == ReferencePathCurveInfo::CurveType::SHARP_CURVE ||
        ref_path_curve_info_.curve_type == ReferencePathCurveInfo::CurveType::S_CURVE ||
        ref_path_curve_info_.curve_type == ReferencePathCurveInfo::CurveType::BIG_CURVE) {
      smooth_bound_filter->Reset();
      bound_val = map_bound_val_[0];
      // weight_fem_pos_deviation = map_weight_fem_pos_deviation_[0];
    }
    smooth_bound_filter->Update(bound_val);
    const double mean_bound_val = smooth_bound_filter->GetMeanValue();
    ref_path_smoother_info_.set_bound_val(mean_bound_val);
    bounds.resize(refined_x_vec.size(), mean_bound_val);
    SetSmoothBounds(mean_bound_val, bounds);
  }
  // update param
  // auto &smoother_config =
  //     ref_path_smoother_.MutableFemPosDeviationSmootherConfig();
  // smoother_config.set_weight_fem_pos_deviation(weight_fem_pos_deviation);
  for (size_t i = 0; i < refined_x_vec.size(); ++i) {
    raw_points_vec.emplace_back(refined_x_vec[i] - init_point.first,
                                refined_y_vec[i] - init_point.second);
    raw_x_vec->Add(refined_x_vec[i]);
    raw_y_vec->Add(refined_y_vec[i]);
  }
  // use last result
  // if (is_use_last_result &&
  //     (!smooth_output_.points_x.empty() &&
  //      !smooth_output_.points_y.empty() &&
  //      !smooth_output_.points_s.empty())) {
  //   pnc::mathlib::spline x_s_spline;
  //   pnc::mathlib::spline y_s_spline;
  //   x_s_spline.set_points(smooth_output_.points_s, smooth_output_.points_x);
  //   y_s_spline.set_points(smooth_output_.points_s, smooth_output_.points_y);
  //   Eigen::Vector2d cur_start_point(refined_x_vec.front(), refined_y_vec.front());
  //   pnc::spline::Projection ref_projection_spline;
  //   ref_projection_spline.CalProjectionPoint(
  //       x_s_spline, y_s_spline,
  //       smooth_output_.points_s.front(), smooth_output_.points_s.back(),
  //       cur_start_point);
  //   double cur_start_point_s = ref_projection_spline.GetOutput().s_proj;
  //   double cur_end_point_s = cur_start_point_s + ref_path_length;
  //   double last_end_point_s = cur_start_point_s;
  //   int raw_size = refined_x_vec.size();
  //   double ds =
  //       (cur_end_point_s - cur_start_point_s) / std::max(raw_size - 1, 1);
  //   initial_points.reserve(refined_x_vec.size());
  //   for (double point_s = cur_start_point_s; point_s < (cur_end_point_s + ds); point_s += ds) {
  //     double point_x = x_s_spline(point_s);
  //     double point_y = y_s_spline(point_s);
  //     last_end_point_s = point_s;
  //     initial_points.emplace_back(point_x - init_point.first,
  //                                 point_y - init_point.second);
  //     initial_points_xvec->Add(point_x);
  //     initial_points_yvec->Add(point_y);
  //     if (initial_points.size() == refined_x_vec.size()) {
  //       break;
  //     }
  //   }
  // }
  return true;
}

void ReferencePath::SetSmoothBounds(
    const double bound_val, std::vector<double> &bounds) {
  if (bounds.size() <= 2) {
    return;
  }
  constexpr double kEpsion = 1e-6;
  bounds.front() = kEpsion;
  bounds.back() = kEpsion;
  for (int i = 1; i < bounds.size() - 1; i++) {
    bounds.at(i) = bound_val;
  }
}

bool ReferencePath::HandleOutputData(
    const std::pair<double, double> &init_point) {
  smooth_output_.points_s.reserve(smooth_output_.points_x.size());
  auto smooth_x_vec = ref_path_smoother_info_.mutable_smooth_x_vec();
  auto smooth_y_vec = ref_path_smoother_info_.mutable_smooth_y_vec();
  auto smooth_s_vec = ref_path_smoother_info_.mutable_smooth_s_vec();
  smooth_x_vec->Reserve(smooth_output_.points_x.size());
  smooth_y_vec->Reserve(smooth_output_.points_x.size());
  smooth_s_vec->Reserve(smooth_output_.points_x.size());
  double points_s = 0.0;
  for (size_t k = 0; k < smooth_output_.points_x.size(); ++k) {
    smooth_output_.points_x[k] += init_point.first;
    smooth_output_.points_y[k] += init_point.second;
    if (k > 0) {
      points_s += std::hypotf(
        smooth_output_.points_x[k] - smooth_output_.points_x[k - 1],
        smooth_output_.points_y[k] - smooth_output_.points_y[k - 1]);
    }
    smooth_output_.points_s.emplace_back(points_s);
    smooth_x_vec->Add(smooth_output_.points_x[k]);
    smooth_y_vec->Add(smooth_output_.points_y[k]);
    smooth_s_vec->Add(points_s);
  }
  smooth_output_.x_s_spline.set_points(smooth_output_.points_s, smooth_output_.points_x);
  smooth_output_.y_s_spline.set_points(smooth_output_.points_s, smooth_output_.points_y);
  smooth_output_.is_available = true;
  return true;
}

bool ReferencePath::GenerateDenseRefPathPoints(
    const double behind_partition_length,
    const double ahead_partition_length,
    const pnc::mathlib::spline &x_s_spline,
    const pnc::mathlib::spline &y_s_spline,
    std::vector<planning_math::PathPoint> &smoothed_path_points) {
  smoothed_path_points.reserve(refined_ref_path_points_.size() + 1);
  BackwardExtendedRefPoints(
      behind_partition_length, x_s_spline, y_s_spline, smoothed_path_points);
  for (double pt_s = 0; pt_s < smooth_output_.points_s.back() + sampling_step_; pt_s += sampling_step_) {
    if (pt_s > smooth_output_.points_s.back()) {
      pt_s = smooth_output_.points_s.back() + 1e-2;
    }
    auto pt =
        planning_math::PathPoint(smooth_output_.x_s_spline(pt_s),
                                 smooth_output_.y_s_spline(pt_s));
    smoothed_path_points.emplace_back(std::move(pt));
  }
  if (is_enable_raw_line_extend_) {
    ForwardStitchRefPoints(
        ahead_partition_length, x_s_spline, y_s_spline, smoothed_path_points);
  } else {
    ForwardExtendedRefPoints(
        ahead_partition_length, smoothed_path_points);
  }
  return true;
}

bool ReferencePath::BackwardExtendedRefPoints(
    const double behind_partition_length,
    const pnc::mathlib::spline &x_s_spline,
    const pnc::mathlib::spline &y_s_spline,
    std::vector<planning_math::PathPoint> &smoothed_path_points) {
  if (behind_partition_length < 1e-3) {
    return true;
  }
  for (double pt_s = 0.; pt_s < behind_partition_length - 0.5 * sampling_step_; pt_s += sampling_step_) {
    double pt_x = x_s_spline(pt_s);
    double pt_y = y_s_spline(pt_s);
    auto pt =
        planning_math::PathPoint(pt_x, pt_y);
    smoothed_path_points.emplace_back(std::move(pt));
  }
  return true;
}

bool ReferencePath::ForwardStitchRefPoints(
    const double ahead_partition_length,
    const pnc::mathlib::spline &x_s_spline,
    const pnc::mathlib::spline &y_s_spline,
    std::vector<planning_math::PathPoint> &smoothed_path_points) {
  if (ahead_partition_length < 1e-3) {
    return true;
  }
  const auto &raw_points_s = x_s_spline.get_x();
  double cur_length = raw_points_s.back() - ahead_partition_length;
  double end_s = std::min(valid_lane_line_length_, raw_points_s.back());
  for (double pt_s = cur_length + sampling_step_; pt_s <= end_s; pt_s += sampling_step_) {
    cur_length = pt_s;
    double pt_x = x_s_spline(pt_s);
    double pt_y = y_s_spline(pt_s);
    auto pt =
        planning_math::PathPoint(pt_x, pt_y);
    smoothed_path_points.emplace_back(std::move(pt));
  }
  double remain_length = std::max(raw_points_s.back() - cur_length, 0.0);
  ForwardExtendedRefPoints(remain_length, smoothed_path_points);
  return true;
}

bool ReferencePath::ForwardExtendedRefPoints(
    const double ahead_partition_length,
    std::vector<planning_math::PathPoint> &smoothed_path_points) {
  if (smoothed_path_points.size() < 2) {
    return false;
  }
  if (ahead_partition_length < 1e-3) {
    return true;
  }
  if (smoothed_path_points.size() == 2 ||
      !is_enable_clothoid_extend_) {
    // 如果点数不足，回退到简单直线延长
    StraightExtendedRefPoints(ahead_partition_length, smoothed_path_points);
    return true;
  }
  ClothoidExtendedRefPoints(ahead_partition_length, 1e-6, smoothed_path_points);
  return true;
}

void ReferencePath::StraightExtendedRefPoints(
    const double extend_length, std::vector<planning_math::PathPoint> &smoothed_path_points) {
  size_t start_idx = smoothed_path_points.size() - 2;
  double dx = smoothed_path_points[start_idx + 1].x() - smoothed_path_points[start_idx].x();
  double dy = smoothed_path_points[start_idx + 1].y() - smoothed_path_points[start_idx].y();
  double line_length = sqrt(dx * dx + dy * dy);
  dx /= line_length;
  dy /= line_length;
  // double extend_pt_x = smoothed_path_points[start_idx + 1].x() + dx * extend_length;
  // double extend_pt_y = smoothed_path_points[start_idx + 1].y() + dy * extend_length;
  // auto pt = planning_math::PathPoint(extend_pt_x, extend_pt_y);
  // smoothed_path_points.emplace_back(std::move(pt));
  double extend_pt_x = smoothed_path_points[start_idx + 1].x();
  double extend_pt_y = smoothed_path_points[start_idx + 1].y();
  for (double sum_s = 0.0; sum_s < extend_length; sum_s += sampling_step_) {
    double ds = sampling_step_;
    if (sum_s + ds >= extend_length) {
      ds = std::max(extend_length - sum_s, 0.0);
    }
    extend_pt_x += dx * ds;
    extend_pt_y += dy * ds;
    auto pt = planning_math::PathPoint(extend_pt_x, extend_pt_y);
    smoothed_path_points.emplace_back(std::move(pt));
  }
}

void ReferencePath::ClothoidExtendedRefPoints(
    const double extend_length, const double target_curv,
    std::vector<planning_math::PathPoint> &smoothed_path_points) {
  // 使用最后三个点计算曲率
  double curvature = 1e-4;
  size_t start_idx = smoothed_path_points.size() - 3;
  double dx1 = smoothed_path_points[start_idx + 1].x() - smoothed_path_points[start_idx].x();
  double dy1 = smoothed_path_points[start_idx + 1].y() - smoothed_path_points[start_idx].y();
  double dx2 = smoothed_path_points[start_idx + 2].x() - smoothed_path_points[start_idx + 1].x();
  double dy2 = smoothed_path_points[start_idx + 2].y() - smoothed_path_points[start_idx + 1].y();
  // 计算向量长度
  double len1 = std::hypot(dx1, dy1);
  double len2 = std::hypot(dx2, dy2);
  if (len1 > 1e-6 && len2 > 1e-6) {
    // 归一化向量
    dx1 /= len1;
    dy1 /= len1;
    dx2 /= len2;
    dy2 /= len2;
    // 计算角度变化
    double dot_product = dx1 * dx2 + dy1 * dy2;
    dot_product = std::max(-1.0, std::min(1.0, dot_product)); // 防止数值误差
    double angle_change = std::acos(dot_product);
    // 计算曲率
    double avg_length = (len1 + len2) / 2.0;
    curvature = angle_change / avg_length;
  }
  // 限制曲率范围，避免数值不稳定
  if (curvature > 1e-6) {
    curvature = std::max(1e-4, std::min(1e-1, curvature));
  } else {
    curvature = std::max(-1e-1, std::min(-1e-4, curvature));
  }
  // 计算曲率半径变化率
  double curv_rate = (curvature - target_curv) / extend_length;
  // 获取最后一个点的信息
  double extend_pt_x = smoothed_path_points.back().x();
  double extend_pt_y = smoothed_path_points.back().y();
  double extend_pt_theta = std::atan2(dy2, dx2);
  double extend_pt_curv = curvature;
  // 使用clothoid积分生成延长点
  for (double sum_s = 0.0; sum_s < extend_length; sum_s += sampling_step_) {
    double ds = sampling_step_;
    if (sum_s + ds >= extend_length) {
      ds = extend_length - sum_s;
    }
    if (std::fabs(extend_pt_curv - target_curv) > 1e-4) {
      extend_pt_curv += curv_rate * ds;
    }
    double last_theta = extend_pt_theta;
    extend_pt_theta +=  extend_pt_curv * ds;
    // 使用平均角度计算位置增量
    double avg_theta = 0.5 * (last_theta + extend_pt_theta);
    extend_pt_x += std::cos(avg_theta) * ds;
    extend_pt_y += std::sin(avg_theta) * ds;
    auto pt = planning_math::PathPoint(extend_pt_x, extend_pt_y);
    smoothed_path_points.emplace_back(std::move(pt));
  }
}

bool ReferencePath::UpdateReferencePathInfo(
    std::vector<planning_math::PathPoint> &smoothed_path_points) {
  if (smoothed_path_points.size() < KDPath::kKDPathMinPathPointSize) {
    return false;
  }
  ReferencePathPoints smoothed_ref_path;
  smoothed_ref_path.reserve(smoothed_path_points.size());
  std::shared_ptr<planning_math::KDPath> smoothed_frenet_coord =
      std::make_shared<planning_math::KDPath>(std::move(smoothed_path_points));
  const auto &frenet_path_points = smoothed_frenet_coord->path_points();
  ReferencePathPoint last_ref_pt = refined_ref_path_points_.front();

  Point2D frenet_point;
  for (const auto& pt : frenet_path_points) {
    ReferencePathPoint ref_pt;
    ref_pt.path_point.set_x(pt.x());
    ref_pt.path_point.set_y(pt.y());
    ref_pt.path_point.set_theta(pt.theta());
    ref_pt.path_point.set_s(pt.s());
    ref_pt.path_point.set_kappa(pt.kappa());
    ref_pt.path_point.set_dkappa(pt.dkappa());
    ref_pt.path_point.set_ddkappa(pt.ddkappa());
    if (frenet_coord_->XYToSL(
            pt.x(), pt.y(), &frenet_point.x, &frenet_point.y)) {
      ReferencePathPoint raw_pt;
      if (get_reference_point_by_lon(frenet_point.x, raw_pt)) {
        ref_pt.path_point.set_z(raw_pt.path_point.z());
        ref_pt.max_velocity = raw_pt.max_velocity;
        ref_pt.min_velocity = raw_pt.min_velocity;
        ref_pt.lane_width = raw_pt.lane_width;
        ref_pt.distance_to_left_lane_border = raw_pt.distance_to_left_lane_border - frenet_point.y;
        ref_pt.distance_to_right_lane_border = raw_pt.distance_to_right_lane_border + frenet_point.y;
        ref_pt.distance_to_left_road_border = raw_pt.distance_to_left_road_border - frenet_point.y;
        ref_pt.distance_to_right_road_border = raw_pt.distance_to_right_road_border + frenet_point.y;
        ref_pt.left_lane_border_type = raw_pt.left_lane_border_type;
        ref_pt.right_lane_border_type = raw_pt.right_lane_border_type;
        ref_pt.left_road_border_type = raw_pt.left_road_border_type;
        ref_pt.right_road_border_type = raw_pt.right_road_border_type;
        ref_pt.type = raw_pt.type;
        ref_pt.is_in_intersection = raw_pt.is_in_intersection;
      } else {
        ref_pt.path_point.set_z(last_ref_pt.path_point.z());
        ref_pt.max_velocity = last_ref_pt.max_velocity;
        ref_pt.min_velocity = last_ref_pt.min_velocity;
        ref_pt.lane_width = last_ref_pt.lane_width;
        ref_pt.distance_to_left_lane_border = last_ref_pt.distance_to_left_lane_border;
        ref_pt.distance_to_right_lane_border = last_ref_pt.distance_to_right_lane_border;
        ref_pt.distance_to_left_road_border = last_ref_pt.distance_to_left_road_border;
        ref_pt.distance_to_right_road_border = last_ref_pt.distance_to_right_road_border;
        ref_pt.left_lane_border_type = last_ref_pt.left_lane_border_type;
        ref_pt.right_lane_border_type = last_ref_pt.right_lane_border_type;
        ref_pt.left_road_border_type = last_ref_pt.left_road_border_type;
        ref_pt.right_road_border_type = last_ref_pt.right_road_border_type;
        ref_pt.type = last_ref_pt.type;
        ref_pt.is_in_intersection = last_ref_pt.is_in_intersection;
      }
    } else {
      ref_pt.path_point.set_z(last_ref_pt.path_point.z());
      ref_pt.max_velocity = last_ref_pt.max_velocity;
      ref_pt.min_velocity = last_ref_pt.min_velocity;
      ref_pt.lane_width = last_ref_pt.lane_width;
      ref_pt.distance_to_left_lane_border = last_ref_pt.distance_to_left_lane_border;
      ref_pt.distance_to_right_lane_border = last_ref_pt.distance_to_right_lane_border;
      ref_pt.distance_to_left_road_border = last_ref_pt.distance_to_left_road_border;
      ref_pt.distance_to_right_road_border = last_ref_pt.distance_to_right_road_border;
      ref_pt.left_lane_border_type = last_ref_pt.left_lane_border_type;
      ref_pt.right_lane_border_type = last_ref_pt.right_lane_border_type;
      ref_pt.left_road_border_type = last_ref_pt.left_road_border_type;
      ref_pt.right_road_border_type = last_ref_pt.right_road_border_type;
      ref_pt.type = last_ref_pt.type;
      ref_pt.is_in_intersection = last_ref_pt.is_in_intersection;
    }
    last_ref_pt = ref_pt;
    smoothed_ref_path.emplace_back(std::move(ref_pt));
  }
  frenet_coord_ = std::move(smoothed_frenet_coord);
  refined_ref_path_points_.clear();
  refined_ref_path_points_ = std::move(smoothed_ref_path);
  return true;
}

void ReferencePath::SaveSmootherDebugInfo() {
#ifdef ENABLE_PROTO_LOG
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_reference_path_smooth_info()
      ->CopyFrom(ref_path_smoother_info_);
#else
  ref_path_smoother_info_.clear_is_smooth_success();
  ref_path_smoother_info_.clear_bound_val();
  ref_path_smoother_info_.clear_smoother_config();
  ref_path_smoother_info_.clear_raw_x_vec();
  ref_path_smoother_info_.clear_raw_y_vec();
  ref_path_smoother_info_.clear_smooth_s_vec();
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_reference_path_smooth_info()
      ->CopyFrom(ref_path_smoother_info_);
#endif
}

}  // namespace planning

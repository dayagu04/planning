#include "../include/basic_pure_pursuit_model.h"
#include <cmath>
#include <memory>

#include "config/basic_type.h"
#include "debug_info_log.h"
#include "define/geometry.h"
#include "log.h"
#include "math_lib.h"
#include "src/common/log.h"
namespace {
constexpr size_t scope_goal_points_num = 20;
};
namespace planning {

ErrorType BasicPurePursuitModel::ProcessReferencePath(
    std::shared_ptr<ReferencePath> reference_path) {
  if (reference_path == nullptr) {
    LOG_ERROR(
        "BasicPurePursuitModel::ProcessReferencePath: reference path is null");
    return ErrorType::kIllegalInput;
  }
  reference_path_ = reference_path;
  const auto& ref_points = reference_path->get_points();
  ref_s_vec_.resize(ref_points.size(), 0.0);
  ref_x_vec_.resize(ref_points.size(), 0.0);
  ref_y_vec_.resize(ref_points.size(), 0.0);
  ref_theta_vec_.resize(ref_points.size(), 0.0);
  for (size_t i = 0; i < ref_points.size(); ++i) {
    ref_s_vec_[i] = ref_points[i].path_point.s();
    ref_x_vec_[i] = ref_points[i].path_point.x();
    ref_y_vec_[i] = ref_points[i].path_point.y();
    ref_theta_vec_[i] = ref_points[i].path_point.theta();
  }
  ref_x_s_spline_.set_points(ref_s_vec_, ref_x_vec_);
  ref_y_s_spline_.set_points(ref_s_vec_, ref_y_vec_);
  ref_theta_s_spline_.set_points(ref_s_vec_, ref_theta_vec_);
  ref_points_size_ = ref_points.size();
  is_input_set_ = true;
  return ErrorType::kSuccess;
}

ErrorType BasicPurePursuitModel::CalculateAlpha(const double lat_offset) {
  if (!is_input_set_) {
    LOG_ERROR("pure pursuit model input not set!!! \n");
    return ErrorType::kWrongStatus;
  }
  Eigen::Vector2d ego_init_pos(mdoel_state_.x, mdoel_state_.y);
  size_t pre_goal_point_index = 0;
  // cal projection point on reference path
  pnc::spline::Projection projection_spline;
  projection_spline.CalProjectionPoint(ref_x_s_spline_, ref_y_s_spline_,
                                       ref_s_vec_.front(), ref_s_vec_.back(),
                                       ego_init_pos);
  const auto projection_result = projection_spline.GetOutput();
  const auto proj_point = projection_result.point_proj;
  const double s_proj = projection_result.s_proj;  // s of projection point
  // find goal point
  size_t init_lookup_index = 0;
  auto it = std::upper_bound(ref_s_vec_.begin(), ref_s_vec_.end(), s_proj);
  if (it != ref_s_vec_.begin()) {
    init_lookup_index = std::distance(ref_s_vec_.begin(), it) - 1;
  } else {
    init_lookup_index = 0;
  }
  double distance2 = std::numeric_limits<double>::max();
  const double ld2 = model_param_.ld * model_param_.ld;
  pre_goal_point_index = init_lookup_index;
  for (size_t i = init_lookup_index; i < ref_points_size_; ++i) {
    distance2 = pow(ref_x_vec_[i] - mdoel_state_.x, 2) +
                pow(ref_y_vec_[i] - mdoel_state_.y, 2);
    if (distance2 > ld2) {
      pre_goal_point_index = i;
      break;
    }
  }
  --pre_goal_point_index;

  Eigen::Vector2d front_goal_pnt(ref_x_vec_[pre_goal_point_index + 1],
                                 ref_y_vec_[pre_goal_point_index + 1]);
  Eigen::Vector2d behind_goal_pnt(ref_x_vec_[pre_goal_point_index],
                                  ref_y_vec_[pre_goal_point_index]);
  double ld_interp = 0.0;
  bool is_find_goal_point = false;

  Eigen::Vector2d scope_goal_point_candidate;
  for (size_t i = 1; i <= scope_goal_points_num; ++i) {
    double t = static_cast<double>(i) / (scope_goal_points_num + 1);
    scope_goal_point_candidate = (1 - t) * behind_goal_pnt + t * front_goal_pnt;
    ld_interp = (scope_goal_point_candidate - ego_init_pos).norm();
    if (ld_interp > model_param_.ld) {
      goal_point_ = scope_goal_point_candidate;
      is_find_goal_point = true;
      break;
    }
  }

  //防止上面的if条件不满足
  if (!is_find_goal_point) {
    goal_point_ = front_goal_pnt;
  }

  //更新横向偏移值
  const auto& ref_coor = reference_path_->get_frenet_coord();

  Point2D xy_point(goal_point_.x(), goal_point_.y());
  Point2D sl_point;
  if (!ref_coor->XYToSL(xy_point, sl_point)) {
    LOG_ERROR("xy_point transforme to sl_point fail!!! \n");
    return ErrorType::kWrongStatus;
  }

  Point2D new_sl_point(sl_point.x, sl_point.y + lat_offset);
  if (!ref_coor->SLToXY(new_sl_point, xy_point)) {
    LOG_ERROR("sl_point transforme to xy_point fail!!! \n");
    return ErrorType::kWrongStatus;
  }

  goal_point_.x() = xy_point.x;
  goal_point_.y() = xy_point.y;

  const double current_state_theta = mdoel_state_.theta;
  Eigen::Vector2d current_state_theta_vec(cos(current_state_theta),
                                          sin(current_state_theta));
  const Eigen::Vector2d ld_vec = goal_point_ - ego_init_pos;
  const Eigen::Vector2d ld_vec_unit = ld_vec.normalized();
  ld_actual_length_ = ld_vec.norm();
  double vec_dot = current_state_theta_vec.dot(ld_vec_unit);
  vec_dot = pnc::mathlib::Clamp(vec_dot, -1.0, 1.0);
  alpha_ = acos(vec_dot);

  double cross_product_current_state_theta_to_ld_vec =
      current_state_theta_vec.x() * ld_vec.y() -
      current_state_theta_vec.y() * ld_vec.x();
  int rotation_direction =
      (cross_product_current_state_theta_to_ld_vec >= 0) ? 1 : -1;
  alpha_ *= rotation_direction;
  JSON_DEBUG_VALUE("goal_point_x", goal_point_.x())
  JSON_DEBUG_VALUE("goal_point_y", goal_point_.y())
  JSON_DEBUG_VALUE("pp_init_x", mdoel_state_.x)
  JSON_DEBUG_VALUE("pp_init_y", mdoel_state_.y)
  return ErrorType::kSuccess;
}

ErrorType BasicPurePursuitModel::CalculateDesiredDelta(const double lat_offset) {
  delta_ = 0;
  alpha_ = 0;
  goal_point_(0, 0);

  if (!is_model_param_set_) {
    LOG_ERROR("pure pursuit model param not set!!! \n");
    return ErrorType::kWrongStatus;
  }

  if (!is_current_state_set_) {
    LOG_ERROR("pure pursuit model state not set!!! \n");
    return ErrorType::kWrongStatus;
  }

  CalculateAlpha(lat_offset);
  delta_ =
      atan2(2.0 * model_param_.wheelbase_length * sin(alpha_), model_param_.ld);
  Reset();
  return ErrorType::kSuccess;
}

ErrorType BasicPurePursuitModel::CalculateDesiredDelta(
    const double wheelbase_len, const double angle_diff /*alpha*/,
    const double look_ahead_dist, double* delta) {
  *delta = atan2(2.0 * wheelbase_len * sin(angle_diff), look_ahead_dist);
  return ErrorType::kSuccess;
}

void BasicPurePursuitModel::Reset() {
  model_param_ = ModelParam();
  mdoel_state_ = ModelState();
  is_current_state_set_ = false;
  is_model_param_set_ = false;
}

}  // namespace planning
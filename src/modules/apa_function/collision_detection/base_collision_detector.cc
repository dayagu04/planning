#include "base_collision_detector.h"

#include "apa_obstacle.h"
#include "apa_param_config.h"
#include "log_glog.h"

namespace planning {
namespace apa_planner {

void BaseCollisionDetector::Init(const bool fold_mirror_flag) {
  const ApaParameters& param = apa_param.GetParam();
  Eigen::Vector2d vertex;

  double max_car_width;
  std::vector<double> car_vertex_x_vec, car_vertex_y_vec;
  if (fold_mirror_flag) {
    max_car_width = param.fold_mirror_max_car_width;
    car_vertex_x_vec = param.fold_mirror_car_vertex_x_vec;
    car_vertex_y_vec = param.fold_mirror_car_vertex_y_vec;
  } else {
    max_car_width = param.max_car_width;
    car_vertex_x_vec = param.car_vertex_x_vec;
    car_vertex_y_vec = param.car_vertex_y_vec;
  }

  // 包含左右后视镜的多边形
  car_with_mirror_polygon_vertex_.clear();
  car_with_mirror_polygon_vertex_.reserve(car_vertex_x_vec.size());
  for (size_t i = 0; i < car_vertex_x_vec.size(); ++i) {
    vertex << car_vertex_x_vec[i], car_vertex_y_vec[i];
    car_with_mirror_polygon_vertex_.emplace_back(vertex);
  }
  // 参数里默认为顺时针 这里转化为逆时针 方便后面形成polygon gjk检测
  std::reverse(car_with_mirror_polygon_vertex_.begin(),
               car_with_mirror_polygon_vertex_.end());

  // 不包含后视镜的多边形
  car_without_mirror_polygon_vertex_.clear();
  car_without_mirror_polygon_vertex_.reserve(
      car_with_mirror_polygon_vertex_.size() - 8);
  for (const Eigen::Vector2d& pt : car_with_mirror_polygon_vertex_) {
    if (pt.x() > 0.0 && pt.x() < param.lon_dist_mirror_to_rear_axle + 0.68) {
      continue;
    }
    car_without_mirror_polygon_vertex_.emplace_back(pt);
  }

  // 左后视镜
  left_mirror_rectangle_vertex_.clear();
  left_mirror_rectangle_vertex_.reserve(4);
  for (const Eigen::Vector2d& pt : car_with_mirror_polygon_vertex_) {
    if (pt.x() > 0.0 && pt.x() < param.lon_dist_mirror_to_rear_axle + 0.68 &&
        pt.y() > 0.0) {
      left_mirror_rectangle_vertex_.emplace_back(pt);
    }
  }

  // 右后视镜
  right_mirror_rectangle_vertex_.clear();
  right_mirror_rectangle_vertex_.reserve(4);
  for (const Eigen::Vector2d& pt : car_with_mirror_polygon_vertex_) {
    if (pt.x() > 0.0 && pt.x() < param.lon_dist_mirror_to_rear_axle + 0.68 &&
        pt.y() < 0.0) {
      right_mirror_rectangle_vertex_.emplace_back(pt);
    }
  }

  // 底盘矩形
  chassis_vertex_.clear();
  chassis_vertex_.resize(4);
  chassis_vertex_[0] << 0.0, 0.5 * param.car_width;
  chassis_vertex_[1] << 0.0, -0.5 * param.car_width;
  chassis_vertex_[2] << param.wheel_base, -0.5 * param.car_width;
  chassis_vertex_[3] << param.wheel_base, 0.5 * param.car_width;

  // 包含左右后视镜的矩形
  car_with_mirror_rectangle_vertex_.clear();
  car_with_mirror_rectangle_vertex_.resize(4);
  car_with_mirror_rectangle_vertex_[0] << -param.rear_overhanging,
      0.5 * max_car_width;
  car_with_mirror_rectangle_vertex_[1] << -param.rear_overhanging,
      -0.5 * max_car_width;
  car_with_mirror_rectangle_vertex_[2]
      << param.wheel_base + param.front_overhanging,
      -0.5 * max_car_width;
  car_with_mirror_rectangle_vertex_[3]
      << param.wheel_base + param.front_overhanging,
      0.5 * max_car_width;

  // 后视镜到前悬矩形
  mirror_to_front_overhanging_rectangle_vertex_expand_front_.clear();
  mirror_to_front_overhanging_rectangle_vertex_expand_front_.resize(4);
  mirror_to_front_overhanging_rectangle_vertex_expand_front_[0] =
      left_mirror_rectangle_vertex_[2];
  mirror_to_front_overhanging_rectangle_vertex_expand_front_[1] =
      right_mirror_rectangle_vertex_[1];
  mirror_to_front_overhanging_rectangle_vertex_expand_front_[2]
      << param.wheel_base + param.front_overhanging,
      -0.5 * max_car_width;
  mirror_to_front_overhanging_rectangle_vertex_expand_front_[3]
      << param.wheel_base + param.front_overhanging,
      0.5 * max_car_width;

  // 后视镜到后悬多边形
  mirror_to_rear_overhanging_polygon_vertex_.clear();
  mirror_to_rear_overhanging_polygon_vertex_.reserve(8);
  mirror_to_rear_overhanging_polygon_vertex_.emplace_back(
      left_mirror_rectangle_vertex_[3]);
  for (const Eigen::Vector2d& pt : car_with_mirror_polygon_vertex_) {
    if (pt.x() > 0.0) {
      continue;
    }
    mirror_to_rear_overhanging_polygon_vertex_.emplace_back(pt);
  }
  mirror_to_rear_overhanging_polygon_vertex_.emplace_back(
      right_mirror_rectangle_vertex_[0]);

  car_with_mirror_circles_list_.Reset();
  car_with_mirror_circles_list_.height_type = ApaObsHeightType::HIGH;
  std::vector<float> circle_x, circle_y, circle_r;
  if (fold_mirror_flag) {
    circle_x = param.fold_mirror_footprint_circle_x;
    circle_y = param.fold_mirror_footprint_circle_y;
    circle_r = param.fold_mirror_footprint_circle_r;
  } else {
    circle_x = param.footprint_circle_x;
    circle_y = param.footprint_circle_y;
    circle_r = param.footprint_circle_r;
  }
  car_with_mirror_circles_list_.max_circle.center_local << circle_x[0],
      circle_y[0];
  car_with_mirror_circles_list_.max_circle.radius = circle_r[0];
  CarFootPrintCircle* circles = car_with_mirror_circles_list_.circles;
  for (size_t i = 1; i < circle_x.size(); ++i) {
    if (car_with_mirror_circles_list_.count >= MAX_CAR_FOOTPRINT_CIRCLE_NUM) {
      break;
    }
    circles[car_with_mirror_circles_list_.count].center_local << circle_x[i],
        circle_y[i];
    circles[car_with_mirror_circles_list_.count].radius = circle_r[i];
    car_with_mirror_circles_list_.count++;
  }

  car_without_mirror_circles_list_.Reset();
  // todo: use right circle and height type
  car_without_mirror_circles_list_.height_type = ApaObsHeightType::HIGH;
  circle_x = param.fold_mirror_footprint_circle_x;
  circle_y = param.fold_mirror_footprint_circle_y;
  circle_r = param.fold_mirror_footprint_circle_r;
  car_without_mirror_circles_list_.max_circle.center_local << circle_x[0],
      circle_y[0];
  car_without_mirror_circles_list_.max_circle.radius = circle_r[0];
  circles = car_without_mirror_circles_list_.circles;
  for (size_t i = 1; i < circle_x.size(); ++i) {
    if (car_without_mirror_circles_list_.count >=
        MAX_CAR_FOOTPRINT_CIRCLE_NUM) {
      break;
    }
    circles[car_without_mirror_circles_list_.count].center_local << circle_x[i],
        circle_y[i];
    circles[car_without_mirror_circles_list_.count].radius = circle_r[i];
    car_without_mirror_circles_list_.count++;
  }

  car_chassis_circles_list_.Reset();
  // todo: use right circle and height type
  car_chassis_circles_list_.height_type = ApaObsHeightType::HIGH;
  circle_x = param.fold_mirror_footprint_circle_x;
  circle_y = param.fold_mirror_footprint_circle_y;
  circle_r = param.fold_mirror_footprint_circle_r;
  car_chassis_circles_list_.max_circle.center_local << circle_x[0], circle_y[0];
  car_chassis_circles_list_.max_circle.radius = circle_r[0];
  circles = car_chassis_circles_list_.circles;
  for (size_t i = 1; i < circle_x.size(); ++i) {
    if (car_chassis_circles_list_.count >= MAX_CAR_FOOTPRINT_CIRCLE_NUM) {
      break;
    }
    circles[car_chassis_circles_list_.count].center_local << circle_x[i],
        circle_y[i];
    circles[car_chassis_circles_list_.count].radius = circle_r[i];
    car_chassis_circles_list_.count++;
  }

  need_update_buffer_ = true;
}

void BaseCollisionDetector::UpdateSafeBuffer(const double lat_buffer,
                                             const double lon_buffer) {
  lon_buffer_ = lon_buffer;
  if (!need_update_buffer_ && mathlib::IsDoubleEqual(lat_buffer, lat_buffer_)) {
    return;
  }
  need_update_buffer_ = false;
  lat_buffer_ = lat_buffer;
  Eigen::Vector2d vertex;

  // 包含左右后视镜的多边形
  car_with_mirror_polygon_vertex_with_buffer_.clear();
  car_with_mirror_polygon_vertex_with_buffer_.reserve(
      car_with_mirror_polygon_vertex_.size());
  for (const Eigen::Vector2d& pt : car_with_mirror_polygon_vertex_) {
    vertex.x() = pt.x();
    vertex.y() = (pt.y() > 0.0) ? pt.y() + lat_buffer : pt.y() - lat_buffer;
    car_with_mirror_polygon_vertex_with_buffer_.emplace_back(vertex);
  }

  // 不包含后视镜的多边形
  car_without_mirror_polygon_vertex_with_buffer_.clear();
  car_without_mirror_polygon_vertex_with_buffer_.reserve(
      car_without_mirror_polygon_vertex_.size());
  for (const Eigen::Vector2d& pt : car_without_mirror_polygon_vertex_) {
    vertex.x() = pt.x();
    vertex.y() = (pt.y() > 0.0) ? pt.y() + lat_buffer : pt.y() - lat_buffer;
    car_without_mirror_polygon_vertex_with_buffer_.emplace_back(vertex);
  }

  // 左后视镜
  left_mirror_rectangle_vertex_with_buffer_.clear();
  left_mirror_rectangle_vertex_with_buffer_.reserve(
      left_mirror_rectangle_vertex_.size());
  for (const Eigen::Vector2d& pt : left_mirror_rectangle_vertex_) {
    vertex << pt.x(), pt.y() + lat_buffer;
    left_mirror_rectangle_vertex_with_buffer_.emplace_back(vertex);
  }

  // 右后视镜
  right_mirror_rectangle_vertex_with_buffer_.clear();
  right_mirror_rectangle_vertex_with_buffer_.reserve(
      right_mirror_rectangle_vertex_.size());
  for (const Eigen::Vector2d& pt : right_mirror_rectangle_vertex_) {
    vertex << pt.x(), pt.y() - lat_buffer;
    right_mirror_rectangle_vertex_with_buffer_.emplace_back(vertex);
  }

  // 底盘矩形
  chassis_vertex_with_buffer_.clear();
  chassis_vertex_with_buffer_.reserve(chassis_vertex_.size());
  for (const Eigen::Vector2d& pt : chassis_vertex_) {
    vertex.x() = pt.x();
    vertex.y() = (pt.y() > 0.0) ? pt.y() + lat_buffer : pt.y() - lat_buffer;
    chassis_vertex_with_buffer_.emplace_back(vertex);
  }

  // 包含左右后视镜的矩形
  car_with_mirror_rectangle_vertex_with_buffer_.clear();
  car_with_mirror_rectangle_vertex_with_buffer_.reserve(
      car_with_mirror_rectangle_vertex_.size());
  for (const Eigen::Vector2d& pt : car_with_mirror_rectangle_vertex_) {
    vertex.x() = pt.x();
    vertex.y() = (pt.y() > 0.0) ? pt.y() + lat_buffer : pt.y() - lat_buffer;
    car_with_mirror_rectangle_vertex_with_buffer_.emplace_back(vertex);
  }

  // 后视镜到前悬矩形
  mirror_to_front_overhanging_rectangle_vertex_expand_front_with_buffer_
      .clear();
  mirror_to_front_overhanging_rectangle_vertex_expand_front_with_buffer_
      .reserve(4);
  for (const Eigen::Vector2d& pt :
       mirror_to_front_overhanging_rectangle_vertex_expand_front_) {
    vertex.x() = pt.x();
    vertex.y() = (pt.y() > 0.0) ? pt.y() + lat_buffer : pt.y() - lat_buffer;
    mirror_to_front_overhanging_rectangle_vertex_expand_front_with_buffer_
        .emplace_back(vertex);
  }

  // 后视镜到后悬多边形
  mirror_to_rear_overhanging_polygon_vertex_with_buffer_.clear();
  mirror_to_rear_overhanging_polygon_vertex_with_buffer_.reserve(
      mirror_to_rear_overhanging_polygon_vertex_.size());
  for (const Eigen::Vector2d& pt : mirror_to_rear_overhanging_polygon_vertex_) {
    vertex.x() = pt.x();
    vertex.y() = (pt.y() > 0.0) ? pt.y() + lat_buffer : pt.y() - lat_buffer;
    mirror_to_rear_overhanging_polygon_vertex_with_buffer_.emplace_back(vertex);
  }
}

const geometry_lib::RectangleBound BaseCollisionDetector::CalCarRectangleBound(
    const geometry_lib::PathPoint& current_pose) {
  geometry_lib::RectangleBound bound;
  geometry_lib::LocalToGlobalTf l2g_tf(current_pose.pos, current_pose.heading);
  std::vector<Eigen::Vector2d> polygon;
  polygon.reserve(car_with_mirror_rectangle_vertex_.size());
  for (const Eigen::Vector2d& pt : car_with_mirror_rectangle_vertex_) {
    polygon.emplace_back(l2g_tf.GetPos(pt));
  }
  bound.CalcBoundByPtVec(polygon);
  return bound;
}

const bool BaseCollisionDetector::CheckObsMovementTypeFeasible(
    const ApaObsMovementType obs_type,
    const ApaObsMovementType obs_type_request) {
  if (obs_type_request == ApaObsMovementType::ALL) {
    return true;
  }
  if (obs_type_request == obs_type) {
    return true;
  }
  return false;
}

}  // namespace apa_planner
}  // namespace planning

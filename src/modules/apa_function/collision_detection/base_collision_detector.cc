#include "base_collision_detector.h"

#include "apa_obstacle.h"
#include "apa_param_config.h"
#include "log_glog.h"

namespace planning {
namespace apa_planner {

void BaseCollisionDetector::Init() {
  const ApaParameters& param = apa_param.GetParam();
  Eigen::Vector2d vertex;

  // 包含左右后视镜的多边形
  car_with_mirror_polygon_vertex_.clear();
  car_with_mirror_polygon_vertex_.reserve(param.car_vertex_x_vec.size());
  for (size_t i = 0; i < param.car_vertex_x_vec.size(); ++i) {
    vertex << param.car_vertex_x_vec[i], param.car_vertex_y_vec[i];
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
      0.5 * param.max_car_width;
  car_with_mirror_rectangle_vertex_[1] << -param.rear_overhanging,
      -0.5 * param.max_car_width;
  car_with_mirror_rectangle_vertex_[2]
      << param.wheel_base + param.front_overhanging,
      -0.5 * param.max_car_width;
  car_with_mirror_rectangle_vertex_[3]
      << param.wheel_base + param.front_overhanging,
      0.5 * param.max_car_width;

  // 后视镜到前悬矩形
  mirror_to_front_overhanging_rectangle_vertex_expand_front_.clear();
  mirror_to_front_overhanging_rectangle_vertex_expand_front_.resize(4);
  mirror_to_front_overhanging_rectangle_vertex_expand_front_[0] =
      left_mirror_rectangle_vertex_[2];
  mirror_to_front_overhanging_rectangle_vertex_expand_front_[1] =
      right_mirror_rectangle_vertex_[1];
  mirror_to_front_overhanging_rectangle_vertex_expand_front_[2]
      << param.wheel_base + param.front_overhanging,
      -0.5 * param.max_car_width;
  mirror_to_front_overhanging_rectangle_vertex_expand_front_[3]
      << param.wheel_base + param.front_overhanging,
      0.5 * param.max_car_width;

  // 后视镜到后悬矩形
  mirror_to_rear_overhanging_rectangle_vertex_.clear();
  mirror_to_rear_overhanging_rectangle_vertex_.resize(4);
  mirror_to_rear_overhanging_rectangle_vertex_[0] =
      left_mirror_rectangle_vertex_[3];
  mirror_to_rear_overhanging_rectangle_vertex_[1]
      << -param.rear_overhanging,
      0.5 * param.max_car_width;
  mirror_to_rear_overhanging_rectangle_vertex_[2]
      << -param.rear_overhanging,
      -0.5 * param.max_car_width;
  mirror_to_rear_overhanging_rectangle_vertex_[3] =
      right_mirror_rectangle_vertex_[0];
}

void BaseCollisionDetector::UpdateSafeBuffer(const double lat_buffer,
                                             const double lon_buffer) {
  lon_buffer_ = lon_buffer;
  if (mathlib::IsDoubleEqual(lat_buffer, lat_buffer_)) {
    return;
  }
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
  mirror_to_front_overhanging_rectangle_vertex_expand_front_with_buffer_.clear();
  mirror_to_front_overhanging_rectangle_vertex_expand_front_with_buffer_.reserve(4);
  for (const Eigen::Vector2d& pt :
       mirror_to_front_overhanging_rectangle_vertex_expand_front_) {
    vertex.x() = pt.x();
    vertex.y() = (pt.y() > 0.0) ? pt.y() + lat_buffer : pt.y() - lat_buffer;
    mirror_to_front_overhanging_rectangle_vertex_expand_front_with_buffer_
        .emplace_back(vertex);
  }

  // 后视镜到后悬矩形
  mirror_to_rear_overhanging_rectangle_vertex_with_buffer_.clear();
  mirror_to_rear_overhanging_rectangle_vertex_with_buffer_.reserve(4);
  for (const Eigen::Vector2d& pt :
       mirror_to_rear_overhanging_rectangle_vertex_) {
    vertex.x() = pt.x();
    vertex.y() = (pt.y() > 0.0) ? pt.y() + lat_buffer : pt.y() - lat_buffer;
    mirror_to_rear_overhanging_rectangle_vertex_with_buffer_
        .emplace_back(vertex);
  }
}

const geometry_lib::RectangleBound BaseCollisionDetector::CalCarRectangleBound(
    const geometry_lib::PathPoint& current_pose) {
  geometry_lib::RectangleBound bound;
  bound.CalcBoundByPtVec(car_with_mirror_rectangle_vertex_);
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

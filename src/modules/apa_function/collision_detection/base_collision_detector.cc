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

  car_with_mirror_polygon_vertex_.clear();
  car_with_mirror_polygon_vertex_.reserve(car_vertex_x_vec.size());
  for (size_t i = 0; i < car_vertex_x_vec.size(); ++i) {
    vertex << car_vertex_x_vec[i], car_vertex_y_vec[i];
    car_with_mirror_polygon_vertex_.emplace_back(vertex);
  }

  std::reverse(car_with_mirror_polygon_vertex_.begin(),
               car_with_mirror_polygon_vertex_.end());

  car_without_mirror_polygon_vertex_.clear();
  car_without_mirror_polygon_vertex_.reserve(
      car_with_mirror_polygon_vertex_.size() - 8);
  for (const Eigen::Vector2d& pt : car_with_mirror_polygon_vertex_) {
    if (pt.x() > 0.0 && pt.x() < param.lon_dist_mirror_to_rear_axle + 0.68) {
      continue;
    }
    car_without_mirror_polygon_vertex_.emplace_back(pt);
  }

  left_mirror_rectangle_vertex_.clear();
  left_mirror_rectangle_vertex_.reserve(4);
  for (const Eigen::Vector2d& pt : car_with_mirror_polygon_vertex_) {
    if (pt.x() > 0.0 && pt.x() < param.lon_dist_mirror_to_rear_axle + 0.68 &&
        pt.y() > 0.0) {
      left_mirror_rectangle_vertex_.emplace_back(pt);
    }
  }

  right_mirror_rectangle_vertex_.clear();
  right_mirror_rectangle_vertex_.reserve(4);
  for (const Eigen::Vector2d& pt : car_with_mirror_polygon_vertex_) {
    if (pt.x() > 0.0 && pt.x() < param.lon_dist_mirror_to_rear_axle + 0.68 &&
        pt.y() < 0.0) {
      right_mirror_rectangle_vertex_.emplace_back(pt);
    }
  }

  chassis_vertex_.clear();
  chassis_vertex_.resize(car_without_mirror_polygon_vertex_.size());
  for (size_t i = 0; i < chassis_vertex_.size(); ++i) {
    chassis_vertex_[i] << car_without_mirror_polygon_vertex_[i].x(),
        car_without_mirror_polygon_vertex_[i].y();
    if (chassis_vertex_[i].x() > 0.0) {
      chassis_vertex_[i].x() -= param.chassis_reduce_length;
    } else {
      chassis_vertex_[i].x() += param.chassis_reduce_length;
    }
  }

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

  car_with_mirror_rectangle_vertexf_.clear();
  car_with_mirror_rectangle_vertexf_.resize(4);
  car_with_mirror_rectangle_vertexf_[0] << -param.rear_overhanging,
      0.5 * max_car_width;
  car_with_mirror_rectangle_vertexf_[1] << -param.rear_overhanging,
      -0.5 * max_car_width;
  car_with_mirror_rectangle_vertexf_[2]
      << param.wheel_base + param.front_overhanging,
      -0.5 * max_car_width;
  car_with_mirror_rectangle_vertexf_[3]
      << param.wheel_base + param.front_overhanging,
      0.5 * max_car_width;

  car_without_mirror_rectangle_vertex_.clear();
  car_without_mirror_rectangle_vertex_.resize(4);
  car_without_mirror_rectangle_vertex_[0] << -param.rear_overhanging,
      0.5 * param.car_width;
  car_without_mirror_rectangle_vertex_[1] << -param.rear_overhanging,
      -0.5 * param.car_width;
  car_without_mirror_rectangle_vertex_[2]
      << param.wheel_base + param.front_overhanging,
      -0.5 * param.car_width;
  car_without_mirror_rectangle_vertex_[3]
      << param.wheel_base + param.front_overhanging,
      0.5 * param.car_width;

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

  mirror_to_rear_overhanging_rectangle_vertex_expand_rear_.clear();
  mirror_to_rear_overhanging_rectangle_vertex_expand_rear_.resize(4);
  mirror_to_rear_overhanging_rectangle_vertex_expand_rear_[0] =
      left_mirror_rectangle_vertex_[1];
  mirror_to_rear_overhanging_rectangle_vertex_expand_rear_[1]
      << -param.rear_overhanging,
      0.5 * max_car_width;
  mirror_to_rear_overhanging_rectangle_vertex_expand_rear_[2]
      << -param.rear_overhanging,
      -0.5 * max_car_width;
  mirror_to_rear_overhanging_rectangle_vertex_expand_rear_[3] =
      right_mirror_rectangle_vertex_[2];

  mirror_to_front_overhanging_polygon_vertex_.clear();
  mirror_to_front_overhanging_polygon_vertex_.reserve(8);
  mirror_to_front_overhanging_polygon_vertex_.emplace_back(
      right_mirror_rectangle_vertex_[3]);
  for (const Eigen::Vector2d& pt : car_with_mirror_polygon_vertex_) {
    if (pt.x() < param.lon_dist_mirror_to_rear_axle + 0.68) {
      continue;
    }
    mirror_to_front_overhanging_polygon_vertex_.emplace_back(pt);
  }
  mirror_to_front_overhanging_polygon_vertex_.emplace_back(
      left_mirror_rectangle_vertex_[0]);

  car_with_mirror_circles_list_.Reset();
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
  car_without_mirror_circles_list_.max_circle.center_local << circle_x[0],
      circle_y[0];
  car_without_mirror_circles_list_.max_circle.radius = circle_r[0];
  circles = car_without_mirror_circles_list_.circles;
  for (size_t i = 1; i < circle_x.size(); ++i) {
    if (car_without_mirror_circles_list_.count >=
        MAX_CAR_FOOTPRINT_CIRCLE_NUM) {
      break;
    }

    if (i == 3 || i == 6) {
      // the circle corresponding to mirror
      continue;
    }

    circles[car_without_mirror_circles_list_.count].center_local << circle_x[i],
        circle_y[i];
    circles[car_without_mirror_circles_list_.count].radius = circle_r[i];
    car_without_mirror_circles_list_.count++;
  }

  car_chassis_circles_list_.Reset();
  car_chassis_circles_list_.max_circle.center_local << circle_x[0], circle_y[0];
  car_chassis_circles_list_.max_circle.radius = circle_r[0];
  circles = car_chassis_circles_list_.circles;
  for (size_t i = 1; i < circle_x.size(); ++i) {
    if (car_chassis_circles_list_.count >= MAX_CAR_FOOTPRINT_CIRCLE_NUM) {
      break;
    }

    if (i == 3 || i == 6) {
      // the circle corresponding to mirror
      continue;
    }

    circles[car_chassis_circles_list_.count].center_local << circle_x[i],
        circle_y[i];
    circles[car_chassis_circles_list_.count].radius = circle_r[i];

    if (i == 1 || i == 2 || i == 7) {
      // the circle corresponding to front_overhanging
      circles[car_chassis_circles_list_.count].center_local.x() -=
          param.chassis_reduce_length;
    } else if (i == 4 || i == 5 || i == 10) {
      // the circle corresponding to rear_overhanging
      circles[car_chassis_circles_list_.count].center_local.x() +=
          param.chassis_reduce_length;
    }

    car_chassis_circles_list_.count++;
  }

  need_update_buffer_ = true;
}

void BaseCollisionDetector::UpdateSafeBuffer(
    const ColDetBuffer& col_det_buffer) {
  if (!need_update_buffer_ &&
      mathlib::IsDoubleEqual(col_det_buffer.body_lat_buffer,
                             col_det_buffer_.body_lat_buffer) &&
      mathlib::IsDoubleEqual(col_det_buffer.mirror_lat_buffer,
                             col_det_buffer_.mirror_lat_buffer)) {
    col_det_buffer_.lon_buffer = col_det_buffer.lon_buffer;
    return;
  }

  need_update_buffer_ = false;
  col_det_buffer_ = col_det_buffer;
  Eigen::Vector2d vertex;

  const ApaParameters& param = apa_param.GetParam();

  car_with_mirror_polygon_vertex_with_buffer_.clear();
  car_with_mirror_polygon_vertex_with_buffer_.reserve(
      car_with_mirror_polygon_vertex_.size());
  for (const Eigen::Vector2d& pt : car_with_mirror_polygon_vertex_) {
    vertex.x() = pt.x();

    if (pt.x() > 0.0 && pt.x() < param.lon_dist_mirror_to_rear_axle + 0.68) {
      vertex.y() = (pt.y() > 0.0) ? pt.y() + col_det_buffer_.mirror_lat_buffer
                                  : pt.y() - col_det_buffer_.mirror_lat_buffer;
    } else {
      vertex.y() = (pt.y() > 0.0) ? pt.y() + col_det_buffer_.body_lat_buffer
                                  : pt.y() - col_det_buffer_.body_lat_buffer;
    }

    car_with_mirror_polygon_vertex_with_buffer_.emplace_back(vertex);
  }

  car_without_mirror_polygon_vertex_with_buffer_.clear();
  car_without_mirror_polygon_vertex_with_buffer_.reserve(
      car_without_mirror_polygon_vertex_.size());
  for (const Eigen::Vector2d& pt : car_without_mirror_polygon_vertex_) {
    vertex.x() = pt.x();
    vertex.y() =
        (pt.y() > 0.0) ? pt.y() + col_det_buffer_.body_lat_buffer : pt.y() - col_det_buffer_.body_lat_buffer;
    car_without_mirror_polygon_vertex_with_buffer_.emplace_back(vertex);
  }

  left_mirror_rectangle_vertex_with_buffer_.clear();
  left_mirror_rectangle_vertex_with_buffer_.reserve(
      left_mirror_rectangle_vertex_.size());
  for (const Eigen::Vector2d& pt : left_mirror_rectangle_vertex_) {
    vertex << pt.x(), pt.y();
    if (std::fabs(pt.y()) > param.car_width * 0.5) {
      vertex.y() += col_det_buffer_.mirror_lat_buffer;
    }
    left_mirror_rectangle_vertex_with_buffer_.emplace_back(vertex);
  }

  right_mirror_rectangle_vertex_with_buffer_.clear();
  right_mirror_rectangle_vertex_with_buffer_.reserve(
      right_mirror_rectangle_vertex_.size());
  for (const Eigen::Vector2d& pt : right_mirror_rectangle_vertex_) {
    vertex << pt.x(), pt.y();
    if (std::fabs(pt.y()) > param.car_width * 0.5) {
      vertex.y() -= col_det_buffer_.mirror_lat_buffer;
    }
    right_mirror_rectangle_vertex_with_buffer_.emplace_back(vertex);
  }

  chassis_vertex_with_buffer_.clear();
  chassis_vertex_with_buffer_.reserve(chassis_vertex_.size());
  for (const Eigen::Vector2d& pt : chassis_vertex_) {
    vertex.x() = pt.x();
    vertex.y() =
        (pt.y() > 0.0) ? pt.y() + col_det_buffer_.body_lat_buffer : pt.y() - col_det_buffer_.body_lat_buffer;
    chassis_vertex_with_buffer_.emplace_back(vertex);
  }

  car_with_mirror_rectangle_vertex_with_buffer_.clear();
  car_with_mirror_rectangle_vertex_with_buffer_.reserve(
      car_with_mirror_rectangle_vertex_.size());
  for (const Eigen::Vector2d& pt : car_with_mirror_rectangle_vertex_) {
    vertex.x() = pt.x();
    vertex.y() = (pt.y() > 0.0) ? pt.y() + col_det_buffer_.mirror_lat_buffer
                                : pt.y() - col_det_buffer_.mirror_lat_buffer;
    car_with_mirror_rectangle_vertex_with_buffer_.emplace_back(vertex);
  }

  car_without_mirror_rectangle_vertex_with_buffer_.clear();
  car_without_mirror_rectangle_vertex_with_buffer_.reserve(
      car_without_mirror_rectangle_vertex_.size());
  for (const Eigen::Vector2d& pt : car_without_mirror_rectangle_vertex_) {
    vertex.x() = pt.x();
    vertex.y() =
        (pt.y() > 0.0) ? pt.y() + col_det_buffer_.body_lat_buffer : pt.y() - col_det_buffer_.body_lat_buffer;
    car_without_mirror_rectangle_vertex_with_buffer_.emplace_back(vertex);
  }

  mirror_to_front_overhanging_rectangle_vertex_expand_front_with_buffer_
      .clear();
  mirror_to_front_overhanging_rectangle_vertex_expand_front_with_buffer_
      .reserve(4);
  for (const Eigen::Vector2d& pt :
       mirror_to_front_overhanging_rectangle_vertex_expand_front_) {
    vertex.x() = pt.x();
    vertex.y() = (pt.y() > 0.0) ? pt.y() + col_det_buffer_.mirror_lat_buffer
                                : pt.y() - col_det_buffer_.mirror_lat_buffer;
    mirror_to_front_overhanging_rectangle_vertex_expand_front_with_buffer_
        .emplace_back(vertex);
  }

  mirror_to_rear_overhanging_polygon_vertex_with_buffer_.clear();
  mirror_to_rear_overhanging_polygon_vertex_with_buffer_.reserve(
      mirror_to_rear_overhanging_polygon_vertex_.size());
  for (const Eigen::Vector2d& pt : mirror_to_rear_overhanging_polygon_vertex_) {
    vertex.x() = pt.x();
    vertex.y() =
        (pt.y() > 0.0) ? pt.y() + col_det_buffer_.body_lat_buffer : pt.y() - col_det_buffer_.body_lat_buffer;
    mirror_to_rear_overhanging_polygon_vertex_with_buffer_.emplace_back(vertex);
  }

  mirror_to_rear_overhanging_rectangle_vertex_expand_rear_with_buffer_.clear();
  mirror_to_rear_overhanging_rectangle_vertex_expand_rear_with_buffer_.reserve(
      mirror_to_rear_overhanging_rectangle_vertex_expand_rear_.size());
  for (const Eigen::Vector2d& pt :
       mirror_to_rear_overhanging_rectangle_vertex_expand_rear_) {
    vertex.x() = pt.x();
    vertex.y() = (pt.y() > 0.0) ? pt.y() + col_det_buffer_.mirror_lat_buffer
                                : pt.y() - col_det_buffer_.mirror_lat_buffer;
    mirror_to_rear_overhanging_rectangle_vertex_expand_rear_with_buffer_
        .emplace_back(vertex);
  }

  mirror_to_front_overhanging_polygon_vertex_with_buffer_.clear();
  mirror_to_front_overhanging_polygon_vertex_with_buffer_.reserve(
      mirror_to_front_overhanging_polygon_vertex_.size());
  for (const Eigen::Vector2d& pt :
       mirror_to_front_overhanging_polygon_vertex_) {
    vertex.x() = pt.x();
    vertex.y() =
        (pt.y() > 0.0) ? pt.y() + col_det_buffer_.body_lat_buffer : pt.y() - col_det_buffer_.body_lat_buffer;
    mirror_to_front_overhanging_polygon_vertex_with_buffer_.emplace_back(
        vertex);
  }

  // todo: 根据轮胎具体角度来加上横向buffer，但是这样会导致每次重复计算sin
  // cos值, 且收益意义不大，可以直接赋值
  left_mirror_rectangle_vertex_with_buffer_ = left_tyre_rectangle_vertex_;
  right_mirror_rectangle_vertex_with_buffer_ = right_tyre_rectangle_vertex_;
}

void BaseCollisionDetector::UpdateObsClearZone(
    const std::vector<Eigen::Vector2d>& pt_vec) {
  obs_clear_zone_decider_.GenerateBoundingBox(pt_vec, obs_manager_ptr_);
}

const bool BaseCollisionDetector::IsPoseInClearZone(
    const geometry_lib::PathPoint& pose) {
  const geometry_lib::RectangleBound bound = CalCarRectangleBound(pose);
  return obs_clear_zone_decider_.IsInClearZone(bound);
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

const bool BaseCollisionDetector::IsPoseInClearZone(
    const common_math::PathPt<float>& pose) {
  return obs_clear_zone_decider_.IsInClearZone(CalCarAABBBoxF(pose));
}

const cdl::AABB2f BaseCollisionDetector::CalCarAABBBoxF(
    const common_math::PathPt<float>& pose) {
  common_math::Local2GlobalTrans<float> l2g_tf(pose.pos, pose.theta);
  std::vector<common_math::Pos<float>> pos_vec;
  pos_vec.resize(car_with_mirror_rectangle_vertexf_.size());
  for (size_t i = 0; i < car_with_mirror_rectangle_vertexf_.size(); ++i) {
    pos_vec[i] = l2g_tf.GetPos(car_with_mirror_rectangle_vertexf_[i]);
  }
  return cdl::AABB2f(pos_vec);
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

const std::vector<Eigen::Vector2d>
BaseCollisionDetector::GetCarBigBoxWithBuffer(
    const double lat_buf, const double lon_buf,
    const geometry_lib::PathPoint& pose) {
  const ApaParameters& param = apa_param.GetParam();
  const double half_car_width = param.max_car_width * 0.5;
  const double max_x = param.wheel_base + param.front_overhanging;
  const double min_x = param.rear_overhanging;
  std::vector<Eigen::Vector2d> car_box;
  car_box.resize(4);
  car_box[0] = Eigen::Vector2d(max_x + lon_buf, half_car_width + lat_buf);
  car_box[1] = Eigen::Vector2d(min_x - lon_buf, half_car_width + lat_buf);
  car_box[2] = Eigen::Vector2d(min_x - lon_buf, -half_car_width - lat_buf);
  car_box[3] = Eigen::Vector2d(max_x + lon_buf, -half_car_width - lat_buf);
  // Transform car_box to global coordinate
  geometry_lib::LocalToGlobalTf l2g_tf(pose.pos, pose.heading);
  for (Eigen::Vector2d& pt : car_box) {
    pt = l2g_tf.GetPos(pt);
  }
  return car_box;
}

void BaseCollisionDetector::GenRealTimeTyrePolygonAccordingToFrontWheelAngle(
    const double front_wheel_angle) {
  const auto& param = apa_param.GetParam();
  const double half_car_width = param.car_width * 0.5;
  const double half_tyre_width = 0.25 * 0.5;
  const double half_tyre_length = 0.75 * 0.5;

  // front_wheel_angle is rads, Positive values indicate rotation to the left,
  // while negative values indicate rotation to the right
  const Eigen::Rotation2Dd rot(front_wheel_angle);
  const Eigen::Vector2d r0 =
      rot * Eigen::Vector2d(half_tyre_length, half_tyre_width);
  const Eigen::Vector2d r1 =
      rot * Eigen::Vector2d(-half_tyre_length, half_tyre_width);
  const Eigen::Vector2d r2 =
      rot * Eigen::Vector2d(-half_tyre_length, -half_tyre_width);
  const Eigen::Vector2d r3 =
      rot * Eigen::Vector2d(half_tyre_length, -half_tyre_width);

  const Eigen::Vector2d left_center(param.wheel_base, half_car_width);
  const Eigen::Vector2d right_center(param.wheel_base, -half_car_width);

  left_tyre_rectangle_vertex_ = {left_center + r0, left_center + r1,
                                 left_center + r2, left_center + r3};
  right_tyre_rectangle_vertex_ = {right_center + r0, right_center + r3,
                                  right_center + r2, right_center + r1};
}

}  // namespace apa_planner
}  // namespace planning

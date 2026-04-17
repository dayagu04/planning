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

  car_shape_vertex_.polygon_with_mirror.clear();
  car_shape_vertex_.polygon_with_mirror.reserve(car_vertex_x_vec.size());
  for (size_t i = 0; i < car_vertex_x_vec.size(); ++i) {
    vertex << car_vertex_x_vec[i], car_vertex_y_vec[i];
    car_shape_vertex_.polygon_with_mirror.emplace_back(vertex);
  }

  std::reverse(car_shape_vertex_.polygon_with_mirror.begin(),
               car_shape_vertex_.polygon_with_mirror.end());

  car_shape_vertex_.polygon_without_mirror.clear();
  car_shape_vertex_.polygon_without_mirror.reserve(
      car_shape_vertex_.polygon_with_mirror.size() - 8);
  for (const Eigen::Vector2d& pt : car_shape_vertex_.polygon_with_mirror) {
    if (pt.x() > 0.0 && pt.x() < param.lon_dist_mirror_to_rear_axle + 0.68) {
      continue;
    }
    car_shape_vertex_.polygon_without_mirror.emplace_back(pt);
  }

  car_shape_vertex_.left_mirror_rectangle.clear();
  car_shape_vertex_.left_mirror_rectangle.reserve(4);
  for (const Eigen::Vector2d& pt : car_shape_vertex_.polygon_with_mirror) {
    if (pt.x() > 0.0 && pt.x() < param.lon_dist_mirror_to_rear_axle + 0.68 &&
        pt.y() > 0.0) {
      car_shape_vertex_.left_mirror_rectangle.emplace_back(pt);
    }
  }

  car_shape_vertex_.right_mirror_rectangle.clear();
  car_shape_vertex_.right_mirror_rectangle.reserve(4);
  for (const Eigen::Vector2d& pt : car_shape_vertex_.polygon_with_mirror) {
    if (pt.x() > 0.0 && pt.x() < param.lon_dist_mirror_to_rear_axle + 0.68 &&
        pt.y() < 0.0) {
      car_shape_vertex_.right_mirror_rectangle.emplace_back(pt);
    }
  }

  car_shape_vertex_.chassis_polygon.clear();
  car_shape_vertex_.chassis_polygon.resize(
      car_shape_vertex_.polygon_without_mirror.size());
  for (size_t i = 0; i < car_shape_vertex_.chassis_polygon.size(); ++i) {
    car_shape_vertex_.chassis_polygon[i]
        << car_shape_vertex_.polygon_without_mirror[i].x(),
        car_shape_vertex_.polygon_without_mirror[i].y();
    if (car_shape_vertex_.chassis_polygon[i].x() > 0.0) {
      car_shape_vertex_.chassis_polygon[i].x() -= param.chassis_reduce_length;
    } else {
      car_shape_vertex_.chassis_polygon[i].x() += param.chassis_reduce_length;
    }
  }

  car_shape_vertex_.rectangle_with_mirror.clear();
  car_shape_vertex_.rectangle_with_mirror.resize(4);
  car_shape_vertex_.rectangle_with_mirror[0] << -param.rear_overhanging,
      0.5 * max_car_width;
  car_shape_vertex_.rectangle_with_mirror[1] << -param.rear_overhanging,
      -0.5 * max_car_width;
  car_shape_vertex_.rectangle_with_mirror[2]
      << param.wheel_base + param.front_overhanging,
      -0.5 * max_car_width;
  car_shape_vertex_.rectangle_with_mirror[3]
      << param.wheel_base + param.front_overhanging,
      0.5 * max_car_width;

  car_shape_vertex_.rectangle_with_mirror_f.clear();
  car_shape_vertex_.rectangle_with_mirror_f.resize(4);
  car_shape_vertex_.rectangle_with_mirror_f[0] << -param.rear_overhanging,
      0.5 * max_car_width;
  car_shape_vertex_.rectangle_with_mirror_f[1] << -param.rear_overhanging,
      -0.5 * max_car_width;
  car_shape_vertex_.rectangle_with_mirror_f[2]
      << param.wheel_base + param.front_overhanging,
      -0.5 * max_car_width;
  car_shape_vertex_.rectangle_with_mirror_f[3]
      << param.wheel_base + param.front_overhanging,
      0.5 * max_car_width;

  car_shape_vertex_.rectangle_without_mirror.clear();
  car_shape_vertex_.rectangle_without_mirror.resize(4);
  car_shape_vertex_.rectangle_without_mirror[0] << -param.rear_overhanging,
      0.5 * param.car_width;
  car_shape_vertex_.rectangle_without_mirror[1] << -param.rear_overhanging,
      -0.5 * param.car_width;
  car_shape_vertex_.rectangle_without_mirror[2]
      << param.wheel_base + param.front_overhanging,
      -0.5 * param.car_width;
  car_shape_vertex_.rectangle_without_mirror[3]
      << param.wheel_base + param.front_overhanging,
      0.5 * param.car_width;

  car_shape_vertex_.mirror_to_front_overhanging_rectangle_expand_front.clear();
  car_shape_vertex_.mirror_to_front_overhanging_rectangle_expand_front.resize(
      4);
  car_shape_vertex_.mirror_to_front_overhanging_rectangle_expand_front[0] =
      car_shape_vertex_.left_mirror_rectangle[2];
  car_shape_vertex_.mirror_to_front_overhanging_rectangle_expand_front[1] =
      car_shape_vertex_.right_mirror_rectangle[1];
  car_shape_vertex_.mirror_to_front_overhanging_rectangle_expand_front[2]
      << param.wheel_base + param.front_overhanging,
      -0.5 * max_car_width;
  car_shape_vertex_.mirror_to_front_overhanging_rectangle_expand_front[3]
      << param.wheel_base + param.front_overhanging,
      0.5 * max_car_width;

  car_shape_vertex_.mirror_to_rear_overhanging_polygon.clear();
  car_shape_vertex_.mirror_to_rear_overhanging_polygon.reserve(8);
  car_shape_vertex_.mirror_to_rear_overhanging_polygon.emplace_back(
      car_shape_vertex_.left_mirror_rectangle[3]);
  for (const Eigen::Vector2d& pt : car_shape_vertex_.polygon_with_mirror) {
    if (pt.x() > 0.0) {
      continue;
    }
    car_shape_vertex_.mirror_to_rear_overhanging_polygon.emplace_back(pt);
  }
  car_shape_vertex_.mirror_to_rear_overhanging_polygon.emplace_back(
      car_shape_vertex_.right_mirror_rectangle[0]);

  car_shape_vertex_.mirror_to_rear_overhanging_rectangle_expand_rear.clear();
  car_shape_vertex_.mirror_to_rear_overhanging_rectangle_expand_rear.resize(4);
  car_shape_vertex_.mirror_to_rear_overhanging_rectangle_expand_rear[0] =
      car_shape_vertex_.left_mirror_rectangle[1];
  car_shape_vertex_.mirror_to_rear_overhanging_rectangle_expand_rear[1]
      << -param.rear_overhanging,
      0.5 * max_car_width;
  car_shape_vertex_.mirror_to_rear_overhanging_rectangle_expand_rear[2]
      << -param.rear_overhanging,
      -0.5 * max_car_width;
  car_shape_vertex_.mirror_to_rear_overhanging_rectangle_expand_rear[3] =
      car_shape_vertex_.right_mirror_rectangle[2];

  car_shape_vertex_.mirror_to_front_overhanging_polygon.clear();
  car_shape_vertex_.mirror_to_front_overhanging_polygon.reserve(8);
  car_shape_vertex_.mirror_to_front_overhanging_polygon.emplace_back(
      car_shape_vertex_.right_mirror_rectangle[3]);
  for (const Eigen::Vector2d& pt : car_shape_vertex_.polygon_with_mirror) {
    if (pt.x() < param.lon_dist_mirror_to_rear_axle + 0.68) {
      continue;
    }
    car_shape_vertex_.mirror_to_front_overhanging_polygon.emplace_back(pt);
  }
  car_shape_vertex_.mirror_to_front_overhanging_polygon.emplace_back(
      car_shape_vertex_.left_mirror_rectangle[0]);

  multi_car_shape_circle_.circles_with_mirror.Reset();
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
  multi_car_shape_circle_.circles_with_mirror.max_circle.center_local
      << circle_x[0],
      circle_y[0];
  multi_car_shape_circle_.circles_with_mirror.max_circle.radius = circle_r[0];
  CarFootPrintCircle* circles =
      multi_car_shape_circle_.circles_with_mirror.circles;
  for (size_t i = 1; i < circle_x.size(); ++i) {
    if (multi_car_shape_circle_.circles_with_mirror.count >=
        MAX_CAR_FOOTPRINT_CIRCLE_NUM) {
      break;
    }
    circles[multi_car_shape_circle_.circles_with_mirror.count].center_local
        << circle_x[i],
        circle_y[i];
    circles[multi_car_shape_circle_.circles_with_mirror.count].radius =
        circle_r[i];
    multi_car_shape_circle_.circles_with_mirror.count++;
  }

  multi_car_shape_circle_.circles_without_mirror.Reset();
  multi_car_shape_circle_.circles_without_mirror.max_circle.center_local
      << circle_x[0],
      circle_y[0];
  multi_car_shape_circle_.circles_without_mirror.max_circle.radius =
      circle_r[0];
  circles = multi_car_shape_circle_.circles_without_mirror.circles;
  for (size_t i = 1; i < circle_x.size(); ++i) {
    if (multi_car_shape_circle_.circles_without_mirror.count >=
        MAX_CAR_FOOTPRINT_CIRCLE_NUM) {
      break;
    }

    if (i == 3 || i == 6) {
      // the circle corresponding to mirror
      continue;
    }

    circles[multi_car_shape_circle_.circles_without_mirror.count].center_local
        << circle_x[i],
        circle_y[i];
    circles[multi_car_shape_circle_.circles_without_mirror.count].radius =
        circle_r[i];
    multi_car_shape_circle_.circles_without_mirror.count++;
  }

  multi_car_shape_circle_.chassis_circles.Reset();
  multi_car_shape_circle_.chassis_circles.max_circle.center_local
      << circle_x[0],
      circle_y[0];
  multi_car_shape_circle_.chassis_circles.max_circle.radius = circle_r[0];
  circles = multi_car_shape_circle_.chassis_circles.circles;
  for (size_t i = 1; i < circle_x.size(); ++i) {
    if (multi_car_shape_circle_.chassis_circles.count >=
        MAX_CAR_FOOTPRINT_CIRCLE_NUM) {
      break;
    }

    if (i == 3 || i == 6) {
      // the circle corresponding to mirror
      continue;
    }

    circles[multi_car_shape_circle_.chassis_circles.count].center_local
        << circle_x[i],
        circle_y[i];
    circles[multi_car_shape_circle_.chassis_circles.count].radius = circle_r[i];

    if (i == 1 || i == 2 || i == 7) {
      // the circle corresponding to front_overhanging
      circles[multi_car_shape_circle_.chassis_circles.count].center_local.x() -=
          param.chassis_reduce_length;
    } else if (i == 4 || i == 5 || i == 10) {
      // the circle corresponding to rear_overhanging
      circles[multi_car_shape_circle_.chassis_circles.count].center_local.x() +=
          param.chassis_reduce_length;
    }

    multi_car_shape_circle_.chassis_circles.count++;
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

  car_shape_vertex_with_buffer_.polygon_with_mirror.clear();
  car_shape_vertex_with_buffer_.polygon_with_mirror.reserve(
      car_shape_vertex_.polygon_with_mirror.size());
  for (const Eigen::Vector2d& pt : car_shape_vertex_.polygon_with_mirror) {
    vertex.x() = pt.x();

    if (pt.x() > 0.0 && pt.x() < param.lon_dist_mirror_to_rear_axle + 0.68) {
      vertex.y() = (pt.y() > 0.0) ? pt.y() + col_det_buffer_.mirror_lat_buffer
                                  : pt.y() - col_det_buffer_.mirror_lat_buffer;
    } else {
      vertex.y() = (pt.y() > 0.0) ? pt.y() + col_det_buffer_.body_lat_buffer
                                  : pt.y() - col_det_buffer_.body_lat_buffer;
    }

    car_shape_vertex_with_buffer_.polygon_with_mirror.emplace_back(vertex);
  }

  car_shape_vertex_with_buffer_.polygon_without_mirror.clear();
  car_shape_vertex_with_buffer_.polygon_without_mirror.reserve(
      car_shape_vertex_.polygon_without_mirror.size());
  for (const Eigen::Vector2d& pt : car_shape_vertex_.polygon_without_mirror) {
    vertex.x() = pt.x();
    vertex.y() = (pt.y() > 0.0) ? pt.y() + col_det_buffer_.body_lat_buffer
                                : pt.y() - col_det_buffer_.body_lat_buffer;
    car_shape_vertex_with_buffer_.polygon_without_mirror.emplace_back(vertex);
  }

  car_shape_vertex_with_buffer_.left_mirror_rectangle.clear();
  car_shape_vertex_with_buffer_.left_mirror_rectangle.reserve(
      car_shape_vertex_.left_mirror_rectangle.size());
  for (const Eigen::Vector2d& pt : car_shape_vertex_.left_mirror_rectangle) {
    vertex << pt.x(), pt.y();
    if (std::fabs(pt.y()) > param.car_width * 0.5) {
      vertex.y() += col_det_buffer_.mirror_lat_buffer;
    }
    car_shape_vertex_with_buffer_.left_mirror_rectangle.emplace_back(vertex);
  }

  car_shape_vertex_with_buffer_.right_mirror_rectangle.clear();
  car_shape_vertex_with_buffer_.right_mirror_rectangle.reserve(
      car_shape_vertex_.right_mirror_rectangle.size());
  for (const Eigen::Vector2d& pt : car_shape_vertex_.right_mirror_rectangle) {
    vertex << pt.x(), pt.y();
    if (std::fabs(pt.y()) > param.car_width * 0.5) {
      vertex.y() -= col_det_buffer_.mirror_lat_buffer;
    }
    car_shape_vertex_with_buffer_.right_mirror_rectangle.emplace_back(vertex);
  }

  car_shape_vertex_with_buffer_.chassis_polygon.clear();
  car_shape_vertex_with_buffer_.chassis_polygon.reserve(
      car_shape_vertex_.chassis_polygon.size());
  for (const Eigen::Vector2d& pt : car_shape_vertex_.chassis_polygon) {
    vertex.x() = pt.x();
    vertex.y() = (pt.y() > 0.0) ? pt.y() + col_det_buffer_.body_lat_buffer
                                : pt.y() - col_det_buffer_.body_lat_buffer;
    car_shape_vertex_with_buffer_.chassis_polygon.emplace_back(vertex);
  }

  car_shape_vertex_with_buffer_.rectangle_with_mirror.clear();
  car_shape_vertex_with_buffer_.rectangle_with_mirror.reserve(
      car_shape_vertex_.rectangle_with_mirror.size());
  for (const Eigen::Vector2d& pt : car_shape_vertex_.rectangle_with_mirror) {
    vertex.x() = pt.x();
    vertex.y() = (pt.y() > 0.0) ? pt.y() + col_det_buffer_.mirror_lat_buffer
                                : pt.y() - col_det_buffer_.mirror_lat_buffer;
    car_shape_vertex_with_buffer_.rectangle_with_mirror.emplace_back(vertex);
  }

  car_shape_vertex_with_buffer_.rectangle_without_mirror.clear();
  car_shape_vertex_with_buffer_.rectangle_without_mirror.reserve(
      car_shape_vertex_.rectangle_without_mirror.size());
  for (const Eigen::Vector2d& pt : car_shape_vertex_.rectangle_without_mirror) {
    vertex.x() = pt.x();
    vertex.y() = (pt.y() > 0.0) ? pt.y() + col_det_buffer_.body_lat_buffer
                                : pt.y() - col_det_buffer_.body_lat_buffer;
    car_shape_vertex_with_buffer_.rectangle_without_mirror.emplace_back(vertex);
  }

  car_shape_vertex_with_buffer_
      .mirror_to_front_overhanging_rectangle_expand_front.clear();
  car_shape_vertex_with_buffer_
      .mirror_to_front_overhanging_rectangle_expand_front.reserve(4);
  for (const Eigen::Vector2d& pt :
       car_shape_vertex_.mirror_to_front_overhanging_rectangle_expand_front) {
    vertex.x() = pt.x();
    vertex.y() = (pt.y() > 0.0) ? pt.y() + col_det_buffer_.mirror_lat_buffer
                                : pt.y() - col_det_buffer_.mirror_lat_buffer;
    car_shape_vertex_with_buffer_
        .mirror_to_front_overhanging_rectangle_expand_front.emplace_back(
            vertex);
  }

  car_shape_vertex_with_buffer_.mirror_to_rear_overhanging_polygon.clear();
  car_shape_vertex_with_buffer_.mirror_to_rear_overhanging_polygon.reserve(
      car_shape_vertex_.mirror_to_rear_overhanging_polygon.size());
  for (const Eigen::Vector2d& pt :
       car_shape_vertex_.mirror_to_rear_overhanging_polygon) {
    vertex.x() = pt.x();
    vertex.y() = (pt.y() > 0.0) ? pt.y() + col_det_buffer_.body_lat_buffer
                                : pt.y() - col_det_buffer_.body_lat_buffer;
    car_shape_vertex_with_buffer_.mirror_to_rear_overhanging_polygon
        .emplace_back(vertex);
  }

  car_shape_vertex_with_buffer_.mirror_to_rear_overhanging_rectangle_expand_rear
      .clear();
  car_shape_vertex_with_buffer_.mirror_to_rear_overhanging_rectangle_expand_rear
      .reserve(car_shape_vertex_
                   .mirror_to_rear_overhanging_rectangle_expand_rear.size());
  for (const Eigen::Vector2d& pt :
       car_shape_vertex_.mirror_to_rear_overhanging_rectangle_expand_rear) {
    vertex.x() = pt.x();
    vertex.y() = (pt.y() > 0.0) ? pt.y() + col_det_buffer_.mirror_lat_buffer
                                : pt.y() - col_det_buffer_.mirror_lat_buffer;
    car_shape_vertex_with_buffer_
        .mirror_to_rear_overhanging_rectangle_expand_rear.emplace_back(vertex);
  }

  car_shape_vertex_with_buffer_.mirror_to_front_overhanging_polygon.clear();
  car_shape_vertex_with_buffer_.mirror_to_front_overhanging_polygon.reserve(
      car_shape_vertex_.mirror_to_front_overhanging_polygon.size());
  for (const Eigen::Vector2d& pt :
       car_shape_vertex_.mirror_to_front_overhanging_polygon) {
    vertex.x() = pt.x();
    vertex.y() = (pt.y() > 0.0) ? pt.y() + col_det_buffer_.body_lat_buffer
                                : pt.y() - col_det_buffer_.body_lat_buffer;
    car_shape_vertex_with_buffer_.mirror_to_front_overhanging_polygon
        .emplace_back(vertex);
  }

  // todo: 根据轮胎具体角度来加上横向buffer，但是这样会导致每次重复计算sin
  // cos值, 且收益意义不大，可以直接赋值
  car_shape_vertex_with_buffer_.left_tyre_rectangle =
      car_shape_vertex_.left_tyre_rectangle;
  car_shape_vertex_with_buffer_.right_tyre_rectangle =
      car_shape_vertex_.right_tyre_rectangle;
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
  polygon.reserve(car_shape_vertex_.rectangle_with_mirror.size());
  for (const Eigen::Vector2d& pt : car_shape_vertex_.rectangle_with_mirror) {
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
  pos_vec.resize(car_shape_vertex_.rectangle_with_mirror_f.size());
  for (size_t i = 0; i < pos_vec.size(); ++i) {
    pos_vec[i] = l2g_tf.GetPos(car_shape_vertex_.rectangle_with_mirror_f[i]);
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

  car_shape_vertex_.left_tyre_rectangle = {left_center + r0, left_center + r1,
                                           left_center + r2, left_center + r3};
  car_shape_vertex_.right_tyre_rectangle = {
      right_center + r0, right_center + r3, right_center + r2,
      right_center + r1};
}

}  // namespace apa_planner
}  // namespace planning

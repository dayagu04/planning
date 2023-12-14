#include "obstacle.h"

#include "common.h"
#include "log.h"
#include "math/linear_interpolation.h"

namespace planning {

// Obstacle::Obstacle(int id,
//                    const Prediction::PredictionObject &prediction_object,
//                    const bool is_static, double start_relative_timestamp)
//     : id_(id),
//       perception_id_(prediction_object.fusion_obstacle().additional_info().track_id()),
//       is_static_(is_static),
//       perception_bounding_box_(
//           {prediction_object.fusion_obstacle().common_info().center_position().x(),
//           prediction_object.fusion_obstacle().common_info().center_position().y()},
//           prediction_object.fusion_obstacle().common_info().heading_angle(),
//           prediction_object.fusion_obstacle().common_info().shape().length(),
//           prediction_object.fusion_obstacle().common_info().shape().width())
//           {

//   perception_obstacle_.CopyFrom(prediction_object.fusion_obstacle());
//   x_center_ = perception_obstacle_.common_info().center_position().x();
//   y_center_ = perception_obstacle_.common_info().center_position().y();
//   perception_velocity_ = perception_obstacle_.common_info().velocity().x();
//   velocity_angle_ = perception_obstacle_.common_info().heading_angle();
//   // perception_obstacle_.heading_yaw =
//   //
//   planning_math::NormalizeAngle(prediction_object.perception_obstacle().heading_angle());

//   std::vector<planning_math::Vec2d> polygon_points;
//   if
//   (prediction_object.fusion_obstacle().additional_info().polygon().points_size()
//   < 3) {
//     perception_bounding_box_.GetAllCorners(&polygon_points);
//   } else {
//     for (int i = 0; i <
//     prediction_object.fusion_obstacle().additional_info().polygon().points_size()
//     - 1;
//          ++i) {
//       auto &point =
//       prediction_object.fusion_obstacle().additional_info().polygon().points(i);
//       polygon_points.emplace_back(planning_math::Vec2d(point.x(),
//       point.y()));
//     }
//   }

//   auto extracted_polygon_points = polygon_points;
//   extract_point_at_specified_resolution(extracted_polygon_points);
//   if (extracted_polygon_points.size() < 3) {
//     extracted_polygon_points = polygon_points;
//   }
//   if (!planning_math::Polygon2d::ComputeConvexHull(extracted_polygon_points,
//                                                    &perception_polygon_)) {
//     LOG_DEBUG("polygon_debug invalid cart polygon");
//   }
//   std::vector<planning_math::Vec2d> ego_polygon_points;
//   for (const auto &point : perception_polygon_.points()) {
//     ego_polygon_points.emplace_back(
//         planning_math::Vec2d(point.x() -
//         perception_obstacle_.common_info().center_position().x(),
//                              point.y() -
//                              perception_obstacle_.common_info().center_position().y()));
//   }
//   // LOG_DEBUG("obstacle[%d] last polygon size : %d", polygon_points.size());
//   if (!planning_math::Polygon2d::ComputeConvexHull(ego_polygon_points,
//                                                    &obstacle_ego_polygon_)) {
//     LOG_DEBUG("polygon_debug invalid ego polygon");
//   }

//   velocity_ =
//   prediction_object.fusion_obstacle().common_info().velocity().x(); acc_ =
//   prediction_object.fusion_obstacle().common_info().acceleration().x();
//   // acc_signed_ = acc_ > 0 ? 1 : -1 ;

//   if (is_static) {
//     return;
//   }

//   auto &prediction_trajectory = prediction_object.trajectory(0);

//   if (prediction_trajectory.trajectory_point_size() == 0) {
//     return;
//   }

//   trajectory_.clear();
//   double relative_time = start_relative_timestamp;
//   double cumulative_s = 0.0;
//   for (int i = 0; i < prediction_trajectory.trajectory_point_size(); ++i) {
//     PncTrajectoryPoint tp;
//     auto &traj_point = prediction_trajectory.trajectory_point(i);
//     tp.v = traj_point.velocity ();
//     tp.a = 0;
//     tp.prediction_prob = traj_point.confidence ();
//     tp.path_point.x = traj_point.position().x();
//     tp.path_point.y = traj_point.position().y();
//     tp.sigma_x = traj_point.gaussian_info().sigma_x();
//     tp.sigma_y = traj_point.gaussian_info().sigma_y();
//     tp.path_point.theta = planning_math::NormalizeAngle(traj_point.yaw());
//     tp.velocity_direction =
//     planning_math::NormalizeAngle(traj_point.theta()); tp.relative_ego_x =
//     traj_point.relative_position().x(); tp.relative_ego_y =
//     traj_point.relative_position().y(); tp.relative_ego_yaw =
//     traj_point.relative_yaw(); tp.relative_ego_speed =
//     traj_point.relative_velocity().x(); //TODO：补成相对绝对速度
//     tp.relative_ego_std_dev_x = 0.; // hack
//     tp.relative_ego_std_dev_y = 0.;
//     tp.relative_ego_std_dev_yaw = 0.;
//     tp.relative_ego_std_dev_speed = 0.;
//     tp.path_point.s = cumulative_s;
//     tp.relative_time = relative_time;
//     // todo: get relative time from prediction msg !!!
//     relative_time += 0.2;  // prediction time step
//     if (i >= 1) {
//       cumulative_s +=
//           planning_math::fast_hypot(trajectory_[i - 1].path_point.x -
//           tp.path_point.x,
//                           trajectory_[i - 1].path_point.y - tp.path_point.y);
//     }
//     trajectory_.push_back(tp);
//     //    trajectory_.emplace_back(tp);
//   }
//   is_static_ = std::fabs(trajectory_.back().path_point.s -
//                          trajectory_.front().path_point.s) < 5.e-3;
//   // DiscretizedTrajectory::CompensateTrajectory(trajectory_,
//   //                                             5.0);

//   // reset perception info matched with current timestamp
//   auto init_point = get_point_at_time(0.0);
//   x_center_ = init_point.path_point.x;
//   y_center_ = init_point.path_point.y;
//   perception_polygon_ = get_polygon_at_point(init_point);
//   perception_bounding_box_ = get_bounding_box(init_point);
//   velocity_angle_ = init_point.velocity_direction;
// }

Obstacle::Obstacle(int id, const PredictionObject &prediction_object,
                   const bool is_static, double start_timestamp)
    : id_(id),
      perception_id_(prediction_object.id),
      is_static_(is_static),
      perception_bounding_box_(
          {prediction_object.position_x, prediction_object.position_y},
          prediction_object.yaw, prediction_object.length,
          prediction_object.width) {
  x_center_ = prediction_object.position_x;
  y_center_ = prediction_object.position_y;
  x_relative_center_ = prediction_object.relative_position_x;
  y_relative_center_ = prediction_object.relative_position_y;
  width_ = prediction_object.width;
  length_ = prediction_object.length;
  yaw_ = planning_math::NormalizeAngle(prediction_object.yaw);
  relative_yaw_ =
      planning_math::NormalizeAngle(prediction_object.relative_theta);
  velocity_ = prediction_object.speed;
  velocity_angle_ = prediction_object.theta;
  relative_velocity_angle_ = std::atan2(prediction_object.relative_speed_y,
                                        prediction_object.relative_speed_x);
  x_relative_velocity_ = prediction_object.relative_speed_x;
  y_relative_velocity_ = prediction_object.relative_speed_y;
  acc_ = prediction_object.acc;
  fusion_source_ = prediction_object.fusion_source;

  std::vector<planning_math::Vec2d> polygon_points;
  if (prediction_object.bottom_polygon_points.size() < 3) {
    perception_bounding_box_.GetAllCorners(&polygon_points);
  } else {
    LOG_DEBUG("raw size %d", prediction_object.bottom_polygon_points.size());
    for (size_t i = 0; i < prediction_object.bottom_polygon_points.size() - 1;
         ++i) {
      auto &point = prediction_object.bottom_polygon_points[i];
      polygon_points.emplace_back(planning_math::Vec2d(point.x, point.y));
      LOG_DEBUG("point x %f y %f", polygon_points[i].x(),
                polygon_points[i].y());
      LOG_DEBUG("rel point x %f y %f", polygon_points[i].x() - x_center_,
                polygon_points[i].y() - y_center_);
    }
    // polygon_points.erase(polygon_points.begin());
  }
  auto extracted_polygon_points = polygon_points;
  extract_point_at_specified_resolution(extracted_polygon_points);
  if (extracted_polygon_points.size() < 3) {
    extracted_polygon_points = polygon_points;
  }

  if (!planning_math::Polygon2d::ComputeConvexHull(extracted_polygon_points,
                                                   &perception_polygon_)) {
    LOG_DEBUG("polygon_debug invalid cart polygon");
  }
  std::vector<planning_math::Vec2d> ego_polygon_points;
  LOG_DEBUG("obstacle[%d] polygon size : %d %d, ego x %f y %f\n", id_,
            prediction_object.bottom_polygon_points.size(),
            polygon_points.size(), x_center_, y_center_);
  for (const auto &point : perception_polygon_.points()) {
    ego_polygon_points.emplace_back(
        planning_math::Vec2d(point.x() - x_center_, point.y() - y_center_));
    // LOG_DEBUG("ego point x %f y %f", ego_polygon_points.back().x(),
    // ego_polygon_points.back().y());
  }
  // LOG_DEBUG("obstacle[%d] last polygon size : %d", polygon_points.size());
  if (!planning_math::Polygon2d::ComputeConvexHull(ego_polygon_points,
                                                   &obstacle_ego_polygon_)) {
    LOG_DEBUG("polygon_debug invalid ego polygon\n");
  }
  is_virtual_ = id_ < 0;

  if (prediction_object.trajectory_array.empty()) return;
  //轨迹默认选第一条
  auto &prediction_trajectory =
      prediction_object.trajectory_array[0].trajectory;
  if (prediction_trajectory.empty()) {
    return;
  }

  trajectory_.clear();
  double relative_time = start_timestamp;
  double cumulative_s = 0.0;
  double start_vel_direction = prediction_trajectory[0].yaw;
  double sigma_t = 0.0;
  constexpr double InvalidSigma = 10.0;
  for (size_t i = 0; i < prediction_trajectory.size(); ++i) {
    PncTrajectoryPoint tp;
    auto &traj_point = prediction_trajectory[i];
    tp.v = traj_point.speed;
    tp.a = 0;
    tp.prediction_prob = traj_point.prob;
    tp.path_point.x = traj_point.x;
    tp.path_point.y = traj_point.y;
    tp.sigma_x = traj_point.std_dev_x;
    tp.sigma_y = traj_point.std_dev_y;
    tp.path_point.theta = planning_math::NormalizeAngle(traj_point.theta);
    tp.velocity_direction = planning_math::NormalizeAngle(traj_point.yaw);
    tp.relative_ego_x = traj_point.relative_ego_x;
    tp.relative_ego_y = traj_point.relative_ego_y;
    tp.relative_ego_yaw = traj_point.relative_ego_yaw;
    tp.relative_ego_speed = traj_point.relative_ego_speed;
    tp.relative_ego_std_dev_x = traj_point.relative_ego_std_dev_x;
    tp.relative_ego_std_dev_y = traj_point.relative_ego_std_dev_y;
    tp.relative_ego_std_dev_yaw = traj_point.relative_ego_std_dev_yaw;
    tp.relative_ego_std_dev_speed = traj_point.relative_ego_std_dev_speed;
    tp.path_point.s = cumulative_s;
    tp.relative_time = traj_point.relative_time;
    // todo: get relative time from prediction msg !!!
    // relative_time += 0.2; // prediction time step

    if (i >= 1) {
      cumulative_s += planning_math::fast_hypot(
          trajectory_[i - 1].path_point.x - tp.path_point.x,
          trajectory_[i - 1].path_point.y - tp.path_point.y);
    }
    trajectory_.emplace_back(tp);
  }
  is_static_ = std::fabs(trajectory_.back().path_point.s -
                         trajectory_.front().path_point.s) < 5.e-3;
  //  DiscretizedTrajectory::CompensateTrajectory(trajectory_, 5.0);

  // reset perception info matched with current timestamp
  auto init_point = get_point_at_time(0.0);
  perception_polygon_ = get_polygon_at_point(init_point);
  perception_bounding_box_ = get_bounding_box(init_point);
  // speed_direction_ = init_point.velocity_direction;
}

// Obstacle::Obstacle(int id,
//                    const FusionObjects::FusionObject &perception_obstacle,
//                    const bool is_static)
//     : id_(id),
//     perception_id_(perception_obstacle.additional_info().track_id()),
//       is_static_(is_static),
//       perception_bounding_box_(
//           {perception_obstacle.common_info().center_position().x(),
//           perception_obstacle.common_info().center_position().y()},
//           perception_obstacle.common_info().heading_angle(),
//           perception_obstacle.common_info().shape().length(),
//           perception_obstacle.common_info().shape().width()) {
//   perception_obstacle_.CopyFrom(perception_obstacle);
//   auto speed = perception_obstacle.common_info().velocity();
//   // todo: add priority for obstacle

//   std::vector<planning_math::Vec2d> polygon_points;
//   if (perception_obstacle.additional_info().polygon().points_size() < 3) {
//     perception_bounding_box_.GetAllCorners(&polygon_points);
//   } else {
//     for (const auto &point :
//     perception_obstacle.additional_info().polygon().points()) {
//       polygon_points.emplace_back(planning_math::Vec2d(point.x(),
//       point.y()));
//     }
//     polygon_points.erase(polygon_points.begin());
//     for (size_t i = 0; i <
//     perception_obstacle.additional_info().polygon().points_size()-1; ++i) {
//       auto &point =
//       perception_obstacle.additional_info().polygon().points(i);
//       polygon_points.emplace_back(planning_math::Vec2d(point.x(),
//       point.y()));
//     }
//   }
//   // LOG_DEBUG("cone type %d points raw size %d static: %d",
//   perception_obstacle.type, polygon_points.size(), is_static); auto
//   extracted_polygon_points = polygon_points;
//   extract_point_at_specified_resolution(extracted_polygon_points);
//   if (extracted_polygon_points.size() < 3) {
//     extracted_polygon_points = polygon_points;
//   }
//   // for (auto point : extracted_polygon_points) {
//   //   LOG_DEBUG("cone after remove redundant point x %f y %f", point.x(),
//   point.y());
//   //   LOG_DEBUG("cone after remove redundant rel point x %f y %f", point.x()
//   - perception_obstacle_.position.x,
//   //         point.y()-perception_obstacle_.position.y);
//   // }
//   if (!planning_math::Polygon2d::ComputeConvexHull(extracted_polygon_points,
//                                                      &perception_polygon_)) {
//     LOG_DEBUG("invalid cone polygon");
//   }
//   std::vector<planning_math::Vec2d> ego_polygon_points;
//   for (const auto &point : perception_polygon_.points()) {
//     ego_polygon_points.emplace_back(planning_math::Vec2d(
//                                     point.x() -
//                                     perception_obstacle.common_info().center_position().x(),
//                                     point.y() -
//                                     perception_obstacle.common_info().center_position().y()));
//   }
//   if (!planning_math::Polygon2d::ComputeConvexHull(ego_polygon_points,
//                                                      &car_ego_polygon_)) {
//     LOG_DEBUG("invalid ego cone polygon");
//   }

//   is_virtual_ = id_ < 0;
//   prob_ = 1.0;
//   speed_ = perception_obstacle.common_info().velocity().x(); // TODO:绝对速度
//   acc_ = std::hypot(0.,
//                       std::hypot(perception_obstacle.common_info().acceleration().x(),
//                                  perception_obstacle.common_info().acceleration().y()));
//                                  //加速度需要有z信息
// }

// Obstacle::Obstacle(int id, const std::vector<Point3d> &points)
//     : id_(id), perception_id_(id), is_static_(true) {
//   type_ = Common::ObjectType::OBJECT_TYPE_UNKNOWN;
//   perception_velocity_ = 0.0;
//   velocity_ = 0.0;
//   if (points.size() > 2) {
//     x_center_ = (points[0].x + points[1].x) / 2;
//     y_center_ = (points[0].y + points[1].y) / 2;
//   }
//   std::vector<planning_math::Vec2d> pillar_points(points.size());
//   for (size_t i = 0; i < points.size(); i++) {
//     pillar_points[i].set_x(points[i].x);
//     pillar_points[i].set_y(points[i].y);
//   }
//   perception_points_ = pillar_points;
//   planning_math::Polygon2d::ComputeConvexHull(pillar_points,
//                                               &perception_polygon_);
// }

Obstacle::Obstacle(int id, const std::vector<planning_math::Vec2d> &points)
    : id_(id),
      perception_id_(id),
      is_static_(true),
      perception_points_(points) {
  type_ = Common::ObjectType::OBJECT_TYPE_UNKNOWN_IMMOVABLE;  // FREESPACE占位
  velocity_ = 0.0;
  if (id_ > 6000000) {
    planning_math::Polygon2d::ComputeConvexHull(perception_points_,
                                                &perception_polygon_);
  }
}

void Obstacle::extract_point_at_specified_resolution(
    std::vector<planning_math::Vec2d> &points) const {
  constexpr double PI = std::atan(1.0) * 4.0;
  constexpr int kMinPointsNum = 4;
  constexpr double kMinDist = 0.5;
  constexpr double kMinCosTheta = cos(0.8 * PI);
  if (points.size() <= kMinPointsNum) {
    return;
  }
  size_t i = 0;
  size_t j = 1;
  while (i < points.size() && j + 1 < points.size()) {
    planning_math::LineSegment2d seg(points.at(i), points.at(j + 1));
    double cos_theta =
        planning_math::InnerProd(points.at(j), points.at(i), points.at(j + 1)) /
        (points.at(j).DistanceTo(points.at(i)) *
         points.at(j).DistanceTo(points.at(j + 1)));

    if (cos_theta > kMinCosTheta ||
        seg.DistanceSquareTo(points.at(j)) > kMinDist * kMinDist) {
      ++i;
      if (i != j) {
        points.at(i) = points.at(j);
      }
    }
    ++j;
  }
  points.at(++i) = points.back();
  points.resize(i + 1);
}

PncTrajectoryPoint Obstacle::get_point_at_time(
    const double relative_time) const {
  const auto &points = trajectory_;
  if (points.size() < 2) {
    PncTrajectoryPoint point;
    point.path_point.x = x_center_;
    point.path_point.y = y_center_;
    point.path_point.z = 0.;  // 障碍物位置需要z信息
    point.path_point.theta = yaw_;
    point.prediction_prob = 1.0;
    point.velocity_direction = 0.0;
    point.path_point.s = 0.0;
    point.path_point.kappa = 0.0;
    point.path_point.dkappa = 0.0;
    point.path_point.ddkappa = 0.0;
    point.v = 0.0;
    point.a = 0.0;
    point.s = 0.0;
    point.sigma_x = 0.0;
    point.sigma_y = 0.0;
    point.relative_time = 0.0;
    point.relative_ego_x = 0.0;
    point.relative_ego_y = 0.0;
    point.relative_ego_yaw = 0.0;
    point.relative_ego_speed = 0.0;
    return point;
  } else {
    auto comp = [](const PncTrajectoryPoint &p, const double time) {
      return p.relative_time < time;
    };

    auto it_lower =
        std::lower_bound(points.begin(), points.end(), relative_time, comp);

    if (it_lower == points.begin()) {
      return *points.begin();
    } else if (it_lower == points.end()) {
      return *points.rbegin();
    }
    return planning_math::InterpolateUsingLinearApproximation(
        *(it_lower - 1), *it_lower, relative_time);
  }
}

planning_math::Box2d Obstacle::get_bounding_box(
    const PncTrajectoryPoint &point) const {
  return planning_math::Box2d({point.path_point.x, point.path_point.y},
                              point.path_point.theta, length_, width_);
}

planning_math::Polygon2d Obstacle::get_polygon_at_point(
    const PncTrajectoryPoint &point) const {
  std::vector<planning_math::Vec2d> polygon_points;
  double rel_theta = point.path_point.theta - yaw_;

  for (const auto &ego_point : obstacle_ego_polygon_.points()) {
    polygon_points.emplace_back(planning_math::Vec2d(
        ego_point.x() * cos(rel_theta) - ego_point.y() * sin(rel_theta) +
            point.path_point.x,
        ego_point.y() * cos(rel_theta) + ego_point.x() * sin(rel_theta) +
            point.path_point.y));
  }
  planning_math::Polygon2d polygon;
  if (!planning_math::Polygon2d::ComputeConvexHull(polygon_points, &polygon)) {
    LOG_DEBUG("polygon_debug : get position %f %f failed\n", point.path_point.x,
              point.path_point.y);
    for (auto p : polygon_points) {
      LOG_DEBUG("polygon_debug invald point x %f y %f",
                p.x() - point.path_point.x, p.y() - point.path_point.y);
    }
    if (!planning_math::Polygon2d::ComputeConvexHull(
            get_bounding_box(point).GetAllCorners(), &polygon)) {
      LOG_DEBUG("polygon_debug : invalid box polygon\n");
    }
  }
  return polygon;
}

}  // namespace planning

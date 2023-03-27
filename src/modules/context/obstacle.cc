#include "modules/context/obstacle.h"

#include "common/log.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/common.h"
// #include "core/modules/common/trajectory/discretized_trajectory.h"

namespace planning {

// Obstacle::abnormal_data_dectection

// Obstacle::Obstacle(int id,
//                    const Asw::Prediction::PredictionObject &prediction_object,
//                    const bool is_static, double start_relative_timestamp)
//     : id_(id),
//       perception_id_(prediction_object.perception_obstacle().id()),
//       is_static_(is_static),
//       perception_bounding_box_(
//           {prediction_object.perception_obstacle().center_position().x(), 
//           prediction_object.perception_obstacle().center_position().y()},
//           prediction_object.perception_obstacle().heading_angle(), 
//           prediction_object.perception_obstacle().shape().length(),
//           prediction_object.perception_obstacle().shape().width()) {

//   perception_obstacle_.CopyFrom(prediction_object.perception_obstacle());
//   x_center_ = perception_obstacle_.common_info().center_position().x();
//   y_center_ = perception_obstacle_.common_info().center_position().y();
//   perception_velocity_ = perception_obstacle_.common_info().velocity().x();
//   velocity_angle_ = perception_obstacle_.common_info().heading_angle();
//   // perception_obstacle_.heading_yaw =
//   //     planning_math::NormalizeAngle(prediction_object.perception_obstacle().heading_angle());

//   std::vector<planning_math::Vec2d> polygon_points;
//   if (prediction_object.bottom_polygon_points_size() < 3) {
//     perception_bounding_box_.GetAllCorners(&polygon_points);
//   } else {
//     for (int i = 0; i < prediction_object.bottom_polygon_points_size() - 1;
//          ++i) {
//       auto &point = prediction_object.bottom_polygon_points(i);
//       polygon_points.emplace_back(planning_math::Vec2d(point.x(), point.y()));
//     }
//   }
//   NLOGI("obstacle debug enable bbox mode: %d", enable_bbox_mode);
//   auto extracted_polygon_points = polygon_points;
//   extract_point_at_specified_resolution(extracted_polygon_points);
//   if (extracted_polygon_points.size() < 3) {
//     extracted_polygon_points = polygon_points;
//   }
//   if (!planning_math::Polygon2d::ComputeConvexHull(extracted_polygon_points,
//                                                    &perception_polygon_)) {
//     NLOGD("polygon_debug invalid cart polygon");
//   }
//   std::vector<planning_math::Vec2d> ego_polygon_points;
//   for (const auto &point : perception_polygon_.points()) {
//     ego_polygon_points.emplace_back(
//         planning_math::Vec2d(point.x() - perception_obstacle_.position.x,
//                              point.y() - perception_obstacle_.position.y));
//   }
//   // NLOGI("obstacle[%d] last polygon size : %d", polygon_points.size());
//   if (!planning_math::Polygon2d::ComputeConvexHull(ego_polygon_points,
//                                                    &obstacle_ego_polygon_)) {
//     NLOGD("polygon_debug invalid ego polygon");
//   }

//   velocity_ = prediction_object.speed();
//   acc_ = prediction_object.acc();
//   acc_signed_ = prediction_object.acc_signed();

//   if (is_static) {
//     return;
//   }

//   auto &prediction_trajectory = prediction_object.trajectory_array(0);

//   if (prediction_trajectory.trajectory_size() == 0) {
//     return;
//   }

//   trajectory_.clear();
//   double relative_time = start_relative_timestamp;
//   double cumulative_s = 0.0;
//   for (int i = 0; i < prediction_trajectory.trajectory_size(); ++i) {
//     ObjectTrajectoryPoint tp;
//     auto &traj_point = prediction_trajectory.trajectory(i);
//     tp.v = traj_point.speed();
//     tp.a = 0;
//     tp.prediction_prob = traj_point.prob();
//     tp.path_point.x = traj_point.x();
//     tp.path_point.y = traj_point.y();
//     tp.sigma_x = traj_point.std_dev_x();
//     tp.sigma_y = traj_point.std_dev_y();
//     tp.path_point.theta = planning_math::NormalizeAngle(traj_point.theta());
//     tp.velocity_direction = planning_math::NormalizeAngle(traj_point.yaw());
//     tp.relative_ego_x = traj_point.relative_ego_x();
//     tp.relative_ego_y = traj_point.relative_ego_y();
//     tp.relative_ego_yaw = traj_point.relative_ego_yaw();
//     tp.relative_ego_speed = traj_point.relative_ego_speed();
//     tp.relative_ego_std_dev_x = traj_point.relative_ego_std_dev_x();
//     tp.relative_ego_std_dev_y = traj_point.relative_ego_std_dev_y();
//     tp.relative_ego_std_dev_yaw = traj_point.relative_ego_std_dev_yaw();
//     tp.relative_ego_std_dev_speed = traj_point.relative_ego_std_dev_speed();
//     tp.path_point.s = cumulative_s;
//     tp.relative_time = relative_time;
//     // todo: get relative time from prediction msg !!!
//     relative_time += 0.2;  // prediction time step
//     if (i >= 1) {
//       cumulative_s +=
//           npp::fast_hypot(trajectory_[i - 1].path_point.x - tp.path_point.x,
//                           trajectory_[i - 1].path_point.y - tp.path_point.y);
//     }
//     trajectory_.push_back(tp);
//     //    trajectory_.emplace_back(tp);
//   }
//   is_static_ = std::fabs(trajectory_.back().path_point.s -
//                          trajectory_.front().path_point.s) < 5.e-3;
//   DiscretizedTrajectory::CompensateTrajectory(trajectory_,
//                                               FLAGS_lon_decision_time_horizon);

//   // reset perception info matched with current timestamp
//   auto init_point = get_point_at_time(0.0);
//   x_center_ = init_point.path_point.x;
//   y_center_ = init_point.path_point.y;
//   perception_polygon_ = get_polygon_at_point(init_point);
//   perception_bounding_box_ = get_bounding_box(init_point);
//   velocity_angle_ = init_point.velocity_direction;
// }

// Obstacle::Obstacle(int id,
//                    const Asw::ObjectFusion::FusionObject &perception_obstacle,
//                    const bool is_static)
//     : id_(id), perception_id_(perception_obstacle.additional_info().track_id()),
//       is_static_(is_static), perception_obstacle_(perception_obstacle),
//       perception_bounding_box_(
//           {perception_obstacle.common_info().center_position().x(),
//           perception_obstacle.common_info().center_position().y()},
//           perception_obstacle.common_info().heading_angle(),
//           perception_obstacle.common_info().shape().length() + kSizeEpsilon,
//           perception_obstacle.common_info().shape().width() + kSizeEpsilon) {
//   is_caution_level_obstacle_ = true;
//   is_truncated_ = false;
//   is_st_boundary_constructed_ = false;
//   is_zoomed_ = false;
//   truncated_time_ = std::numeric_limits<double>::infinity();
//   auto speed = perception_obstacle.velocity;
//   perception_speed_ = std::hypot(speed.x, std::hypot(speed.y, speed.z));
//   // todo: add priority for obstacle

//   std::vector<planning_math::Vec2d> polygon_points;
//   if (perception_obstacle.polygon_bottom.points.size() < 3) {
//     perception_bounding_box_.GetAllCorners(&polygon_points);
//   } else {
//     for (const auto &point : perception_obstacle.polygon_bottom.points) {
//       polygon_points.emplace_back(planning_math::Vec2d(point.x, point.y));
//     }
//     polygon_points.erase(polygon_points.begin());
//     for (size_t i = 0; i < perception_obstacle.polygon_bottom.points.size()-1; ++i) {
//       auto &point = perception_obstacle.polygon_bottom.points[i];
//       polygon_points.emplace_back(planning_math::Vec2d(point.x, point.y));
//     }
//   }
//   // LOG_DEBUG("cone type %d points raw size %d static: %d", perception_obstacle.type, polygon_points.size(), is_static);
//   auto extracted_polygon_points = polygon_points;
//   ExtractPointAtSpecifiedResolution(extracted_polygon_points);
//   if (extracted_polygon_points.size() < 3) {
//     extracted_polygon_points = polygon_points;
//   }
//   // for (auto point : extracted_polygon_points) {
//   //   LOG_DEBUG("cone after remove redundant point x %f y %f", point.x(), point.y());
//   //   LOG_DEBUG("cone after remove redundant rel point x %f y %f", point.x() - perception_obstacle_.position.x,
//   //         point.y()-perception_obstacle_.position.y);
//   // }
//   if (!planning_math::Polygon2d::ComputeConvexHull(extracted_polygon_points,
//                                                      &perception_polygon_)) {
//     LOG_DEBUG("invalid cone polygon");
//   }
//   std::vector<planning_math::Vec2d> ego_polygon_points;
//   for (const auto &point : perception_polygon_.points()) {
//     ego_polygon_points.emplace_back(planning_math::Vec2d(
//                                     point.x() - perception_obstacle.position.x,
//                                     point.y() - perception_obstacle.position.y));
//   }
//   if (!planning_math::Polygon2d::ComputeConvexHull(ego_polygon_points,
//                                                      &car_ego_polygon_)) {
//     LOG_DEBUG("invalid ego cone polygon");
//   }

//   is_virtual_ = id_ < 0;
//   prob_ = 1.0;
//   speed_ = perception_speed_;
//   acc_ = std::hypot(perception_obstacle.acceleration.z,
//                       std::hypot(perception_obstacle.acceleration.x,
//                                  perception_obstacle.acceleration.y));
// }

// Obstacle::Obstacle(int id, const std::vector<Point3d> &points)
//     : id_(id), perception_id_(id), is_static_(true) {
//   type_ = ObjectType::NOT_KNOW;
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

// Obstacle::Obstacle(int id, const std::vector<planning_math::Vec2d> &points)
//     : id_(id),
//       perception_id_(id),
//       is_static_(true),
//       perception_points_(points) {
//   type_ = ObjectType::FREESPACE;
//   perception_velocity_ = 0.0;
//   velocity_ = 0.0;
// }

// void Obstacle::extract_point_at_specified_resolution(
//     std::vector<planning_math::Vec2d> &points) const {
//   constexpr double PI = std::atan(1.0) * 4.0;
//   constexpr int kMinPointsNum = 4;
//   constexpr double kMinDist = 0.5;
//   constexpr double kMinCosTheta = cos(0.8 * PI);
//   if (points.size() <= kMinPointsNum) {
//     return;
//   }
//   size_t i = 0;
//   size_t j = 1;
//   while (i < points.size() && j + 1 < points.size()) {
//     planning_math::LineSegment2d seg(points.at(i), points.at(j + 1));
//     double cos_theta =
//         planning_math::InnerProd(points.at(j), points.at(i), points.at(j + 1)) /
//         (points.at(j).DistanceTo(points.at(i)) *
//          points.at(j).DistanceTo(points.at(j + 1)));

//     if (cos_theta > kMinCosTheta ||
//         seg.DistanceSquareTo(points.at(j)) > kMinDist * kMinDist) {
//       ++i;
//       if (i != j) {
//         points.at(i) = points.at(j);
//       }
//     }
//     ++j;
//   }
//   points.at(++i) = points.back();
//   points.resize(i + 1);
// }

// ObjectTrajectoryPoint Obstacle::get_point_at_time(
//     const double relative_time) const {
//   const auto &points = trajectory_;
//   if (points.size() < 2) {
//     ObjectTrajectoryPoint point;
//     point.path_point.x = perception_obstacle_.position.x;
//     point.path_point.y = perception_obstacle_.position.y;
//     point.path_point.z = perception_obstacle_.position.z;
//     point.path_point.theta = perception_obstacle_.heading_yaw;
//     point.prediction_prob = 1.0;
//     point.velocity_direction = 0.0;
//     point.path_point.s = 0.0;
//     point.path_point.kappa = 0.0;
//     point.path_point.dkappa = 0.0;
//     point.path_point.ddkappa = 0.0;
//     point.v = 0.0;
//     point.a = 0.0;
//     point.s = 0.0;
//     point.sigma_x = 0.0;
//     point.sigma_y = 0.0;
//     point.relative_time = 0.0;
//     point.relative_ego_x = 0.0;
//     point.relative_ego_y = 0.0;
//     point.relative_ego_yaw = 0.0;
//     point.relative_ego_speed = 0.0;
//     return point;
//   } else {
//     auto comp = [](const ObjectTrajectoryPoint &p, const double time) {
//       return p.relative_time < time;
//     };

//     auto it_lower =
//         std::lower_bound(points.begin(), points.end(), relative_time, comp);

//     if (it_lower == points.begin()) {
//       return *points.begin();
//     } else if (it_lower == points.end()) {
//       return *points.rbegin();
//     }
//     return planning_math::InterpolateUsingLinearApproximation(
//         *(it_lower - 1), *it_lower, relative_time);
//   }
// }

// planning_math::Box2d Obstacle::get_bounding_box(
//     const ObjectTrajectoryPoint &point) const {
//   return planning_math::Box2d(
//       {point.path_point.x, point.path_point.y}, point.path_point.theta,
//       perception_obstacle_.shape.length, perception_obstacle_.shape.width);
// }

// planning_math::Polygon2d Obstacle::get_polygon_at_point(
//     const ObjectTrajectoryPoint &point) const {
//   std::vector<planning_math::Vec2d> polygon_points;
//   double rel_theta = point.path_point.theta - perception_obstacle_.heading_yaw;

//   for (const auto &ego_point : obstacle_ego_polygon_.points()) {
//     polygon_points.emplace_back(planning_math::Vec2d(
//         ego_point.x() * cos(rel_theta) - ego_point.y() * sin(rel_theta) +
//             point.path_point.x,
//         ego_point.y() * cos(rel_theta) + ego_point.x() * sin(rel_theta) +
//             point.path_point.y));
//   }
//   planning_math::Polygon2d polygon;
//   if (!planning_math::Polygon2d::ComputeConvexHull(polygon_points, &polygon)) {
//     NLOGD("polygon_debug : get position %f %f failed", point.path_point.x,
//           point.path_point.y);
//     for (auto p : polygon_points) {
//       NLOGD("polygon_debug invald point x %f y %f", p.x() - point.path_point.x,
//             p.y() - point.path_point.y);
//     }
//     if (!planning_math::Polygon2d::ComputeConvexHull(
//             get_bounding_box(point).GetAllCorners(), &polygon)) {
//       NLOGD("polygon_debug : invalid box polygon");
//     }
//   }
//   return polygon;
// }

}  // namespace planning

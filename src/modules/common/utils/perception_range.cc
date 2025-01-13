#include "utils/perception_range.h"
#include "common.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace planning {

template <class ForwardIterator>
inline size_t argmax(ForwardIterator first, ForwardIterator last) {
  return std::distance(first, std::max_element(first, last));
}

class Line {
 public:
  explicit Line(const Point2D &A, const Point2D &B) {
    a_ = A.y - B.y;
    b_ = B.x - A.x;
    c_ = A.x * B.y - A.y * B.x;
  };
  Point2D operator()(double x) const { return {x, -c_ - (a_ * x / b_)}; };
  Point2D operator*(const Line &line) const {
    double d = a_ * line.b_ - line.a_ * b_;
    if (std::abs(d) < std::numeric_limits<double>::epsilon()) {
      return {std::numeric_limits<double>::min(),
              std::numeric_limits<double>::min()};
    } else {
      return {(b_ * line.c_ - line.b_ * c_) / d,
              (c_ * line.a_ - line.c_ * a_) / d};
    }
  }

 private:
  double a_;
  double b_;
  double c_;
};

class PerceptionRangeEstimatorImpl : public PerceptionRangeEstimator {
 public:
  struct Config {
    double half_car_width = 1.1;
    double lidar_position_x = -2.8;
    double lidar_position_z = 1.8;
    double obstacle_length_scale = 0.75;
    double obstacle_width_scale = 0.65;
    double side_view_target_height = 0.8;
    double side_view_height_scale = 0.75;
    double side_view_x_percent = 0.3;
  };

  PerceptionRangeEstimatorImpl() /* : ego_yaw_(0.0), ego_frenet_s_(0.0) */ {};
  ~PerceptionRangeEstimatorImpl() = default;

  void updateFrenet(
      const std::shared_ptr<FrenetCoordinateSystem> &frenet) override {
    frenet_coord_ = frenet;
  };

  // void FeedMap(const MSDMapInfo &map_info) override {
  //   refline_points_.clear();
  //   left_bound_points_.clear();
  //   right_bound_points_.clear();

  //   size_t points_num = map_info.current_refline_points().size();
  //   refline_points_.reserve(points_num);
  //   left_bound_points_.reserve(points_num);
  //   right_bound_points_.reserve(points_num);

  //   for (auto &p : map_info.current_refline_points()) {
  //     auto relative_yaw = p.yaw - ego_yaw_;
  //     auto sin_yaw_l = std::sin(relative_yaw) * config_.half_car_width;
  //     auto cos_yaw_l = std::cos(relative_yaw) * config_.half_car_width;
  //     refline_points_.emplace_back(p.car_point.x, p.car_point.y);
  //     left_bound_points_.emplace_back(p.car_point.x - sin_yaw_l,
  //                                     p.car_point.y + cos_yaw_l);
  //     right_bound_points_.emplace_back(p.car_point.x + sin_yaw_l,
  //                                       p.car_point.y - cos_yaw_l);
  //   }
  // }

  // void feedEgoState(const EgoState &ego_state,
  //                   const Transform &car2enu) override {
  //   ego_yaw_ = ego_state.ego_pose.theta;
  //   ego_frenet_s_ = ego_state.ego_frenet.x;
  //   car2enu_ = car2enu;
  // }

  // double calculate(const TrackedObject &obstacle) const override {
  //   if (frenet_coord_ == nullptr) {
  //     return 0;
  //   }
  //   // p1 -> bottom left, p2 -> bottom right, p3 -> top right, p4 -> top left
  //   double sin_yaw = std::sin(obstacle.theta);
  //   double cos_yaw = std::cos(obstacle.theta);
  //   double scaled_half_length =
  //       obstacle.length * 0.5 * config_.obstacle_length_scale;
  //   double scaled_half_width =
  //       obstacle.width * 0.5 * config_.obstacle_width_scale;
  //   Point2D origin(obstacle.center_x, obstacle.center_y);
  //   Point2D p1 = rotatePoint(origin,
  //                            Point2D(obstacle.center_x - scaled_half_length,
  //                                    obstacle.center_y - scaled_half_width),
  //                            obstacle.theta);
  //   Point2D p2 = rotatePoint(origin,
  //                            Point2D(obstacle.center_x + scaled_half_length,
  //                                    obstacle.center_y - scaled_half_width),
  //                            obstacle.theta);
  //   Point2D p3 = rotatePoint(origin,
  //                            Point2D(obstacle.center_x + scaled_half_length,
  //                                    obstacle.center_y + scaled_half_width),
  //                            obstacle.theta);
  //   Point2D p4 = rotatePoint(origin,
  //                            Point2D(obstacle.center_x - scaled_half_length,
  //                                    obstacle.center_y + scaled_half_width),
  //                            obstacle.theta);
  //   double obstacle_min_x = std::min({p1.x, p2.x, p3.x, p4.x});
  //   double obstacle_max_x = std::max({p1.x, p2.x, p3.x, p4.x});
  //   double ret = std::numeric_limits<double>::max();

  //   // Bird view
  //   Point2D lidar_origin(config_.lidar_position_x, 0);
  //   auto cross_1_l = findReflineCrossing(lidar_origin, p1,
  //   left_bound_points_); auto cross_1_r = findReflineCrossing(lidar_origin,
  //   p1, right_bound_points_); auto cross_2_l =
  //   findReflineCrossing(lidar_origin, p2, left_bound_points_); auto cross_2_r
  //   = findReflineCrossing(lidar_origin, p2, right_bound_points_); auto
  //   cross_3_l = findReflineCrossing(lidar_origin, p3, left_bound_points_);
  //   auto cross_3_r = findReflineCrossing(lidar_origin, p3,
  //   right_bound_points_); auto cross_4_l = findReflineCrossing(lidar_origin,
  //   p4, left_bound_points_); auto cross_4_r =
  //   findReflineCrossing(lidar_origin, p4, right_bound_points_);
  //   LOG_DEBUG("lymdbg: 1_l(%.3f, %.3f)", cross_1_l.x, cross_1_l.y);
  //   LOG_DEBUG("lymdbg: 1_r(%.3f, %.3f)", cross_1_r.x, cross_1_r.y);
  //   LOG_DEBUG("lymdbg: 2_l(%.3f, %.3f)", cross_2_l.x, cross_2_l.y);
  //   LOG_DEBUG("lymdbg: 2_r(%.3f, %.3f)", cross_2_r.x, cross_2_r.y);
  //   LOG_DEBUG("lymdbg: 3_l(%.3f, %.3f)", cross_3_l.x, cross_3_l.y);
  //   LOG_DEBUG("lymdbg: 3_r(%.3f, %.3f)", cross_3_r.x, cross_3_r.y);
  //   LOG_DEBUG("lymdbg: 4_l(%.3f, %.3f)", cross_4_l.x, cross_4_l.y);
  //   LOG_DEBUG("lymdbg: 4_r(%.3f, %.3f)", cross_4_r.x, cross_4_r.y);
  //   std::vector<double> x_vec({cross_1_l.x, cross_1_r.x, cross_2_l.x,
  //                              cross_2_r.x, cross_3_l.x, cross_3_r.x,
  //                              cross_4_l.x, cross_4_r.x});
  //   std::vector<double> y_vec({cross_1_l.y, cross_1_r.y, cross_2_l.y,
  //                              cross_2_r.y, cross_3_l.y, cross_3_r.y,
  //                              cross_4_l.y, cross_4_r.y});
  //   double max_idx = argmax(x_vec.begin(), x_vec.end());
  //   LOG_DEBUG("lymdbg: bird_view_pt (%f, %f)", x_vec[max_idx],
  //           y_vec[max_idx]);
  //   if (x_vec[max_idx] >= obstacle_min_x) {
  //     Point2D frenet_bv_coord;
  //     Eigen::Vector3d cart_car;
  //     cart_car << x_vec[max_idx], y_vec[max_idx], 0;
  //     auto cart_enu = car2enu_ * cart_car;
  //     if (frenet_coord_->CartCoord2FrenetCoord(
  //             Point2D(cart_enu.x(), cart_enu.y()), frenet_bv_coord) ==
  //         TRANSFORM_SUCCESS) {
  //       ret = std::min({ret, frenet_bv_coord.x - ego_frenet_s_});
  //       LOG_DEBUG("lymdbg: bird_view = %f",
  //               frenet_bv_coord.x - ego_frenet_s_);
  //     }
  //   }

  //   // Side view
  //   double obstacle_key_point_x =
  //       obstacle_min_x +
  //       (obstacle_max_x - obstacle_min_x) * config_.side_view_x_percent;
  //   double scaled_obstacle_height =
  //       obstacle.height * config_.side_view_height_scale;
  //   // TODO: Use real height
  //   scaled_obstacle_height = obstacle.width * config_.side_view_height_scale;
  //   double side_view_ret =
  //       (config_.side_view_target_height - scaled_obstacle_height) *
  //           (config_.lidar_position_x - obstacle_key_point_x) /
  //           (config_.lidar_position_z - scaled_obstacle_height) +
  //       obstacle_key_point_x;
  //   auto cross_point = findReflineCrossing(side_view_ret, refline_points_);
  //   LOG_DEBUG("lymdbg: side_view_cross_pt (%f, %f)", cross_point.x,
  //           cross_point.y);
  //   Point2D frenet_sv_coord;
  //   Eigen::Vector3d cart_sv_car;
  //   cart_sv_car << cross_point.x, cross_point.y, 0;
  //   auto cart_sv_enu = car2enu_ * cart_sv_car;
  //   if (frenet_coord_->CartCoord2FrenetCoord(
  //           Point2D(cart_sv_enu.x(), cart_sv_enu.y()), frenet_sv_coord) ==
  //       TRANSFORM_SUCCESS) {
  //     ret = std::min({ret, frenet_sv_coord.x - ego_frenet_s_});
  //     LOG_DEBUG("lymdbg: side_view = %f",
  //             frenet_sv_coord.x - ego_frenet_s_);
  //   }
  //   LOG_DEBUG("lymdbg: perception_range = %f", ret);
  //   return ret;
  // }

 private:
  // double ego_yaw_;
  // double ego_frenet_s_;
  define::Transform car2enu_;
  std::shared_ptr<FrenetCoordinateSystem> frenet_coord_;
  std::vector<Point2D> refline_points_;
  std::vector<Point2D> left_bound_points_;
  std::vector<Point2D> right_bound_points_;

  const Config config_;

  // Rotate a point
  // Param angle should be in radius
  static Point2D rotatePoint(const Point2D &origin, const Point2D &point,
                             double angle) {
    double sin_yaw = std::sin(angle);
    double cos_yaw = std::cos(angle);
    return {cos_yaw * (point.x - origin.x) - sin_yaw * (point.y - origin.y) +
                origin.x,
            sin_yaw * (point.x - origin.x) + cos_yaw * (point.y - origin.y) +
                origin.y};
  }

  static Point2D findReflineCrossing(double x,
                                     const std::vector<Point2D> &refline) {
    for (size_t i = 0; (i + 1) < refline.size(); ++i) {
      if ((i == 0 && x < refline[i].x) ||
          (i == refline.size() - 2 && x > refline[i + 1].x) ||
          (refline[i].x <= x && refline[i + 1].x >= x)) {
        Line curr_segment(Point2D(refline[i].x, refline[i].y),
                          Point2D(refline[i + 1].x, refline[i + 1].y));
        return curr_segment(x);
      }
    }
    return {0, 0};
  }

  static Point2D findReflineCrossing(const Point2D &A, const Point2D &B,
                                     const std::vector<Point2D> &refline) {
    Line line_a(A, B);
    for (size_t i = 0; (i + 1) < refline.size(); ++i) {
      Line curr_segment(Point2D(refline[i].x, refline[i].y),
                        Point2D(refline[i + 1].x, refline[i + 1].y));
      auto cross_point = line_a * curr_segment;
      if (cross_point.x >= refline[i].x && cross_point.x <= refline[i + 1].x &&
          cross_point.y >= std::min({refline[i].y, refline[i + 1].y}) &&
          cross_point.y <= std::max({refline[i].y, refline[i + 1].y})) {
        return cross_point;
      }
    }
    return {std::numeric_limits<double>::min(),
            std::numeric_limits<double>::min()};
  }
};

std::shared_ptr<PerceptionRangeEstimator> PerceptionRangeEstimator::make() {
  return std::make_shared<PerceptionRangeEstimatorImpl>();
}

}  // namespace planning

#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

// #include "modules/common/config/basic_types.h"
#include "modules/common/config/message_type.h"
#include "modules/common/math/box2d.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/math/vec2d.h"
#include "../res/include/proto/prediction.pb.h"
#include "../res/include/proto/fusion_objects.pb.h"
#include "modules/common/prediction_object.h"

namespace planning {

class Obstacle {
 public:
  explicit Obstacle(int id,
                    const Prediction::PredictionObject &prediction_object,
                    bool is_static, double start_relative_timestamp);

  explicit Obstacle(int id,
                    const PredictionObject &prediction_object,
                    bool is_static, double start_relative_timestamp);

  explicit Obstacle(int id,
                    const FusionObjects::FusionObject &perception_obstacle,
                    bool is_static);

  explicit Obstacle(int id, double x, double y, double heading_angle,
                    double length, double width, double height,
                    Common::ObjectType type);

  // for ground line
  explicit Obstacle(int id, const std::vector<Common::Point3d> &points);
  explicit Obstacle(int id, const std::vector<planning_math::Vec2d> &points);

  const std::vector<planning_math::Vec2d> &perception_points() const {
    return perception_points_;
  }

  int id() const { return id_; }

  double x_center() const { return x_center_; }
  double y_center() const { return y_center_; }
  double length() const { return length_; }
  double width() const { return width_; }
  //  relative_heading_angle
  double relative_heading_angle() const { return perception_obstacle_.common_info().relative_heading_angle(); }
  double heading_angle() const { return perception_obstacle_.common_info().heading_angle(); }
  double velocity() const { return velocity_; }
  double perception_velocity() const { return perception_velocity_; }
  double acceleration() const { return acc_; }
  // double acceleration_signed() const { return acc_signed_; }
  double velocity_angle() const { return velocity_angle_; }
  bool is_static() const { return is_static_; }
  Common::ObjectType type() const { return type_; }
  bool is_vaild() const { return valid_; }
  bool abnormal_data_dectection(
      const PredictionObject &prediction_object);

  const std::vector<PncTrajectoryPoint> &trajectory() const {
    return trajectory_;
  }

  const planning_math::Box2d &perception_bounding_box() const {
    return perception_bounding_box_;
  }

  const planning_math::Polygon2d &perception_polygon() const {
    return perception_polygon_;
  }

  PncTrajectoryPoint get_point_at_time(const double relative_time) const;

  planning_math::Polygon2d get_polygon_at_point(
      const PncTrajectoryPoint &point) const;

 private:
  void extract_point_at_specified_resolution(
      std::vector<planning_math::Vec2d> &points) const;

  planning_math::Box2d get_bounding_box(
      const PncTrajectoryPoint &point) const;

 private:
  int id_{};
  int perception_id_ = 0;
  bool is_static_ = false;
  double x_center_;
  double y_center_;
  double z_center_ = 0.0;
  double yaw_;
  double velocity_ = 0.0;
  // double perception_velocity_{};
  double velocity_angle_ = 0.0;
  double acc_ = 0.0;
  // double acc_signed_ = 0.0;
  Common::ObjectType type_;
  bool valid_ = true;
  bool is_virtual_ = false;
  double prob_;
  // double speed_ = 0.0;
  // double speed_direction_ = 0.0;
  double width_;
  double length_;


  std::vector<PncTrajectoryPoint> trajectory_{};
  // FusionObjects::FusionObject perception_obstacle_;
  planning_math::Box2d perception_bounding_box_;
  planning_math::Polygon2d perception_polygon_;
  planning_math::Polygon2d obstacle_ego_polygon_;
  planning_math::Polygon2d car_ego_polygon_;
  std::vector<planning_math::Vec2d> perception_points_;
};

struct HistoryObstacle {
  int id{};
  Common::ObjectType type;
  double time = 0.0;
  double x_center = 0.0;
  double y_center = 0.0;
  double velocity_x = 0.0;
  double velocity_y = 0.0;
  double heading_angle = 0.0;
};

// typedef IndexedList<int, Obstacle> IndexedObstacles;
}  // namespace planning

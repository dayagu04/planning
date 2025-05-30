#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "config/message_type.h"
#include "fusion_groundline_c.h"
#include "fusion_objects_c.h"
#include "fusion_occupancy_objects_c.h"
#include "math/box2d.h"
#include "math/math_utils.h"
#include "math/polygon2d.h"
#include "prediction_c.h"
#include "prediction_object.h"
#include "vec2d.h"

namespace planning {

enum class SourceType {
    NONE,
    OD,
    GroundLine,
    OCC,
    ParkingSlot,
    MAP,
};

class Obstacle {
 public:
  // explicit Obstacle(int id,
  //                   const iflyauto::PredictionObject &prediction_object,
  //                   bool is_static, double start_relative_timestamp);

  explicit Obstacle(int id, const PredictionObject &prediction_object,
                    bool is_static, double start_relative_timestamp);

  explicit Obstacle(const Obstacle *obstacle);

  // explicit Obstacle(int id,
  //                   const iflyauto::FusionObject &perception_obstacle,
  //                   bool is_static);

  // explicit Obstacle(int id, double x, double y, double heading_angle,
  //                   double length, double width, double height,
  //                   iflyauto::ObjectType type);

  // for ground line
  explicit Obstacle(int id, const std::vector<Common::Point3d> &points);
  explicit Obstacle(int id, const std::vector<planning_math::Vec2d> &points);
  explicit Obstacle(int id,
                    const iflyauto::FusionGroundLine &groundline_cluster);
  explicit Obstacle(int id, const std::vector<planning_math::Vec2d> &points,
                    iflyauto::ObjectType type);
  const std::vector<planning_math::Vec2d> &perception_points() const {
    return perception_points_;
  }

  int id() const { return id_; }
  double timestamp() const { return timestamp_; }
  void set_x_center(const double x_center) {
    x_center_ = x_center;
  }  // for pybind debug
  void set_y_center(const double y_center) { y_center_ = y_center; }
  void set_v(const double v) { velocity_ = v; }

  double x_center() const { return x_center_; }
  double y_center() const { return y_center_; }
  double x_relative_center() const { return x_relative_center_; }
  double y_relative_center() const { return y_relative_center_; }
  double length() const { return length_; }
  double width() const { return width_; }
  //  relative_heading_angle
  double relative_heading_angle() const { return relative_yaw_; }
  double heading_angle() const { return yaw_; }
  double velocity() const { return velocity_; }
  double x_relative_velocity() const { return x_relative_velocity_; }
  double y_relative_velocity() const { return y_relative_velocity_; }
  double relative_velocity_angle() const { return relative_velocity_angle_; }
  unsigned int fusion_source() const { return fusion_source_; };

  // double perception_velocity() const { return perception_velocity_; }
  double acceleration() const { return acc_; }
  // double acceleration_signed() const { return acc_signed_; }
  double velocity_angle() const { return velocity_angle_; }
  bool is_static() const { return is_static_; }
  iflyauto::ObjectType type() const { return type_; }
  SourceType source_type() const { return source_type_; }
  bool is_vaild() const { return valid_; }
  bool is_reverse() const { return is_reverse_; }
  void set_is_reverse(bool is_reverse) {
    is_reverse_ = is_reverse;
  }
  bool abnormal_data_dectection(const PredictionObject &prediction_object);
  bool is_oversize_vehicle() const { return is_oversize_vehicle_; }
  bool is_VRU() const { return is_VRU_; }
  bool is_traffic_facilities() const { return is_traffic_facilities_; }
  bool is_car() const { return is_car_; }
  bool trajectory_valid() const { return trajectory_valid_; }

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

  planning_math::Box2d get_bounding_box(const PncTrajectoryPoint &point) const;

 private:
  int id_{};
  int perception_id_ = 0;
  double timestamp_ = 0;  // 单位：s
  bool is_static_ = false;
  double x_center_;
  double y_center_;
  double x_relative_center_;
  double y_relative_center_;
  double z_center_ = 0.0;
  double yaw_;
  double relative_yaw_;
  double velocity_ = 0.0;
  // double perception_velocity_{};
  double velocity_angle_ = 0.0;
  double x_relative_velocity_ = 0.0;
  double y_relative_velocity_ = 0.0;
  double relative_velocity_angle_ = 0.0;
  double acc_ = 0.0;
  // double acc_signed_ = 0.0;
  iflyauto::ObjectType type_;
  bool valid_ = true;
  bool is_virtual_ = false;
  double prob_;
  // double speed_ = 0.0;
  // double speed_direction_ = 0.0;
  double width_;
  double length_;
  bool is_oversize_vehicle_ = false;
  bool is_VRU_ = false;
  bool is_traffic_facilities_ = false;
  bool is_car_ = false;
  bool trajectory_valid_ = false;
  bool is_reverse_ = false;
  std::vector<PncTrajectoryPoint> trajectory_{};
  // iflyauto::FusionObject perception_obstacle_;
  planning_math::Box2d perception_bounding_box_;
  planning_math::Polygon2d perception_polygon_;
  planning_math::Polygon2d obstacle_ego_polygon_;
  planning_math::Polygon2d car_ego_polygon_;
  std::vector<planning_math::Vec2d> perception_points_;
  unsigned int fusion_source_;
  SourceType source_type_;
};

// typedef IndexedList<int, Obstacle> IndexedObstacles;
}  // namespace planning

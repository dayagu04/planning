#pragma once

#include <algorithm>
#include <limits>
#include "modules/common/config/basic_type.h"
#include "modules/common/speed/sl_polygon_seq.h"
#include "modules/context/obstacle.h"
#include "modules/context/ego_state_manager.h"

namespace planning {

class ReferencePath;

class FrenetObstacle {
 public:
  FrenetObstacle(const Obstacle* obstacle_ptr,
                 const ReferencePath& reference_path,
                 const std::shared_ptr<EgoStateManager> ego_state_info);

  int id() const { return id_; }
  Common::ObjectType type() const { return obstacle_ptr_->type(); }

  double frenet_s() const { return frenet_s_; }
  double frenet_l() const { return frenet_l_; }
  double frenet_velocity_s() const { return frenet_velocity_s_; }
  double frenet_velocity_l() const { return frenet_velocity_l_; }
  double frenet_relative_velocity_angle() const {
    return frenet_relative_velocity_angle_;
  }
  double rel_s() const { return rel_s_; }
  Point2D s_min_l() const { return s_with_min_l_; }
  Point2D s_max_l() const { return s_with_max_l_; }
  double l_relative_to_ego() const { return l_relative_to_ego_; }

  const Obstacle* obstacle() const { return obstacle_ptr_; }
  double velocity() const { return obstacle_ptr_->velocity(); }
  const bool b_frenet_valid() const { return b_frenet_valid_; }

  const FrenetObstacleBoundary& frenet_obstacle_boundary() const {
    return frenet_obstacle_boundary_;
  }

  const FrenetBoundaryCorners& frenet_obstacle_corners() const {
    return frenet_obstacle_corners_;
  }

  const SLPolygonSeq& frenet_polygon_sequence() const {
    return frenet_polygon_sequence_;
  }

  bool get_polygon_at_time(const double relative_time,
                           const std::shared_ptr<ReferencePath>& reference_path,
                           planning_math::Polygon2d& obstacle_polygon) const;

 private:
  void compute_frenet_obstacle_boundary(const ReferencePath& reference_path);

  void compute_frenet_polygon_sequence(const ReferencePath& reference_path);

  static void generate_precise_frenet_polygon(
      planning_math::Polygon2d& polygon,
      std::shared_ptr<FrenetCoordinateSystem> frenet_coord);

 private:
  int id_;
  double adc_cart_x_;
  double adc_cart_y_;
  double frenet_s_;
  double frenet_l_;
  double frenet_velocity_s_;
  double frenet_velocity_l_;
  double frenet_relative_velocity_angle_;
  double rel_s_;

  Point2D s_with_min_l_; // x:l, y:s
  Point2D s_with_max_l_;

  double l_relative_to_ego_;

  const Obstacle* obstacle_ptr_ = nullptr;

  FrenetObstacleBoundary frenet_obstacle_boundary_;
  FrenetBoundaryCorners frenet_obstacle_corners_;
  SLPolygonSeq frenet_polygon_sequence_;

  bool b_frenet_valid_ = false;
  bool b_frenet_polygon_sequence_invalid_ = false;
};

}  // namespace planning

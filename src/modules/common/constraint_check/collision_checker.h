#pragma once
#include <limits>
#include <memory>

#include "common/math/box2d.h"
#include "common/math/polygon2d.h"
#include "context/ego_model_manager.h"
#include "pose2d.h"
#include "src/modules/common/utils/path_point.h"

namespace planning {
namespace planning_math {
struct CollisionCheckStatus {
  enum class CollisionType {
    NONE_COLLISION = 0,
    AGENT_COLLISION = 1,
    ROAD_EDGE_COLLISION = 2,
    GROUNDLINE_COLLISION = 3,
  };
  bool is_collision = false;
  // distance to collision or distance to the traj point closest to obstacle
  double s = 0.0;
  double min_distance = 0.0;  // minimum distacne to obstacles (>=0)
  Pose2D ego_poit{0, 0, 0};
  int16_t point_index = 0;
  int obstacle_id = 0;
  CollisionType collision_type = CollisionType::NONE_COLLISION;
  Pose2D collision_object_position{std::numeric_limits<double>::max(),
                                   std::numeric_limits<double>::max(), 0};
};

class CollisionChecker {
 public:
  CollisionChecker();

  // deviation_length : the distance between trajectory point and center of ego
  // vehicle
  bool set_params(const double deviation_length);

  // set ego position/trajectory
  bool set_trajectory(const std::vector<PathPoint> &trajectory);
  bool set_trajectory(const std::vector<Pose2D> &trajectory);
  bool set_point(const PathPoint &point);
  bool set_point(const Pose2D &point);

  // box
  CollisionCheckStatus collision_check(
      const planning_math::Box2d &box, const double collision_threshold,
      CollisionCheckStatus::CollisionType collision_type =
          CollisionCheckStatus::CollisionType::NONE_COLLISION);
  // line
  CollisionCheckStatus collision_check(
      const planning_math::LineSegment2d &line,
      const double collision_threshold,
      CollisionCheckStatus::CollisionType collision_type =
          CollisionCheckStatus::CollisionType::NONE_COLLISION);
  // point
  CollisionCheckStatus collision_check(
      const planning_math::Vec2d &point, const double collision_threshold,
      CollisionCheckStatus::CollisionType collision_type =
          CollisionCheckStatus::CollisionType::NONE_COLLISION);

  CollisionCheckStatus collision_check(
      const planning_math::Polygon2d &polygon, const double collision_threshold,
      CollisionCheckStatus::CollisionType collision_type);

  bool is_polygon_within_range(const planning_math::Polygon2d &polygon,
                               double xy_range);

  bool is_trajectory() const { return check_type_ == CheckType::TRAJECTORY; }
  bool is_point() const { return check_type_ == CheckType::POINT; }
  const std::shared_ptr<EgoModelManager> &get_ego_model() const {
    return ego_model_;
  }

 private:
  enum CheckType {
    TRAJECTORY = 0,
    POINT,
  };

  double deviation_length_;
  std::vector<PathPoint> trajectory_;
  PathPoint point_;
  CheckType check_type_;
  bool use_double_box_ = false;
  bool use_polygon_ = false;
  bool use_decagon_ = false;
  bool use_tetradecagon_ = false;
  bool use_rectangle_hexagon_ = false;
  std::shared_ptr<EgoModelManager> ego_model_;
};
}  // namespace planning_math
}  // namespace planning
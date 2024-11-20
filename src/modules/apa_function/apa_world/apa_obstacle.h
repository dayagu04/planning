#pragma once

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "pose2d.h"
#include "ad_common/math/box2d.h"
#include "config/message_type.h"
#include "utils/index_list.h"
#include "src/library/collision_detection/polygon_base.h"
#include "src/library/collision_detection/aabb2d.h"

namespace planning {

enum class ParkObstacleType {
  FUSION_OBJECT_POINT_CLOUD,
  FUSION_OBJECT_POLYGON,
  GROUND_LINE,
  USS,
  VIRTUAL,
  MAP_BOUND,
  SLOT_LIMITER,
  SLOT_LINE,
  HOLE,
  COUNT,
  INVALID,
};

/**
 * @class Obstacle
 * @brief This is the class that associates an Obstacle with its path
 * properties. An obstacle's path properties relative to a path.
 * The `s` and `l` values are examples of path properties.
 * The decision of an obstacle is also associated with a path.
 *
 * The decisions have two categories: lateral decision and longitudinal
 * decision.
 * Lateral decision includes: nudge, ignore.
 * Lateral decision safety priority: nudge > ignore.
 * Longitudinal decision includes: stop, yield, follow, overtake, ignore.
 * Decision safety priorities order: stop > yield >= follow > overtake > ignore
 *
 * Ignore decision belongs to both lateral decision and longitudinal decision,
 * and it has the lowest priority.
 */
class ParkObstacle {
 public:
  ParkObstacle() = default;

  void Init();

  double speed() const { return speed_; }

  int PerceptionId() const { return perception_id_; }

  void SetId(const int id) {
    id_ = id;
    return;
  }

  int Id() const { return id_; }

  bool IsStatic() const { return is_static_; }

  bool IsVirtual() const { return is_virtual_; }

  const cdl::AABB& PerceptionBoundingBox() const {
    return perception_box_;
  }

  void SetPerceptionBox(const cdl::AABB &box)  {
    perception_box_ = box;

    return;
  }

  const Polygon2D& PerceptionPolygon() const {
    return perception_polygon_;
  }

  const std::vector<PncTrajectoryPoint>& Trajectory() const {
    return prediction_traj_;
  }

  bool HasTrajectory() const {
    return !(prediction_traj_.empty());
  }

  std::string DebugString() const { return "none"; };

  void SetPerceptionSourceType(const ParkObstacleType type) {
    perception_source_type_ = type;

    return;
  }

  void SetPoints(const std::vector<Position3D>& points) {
    perception_points_ = points;

    return;
  }

 private:
  int id_ = 0;
  int perception_id_ = 0;
  bool is_static_ = false;
  bool is_virtual_ = false;

  double speed_ = 0.0;
  double acc_;

  double height_top_;
  double height_bottom_;
  Pose2D pose_;

  ParkObstacleType perception_source_type_;
  ObjectType semantic_type_;

  std::vector<PncTrajectoryPoint> prediction_traj_;

  cdl::AABB perception_box_;
  Polygon2D perception_polygon_;
  Polygon2D obs_slot_polygon_;

  std::vector<Position3D> perception_points_;
};

typedef IndexedList<int, ParkObstacle> IndexedParkObstacles;
typedef ThreadSafeIndexedList<int, ParkObstacle> ThreadSafeParkObstacles;

}  // namespace planning

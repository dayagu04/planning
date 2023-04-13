#pragma once

#include <memory>
#include <vector>

// #include "src/modules/common/define/path_point.h"
#include "src/framework/session.h"
#include "src/modules/common/config/basic_type.h"
#include "src/modules/context/frenet_ego_state.h"
#include "src/modules/context/frenet_obstacle.h"

#include "res/include/proto/common.pb.h"
// #include "src/modules/environmental_model/mdk/horizonkit/horizonkit.h"
// #include "mdk.h"

namespace planning {

enum class ReferencePathPointType { MAP, TRAJ, INTERPOLATE };

struct ReferencePathPoint {
  Point2D frenet_point;
  Point3D enu_point;
  double curvature;
  double yaw;
  double distance_to_left_road_border;
  double distance_to_right_road_border;
  double distance_to_left_lane_border;
  double distance_to_right_lane_border;
  Common::LaneBoundaryType left_road_border_type;
  Common::LaneBoundaryType right_road_border_type;
  Common::LaneBoundaryType left_lane_border_type;
  Common::LaneBoundaryType right_lane_border_type;
  ReferencePathPointType type;
  bool is_in_intersection;
};
using ReferencePathPoints = std::vector<ReferencePathPoint>;

class ReferencePath {
 public:
  ReferencePath();
  virtual ~ReferencePath() = default;

  bool valid() { return valid_; }
  virtual void update(planning::framework::Session *session);
  virtual void update_obstacles();

  const std::vector<ReferencePathPoint> &get_points() const { return points_; }

  const std::shared_ptr<FrenetCoordinateSystem> &get_frenet_coord() const {
    return frenet_coord_;
  }

  const FrenetEgoState &get_frenet_ego_state() const {
    return frenet_ego_state_;
  }

  const FrenetBoundary &get_ego_frenet_boundary() const {
    return frenet_ego_state_.boundary();
  }

  const std::vector<FrenetObstacle> &get_obstacles() const {
    return frenet_obstacles_;
  }

  const std::unordered_map<int, FrenetObstacle> &get_obstacles_map() const {
    return frenet_obstacles_map_;
  }
  virtual bool is_obstacle_ignorable(const FrenetObstacle &obstacle);

  const std::vector<const Obstacle *> &get_parking_space() const {
    return parking_spaces_;
  }

  const std::vector<const Obstacle *> &get_free_space_ground_lines() const {
    return free_space_ground_lines_;
  }

  const std::vector<const Obstacle *> &get_road_edges() const {
    return road_edges_;
  }

  bool get_reference_point_by_lon(
      double s, ReferencePathPoint &reference_path_point) const;
  bool transform_trajectory_points(TrajectoryPoints &trajectory_points) const;
  bool transform_trajectory_point(TrajectoryPoint &trajectory_point) const;

 protected:
  void init();
  void update_refpath_points(const ReferencePathPoints &points);

 protected:
  bool valid_;
  ReferencePathPoints points_;

  // frenet coord system
  FrenetCoordinateSystemParameters frenet_parameters_;
  std::shared_ptr<FrenetCoordinateSystem> frenet_coord_;

  // ego_state
  FrenetEgoState frenet_ego_state_;

  // obstacles
  std::vector<FrenetObstacle> frenet_obstacles_;

  std::unordered_map<int, FrenetObstacle> frenet_obstacles_map_;

  std::vector<const Obstacle *> parking_spaces_;
  std::vector<const Obstacle *> free_space_ground_lines_;
  std::vector<const Obstacle *> road_edges_;

  // session
  planning::framework::Session *session_;

  //  DISALLOW_COPY_AND_ASSIGN(ReferencePath);
};

}  // namespace planning

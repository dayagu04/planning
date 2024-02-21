#pragma once

#include <memory>
#include <vector>

#include "common.pb.h"
#include "config/basic_type.h"
#include "frenet_ego_state.h"
#include "frenet_obstacle.h"
#include "session.h"
#include "utils/kd_path.h"
#include "utils/path_point.h"

namespace planning {

enum class ReferencePathPointType { MAP, TRAJ, INTERPOLATE };

struct ReferencePathPoint {
  PathPoint path_point;
  double distance_to_left_road_border;
  double distance_to_right_road_border;
  double distance_to_left_lane_border;
  double distance_to_right_lane_border;
  Common::LaneBoundaryType left_road_border_type;
  Common::LaneBoundaryType right_road_border_type;
  Common::LaneBoundaryType left_lane_border_type;
  Common::LaneBoundaryType right_lane_border_type;
  double lane_width;
  double max_velocity;
  double min_velocity;
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

  const std::vector<ReferencePathPoint> &get_points() const {
    return refined_ref_path_points_;
  }

  // const std::shared_ptr<FrenetCoordinateSystem> &get_frenet_coord() const {
  //   return frenet_coord_;
  // }

  const std::shared_ptr<KDPath> &get_frenet_coord() const {
    return frenet_coord_;
  }

  const FrenetEgoState &get_frenet_ego_state() const {
    return frenet_ego_state_;
  }

  const FrenetBoundary &get_ego_frenet_boundary() const {
    return frenet_ego_state_.boundary();
  }

  const std::vector<std::shared_ptr<FrenetObstacle>> &get_obstacles() const {
    return frenet_obstacles_;
  }

  std::vector<std::shared_ptr<FrenetObstacle>> &mutable_obstacles() {
    return frenet_obstacles_;
  }

  std::unordered_map<int, std::shared_ptr<FrenetObstacle>>
      &mutable_obstacles_map() {
    return frenet_obstacles_map_;
  }

  const std::unordered_map<int, std::shared_ptr<FrenetObstacle>>
      &get_obstacles_map() const {
    return frenet_obstacles_map_;
  }

  std::vector<int> &mutable_obstacles_in_lane_map() {
    return obstacles_in_lane_map_;
  }

  const std::vector<int> &get_obstacles_in_lane_map() const {
    return obstacles_in_lane_map_;
  }

  virtual bool is_obstacle_ignorable(
      const std::shared_ptr<FrenetObstacle> obstacle);

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

 public:
  // 用在sort函数中，应使用全局量或Lambda函数
  inline static bool compare_obstacle_s_descend(
      const std::shared_ptr<FrenetObstacle> o1,
      const std::shared_ptr<FrenetObstacle> o2) {
    return (o1->frenet_s() > o2->frenet_s());
  }

  inline static bool compare_obstacle_s_ascend(
      const std::shared_ptr<FrenetObstacle> o1,
      const std::shared_ptr<FrenetObstacle> o2) {
    return (o1->frenet_s() < o2->frenet_s());
  }

 protected:
  void init();
  void update_refpath_points(
      const ReferencePathPoints &raw_reference_path_points);

  bool get_reference_point_by_lon_from_raw_ref_path_points(
      double s, const ReferencePathPoints &raw_reference_path_point,
      ReferencePathPoint &reference_path_point);
  void discrete(double start, double end, double gap,
                std::vector<double> &output) {
    output.clear();
    for (double value = start; value < end; value += gap) {
      output.push_back(value);
    }
  }

 protected:
  bool valid_;
  ReferencePathPoints refined_ref_path_points_;
  // frenet coord system
  // FrenetCoordinateSystemParameters frenet_parameters_;
  // std::shared_ptr<FrenetCoordinateSystem> frenet_coord_;

  // kd_path
  std::shared_ptr<KDPath> frenet_coord_;

  // ego_state
  FrenetEgoState frenet_ego_state_;

  // obstacles
  std::vector<std::shared_ptr<FrenetObstacle>> frenet_obstacles_;

  std::unordered_map<int, std::shared_ptr<FrenetObstacle>>
      frenet_obstacles_map_;

  std::vector<int> obstacles_in_lane_map_;

  std::vector<const Obstacle *> parking_spaces_;
  std::vector<const Obstacle *> free_space_ground_lines_;
  std::vector<const Obstacle *> road_edges_;

  // session
  planning::framework::Session *session_;

  //  DISALLOW_COPY_AND_ASSIGN(ReferencePath);
};

}  // namespace planning

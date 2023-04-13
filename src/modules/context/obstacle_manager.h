#pragma once

#include "src/modules/context/ego_planning_config.h"
#include "src/framework/session.h"
#include "src/modules/common/utils/index_list.h"
#include "src/modules/context/frenet_obstacle.h"
#include "src/modules/context/obstacle.h"
#include "src/modules/context/reference_path.h"
#include "src/modules/context/environmental_model.h"

#include "../res/include/proto/fusion_objects.pb.h"

namespace planning {

class ObstacleManager {

  public:
  ObstacleManager(const EgoPlanningConfigBuilder *config_builder,
                  planning::framework::Session *session);

  void update();

  Obstacle *add_obstacle(const Obstacle &obstacle);

  const Obstacle *find_obstacle(int object_id) const;

  Obstacle *find_obstacle(int object_id);

  const IndexedList<int, Obstacle> &get_obstacles() const;

  std::vector<FrenetObstacle> get_reference_path_obstacles(
      const ReferencePath &reference_path) const;

  std::unordered_map<int, FrenetObstacle> get_reference_path_obstacles_map(
      const ReferencePath &reference_path) const;

  const std::vector<int> &get_lane_obstacles(int lane_virtual_id) const {
    if (lanes_obstacles_.count(lane_virtual_id) > 0) {
      return lanes_obstacles_.at(lane_virtual_id);
    } else {
      LOG_ERROR("[ObstacleManager] lane_virtual_id does not exist");
      return lanes_obstacles_.at(-1);
    }
  }

  int get_lane_leadone_obstacle(int lane_virtual_id) const {
    if (lanes_leadone_obstacle_.count(lane_virtual_id) > 0) {
      return lanes_leadone_obstacle_.at(lane_virtual_id);
    } else {
      LOG_ERROR("[ObstacleManager] lane_virtual_id does not exist");
      return -1;
    }
  }

  int get_lane_leadtwo_obstacle(int lane_virtual_id) const {
    if (lanes_leadtwo_obstacle_.count(lane_virtual_id) > 0) {
      return lanes_leadtwo_obstacle_.at(lane_virtual_id);
    } else {
      LOG_ERROR("[ObstacleManager] lane_virtual_id does not exist");
      return -1;
    }
  }

  void assign_obstacles_to_lanes();

  // lidar road edge
  Obstacle *add_road_edge_obstacle(const Obstacle &obstacle) {
    return road_edge_obstacles_.Add(obstacle.id(), obstacle);
  }

  const IndexedList<int, Obstacle> &get_road_edge_obstacles() const {
    return road_edge_obstacles_;
  }

  Obstacle *add_groundline_obstacle(const Obstacle &obstacle) {
    return groundline_obstacles_.Add(obstacle.id(), obstacle);
  }

  const IndexedList<int, Obstacle> &get_groundline_obstacles() const {
    return groundline_obstacles_;
  }

  Obstacle *add_map_static_obstacle(const Obstacle &obstacle) {
    return map_static_obstacles_.Add(obstacle.id(), obstacle);
  }

  const IndexedList<int, Obstacle> &get_map_static_obstacles() const {
    return map_static_obstacles_;
  }

  Obstacle *add_parking_space(const Obstacle &obstacle) {
    return parking_space_obstacles_.Add(obstacle.id(), obstacle);
  }

  const IndexedList<int, Obstacle> &get_parking_space() const {
    return parking_space_obstacles_;
  }

 public:
  // 用在sort函数中，应使用全局量或Lambda函数
  inline static bool compare_obstacle_s_descend(const FrenetObstacle &o1,
                                  const FrenetObstacle &o2) {
    return (o1.frenet_s() > o2.frenet_s());
  }

  inline static bool compare_obstacle_s_ascend(const FrenetObstacle &o1,
                                const FrenetObstacle &o2) {
    return (o1.frenet_s() < o2.frenet_s());
  }

 private:
  void clear();

 private:
  planning::framework::Session *session_ = nullptr;
  IndexedList<int, Obstacle> obstacles_;
  IndexedList<int, Obstacle> groundline_obstacles_;
  IndexedList<int, Obstacle> map_static_obstacles_;
  IndexedList<int, Obstacle> parking_space_obstacles_;
  IndexedList<int, Obstacle> road_edge_obstacles_;
  EgoPlanningObstacleManagerConfig config_;
  std::unordered_map<int, std::vector<int>> lanes_obstacles_;
  std::unordered_map<int, int> lanes_leadone_obstacle_;
  std::unordered_map<int, int> lanes_leadtwo_obstacle_;
  std::unordered_map<int, std::vector<int>> lanes_virtual_obstacles_;
};

} // namespace planning

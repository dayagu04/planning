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

  std::vector<std::shared_ptr<FrenetObstacle>> get_reference_path_obstacles(
      const ReferencePath &reference_path) const;

  std::unordered_map<int, std::shared_ptr<FrenetObstacle>> get_reference_path_obstacles_map(
      const ReferencePath &reference_path) const;

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

 private:
  void clear();
  // bool is_potential_current_leadone_leadtwo_to_ego(const std::shared_ptr<FrenetObstacle> &frenet_obstacle);
 private:
  planning::framework::Session *session_ = nullptr;
  IndexedList<int, Obstacle> obstacles_;
  IndexedList<int, Obstacle> groundline_obstacles_;
  IndexedList<int, Obstacle> map_static_obstacles_;
  IndexedList<int, Obstacle> parking_space_obstacles_;
  IndexedList<int, Obstacle> road_edge_obstacles_;
  EgoPlanningObstacleManagerConfig config_;
  // std::unordered_map<int, std::vector<int>> lanes_obstacles_;
  
  std::unordered_map<int, std::vector<int>> lanes_virtual_obstacles_;
};

} // namespace planning

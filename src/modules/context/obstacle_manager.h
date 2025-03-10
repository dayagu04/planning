#pragma once

#include <memory>
#include "ego_planning_config.h"
#include "environmental_model.h"
#include "euler_distance_transform.h"
#include "frenet_obstacle.h"
#include "fusion_objects_c.h"
#include "obstacle.h"
#include "reference_path.h"
#include "session.h"
#include "uss_obstacle.h"
#include "utils/index_list.h"

using namespace planning::planning_math;
namespace planning {

class ObstacleManager {
 public:
  ObstacleManager(const EgoPlanningConfigBuilder *config_builder,
                  planning::framework::Session *session);

  void SetConfig(const EgoPlanningConfigBuilder *config_builder);

  void update();

  Obstacle *add_obstacle(const Obstacle &obstacle);

  const Obstacle *find_obstacle(int object_id) const;

  Obstacle *find_obstacle(int object_id);

  const IndexedList<int, Obstacle> &get_obstacles() const;

  void generate_frenet_obstacles(ReferencePath &reference_path);

  Obstacle *find_gs_care_obstacle(int object_id);

  Obstacle *add_gs_care_obstacles(const Obstacle &obstacle) {
    return gs_care_obstacles_.Add(obstacle.id(), obstacle);
  }

  const IndexedList<int, Obstacle> &get_gs_care_obstacles() const {
    return gs_care_obstacles_;
  }

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

  Obstacle *add_occupancy_obstacle(const Obstacle &obstacle) {
    return occupancy_obstacles_.Add(obstacle.id(), obstacle);
  }

  const IndexedList<int, Obstacle> &get_occupancy_obstacles() const {
    return occupancy_obstacles_;
  }

  double GetUssRemainDistance() {
    double remain_dist_uss = 5.01;
    const double kSafeUssRemainDist = 0.35;
    if (uss_obstacle_.GetAvailable()) {
      // update remain_dist_uss
      remain_dist_uss = uss_obstacle_.GetRemainDist() - kSafeUssRemainDist;
    }
    return remain_dist_uss;
  }

  void add_frenet_obstacle(
      IndexedList<int, Obstacle> &obstacles, ReferencePath &reference_path,
      std::vector<std::shared_ptr<FrenetObstacle>> &frenet_obstacles,
      std::unordered_map<int, std::shared_ptr<FrenetObstacle>>
          &frenet_obstacles_map,
      std::vector<int> &obstacles_ids_in_lane_map);

  void UpdateOccObstacle();

  void UpdateGroundLineObstacle();

  void UpdateParkingSpaceObstacle();

  void UpdateMapStaticObstacle();

  bool IsOnBend(const std::shared_ptr<ReferencePath> &reference_path,
                double ego_s);

 private:
  void clear();
  // bool is_potential_current_leadone_leadtwo_to_ego(const
  // std::shared_ptr<FrenetObstacle> &frenet_obstacle);

 private:
  planning::framework::Session *session_ = nullptr;
  IndexedList<int, Obstacle> obstacles_;
  IndexedList<int, Obstacle> groundline_obstacles_;
  IndexedList<int, Obstacle> gs_care_obstacles_;
  IndexedList<int, Obstacle> map_static_obstacles_;
  IndexedList<int, Obstacle> parking_space_obstacles_;
  IndexedList<int, Obstacle> road_edge_obstacles_;
  IndexedList<int, Obstacle> occupancy_obstacles_;
  EgoPlanningObstacleManagerConfig config_;
  // std::unordered_map<int, std::vector<int>> lanes_obstacles_;
  UssObstacle uss_obstacle_;
  std::unordered_map<int, std::vector<int>> lanes_virtual_obstacles_;
  std::shared_ptr<planning::GroundLineManager> ground_line_manager_ptr_ =
      nullptr;
};

}  // namespace planning

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

  Obstacle *add_speed_bump_obstacle(const Obstacle &obstacle) {
    return speed_bump_obstacles_.Add(obstacle.id(), obstacle);
  }

  const IndexedList<int, Obstacle> &get_speed_bump_obstacles() const {
    return speed_bump_obstacles_;
  }

  Obstacle *add_turnstile_obstacle(const Obstacle &obstacle) {
    return turnstile_obstacles_.Add(obstacle.id(), obstacle);
  }

  const IndexedList<int, Obstacle> &get_turnstile_obstacles() const {
    return turnstile_obstacles_;
  }

  Obstacle *add_semantic_sign_obstacle(const Obstacle &obstacle) {
    return semantic_sign_obstacles_.Add(obstacle.id(), obstacle);
  }

  const IndexedList<int, Obstacle> &get_semantic_sign_obstacles() const {
    return semantic_sign_obstacles_;
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

  Obstacle *add_speed_bump_obstacle(const Obstacle &obstacle) {
    return speed_bump_obstacles_.Add(obstacle.id(), obstacle);
  }

  const IndexedList<int, Obstacle> &get_speed_bump_obstacles() const {
    return speed_bump_obstacles_;
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
          &frenet_obstacles_map);

  void UpdateOccObstacle();

  void UpdateGroundLineObstacle();

  void UpdateParkingSpaceObstacle();

  void UpdateMapStaticObstacle();

  /******************* for hpp start *********************/
  // 从 local_view.fusion_speed_bump_info 获取减速带（视觉 OD + 地图减速带融合结果）
  void UpdateSpeedBumpObstacle();

  // 从 local_view.fusion_obstacles_info 获取闸机（视觉 OD + 地图闸机融合结果）
  void UpdateTurnStileObstacle();

  // 从 local_view 获取语义标志（比如路口、转弯箭头……）
  void UpdateSemanticSignObstacle();
  /******************* for hpp end   *********************/

  bool IsOnBend(const std::shared_ptr<ReferencePath> &reference_path,
                double ego_s);

  void split_points(const iflyauto::Point2f *points,
                    const double polygon_points_size,
                    const std::shared_ptr<planning_math::KDPath> &frenet_coord,
                    std::vector<std::vector<planning_math::Vec2d>> &result);

  void split_points_by_s(
      const std::vector<std::pair<std::pair<double, double>, planning_math::Vec2d>>& points_vec,
      std::vector<std::vector<planning_math::Vec2d>> &result);

  void ProcessOccupancyWall(
      const iflyauto::FusionOccupancyObject &object,
      const iflyauto::Point2f *polygon_points, size_t polygon_size,
      const std::shared_ptr<planning_math::KDPath> &frenet_coord,
      const Point2D &ego_point, int &index_offset);

  void ProcessOccupancyObject(
      const iflyauto::FusionOccupancyObject &object,
      const iflyauto::Point2f *polygon_points, size_t polygon_size,
      const std::shared_ptr<planning_math::KDPath> &frenet_coord,
      const Point2D &ego_point);

  bool FilterObstacleByDistance(
      const Obstacle &obstacle,
      const std::shared_ptr<planning_math::KDPath> &frenet_coord,
      const Point2D &ego_point);

  bool FilterGroundLineByDistance(
      const std::vector<planning_math::Vec2d> &points,
      const std::shared_ptr<planning_math::KDPath> &frenet_coord,
      const Point2D &ego_point, const iflyauto::GroundLineType type,
      const iflyauto::StaticFusionResourceType resource_type);

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
  /*********************** for hpp start *************************/
  IndexedList<int, Obstacle> speed_bump_obstacles_;       // 减速带障碍物：需要控速，不需要刹停或绕障
  IndexedList<int, Obstacle> turnstile_obstacles_;        // 闸机障碍物：需要走闸机通行逻辑
  IndexedList<int, Obstacle> semantic_sign_obstacles_;    // 语义标识障碍物：不需要停障或绕障，只是表明该处的语义环境，具体处理逻辑右下游决定
  /*********************** for hpp end   *************************/
  EgoPlanningObstacleManagerConfig config_;
  // std::unordered_map<int, std::vector<int>> lanes_obstacles_;
  UssObstacle uss_obstacle_;
  std::unordered_map<int, std::vector<int>> lanes_virtual_obstacles_;
  std::shared_ptr<planning::GroundLineManager> ground_line_manager_ptr_ =
      nullptr;
};

using ObstacleManagerPtr = std::shared_ptr<ObstacleManager>;
using ConstObstacleManagerPtr = std::shared_ptr<const ObstacleManager>;

}  // namespace planning

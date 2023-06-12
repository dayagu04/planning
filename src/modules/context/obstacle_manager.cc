#include <tuple>

#include "math/math_utils.h"
#include "obstacle_manager.h"
#include "ego_state_manager.h"
#include "reference_path_manager.h"
#include "virtual_lane_manager.h"

namespace planning {

ObstacleManager::ObstacleManager(
    const EgoPlanningConfigBuilder *config_builder,
    planning::framework::Session *session)
    : session_(session) {
  config_ = config_builder->cast<EgoPlanningObstacleManagerConfig>();
}

void ObstacleManager::update() {
  clear();
  const auto &ego_state =
      *session_->mutable_environmental_model()->get_ego_state_manager();
  double ego_init_relative_time = ego_state.planning_init_point().relative_time;
  bool enable_bbox_mode = config_.enable_bbox_mode;
  const auto &prediction_objects =
        session_->environmental_model().get_prediction_info();
  for (int i = 0; i < session_->environmental_model().get_prediction_info().size(); i++) {
    auto prediction_object = prediction_objects[i];
    bool is_static = prediction_object.speed < 0.1 ||
                     prediction_object.trajectory_array.size() == 0;
    double prediction_relative_time =
        prediction_object.delay_time - ego_init_relative_time;
    if (prediction_object.trajectory_array.size() == 0) {
      auto obstacle =
          Obstacle(prediction_object.id, prediction_object, is_static,
                   prediction_relative_time);
      add_obstacle(obstacle);
      continue;
    }
    for (int i = 0; i < prediction_object.trajectory_array.size(); ++i) {
      Obstacle obstacle(prediction_object.id, prediction_object, is_static,
                        prediction_relative_time);
      if (obstacle.is_vaild()) {
        add_obstacle(obstacle);
      }
      break;  // only use the first traj
    }
  }

}

void ObstacleManager::clear() {
  obstacles_ = IndexedList<int, Obstacle>();
  groundline_obstacles_ = IndexedList<int, Obstacle>();
  map_static_obstacles_ = IndexedList<int, Obstacle>();
  parking_space_obstacles_ = IndexedList<int, Obstacle>();
  road_edge_obstacles_ = IndexedList<int, Obstacle>();
}

Obstacle *ObstacleManager::add_obstacle(const Obstacle &obstacle) {
  return obstacles_.Add(obstacle.id(), obstacle);
}

Obstacle *ObstacleManager::find_obstacle(int object_id) {
  return obstacles_.Find(object_id);
}

const Obstacle *ObstacleManager::find_obstacle(int object_id) const {
  return obstacles_.Find(object_id);
}

const IndexedList<int, Obstacle> &ObstacleManager::get_obstacles() const {
  return obstacles_;
}

void ObstacleManager::generate_frenet_obstacles(
    ReferencePath &reference_path) {
  const auto &frenet_coord = reference_path.get_frenet_coord();
  auto &frenet_obstacles = reference_path.mutable_obstacles();
  auto &frenet_obstacles_map = reference_path.mutable_obstacles_map();
  frenet_obstacles.clear();
  frenet_obstacles_map.clear();
  constexpr double kCareDistance = 30.0;
  auto ego_s = reference_path.get_frenet_ego_state().s();
  auto ego_l = reference_path.get_frenet_ego_state().l();
  auto planning_init_x =
      reference_path.get_frenet_ego_state().planning_init_point().x;
  auto planning_init_y =
      reference_path.get_frenet_ego_state().planning_init_point().y;

  auto is_location_valid = session_->environmental_model().location_valid();

  for (const Obstacle *obstacle_ptr : obstacles_.Items()) {
    // filter some obstacle

    Point2D frenet_point, cart_point;
    if (is_location_valid) {
      cart_point.x = obstacle_ptr->x_center();
      cart_point.y = obstacle_ptr->y_center();
    } else {
      cart_point.x = obstacle_ptr->x_relative_center();
      cart_point.y = obstacle_ptr->y_relative_center();
    }
      

    // 在自车30m范围内，即使frenet失败也不能丢弃
    auto obstacle_care_flag =
        (fabs(cart_point.x - planning_init_x) < kCareDistance) &&
        (fabs(cart_point.y - planning_init_y) < kCareDistance);
    if ((frenet_coord->CartCoord2FrenetCoord(cart_point, frenet_point) ==
             TRANSFORM_FAILED &&
         obstacle_care_flag == false) ||
        std::isnan(frenet_point.x) || std::isnan(frenet_point.y) ||
        frenet_point.x > (config_.frenet_obstacle_range_s_max + ego_s) ||
        frenet_point.x < (config_.frenet_obstacle_range_s_min + ego_s) ||
        frenet_point.y > (config_.frenet_obstacle_range_l_max + ego_l) ||
        frenet_point.y < (config_.frenet_obstacle_range_l_min + ego_l)) {
      continue;
    }
    // construct frenet_obstacle
    std::shared_ptr<FrenetObstacle> frenet_obstacle =
                  std::make_shared<FrenetObstacle>(obstacle_ptr, reference_path,
                  session_->environmental_model().get_ego_state_manager());
    frenet_obstacles.emplace_back(frenet_obstacle);
    frenet_obstacles_map[obstacle_ptr->id()] = frenet_obstacle;
  }
}

// std::unordered_map<int, std::shared_ptr<FrenetObstacle>>
// ObstacleManager::get_reference_path_obstacles_map(
//     const ReferencePath &reference_path) const {
//   const auto &frenet_coord = reference_path.get_frenet_coord();
//   std::unordered_map<int, std::shared_ptr<FrenetObstacle>> frenet_ob_map;
//   auto ego_s = reference_path.get_frenet_ego_state().s();
//   auto ego_l = reference_path.get_frenet_ego_state().l();
//   for (const Obstacle *obstacle_ptr : obstacles_.Items()) {
//     // filter some obstacle

//     Point2D frenet_point, cart_point;
//     cart_point.x = obstacle_ptr->x_center();
//     cart_point.y = obstacle_ptr->y_center();
//     std::vector<std::shared_ptr<FrenetObstacle>> frenet_obstacles;
//     const double frenet_obstacle_range_s_max = 200.0;
//     const double frenet_obstacle_range_s_min = -50.0;
//     bool frenet_transform_flag =
//         frenet_coord->CartCoord2FrenetCoord(cart_point, frenet_point);
//     if (frenet_transform_flag == TRANSFORM_FAILED ||
//         std::isnan(frenet_point.x) || std::isnan(frenet_point.y) ||
//         frenet_point.x > (frenet_obstacle_range_s_max + ego_s) ||
//         frenet_point.x < (frenet_obstacle_range_s_min + ego_s) ||
//         frenet_point.y > (config_.frenet_obstacle_range_l_max + ego_l) ||
//         frenet_point.y < (config_.frenet_obstacle_range_l_min + ego_l)) {
//       continue;
//     }

//     frenet_ob_map.emplace(
//         std::piecewise_construct, std::forward_as_tuple(obstacle_ptr->id()),
//         std::forward_as_tuple(std::make_shared<FrenetObstacle>(
//             obstacle_ptr, reference_path,
//             session_->environmental_model().get_ego_state_manager())));
//   }
//   return frenet_ob_map;
// }

}  // namespace planning

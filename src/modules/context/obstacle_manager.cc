#include <tuple>

#include "modules/common/math/math_utils.h"

#include "modules/context/obstacle_manager.h"
#include "modules/context/ego_state_manager.h"
// #include "modules/context/groundline_decider.h"
#include "modules/context/reference_path_manager.h"
#include "modules/context/virtual_lane_manager.h"

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
        session_->environmental_model().get_mixed_prediction_info();
  for (int i = 0;
       i < session_->environmental_model().get_mixed_prediction_info().size();
       i++) {
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

std::vector<FrenetObstacle> ObstacleManager::get_reference_path_obstacles(
    const ReferencePath &reference_path) const {
  const auto &frenet_coord = reference_path.get_frenet_coord();
  std::vector<FrenetObstacle> frenet_obstacles;
  constexpr double kCareDistance = 30.0;
  auto ego_s = reference_path.get_frenet_ego_state().s();
  auto ego_l = reference_path.get_frenet_ego_state().l();
  auto planning_init_x =
      reference_path.get_frenet_ego_state().planning_init_point().x;
  auto planning_init_y =
      reference_path.get_frenet_ego_state().planning_init_point().y;

  for (const Obstacle *obstacle_ptr : obstacles_.Items()) {
    // filter some obstacle

    Point2D frenet_point, cart_point;
    cart_point.x = obstacle_ptr->x_center();
    cart_point.y = obstacle_ptr->y_center();
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
    frenet_obstacles.emplace_back(
        obstacle_ptr, reference_path,
        session_->environmental_model().get_ego_state_manager());
  }

  return frenet_obstacles;
}

std::unordered_map<int, FrenetObstacle>
ObstacleManager::get_reference_path_obstacles_map(
    const ReferencePath &reference_path) const {
  const auto &frenet_coord = reference_path.get_frenet_coord();
  std::unordered_map<int, FrenetObstacle> frenet_ob_map;
  auto ego_s = reference_path.get_frenet_ego_state().s();
  auto ego_l = reference_path.get_frenet_ego_state().l();
  for (const Obstacle *obstacle_ptr : obstacles_.Items()) {
    // filter some obstacle

    Point2D frenet_point, cart_point;
    cart_point.x = obstacle_ptr->x_center();
    cart_point.y = obstacle_ptr->y_center();
    std::vector<FrenetObstacle> frenet_obstacles;
    const double frenet_obstacle_range_s_max = 200.0;
    const double frenet_obstacle_range_s_min = -50.0;
    bool frenet_transform_flag =
        frenet_coord->CartCoord2FrenetCoord(cart_point, frenet_point);
    if (frenet_transform_flag == TRANSFORM_FAILED ||
        std::isnan(frenet_point.x) || std::isnan(frenet_point.y) ||
        frenet_point.x > (frenet_obstacle_range_s_max + ego_s) ||
        frenet_point.x < (frenet_obstacle_range_s_min + ego_s) ||
        frenet_point.y > (config_.frenet_obstacle_range_l_max + ego_l) ||
        frenet_point.y < (config_.frenet_obstacle_range_l_min + ego_l)) {
      continue;
    }

    frenet_ob_map.emplace(
        std::piecewise_construct, std::forward_as_tuple(obstacle_ptr->id()),
        std::forward_as_tuple(
            obstacle_ptr, reference_path,
            session_->environmental_model().get_ego_state_manager()));
  }
  return frenet_ob_map;
}

void ObstacleManager::assign_obstacles_to_lanes() {

  lanes_obstacles_.clear();
  lanes_leadone_obstacle_.clear();
  lanes_leadtwo_obstacle_.clear();
  // Insert default, in case virtual_lane does not exist
  std::vector<int> lane_obstacles{};
  lanes_obstacles_.insert(std::make_pair(-1, lane_obstacles));

  auto &virtual_lane_manager =
      session_->mutable_environmental_model()->virtual_lane_manager();
  auto &reference_path_manager =
      session_->mutable_environmental_model()->reference_path_manager();

  for (auto lane_ptr : virtual_lane_manager->get_virtual_lanes()) {
    int virtual_id = lane_ptr->get_virtual_id();
    auto reference_path_ptr =
        reference_path_manager->get_reference_path_by_lane(virtual_id);
    if (reference_path_ptr == nullptr) {
      continue;
    }

    std::vector<FrenetObstacle> sorted_obstacles;
    for (auto &frenet_obstacle : reference_path_ptr->get_obstacles()) {
      if (std::fabs(frenet_obstacle.frenet_l()) < 1.6) {
        sorted_obstacles.push_back(frenet_obstacle);
      }
    }
    std::sort(sorted_obstacles.begin(), sorted_obstacles.end(),
              compare_obstacle_s_ascend);

    std::vector<int> lane_obstacles{};
    int leadone{-1};
    int leadtwo{-1};
    double s_ego = reference_path_ptr->get_frenet_ego_state().s();
    for (auto &frenet_obstacle : sorted_obstacles) {
      lane_obstacles.push_back(frenet_obstacle.id());
      if (frenet_obstacle.frenet_s() > s_ego && leadone == -1) {
        leadone = frenet_obstacle.id();
        continue;
      }
      if (frenet_obstacle.frenet_s() > s_ego && leadone != -1 &&
          leadtwo == -1) {
        leadtwo = frenet_obstacle.id();
        break;
      }
    }
    lanes_obstacles_.insert(std::make_pair(virtual_id, lane_obstacles));
    lanes_leadone_obstacle_.insert(std::make_pair(virtual_id, leadone));
    lanes_leadtwo_obstacle_.insert(std::make_pair(virtual_id, leadtwo));
  }
}

void ObstacleManager::cal_current_leadone_leadtwo_to_ego(int lane_virtual_id) {
  int current_leadone_id{-1};
  int current_leadtwo_id{-1};
  auto &reference_path_manager =
      session_->mutable_environmental_model()->reference_path_manager();
  auto reference_path_ptr =
    reference_path_manager->get_reference_path_by_lane(lane_virtual_id);
  double s_ego = reference_path_ptr->get_frenet_ego_state().s();
  auto lane_obstacles = get_lane_obstacles(lane_virtual_id);
  std::vector<FrenetObstacle> sorted_obstacles;
  for (auto &frenet_obstacle : reference_path_ptr->get_obstacles()) {
    if (std::find(lane_obstacles.begin(), lane_obstacles.end(),frenet_obstacle.id()) == lane_obstacles.end() ||
       frenet_obstacle.frenet_s() > s_ego) {
      continue;
    }
    if (is_potential_current_leadone_leadtwo_to_ego(frenet_obstacle)) {
      sorted_obstacles.emplace_back(frenet_obstacle);
    }
  }

  std::sort(sorted_obstacles.begin(), sorted_obstacles.end(),
              compare_obstacle_s_ascend);

  for (auto &frenet_obstacle : sorted_obstacles) {
    if (current_leadone_id == -1) {
      current_leadone_id = frenet_obstacle.id();
      continue;
    }
    if (current_leadone_id != -1 &&
        current_leadtwo_id == -1) {
      current_leadtwo_id = frenet_obstacle.id();
      break;
    }
  }
  current_leadone_obstacle_ = current_leadone_id;
  current_leadtwo_obstacle_ = current_leadtwo_id;
}

bool ObstacleManager::is_potential_current_leadone_leadtwo_to_ego(const FrenetObstacle &frenet_obstacle) {
  double l_relative_to_ego = frenet_obstacle.l_relative_to_ego();
  if (fabs(l_relative_to_ego) < 1.6) {
    return true;
  } else {
    return false;
  }
}

}  // namespace planning

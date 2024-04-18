#include "obstacle_manager.h"

#include <tuple>

#include "ego_state_manager.h"
#include "math/math_utils.h"
#include "parking_slot_manager.h"
#include "reference_path_manager.h"
#include "virtual_lane_manager.h"
namespace planning {

ObstacleManager::ObstacleManager(const EgoPlanningConfigBuilder *config_builder,
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
  for (int i = 0;
       i < session_->environmental_model().get_prediction_info().size(); i++) {
    auto prediction_object = prediction_objects[i];
    if (prediction_object.type == 0 ||
        ((!(prediction_object.fusion_source & OBSTACLE_SOURCE_CAMERA)) &&
         (prediction_object.relative_position_x > 0 &&
          tan(25) > fabs(prediction_object.relative_position_y /
                         prediction_object.relative_position_x))) ||
        fabs(prediction_object.relative_position_y) > 10 ||
        prediction_object.length == 0 || prediction_object.width == 0) {
      LOG_DEBUG("[obstacle_prediction_update] ignore obstacle! : [%d] \n",
                prediction_object.id);
      continue;
    }

    double prediction_trajectory_length = 10.0;
    double prediction_duration = 0.0;
    if (prediction_object.trajectory_array.size() > 0) {
      const auto &trajectory_array = prediction_object.trajectory_array.at(0);
      if (trajectory_array.trajectory.size() > 0) {
        const auto &start_point = trajectory_array.trajectory.at(0);
        const auto &end_point = trajectory_array.trajectory.at(
            trajectory_array.trajectory.size() - 1);
        prediction_trajectory_length = std::sqrt(
            (start_point.x - end_point.x) * (start_point.x - end_point.x) +
            (start_point.y - end_point.y) * (start_point.y - end_point.y));
        prediction_duration = end_point.relative_time;
      }
    }

    const double kMaxStaticPredictionLength =
        config_.max_speed_static_obstacle * prediction_duration;
    bool is_static = prediction_object.speed < 0.1 ||
                     prediction_object.trajectory_array.size() == 0 ||
                     prediction_trajectory_length < kMaxStaticPredictionLength;
    double prediction_relative_time =
        prediction_object.delay_time - ego_init_relative_time;
    if (prediction_object.trajectory_array.size() == 0) {
      auto obstacle = Obstacle(prediction_object.id, prediction_object,
                               is_static, prediction_relative_time);
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

  if (!session_->is_hpp_scene()) {
    // add gs care obstacles
    for (int i = 0;
         i < session_->environmental_model().get_prediction_info().size();
         i++) {
      const auto &prediction_object = prediction_objects[i];
      const bool is_static = prediction_object.speed < 0.1 ||
                             prediction_object.trajectory_array.size() == 0;
      const double prediction_relative_time =
          prediction_object.delay_time - ego_init_relative_time;

      // add gs care obstacles
      const bool gs_care_fusion_source =
          (prediction_object.fusion_source == OBSTACLE_SOURCE_CAMERA) ||
          (prediction_object.fusion_source == OBSTACLE_SOURCE_F_RADAR_CAMERA) ||
          (prediction_object.fusion_source ==
           OBSTACLE_SOURCE_CAMERA_AND_FLRADAR) ||
          (prediction_object.fusion_source ==
           OBSTACLE_SOURCE_CAMERA_AND_FRRADAR) ||
          (prediction_object.fusion_source ==
           OBSTACLE_SOURCE_CAMERA_AND_RLRADAR) ||
          (prediction_object.fusion_source ==
           OBSTACLE_SOURCE_CAMERA_AND_RRRADAR);

      if (gs_care_fusion_source &&
          prediction_object.type != Common::ObjectType::OBJECT_TYPE_UNKNOWN &&
          prediction_object.length > 0. && prediction_object.width > 0. &&
          std::fabs(prediction_object.relative_position_y) < 10. &&
          std::fabs(prediction_object.relative_position_x) < 100.) {
        auto obstacle = Obstacle(prediction_object.id, prediction_object,
                                 is_static, prediction_relative_time);
        add_gs_care_obstacles(obstacle);
      }
    }
  } else {
    // fusion ground line
    static constexpr int kGroundLineIdOffset = 5000000;
    const std::vector<GroundLinePoint> &groundline =
        session_->environmental_model().get_ground_line_point_info();
    auto groundline_clusters = GroundLineDecider::execute(groundline);
    std::cout << "groundline_clusters.size = " << groundline_clusters.size()
              << std::endl;
    LOG_DEBUG("groundline_clusters.size = %d", groundline_clusters.size());
    int cluster_id = kGroundLineIdOffset;
    for (auto &groundline_cluster : groundline_clusters) {
      cluster_id += 1;
      Obstacle obstacle(cluster_id, groundline_cluster);
      add_groundline_obstacle(obstacle);
    }

    // parking space
    static constexpr int kParkingSlotIdOffset = 6000000;
    const std::shared_ptr<ParkingSlotManager> parking_slot_manager =
        session_->environmental_model().get_parking_slot_manager();
    const std::vector<ParkingSlotPoints> &parking_slot_points =
        parking_slot_manager->get_points();
    int parking_slot_id = kParkingSlotIdOffset;
    for (auto &parking_slot_point : parking_slot_points) {
      if (parking_slot_point.size() != 4U) {
        LOG_DEBUG("invalid parking_slot_point.size = %lu",
                  parking_slot_point.size());
        continue;
      }
      parking_slot_id += 1;
      Obstacle obstacle(parking_slot_id, parking_slot_point);
      add_parking_space(obstacle);
    }

    // update uss
    uss_obstacle_.SetLocalView(
        &session_->environmental_model().get_local_view());
    uss_obstacle_.Update();
  }
}

void ObstacleManager::clear() {
  obstacles_ = IndexedList<int, Obstacle>();
  groundline_obstacles_ = IndexedList<int, Obstacle>();
  map_static_obstacles_ = IndexedList<int, Obstacle>();
  parking_space_obstacles_ = IndexedList<int, Obstacle>();
  road_edge_obstacles_ = IndexedList<int, Obstacle>();
  gs_care_obstacles_ = IndexedList<int, Obstacle>();
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

Obstacle *ObstacleManager::find_gs_care_obstacle(int object_id) {
  return gs_care_obstacles_.Find(object_id);
}

const IndexedList<int, Obstacle> &ObstacleManager::get_obstacles() const {
  return obstacles_;
}

void ObstacleManager::generate_frenet_obstacles(ReferencePath &reference_path) {
  const auto &frenet_coord = reference_path.get_frenet_coord();
  auto &frenet_obstacles = reference_path.mutable_obstacles();
  auto &frenet_obstacles_map = reference_path.mutable_obstacles_map();
  auto &bstacles_ids_in_lane_map =
      reference_path.mutable_obstacles_in_lane_map();
  bstacles_ids_in_lane_map.clear();
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
    if ((!frenet_coord->XYToSL(cart_point, frenet_point) &&
         obstacle_care_flag == false) ||
        std::isnan(frenet_point.x) || std::isnan(frenet_point.y) ||
        frenet_point.x > (config_.frenet_obstacle_range_s_max + ego_s) ||
        frenet_point.x < (config_.frenet_obstacle_range_s_min + ego_s) ||
        frenet_point.y > (config_.frenet_obstacle_range_l_max + ego_l) ||
        frenet_point.y < (config_.frenet_obstacle_range_l_min + ego_l)) {
      auto iter = gs_care_obstacles_.Dict().find(obstacle_ptr->id());
      if (iter != gs_care_obstacles_.Dict().end()) {
        LOG_ERROR("This unnormal obj need to consider in gs");
      } else {
        continue;
      }
    }
    // construct frenet_obstacle
    std::shared_ptr<FrenetObstacle> frenet_obstacle =
        std::make_shared<FrenetObstacle>(
            obstacle_ptr, reference_path,
            session_->environmental_model().get_ego_state_manager(),
            is_location_valid);
    frenet_obstacles.emplace_back(frenet_obstacle);
    frenet_obstacles_map[obstacle_ptr->id()] = frenet_obstacle;

    // judge the strict lane obstacles
    // TODO:@cailiu, this condition can be released, its strict!
    const double half_width = obstacle_ptr->width() * 0.5;
    const double lat_buffer = 0.8;
    if (frenet_point.y + half_width < 1.5 + lat_buffer &&
        frenet_point.y - half_width > -1.5 - lat_buffer) {
      bstacles_ids_in_lane_map.emplace_back(obstacle_ptr->id());
    }
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
//     if (frenet_transform_flag  ||
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

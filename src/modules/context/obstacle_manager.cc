#include "obstacle_manager.h"

#include <tuple>

#include "ego_state_manager.h"
#include "interface/src/c/common_c.h"
#include "math/math_utils.h"
#include "parking_slot_manager.h"
#include "reference_path_manager.h"
#include "virtual_lane_manager.h"
namespace planning {

ObstacleManager::ObstacleManager(const EgoPlanningConfigBuilder *config_builder,
                                 planning::framework::Session *session)
    : session_(session) {
  SetConfig(config_builder);
}

void ObstacleManager::SetConfig(
    const EgoPlanningConfigBuilder *config_builder) {
  config_ = config_builder->cast<EgoPlanningObstacleManagerConfig>();
}

void ObstacleManager::update() {
  const double HALF_FOV = 25.0;
  clear();
  const auto &ego_state =
      *session_->mutable_environmental_model()->get_ego_state_manager();
  double ego_init_relative_time = ego_state.planning_init_point().relative_time;
  const auto &prediction_objects =
      session_->environmental_model().get_prediction_info();
  for (int i = 0;
       i < session_->environmental_model().get_prediction_info().size(); i++) {
    auto prediction_object = prediction_objects[i];
    if (prediction_object.type == iflyauto::ObjectType::OBJECT_TYPE_UNKNOWN) {
      LOG_DEBUG("[ObstacleManager Update] ignore unknown obstacle : [%d] \n",
                prediction_object.id);
      continue;
    }
    bool is_in_fov =
        prediction_object.relative_position_x > 0 &&
        (tan(HALF_FOV) > fabs(prediction_object.relative_position_y /
                              prediction_object.relative_position_x));
    bool is_fusion_with_camera =
        prediction_object.fusion_source & OBSTACLE_SOURCE_CAMERA;
    bool is_ignore_by_fov = is_in_fov && (is_fusion_with_camera == false);
    bool is_ignore_by_size =
        prediction_object.length == 0 || prediction_object.width == 0;

    if (is_ignore_by_fov || is_ignore_by_size) {
      LOG_DEBUG("[ObstacleManager Update] ignore obstacle! : [%d] \n",
                prediction_object.id);
      continue;
    }

    double prediction_relative_time =
        prediction_object.delay_time - ego_init_relative_time;
    if (prediction_object.trajectory_array.size() == 0) {
      auto obstacle =
          Obstacle(prediction_object.id, prediction_object,
                   prediction_object.is_static, prediction_relative_time);
      add_obstacle(obstacle);
      continue;
    }
    for (int i = 0; i < prediction_object.trajectory_array.size(); ++i) {
      Obstacle obstacle(prediction_object.id, prediction_object,
                        prediction_object.is_static, prediction_relative_time);
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
          prediction_object.type != iflyauto::ObjectType::OBJECT_TYPE_UNKNOWN &&
          prediction_object.length > 0. && prediction_object.width > 0. &&
          std::fabs(prediction_object.relative_position_y) < 10. &&
          std::fabs(prediction_object.relative_position_x) < 100.) {
        auto obstacle = Obstacle(prediction_object.id, prediction_object,
                                 is_static, prediction_relative_time);
        add_gs_care_obstacles(obstacle);
      }
    }
  } else {
    // ground line
    static constexpr int kGroundLineIdOffset = 5000000;
    const std::shared_ptr<GroundLineManager> ground_line_manager =
        session_->environmental_model().get_ground_line_manager();
    const std::vector<GroundLinePoints> &ground_line_points =
        ground_line_manager->GetPoints();
    std::cout << "ground_line_points.size = " << ground_line_points.size()
              << std::endl;
    // LOG_DEBUG("ground_line_points.size = %lu", ground_line_points.size());
    int ground_line_id = kGroundLineIdOffset;
    for (const auto &ground_line_point : ground_line_points) {
      if (ground_line_point.size() >= 3) {
        ground_line_id += 1;
        Obstacle obstacle(ground_line_id, ground_line_point);
        add_groundline_obstacle(obstacle);
      }
    }

    // parking space
    static constexpr int kParkingSlotIdOffset = 6000000;
    const std::shared_ptr<ParkingSlotManager> parking_slot_manager =
        session_->environmental_model().get_parking_slot_manager();
    const std::vector<ParkingSlotPoints> &parking_slot_points =
        parking_slot_manager->GetPoints();
    int parking_slot_id = kParkingSlotIdOffset;
    for (const auto &parking_slot_point : parking_slot_points) {
      if (parking_slot_point.size() != 4U) {
        LOG_DEBUG("invalid parking_slot_point.size = %lu",
                  parking_slot_point.size());
        continue;
      }
      parking_slot_id += 1;
      Obstacle obstacle(parking_slot_id, parking_slot_point);
      add_parking_space(obstacle);
    }

    // occupancy objects
    // static constexpr int kOccupancyObjectIdOffset = 7000000;
    // const std::shared_ptr<OccupancyObjectManager> occupancy_object_manager =
    //     session_->environmental_model().get_occupancy_object_manager();
    // const auto &occupancy_objects_points =
    //     occupancy_object_manager->occupancy_object();
    // int occupancy_object_id = kOccupancyObjectIdOffset;
    // for (auto &object_points : occupancy_objects_points) {
    //   if (object_points.second.size() >= 3) {
    //     // occupancy_object_id += 1;
    //     Obstacle obstacle(occupancy_object_id +, object_points.second,
    //                       object_points.first);
    //     add_occupancy_obstacle(obstacle);
    //   }
    // }

    static constexpr int kOccupancyObjectIdOffset = 7000000;
    const auto &local_view = session_->environmental_model().get_local_view();
    if (local_view.fusion_occupancy_objects_info.local_point_valid) {
      const size_t occupancy_objects_size =
          local_view.fusion_occupancy_objects_info.fusion_object_size;
      const auto occupancy_objects =
          local_view.fusion_occupancy_objects_info.fusion_object;
      for (uint8 i = 0; i < occupancy_objects_size; i++) {
        std::vector<planning_math::Vec2d> object_points;
        size_t polygon_points_size =
            occupancy_objects[i].additional_occupancy_info.polygon_points_size;
        if (polygon_points_size >= 3) {
          auto polygon_points =
              occupancy_objects[i].additional_occupancy_info.polygon_points;
          for (uint j = 0; j < polygon_points_size; j++) {
            object_points.emplace_back(
                planning_math::Vec2d(polygon_points[j].x, polygon_points[j].y));
          }
          Obstacle obstacle(
              kOccupancyObjectIdOffset +
                  occupancy_objects[i].additional_occupancy_info.track_id,
              std::move(object_points),
              occupancy_objects[i].common_occupancy_info.type);
          add_occupancy_obstacle(obstacle);
        }
      }
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
  occupancy_obstacles_ = IndexedList<int, Obstacle>();
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
  auto &frenet_obstacles = reference_path.mutable_obstacles();
  auto &frenet_obstacles_map = reference_path.mutable_obstacles_map();
  auto &obstacles_ids_in_lane_map =
      reference_path.mutable_obstacles_in_lane_map();
  obstacles_ids_in_lane_map.clear();
  frenet_obstacles.clear();
  frenet_obstacles_map.clear();

  if (session_->is_hpp_scene()) {
    frenet_obstacles.reserve(obstacles_.Items().size() +
                             groundline_obstacles_.Items().size());
    obstacles_ids_in_lane_map.reserve(obstacles_.Items().size() +
                                      groundline_obstacles_.Items().size());
    add_frenet_obstacle(obstacles_, reference_path, frenet_obstacles,
                        frenet_obstacles_map, obstacles_ids_in_lane_map);
    add_frenet_obstacle(groundline_obstacles_, reference_path, frenet_obstacles,
                        frenet_obstacles_map, obstacles_ids_in_lane_map);
    add_frenet_obstacle(occupancy_obstacles_, reference_path, frenet_obstacles,
                        frenet_obstacles_map, obstacles_ids_in_lane_map);
  } else {
    frenet_obstacles.reserve(obstacles_.Items().size());
    obstacles_ids_in_lane_map.reserve(obstacles_.Items().size());
    add_frenet_obstacle(obstacles_, reference_path, frenet_obstacles,
                        frenet_obstacles_map, obstacles_ids_in_lane_map);
  }
}

void ObstacleManager::add_frenet_obstacle(
    IndexedList<int, Obstacle> &obstacles, ReferencePath &reference_path,
    std::vector<std::shared_ptr<FrenetObstacle>> &frenet_obstacles,
    std::unordered_map<int, std::shared_ptr<FrenetObstacle>>
        &frenet_obstacles_map,
    std::vector<int> &obstacles_ids_in_lane_map) {
  const auto &frenet_coord = reference_path.get_frenet_coord();
  constexpr double kCareDistance = 30.0;
  auto ego_s = reference_path.get_frenet_ego_state().s();
  auto ego_l = reference_path.get_frenet_ego_state().l();
  auto planning_init_x =
      reference_path.get_frenet_ego_state().planning_init_point().x;
  auto planning_init_y =
      reference_path.get_frenet_ego_state().planning_init_point().y;

  auto is_location_valid = session_->environmental_model().location_valid();

  for (const Obstacle *obstacle_ptr : obstacles.Items()) {
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
      obstacles_ids_in_lane_map.emplace_back(obstacle_ptr->id());
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

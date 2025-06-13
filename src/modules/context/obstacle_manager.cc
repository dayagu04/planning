#include "obstacle_manager.h"

#include <climits>
#include <cmath>
#include <cstddef>
#include <tuple>
#include <vector>

#include "ego_planning_config.h"
#include "ego_state_manager.h"
#include "interface/src/c/common_c.h"
#include "math/math_utils.h"
#include "parking_slot_manager.h"
#include "planning_context.h"
#include "reference_path_manager.h"
#include "virtual_lane_manager.h"
namespace planning {

namespace {
  static constexpr int kEHRColumnIdOffset = 8000000;
  static constexpr int kOccupancyObjectIdOffset = 7000000;
  static constexpr int kParkingSlotIdOffset = 6000000;
  static constexpr int kGroundLineIdOffset = 5000000;

}  // namespace

ObstacleManager::ObstacleManager(const EgoPlanningConfigBuilder *config_builder,
                                 planning::framework::Session *session)
    : session_(session) {
  SetConfig(config_builder);
}

void ObstacleManager::SetConfig(
    const EgoPlanningConfigBuilder *config_builder) {
  config_ = config_builder->cast<EgoPlanningObstacleManagerConfig>();
  ground_line_manager_ptr_ = std::make_shared<GroundLineManager>();
}

void ObstacleManager::update() {
  const double HALF_FOV = 25.0;
  clear();
  const auto &ego_state =
      *session_->mutable_environmental_model()->get_ego_state_manager();
  double ego_init_relative_time = ego_state.planning_init_point().relative_time;
  const auto &prediction_objects =
      session_->environmental_model().get_prediction_info();
  const auto &reference_path = session_->planning_context()
                                   .lane_change_decider_output()
                                   .coarse_planning_info.reference_path;
  const auto &frenet_coord = reference_path->get_frenet_coord();
  // 自车sl
  Point2D ego_point;
  constexpr int kSLowerLimitForOD = -3;
  double kSUpperLimitForOD = config_.supper_limit_for_OD_straight;
  if (reference_path != nullptr) {
    if (frenet_coord != nullptr) {
      if (!frenet_coord->XYToSL(
              Point2D(ego_state.ego_carte().x, ego_state.ego_carte().y),
              ego_point) ||
          std::isnan(ego_point.x) || std::isnan(ego_point.y)) {
        return;
      }
      if (IsOnBend(reference_path, ego_point.x)) {
        kSUpperLimitForOD = config_.supper_limit_for_OD_bend;
      }
    }
  }

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

    // hpp中过滤近处的OD
    if (session_->is_hpp_scene()) {
      if (reference_path != nullptr) {
        if (frenet_coord != nullptr) {
          bool in_range = true;
          Box2d bounding_box(
              {prediction_object.position_x, prediction_object.position_y},
              prediction_object.yaw, prediction_object.length,
              prediction_object.width);
          std::vector<planning_math::Vec2d> polygon_points;
          bounding_box.GetAllCorners(&polygon_points);
          for (auto &point : polygon_points) {
            Point2D frenet_point;
            if (!frenet_coord->XYToSL(point.x(), point.y(),
                                      &frenet_point.x, &frenet_point.y) ||
                std::isnan(frenet_point.x) || std::isnan(frenet_point.y) ||
                ((frenet_point.x > ego_point.x + kSLowerLimitForOD) &&
                 (frenet_point.x < ego_point.x + kSUpperLimitForOD))) {
              in_range = false;
              break;
            }
          }
          if (!in_range) {
            continue;
          }
        }
      }
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
    double time_start = IflyTime::Now_ms();
    UpdateGroundLineObstacle();
    double time_end = IflyTime::Now_ms();
    LOG_DEBUG("UpdateGroundLineObstacle cost:%f\n",
              time_end - time_start);
    JSON_DEBUG_VALUE("UpdateGroundLineObstacleCost", time_end - time_start);

    // parking space
    time_start = IflyTime::Now_ms();
    if (config_.enable_fusion_parking_slot) {  // fusion parking slot
      UpdateParkingSpaceObstacle();
    } else {  // ehr parking space
    }
    time_end = IflyTime::Now_ms();
    LOG_DEBUG("UpdateParkingSpaceObstacle cost:%f\n",
              time_end - time_start);
    JSON_DEBUG_VALUE("UpdateParkingSpaceObstacleCost", time_end - time_start);

    // occupancy objects
    time_start = IflyTime::Now_ms();
    if (config_.enable_fusion_occupancy_objects) {
      UpdateOccObstacle();
    }
    time_end = IflyTime::Now_ms();
    LOG_DEBUG("UpdateOccObstacle cost:%f\n", time_end - time_start);
    JSON_DEBUG_VALUE(" UpdateOccObstacleCost", time_end - time_start);

    // ehr column box
    time_start = IflyTime::Now_ms();
    if (config_.enable_ehr_column_box) {
      UpdateMapStaticObstacle();
    }
    time_end = IflyTime::Now_ms();
    LOG_DEBUG("UpdateMapStaticObstacle cost:%f\n", time_end - time_start);
    JSON_DEBUG_VALUE(" UpdateMapStaticObstacleCost", time_end - time_start);

    // update uss
    uss_obstacle_.SetLocalView(
        &session_->environmental_model().get_local_view());
    uss_obstacle_.Update();

    // ehr column box

    // look path in ogm
    // const auto &motion_planner_output =
    //     session_->planning_context().motion_planner_output();
    // double final_t = 5.0;
    // double tmp_t = 0.0;
    // std::vector<planning_math::Vec2d> last_lat_path;
    // for (size_t i = 0; i < motion_planner_output.s_lat_vec.size(); ++i) {
    //   tmp_t = std::fmin(0.1 + i * 0.2, final_t);
    //   double last_lat_path_x = motion_planner_output.lateral_x_t_spline(tmp_t);
    //   double last_lat_path_y = motion_planner_output.lateral_y_t_spline(tmp_t);
    //   last_lat_path.emplace_back(planning_math::Vec2d(last_lat_path_x, last_lat_path_y));
    // }
  }
}

bool ObstacleManager::IsOnBend(
    const std::shared_ptr<ReferencePath> &reference_path, double ego_s) {
  constexpr double kBendRoadRadius = 30;
  ReferencePathPoint temp_ref_path_point;
  std::array<int8_t, 3> s_range{11, 4, -1};
  for (auto s : s_range) {
    if (reference_path->get_reference_point_by_lon(ego_s + s,
                                                   temp_ref_path_point)) {
      if (std::abs(temp_ref_path_point.path_point.kappa()) >
          1 / kBendRoadRadius) {
        return true;
      }
    }
  }
  return false;
}

void ObstacleManager::UpdateParkingSpaceObstacle() {
  const double kMaxDistanceY = 5;
  const double kMaxDistanceFrontX = 40;  // 后续根据实际需求更改
  const double kMaxDistanceBackX = 30;
  const auto &local_view = session_->environmental_model().get_local_view();
  const auto &ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  Pose2D base_pose(ego_state_manager->ego_pose().x,
                   ego_state_manager->ego_pose().y,
                   ego_state_manager->ego_pose().theta);
  Transform2d ego_base;
  ego_base.SetBasePose(base_pose);
  const size_t parking_slot_lists_size =
      local_view.parking_fusion_info.parking_fusion_slot_lists_size;
  const auto &parking_slot_lists =
      local_view.parking_fusion_info.parking_fusion_slot_lists;
  // const auto &enu2car_matrix = ego_state_manager->get_enu2car();
  // Eigen::Vector3d v;
  double min_x, min_y, max_x, max_y;
  for (uint8 i = 0; i < parking_slot_lists_size; i++) {
    std::vector<planning_math::Vec2d> slot_point;
    const auto &parking_slot = parking_slot_lists[i];
    const size_t slot_id = parking_slot.id;
    // TBD: 这里没有判断size, c结构体没有设置size，默认4
    min_x = std::numeric_limits<double>::max();
    min_y = std::numeric_limits<double>::max();
    max_x = std::numeric_limits<double>::lowest();
    max_y = std::numeric_limits<double>::lowest();
    for (const auto &corner_point : parking_slot.corner_points) {
      // v.x() = corner_point.x;
      // v.y() = corner_point.y;
      // v.z() = corner_point.z;
      // auto park_space_point_car = enu2car_matrix(v);
      // min_x = std::fmin(min_x, park_space_point_car.x());
      // min_y = std::fmin(min_y, park_space_point_car.y());
      // max_x = std::fmax(max_x, park_space_point_car.x());
      // max_y = std::fmax(max_y, park_space_point_car.y());
      Pose2D local;
      ego_base.GlobalPointToULFLocal(
          &local,
          Pose2D(corner_point.x, corner_point.y, 0));
      min_x = std::fmin(min_x, local.x);
      min_y = std::fmin(min_y, local.y);
      max_x = std::fmax(max_x, local.x);
      max_y = std::fmax(max_y, local.y);
      slot_point.emplace_back(
          planning_math::Vec2d(corner_point.x, corner_point.y));
    }
    if (((min_y > 0 && min_y < kMaxDistanceY) ||
         (max_y < 0 && max_y > -kMaxDistanceY) || (min_y <= 0 && max_y >= 0)) &&
        min_x < kMaxDistanceFrontX && max_x > -kMaxDistanceBackX) {
      Obstacle obstacle(kParkingSlotIdOffset + slot_id,
                    std::move(slot_point));
      add_parking_space(obstacle);
    }
  }
}

void ObstacleManager::UpdateOccObstacle() {
  int index_offset = 900000;
  const auto &local_view = session_->environmental_model().get_local_view();
  const auto &ego_state =
      *session_->mutable_environmental_model()->get_ego_state_manager();
  const auto &ref_path_ptr = session_->planning_context()
                                 .lane_change_decider_output()
                                 .coarse_planning_info.reference_path;
  // 自车sl
  if (ref_path_ptr == nullptr) {
    return;
  }
  const auto &frenet_coord = ref_path_ptr->get_frenet_coord();
  if (frenet_coord == nullptr) {
    return;
  }
  Point2D ego_point;
  if (!frenet_coord->XYToSL(
          Point2D(ego_state.ego_carte().x, ego_state.ego_carte().y),
          ego_point) ||
      std::isnan(ego_point.x) || std::isnan(ego_point.y)) {
    return;
  }

  if (local_view.fusion_occupancy_objects_info.local_point_valid) {
    const size_t occupancy_objects_size =
        local_view.fusion_occupancy_objects_info.fusion_object_size;
    const auto occupancy_objects =
        local_view.fusion_occupancy_objects_info.fusion_object;
    for (uint8 i = 0; i < occupancy_objects_size; i++) {
      size_t polygon_points_size =
          occupancy_objects[i].additional_occupancy_info.polygon_points_size;
      auto polygon_points =
          occupancy_objects[i].additional_occupancy_info.polygon_points;
      // 防止出现直角墙构成三角形区域
      if (polygon_points_size > 100 &&
          (occupancy_objects[i].common_occupancy_info.type ==
              iflyauto::OBJECT_TYPE_OCC_WALL ||
           occupancy_objects[i].common_occupancy_info.type ==
              iflyauto::OBJECT_TYPE_OCC_EMPTY)) {
        ProcessOccupancyWall(occupancy_objects[i], polygon_points,
                             polygon_points_size, frenet_coord,
                             ego_point, index_offset);

      } else {
        ProcessOccupancyObject(occupancy_objects[i], polygon_points,
                               polygon_points_size, frenet_coord, ego_point);
      }
    }
  }
}

void ObstacleManager::ProcessOccupancyWall(
    const iflyauto::FusionOccupancyObject &object,
    const iflyauto::Point2f *polygon_points, size_t polygon_size,
    const std::shared_ptr<planning_math::KDPath> &frenet_coord,
    const Point2D &ego_point, int &index_offset) {
  std::vector<std::vector<planning_math::Vec2d>> points_vec;
  split_points(polygon_points, polygon_size, frenet_coord, points_vec);

  for (auto &object_points : points_vec) {
    if (object_points.size() < 5) {
      continue;
    }
    Obstacle obstacle(kOccupancyObjectIdOffset + index_offset,
                      std::move(object_points), iflyauto::OBJECT_TYPE_OCC_WALL);
    if (obstacle.is_vaild() &&
        FilterObstacleByDistance(obstacle, frenet_coord, ego_point)) {
      add_occupancy_obstacle(obstacle);
    }
    ++index_offset;
  }
}

void ObstacleManager::ProcessOccupancyObject(
    const iflyauto::FusionOccupancyObject &object,
    const iflyauto::Point2f *polygon_points, size_t polygon_size,
    const std::shared_ptr<planning_math::KDPath> &frenet_coord,
    const Point2D &ego_point) {
  std::vector<planning_math::Vec2d> object_points;
  object_points.reserve(polygon_size);
  for (size_t j = 0; j < polygon_size; ++j) {
    object_points.emplace_back(polygon_points[j].x, polygon_points[j].y);
  }
  if (object_points.size() < 5) {
    return;
  }
  Obstacle obstacle(
      kOccupancyObjectIdOffset + object.additional_occupancy_info.track_id,
      std::move(object_points), object.common_occupancy_info.type);
  if (obstacle.is_vaild() &&
      FilterObstacleByDistance(obstacle, frenet_coord, ego_point)) {
    add_occupancy_obstacle(obstacle);
  }
}

bool ObstacleManager::FilterObstacleByDistance(
    const Obstacle &obstacle,
    const std::shared_ptr<planning_math::KDPath> &frenet_coord,
    const Point2D &ego_point) {
  const double kMaxFrontDistance = 8;
  const double kMinPolygonArea = 0.5;
  const double kMinLength = 0.7;
  const double kMinWidth = 0.7;
  int in_range_count = 0;
  for (const auto &point : obstacle.perception_polygon().points()) {
    Point2D sl_point;
    if (!frenet_coord->XYToSL({point.x(), point.y()}, sl_point) ||
        std::isnan(sl_point.x) || std::isnan(sl_point.y) ||
        sl_point.x > ego_point.x + kMaxFrontDistance) {
      ++in_range_count;
    }
  }

  return !(in_range_count == obstacle.perception_polygon().points().size() ||
           (in_range_count > 0 && obstacle.perception_polygon().area() < kMinPolygonArea &&
            obstacle.perception_bounding_box().length() < kMinLength &&
            obstacle.perception_bounding_box().width() < kMinWidth));
}

void ObstacleManager::split_points(
    const iflyauto::Point2f *points, const double polygon_points_size,
    const std::shared_ptr<planning_math::KDPath> &frenet_coord,
    std::vector<std::vector<planning_math::Vec2d>> &result) {
  constexpr double LATTHRESHOLD = 1;
  constexpr double LONTHRESHOLD = 1;
  constexpr double LATTHRESHOLD2 = 0.5;
  constexpr double LONTHRESHOLD2 = 3;
  constexpr double LONTHRESHOLD3 = 5;
  constexpr double MINSPLITLONTHRESHOLD = 3;
  constexpr double THRESHOLD = 2;
  constexpr double MAXDISTANCETHRESHOLD = 4;
  std::vector<planning_math::Vec2d> current_segment;
  if (frenet_coord != nullptr) {
    std::vector<std::pair<std::pair<double, double>, planning_math::Vec2d>> points_vec;
    points_vec.reserve(polygon_points_size);
    for (size_t i = 0; i < polygon_points_size; ++i) {
      double point_s, point_l;
      if(frenet_coord->XYToSL(points[i].x, points[i].y, &point_s, &point_l)) {
        std::pair<double, double> point_sl(point_s, point_l);
        points_vec.emplace_back(
            std::pair<std::pair<double, double>, planning_math::Vec2d>(
                point_sl, planning_math::Vec2d(points[i].x, points[i].y)));
      }
    }
    auto compare_s = [&](std::pair<std::pair<double, double>, planning_math::Vec2d> p1,
                                   std::pair<std::pair<double, double>, planning_math::Vec2d> p2) {
      return p1.first.first < p2.first.first;
    };
    std::sort(points_vec.begin(), points_vec.end(),
              compare_s);
    size_t init_index = 0;
    double max_diff_s = 0;
    double max_diff_l = 0;
    current_segment.push_back(points_vec.front().second);
    for (size_t i = 1; i < points_vec.size(); ++i) {
      double ds = points_vec[i].first.first - points_vec[i - 1].first.first;
      double dl = fabs(points_vec[i].first.second - points_vec[i - 1].first.second);
      max_diff_s = std::max(fabs(points_vec[i].first.first - points_vec[init_index].first.first), max_diff_s);
      max_diff_l = std::max(fabs(points_vec[i].first.second - points_vec[init_index].first.second), max_diff_l);
      if (points_vec.back().first.first - points_vec.front().first.first > MINSPLITLONTHRESHOLD) {
        if (ds > LONTHRESHOLD ||
            (max_diff_l > LATTHRESHOLD && max_diff_s > LONTHRESHOLD) ||
            (max_diff_l > LATTHRESHOLD2 && max_diff_s > LONTHRESHOLD2) ||
            (max_diff_s > LONTHRESHOLD3)) {
          // 当前点与前一个点相差超过阈值，开启新分割
          init_index = i;
          max_diff_s = 0;
          max_diff_l = 0;
          result.push_back(current_segment);
          current_segment.clear();
        }
      }
      current_segment.push_back(points_vec[i].second);
    }
  } else {
    double sum_distance = 0;
    current_segment.push_back(planning_math::Vec2d(points[0].x, points[0].y));
    for (size_t i = 1; i < polygon_points_size; ++i) {
      double dx = fabs(points[i].x - points[i - 1].x);
      double dy = fabs(points[i].y - points[i - 1].y);
      double ds = std::hypot(dx, dy);
      sum_distance += ds;
      if (dx > THRESHOLD || dy > THRESHOLD || sum_distance > MAXDISTANCETHRESHOLD) {
        // 当前点与前一个点相差超过阈值，开启新分割
        sum_distance = 0;
        result.push_back(current_segment);
        current_segment.clear();
      }
      current_segment.push_back(planning_math::Vec2d(points[i].x, points[i].y));
    }
  }

  if (!current_segment.empty() && current_segment.size() > 2) {
    result.push_back(current_segment);
  }
}

void ObstacleManager::UpdateGroundLineObstacle() {
  const double kMinFrontDistance = 6;
  const double kMinFrontDistanceForMap = 7;
  const double kMaxSideDistance = 5;
  int index_offset = 900000;
  const auto &local_view = session_->environmental_model().get_local_view();
  if (local_view.ground_line_perception.groundline_size > 0) {
    // 是否自行聚类成多个小矩形
    if (config_.is_ground_line_cluster) {
      ground_line_manager_ptr_->Update(local_view.ground_line_perception);
      const std::vector<GroundLinePoints> &ground_line_points =
          ground_line_manager_ptr_->GetPoints();
      std::cout << "ground_line_points.size = " << ground_line_points.size()
                << std::endl;
      for (const auto &ground_line_point : ground_line_points) {
        index_offset += 1;
        if (ground_line_point.size() >= 3) {
          Obstacle obstacle(kGroundLineIdOffset + index_offset, ground_line_point);
          if (obstacle.is_vaild()) {
            add_groundline_obstacle(obstacle);
          }
        }
      }
    } else {
      const auto &ref_path_ptr = session_->planning_context()
                                .lane_change_decider_output()
                                .coarse_planning_info.reference_path;
      if (ref_path_ptr == nullptr) {
        return;
      }
      const auto &frenet_coord = ref_path_ptr->get_frenet_coord();
      if (frenet_coord == nullptr) {
        return;
      }
      const auto &ego_state =
          session_->environmental_model().get_ego_state_manager();
      // 自车sl
      Point2D ego_point;
      if (!frenet_coord->XYToSL(Point2D(ego_state->ego_carte().x,
                                        ego_state->ego_carte().y),
                                ego_point) ||
          std::isnan(ego_point.x) || std::isnan(ego_point.y)) {
        return;
      }

      const size_t groundline_size = local_view.ground_line_perception.groundline_size;
      for (size_t i = 0; i < groundline_size; ++i) {
        const auto &groundline = local_view.ground_line_perception.groundline[i];
        const size_t groundline_point_size = groundline.groundline_point_size;
        if (groundline_point_size > 50 &&
            groundline.type == iflyauto::GROUND_LINE_TYPE_WALL) {
          std::vector<std::vector<planning_math::Vec2d>> points_vec;
          split_points(groundline.groundline_point,
                       groundline_point_size,
                       frenet_coord,
                       points_vec);
          for (auto &object_points : points_vec) {
            if (object_points.size() < 3) {
              continue;
            }
            if (FilterGroundLineByDistance(object_points,
                                           frenet_coord,
                                           ego_point,
                                           groundline.type,
                                           groundline.resource_type)) {
              continue;
            }
            Obstacle obstacle(kGroundLineIdOffset + index_offset,
                              std::move(object_points));
            if (obstacle.is_vaild()) {
              add_groundline_obstacle(obstacle);
              ++index_offset;
            }
          }
          continue;
        }
        bool in_range = true;
        bool is_lat_valid = false;
        double min_dist_to_ref = NL_NMAX;
        std::vector<planning_math::Vec2d> points;
        points.reserve(groundline_point_size);
        for (size_t j = 0; j < groundline_point_size; ++j) {
          // filter
          if ((groundline.groundline_point[j].x == 0) && (groundline.groundline_point[j].y == 0)) {
            continue;
          }
          Point2D sl_point;
          if (!frenet_coord->XYToSL(
                  Point2D(groundline.groundline_point[j].x, groundline.groundline_point[j].y),
                  sl_point) ||
              std::isnan(sl_point.x) || std::isnan(sl_point.y) ||
              groundline.type == iflyauto::GROUND_LINE_TYPE_COLUMN ||
              sl_point.x < ego_point.x + kMinFrontDistance ||
              ((sl_point.x < ego_point.x + kMinFrontDistanceForMap) &&
                groundline.resource_type ==
                    iflyauto::StaticFusionResourceType::RESOURCE_TYPE_MAP)) {
            in_range = false;
            break;
          } else {
            min_dist_to_ref = std::min(std::fabs(sl_point.y), min_dist_to_ref);
            is_lat_valid = true;
          }
          points.emplace_back(planning_math::Vec2d(groundline.groundline_point[j].x,
                                                  groundline.groundline_point[j].y));
        }
        if (!in_range) {
          continue;
        } else if (is_lat_valid && min_dist_to_ref > kMaxSideDistance) {
          continue;
        }
        if (points.size() >= 3) {
          Obstacle obstacle(groundline.id + kGroundLineIdOffset, std::move(points));
          if (obstacle.is_vaild()) {
            add_groundline_obstacle(obstacle);
          }
        }
      }
    }
  }
}

bool ObstacleManager::FilterGroundLineByDistance(
    const std::vector<planning_math::Vec2d> &points,
    const std::shared_ptr<planning_math::KDPath> &frenet_coord,
    const Point2D &ego_point,
    const iflyauto::GroundLineType type,
    const iflyauto::StaticFusionResourceType resource_type) {
  const double kMinFrontDistance = 6;
  const double kMinFrontDistanceForMap = 7;
  const double kMaxSideDistance = 5;
  bool in_range = true;
  bool is_lat_valid = false;
  double min_dist_to_ref = NL_NMAX;
  for (auto& point : points) {
    Point2D sl_point;
    if (!frenet_coord->XYToSL(
            Point2D(point.x(), point.y()),
            sl_point) ||
        std::isnan(sl_point.x) || std::isnan(sl_point.y) ||
        type == iflyauto::GROUND_LINE_TYPE_COLUMN ||
        sl_point.x < ego_point.x + kMinFrontDistance ||
        ((sl_point.x < ego_point.x + kMinFrontDistanceForMap) &&
          resource_type ==
              iflyauto::StaticFusionResourceType::RESOURCE_TYPE_MAP)) {
      in_range = false;
      break;
    } else {
      min_dist_to_ref = std::min(std::fabs(sl_point.y), min_dist_to_ref);
      is_lat_valid = true;
    }
  }
  return ((!in_range) || (is_lat_valid && min_dist_to_ref > kMaxSideDistance));
}

void ObstacleManager::UpdateMapStaticObstacle() {
  // todo select
  // ehr column box
  const double kMinFrontDistance = 11;
  const double kMaxFrontDistance = 50;
  const double kMaxSideDistance = 5;
  const auto &local_view = session_->environmental_model().get_local_view();
  const size_t polygon_obstacles_size = local_view.static_map_info.parking_assist_info().polygon_obstacles_size();
  bool has_target_lane = session_->planning_context()
                                 .lane_change_decider_output()
                                 .has_target_lane;
  const auto &ref_path_ptr = session_->planning_context()
                                     .lane_change_decider_output()
                                     .coarse_planning_info.reference_path;
  const auto &ego_state =
      session_->environmental_model().get_ego_state_manager();
  if (polygon_obstacles_size > 0) {
    const auto& polygon_obstacles = local_view.static_map_info.parking_assist_info().polygon_obstacles();
    int ehr_column_id = kEHRColumnIdOffset;
    for (const Map::PolygonObject &polygon_object : polygon_obstacles) {
      ehr_column_id += 1;
      if (polygon_object.shape_size() != 4U) {
        LOG_DEBUG("invalid polygon_object.shape_size = %d",
                  polygon_object.shape_size());
        continue;
      }
      bool in_range = true;
      bool is_lat_valid = false;
      double min_dist_to_ref = NL_NMAX;
      std::vector<planning::planning_math::Vec2d> column_box;
      column_box.reserve(polygon_object.shape_size());
      for (const auto& shape : polygon_object.shape()) {
        // filter
        if (has_target_lane) {
          if (ref_path_ptr != nullptr) {
            const auto &frenet_coord = ref_path_ptr->get_frenet_coord();
            if (frenet_coord != nullptr) {
              // 自车sl
              Point2D ego_point;
              if (!frenet_coord->XYToSL(
                      Point2D(ego_state->ego_carte().x, ego_state->ego_carte().y),
                      ego_point) ||
                  std::isnan(ego_point.x) || std::isnan(ego_point.y)) {
                continue;
              }
              Point2D sl_point;
              if (!frenet_coord->XYToSL(
                      Point2D(shape.x(), shape.y()),
                      sl_point) ||
                  std::isnan(sl_point.x) || std::isnan(sl_point.y) ||
                  sl_point.x < ego_point.x + kMinFrontDistance ||
                  sl_point.x > ego_point.x + kMaxFrontDistance) {
                in_range = false;
                break;
              } else {
                min_dist_to_ref = std::min(std::fabs(sl_point.y), min_dist_to_ref);
                is_lat_valid = true;
              }
            }
          }
        }
        column_box.emplace_back(planning::planning_math::Vec2d(shape.x(), shape.y()));
      }
      if (!in_range) {
        continue;
      } else if (is_lat_valid && min_dist_to_ref > kMaxSideDistance) {
        continue;
      }
      if (column_box.size() >= 3) {
        Obstacle obstacle(ehr_column_id, column_box);
        add_map_static_obstacle(obstacle);
      }
    }
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
                             groundline_obstacles_.Items().size() +
                             occupancy_obstacles_.Items().size() +
                             map_static_obstacles_.Items().size());
    obstacles_ids_in_lane_map.reserve(obstacles_.Items().size() +
                                      groundline_obstacles_.Items().size() +
                                      occupancy_obstacles_.Items().size() +
                                      map_static_obstacles_.Items().size());
    add_frenet_obstacle(obstacles_, reference_path, frenet_obstacles,
                        frenet_obstacles_map, obstacles_ids_in_lane_map);
    add_frenet_obstacle(groundline_obstacles_, reference_path, frenet_obstacles,
                        frenet_obstacles_map, obstacles_ids_in_lane_map);
    add_frenet_obstacle(occupancy_obstacles_, reference_path, frenet_obstacles,
                        frenet_obstacles_map, obstacles_ids_in_lane_map);
    add_frenet_obstacle(map_static_obstacles_, reference_path, frenet_obstacles,
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
//         std::piecewise_construct,
//         std::forward_as_tuple(obstacle_ptr->id()),
//         std::forward_as_tuple(std::make_shared<FrenetObstacle>(
//             obstacle_ptr, reference_path,
//             session_->environmental_model().get_ego_state_manager())));
//   }
//   return frenet_ob_map;
// }

}  // namespace planning

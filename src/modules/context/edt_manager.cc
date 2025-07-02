#include "edt_manager.h"

#include <cstddef>

#include "obstacle.h"
#include "planning_context.h"
#include "pose2d.h"

namespace planning {

EdtManager::EdtManager(const EgoPlanningConfigBuilder *config_builder,
                       planning::framework::Session *session)
    : session_(session) {
  SetConfig(config_builder);
  InitEDT();
}

void EdtManager::SetConfig(const EgoPlanningConfigBuilder *config_builder) {
  config_ = config_builder->cast<EgoPlanningEdtManagerConfig>();
}

void EdtManager::InitEDT() {
  is_edt_valid_ = false;
  resolution_ = 0.05;  // 5cm
  ogm_.Clear();
  ogm_.Init();  // is null
  edt_.Init(config_.car_body_lat_safe_buffer, config_.lon_safe_buffer,
            config_.mirror_buffer);
  // edt_.UpdateSafeBuffer(car_body_lat_safe_buffer, lon_safe_buffer,
  // mirror_buffer);
}

OccupancyGridBound EdtManager::GenerateOGM(const Pose2D &base_pose) {
  ogm_.Clear();
  Transform2d ego_base;
  ego_base.SetBasePose(base_pose);
  // base grid bound
  const auto &motion_planner_output =
      session_->planning_context().motion_planner_output();
  double final_t = 5.0;
  double tmp_t = 0.0;
  double front_x = 0.0;
  double back_x = 0.0;
  double left_y = 0.0;
  double right_y = 0.0;
  double last_end_x = 0.0;
  double last_end_y = 0.0;
  for (size_t i = 0; i < motion_planner_output.s_lat_vec.size(); ++i) {
    tmp_t = std::fmin(0.1 + i * 0.2, final_t);
    double last_lat_path_x = motion_planner_output.lateral_x_t_spline(tmp_t);
    double last_lat_path_y = motion_planner_output.lateral_y_t_spline(tmp_t);
    Pose2D local;
    ego_base.GlobalPointToULFLocal(&local,
                                   Pose2D(last_lat_path_x, last_lat_path_y, 0));
    last_end_x = local.x;
    last_end_y = local.y;
    front_x = std::max(front_x, local.x);
    back_x = std::min(back_x, local.x);
    left_y = std::max(left_y, local.y);
    right_y = std::min(right_y, local.y);
  }
  //
  if ((last_end_x < front_x) && (last_end_x > back_x)) {
    // front_x += 3.0;
    back_x -= 5.0;
  }
  // if (last_end_x == front_x) {
  //   front_x += 3.0;
  // }
  if (last_end_x == back_x) {
    back_x -= 5.0;
  }
  // if ((last_end_y < left_y) && (last_end_y > right_y)) {
  //   left_y += 3.0;
  //   right_y -= 3.0;
  // }
  // if (last_end_y == left_y) {
  //   left_y += 3.0;
  // }
  // if (last_end_y == right_y) {
  //   right_y -= 3.0;
  // }
  //
  if (front_x >= 0.0) {
    front_x += 7.0;
  }
  if (back_x <= 0.0) {
    back_x -= 2.0;
  }
  if (left_y >= 0.0) {
    left_y += 7.0;
  }
  if (right_y <= 0.0) {
    right_y -= 7.0;
  }

  // 根据最短搜索范围，重新计算边界
  // 规划起始点
  const PlanningInitPoint &planning_init_point =
      session_->environmental_model()
          .get_ego_state_manager()
          ->planning_init_point();
  if (session_->is_hpp_scene()) {
    const auto &reference_path = session_->planning_context()
                                     .lane_change_decider_output()
                                     .coarse_planning_info.reference_path;
    if (reference_path != nullptr) {
      const auto &frenet_coord = reference_path->get_frenet_coord();
      if (frenet_coord != nullptr) {
        Point2D end_point;
        if (frenet_coord->SLToXY(
                Point2D(reference_path->get_frenet_ego_state().s() + config_.hpp_min_search_range + 7, 0),
                end_point) &&
            !std::isnan(end_point.x) && !std::isnan(end_point.y)) {
          Pose2D local;
          ego_base.GlobalPointToULFLocal(&local,
                                          Pose2D(end_point.x, end_point.y, 0));
          front_x = std::max(front_x, local.x);
          back_x = std::min(back_x, local.x);
          left_y = std::max(left_y, local.y);
          right_y = std::min(right_y, local.y);
        }
      }
    }
  }
  if (std::fabs(front_x - back_x) > 40) {
    front_x = planning_init_point.lat_init_state.x() + 20;
    back_x = planning_init_point.lat_init_state.x() - 20;
  }
  if (std::fabs(left_y - right_y) > 40) {
    left_y = planning_init_point.lat_init_state.y() + 20;
    right_y = planning_init_point.lat_init_state.y() - 20;
  }
  OccupancyGridBound grid_bound(back_x, right_y, front_x, left_y);
  // reload grid bound
  ogm_.Process(grid_bound, resolution_);
  return grid_bound;
}

void EdtManager::AddPointClouds(
    const std::vector<planning_math::Vec2d> &point_clouds,
    size_t step) {  // can be sparse
  const auto &ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  // method 1:  // time-cost more
  // const auto &enu2car_matrix = ego_state_manager->get_enu2car();

  // method 2:  // time-cost less
  Transform2d ego_base;
  ego_base.SetBasePose(Pose2D(ego_state_manager->ego_pose().x,
                              ego_state_manager->ego_pose().y,
                              ego_state_manager->ego_pose().theta));
  for (size_t i = 0; i < point_clouds.size(); i += step) {
    // method 1:
    // Eigen::Vector3d v;
    // v.x() = point_clouds[i].x();
    // v.y() = point_clouds[i].y();
    // v.z() = 0;
    // const auto& polygon_point_car = enu2car_matrix(v);
    // ogm_.AddSlotCoordinatePoint(Position2D(polygon_point_car.x(),
    // polygon_point_car.y()));

    // method 2:
    Pose2D local;
    ego_base.GlobalPointToULFLocal(
        &local, Pose2D(point_clouds[i].x(), point_clouds[i].y(), 0));
    ogm_.AddSlotCoordinatePoint(Position2D(local.x, local.y));
  }
}

void EdtManager::AddODPoint(const Obstacle &obstalce) {
  std::vector<planning_math::Vec2d> polygon_points;
  obstalce.perception_bounding_box().GetAllCorners(&polygon_points);
  AddPointClouds(polygon_points);
  std::vector<planning_math::Vec2d> extracted_polygon_points;
  if (polygon_points.size() > 1) {
    for (size_t i = 0; i < polygon_points.size() - 1; ++i) {
      double mid_x = 0.5 * (polygon_points[i].x() + polygon_points[i + 1].x());
      double mid_y = 0.5 * (polygon_points[i].y() + polygon_points[i + 1].y());
      extracted_polygon_points.emplace_back(planning_math::Vec2d(mid_x, mid_y));
    }
    if (polygon_points.size() > 2) {
      double mid_x =
          0.5 * (polygon_points.front().x() + polygon_points.back().x());
      double mid_y =
          0.5 * (polygon_points.front().y() + polygon_points.back().y());
      extracted_polygon_points.emplace_back(planning_math::Vec2d(mid_x, mid_y));
    }
    AddPointClouds(extracted_polygon_points);
  }
}

bool EdtManager::UpdateEDT(const OccupancyGridBound &grid_bound) {
  // edt handle
  edt_.Excute(ogm_, grid_bound, resolution_);
  return true;
}

bool EdtManager::FilterObstacleForAra(
    const planning::FrenetObstacle &frenet_obstacle) {
  constexpr double kLVelocity = 0.4;
  constexpr double kVruLThreshold = 1.1;
  double min_l = frenet_obstacle.frenet_polygon_sequence()[0].second.min_y();
  double max_l = frenet_obstacle.frenet_polygon_sequence()[0].second.max_y();
  if (frenet_obstacle.is_static() ||
      (frenet_obstacle.obstacle()->is_VRU() &&
       std::fabs(frenet_obstacle.frenet_velocity_l()) < kLVelocity) ||
      (frenet_obstacle.obstacle()->is_car() &&
       (std::fabs(frenet_obstacle.frenet_velocity_l()) < kLVelocity &&
        frenet_obstacle.frenet_velocity_s() < 0.25))) {
    if ((frenet_obstacle.obstacle()->is_VRU() ||
         frenet_obstacle.obstacle()->type() ==
             iflyauto::OBJECT_TYPE_OCC_PEOPLE) &&
        (max_l < kVruLThreshold && min_l > -kVruLThreshold)) {
      return false;
    }
    return true;
  }
  return false;
}

void EdtManager::update() {
  if (!session_->is_hpp_scene() ||
      !session_->planning_context().last_planning_success()) {
    return;
  }
  is_edt_valid_ = false;
  const auto &ego_state =
      *session_->mutable_environmental_model()->get_ego_state_manager();
  OccupancyGridBound grid_bound =
      GenerateOGM(Pose2D(ego_state.ego_pose().x, ego_state.ego_pose().y,
                         ego_state.ego_pose().theta));

  const auto &reference_path_ptr = session_->environmental_model()
                                       .get_reference_path_manager()
                                       ->get_reference_path_by_current_lane();
  if (reference_path_ptr != nullptr) {
    for (auto &frenet_obstacle : reference_path_ptr->get_obstacles()) {
      if (!frenet_obstacle->b_frenet_valid() ||
          frenet_obstacle->b_frenet_polygon_sequence_invalid()) {
        continue;
      }
      if (frenet_obstacle->source_type() == SourceType::OD) {
        if (FilterObstacleForAra(*frenet_obstacle)) {
          AddODPoint(*frenet_obstacle->obstacle());
        }
      } else if (frenet_obstacle->source_type() == SourceType::OCC) {
        if (FilterObstacleForAra(*frenet_obstacle)) {
          AddPointClouds(frenet_obstacle->obstacle()->perception_points());
        }
      } else if (frenet_obstacle->source_type() == SourceType::GroundLine) {
        AddPointClouds(frenet_obstacle->obstacle()->perception_points());
      } else if (frenet_obstacle->source_type() == SourceType::MAP) {
        AddPointClouds(frenet_obstacle->obstacle()->perception_points());
      }
    }
  }
  double time_start = IflyTime::Now_ms();
  is_edt_valid_ = UpdateEDT(grid_bound);
  double time_end = IflyTime::Now_ms();
  LOG_DEBUG("EulerDistanceTransform cost:%f\n", time_end - time_start);
  JSON_DEBUG_VALUE("UpdateEulerDistanceTransformCost", time_end - time_start);
}
}  // namespace planning

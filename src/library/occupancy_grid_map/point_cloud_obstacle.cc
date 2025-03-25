
#include "point_cloud_obstacle.h"

#include "./../convex_collision_detection/gjk2d_interface.h"
#include "aabb2d.h"
#include "log_glog.h"
#include "modules/apa_function/apa_world/apa_obstacle.h"
#include "polygon_base.h"
#include "pose2d.h"
#include "src/modules/apa_function/apa_param_config.h"

namespace planning {

#define DEBUG_POINT_CLOUD_OBS (0)

void PointCloudObstacleTransform::GenerateLocalObstacleByLocalView(
    ParkObstacleList& obs_list, const LocalView* local_view,
    const float slot_length, const float slot_width,
    const Pose2D& slot_base_pose, const Pose2D& ego_start,
    const bool enable_limiter_obs) {
  Transform2d slot_tf;
  slot_tf.SetBasePose(slot_base_pose);

  // hack: delete obstacle around ego and slot. In the future, it will be
  // retired.
  const apa_planner::ApaParameters& config = apa_param.GetParam();
  Polygon2D ego_local_polygon;
  Polygon2D ego_global_polygon;
  float veh_x_buffer = 0.3;
  float veh_y_buffer = 0.11;

  GenerateUpLeftFrameBox(
      &ego_local_polygon,
      -static_cast<float>(config.rear_overhanging) - veh_x_buffer,
      -static_cast<float>(config.max_car_width / 2) - veh_y_buffer,
      static_cast<float>(config.car_length) -
          static_cast<float>(config.rear_overhanging) + veh_x_buffer,
      static_cast<float>(config.max_car_width / 2) + veh_y_buffer);
  ULFLocalPolygonToGlobal(&ego_global_polygon, &ego_local_polygon, ego_start);

  // generate local obs
  if (local_view == nullptr) {
    ILOG_ERROR << "local view is null";

    return;
  }

  size_t number =
      static_cast<size_t>(
          local_view->fusion_occupancy_objects_info.fusion_object_size) +
      static_cast<size_t>(local_view->ground_line_perception.groundline_size);

  obs_list.point_cloud_list.resize(number + 1);

  // slot aabb
  cdl::AABB slot_box;

  float slot_x_buffer = 1.0;
  float slot_y_buffer = 0.05;
  float safe_slot_width =
      std::max(slot_width,
               static_cast<float>(config.max_car_width) + veh_x_buffer) +
      slot_y_buffer;
  float safe_slot_length =
      std::max(slot_length, static_cast<float>(config.car_length)) +
      slot_x_buffer;

  slot_box.min_ = cdl::Vector2r(-0.5, -safe_slot_width / 2);
  slot_box.max_ = cdl::Vector2r(safe_slot_length, safe_slot_width / 2);

  Pose2D global;
  Pose2D local;
  bool is_collision;
  GJK2DInterface gjk;
  planning::PointCloudObstacle* obs;

  // fusion obj
  uint8 fusion_obj_number =
      local_view->fusion_occupancy_objects_info.fusion_object_size;
  for (uint8 i = 0; i < fusion_obj_number; i++) {
    const iflyauto::FusionOccupancyAdditional& points =
        local_view->fusion_occupancy_objects_info.fusion_object[i]
            .additional_occupancy_info;

    const iflyauto::ObjectType obs_type =
        local_view->fusion_occupancy_objects_info.fusion_object[i]
            .common_occupancy_info.type;
    if (obs_type == iflyauto::OBJECT_TYPE_PEDESTRIAN ||
        obs_type == iflyauto::OBJECT_TYPE_UNKNOWN_MOVABLE ||
        obs_type == iflyauto::OBJECT_TYPE_OCC_PEOPLE ||
        obs_type == iflyauto::OBJECT_TYPE_OCC_GENERAL_DYNAMIC) {
      continue;
    }

    obs = &obs_list.point_cloud_list[i];
    obs->obs_type = apa_planner::ApaObsAttributeType::FUSION_POINT_CLOUD;
    obs->points.clear();
    cdl::AABB box = cdl::AABB();

    for (uint32 j = 0; j < points.polygon_points_size; j++) {
      global.x = points.polygon_points[j].x;
      global.y = points.polygon_points[j].y;

      slot_tf.GlobalPointToULFLocal(&local, global);

      if (config.astar_config.enable_delete_occ_in_slot) {
        is_collision = slot_box.contain(cdl::Vector2r(local.x, local.y));

        if (is_collision) {
          // ILOG_INFO << "xy " << local.x << " " << local.y
          //           << " delete fusion_obj";
          continue;
        }
      }

      // delete by ego
      if (config.astar_config.enable_delete_occ_in_ego) {
        gjk.PolygonPointCollisionDetect(&is_collision, &ego_global_polygon,
                                        Position2D(local.x, local.y));

        if (is_collision) {
          // ILOG_INFO << "xy " << local.x << " " << local.y
          //           << " ego delete fusion_obj";
          continue;
        }
      }

      box.MergePoint(cdl::Vector2r(local.x, local.y));
      obs->points.emplace_back(Position2D(local.x, local.y));
    }

    // use box to generate polygon for future safe check
    if (obs->points.size() > 0) {
      GeneratePolygonByAABB(&obs->envelop_polygon, box);
      obs->box = box;
    }
  }

  uint8 ground_line_number = local_view->ground_line_perception.groundline_size;
  for (uint8 i = 0; i < ground_line_number; i++) {
    const iflyauto::FusionGroundLine& gl =
        local_view->ground_line_perception.groundline[i];

    obs = &obs_list.point_cloud_list[i + fusion_obj_number];
    obs->obs_type = apa_planner::ApaObsAttributeType::GROUND_LINE_POINT_CLOUD;
    obs->points.clear();
    cdl::AABB box = cdl::AABB();

    for (uint8 j = 0; j < gl.groundline_point_size; j++) {
      global.x = gl.groundline_point[j].x;
      global.y = gl.groundline_point[j].y;

      slot_tf.GlobalPointToULFLocal(&local, global);

      if (config.astar_config.enable_delete_occ_in_slot) {
        is_collision = slot_box.contain(cdl::Vector2r(local.x, local.y));

        if (is_collision) {
          // ILOG_INFO << "xy " << local.x << " " << local.y
          //           << "ego delete fusion_obj";
          continue;
        }
      }

      // delete by ego
      if (config.astar_config.enable_delete_occ_in_ego) {
        gjk.PolygonPointCollisionDetect(&is_collision, &ego_global_polygon,
                                        Position2D(local.x, local.y));

        if (is_collision) {
          // ILOG_INFO << "xy " << local.x << " " << local.y << " delete
          // fusion_obj";
          continue;
        }
      }

      box.MergePoint(cdl::Vector2r(local.x, local.y));
      obs->points.emplace_back(Position2D(local.x, local.y));
    }

    if (obs->points.size() > 0) {
      GeneratePolygonByAABB(&obs->envelop_polygon, box);

      obs->box = box;
    }
  }

  // limiters
  if (enable_limiter_obs) {
    obs = &obs_list.point_cloud_list[fusion_obj_number + ground_line_number];
    obs->obs_type = apa_planner::ApaObsAttributeType::SLOT_LIMITER;
    obs->points.clear();
    cdl::AABB box = cdl::AABB();
    std::vector<Position2D> limiter_points;

    const iflyauto::ParkingFusionInfo* slot_list =
        &local_view->parking_fusion_info;
    for (uint8 i = 0; i < slot_list->parking_fusion_slot_lists_size; i++) {
      const iflyauto::ParkingFusionSlot* slot =
          &slot_list->parking_fusion_slot_lists[i];

      if (slot_list->select_slot_id == slot->id) {
        continue;
      }

      for (uint8 j = 0; j < slot->limiters_size; j++) {
        const iflyauto::ParkingFusionLimiter* limiter = &slot->limiters[j];

        SampleInLineSegment(
            Eigen::Vector2f(limiter->end_points[0].x, limiter->end_points[0].y),
            Eigen::Vector2f(limiter->end_points[1].x, limiter->end_points[1].y),
            &limiter_points);

        // ILOG_INFO << "limiter point size " << limiter_points.size();

        for (size_t point_id = 0; point_id < limiter_points.size();
             point_id++) {
          global.x = limiter_points[point_id].x;
          global.y = limiter_points[point_id].y;
          slot_tf.GlobalPointToULFLocal(&local, global);

          if (config.astar_config.enable_delete_occ_in_slot) {
            is_collision = slot_box.contain(cdl::Vector2r(local.x, local.y));

            if (is_collision) {
              // ILOG_INFO << "xy " << local.x << " " << local.y
              //           << " delete fusion_obj";
              continue;
            }
          }

          // delete by ego
          if (config.astar_config.enable_delete_occ_in_ego) {
            gjk.PolygonPointCollisionDetect(&is_collision, &ego_global_polygon,
                                            Position2D(local.x, local.y));

            if (is_collision) {
              // ILOG_INFO << "xy " << local.x << " " << local.y
              //           << " ego delete fusion_obj";
              continue;
            }
          }

          box.MergePoint(cdl::Vector2r(local.x, local.y));
          obs->points.emplace_back(Position2D(local.x, local.y));

          // ILOG_INFO << "id = " << point_id << ",x = " << local.x
          //           << ",y = " << local.y;
        }
      }
    }

    // use box to generate polygon for future safe check
    if (obs->points.size() > 0) {
      GeneratePolygonByAABB(&obs->envelop_polygon, box);
      obs->box = box;
    }
  }

  ILOG_INFO << "GenerateFusionPolygon, size = "
            << obs_list.point_cloud_list.size();

  return;
}

void PointCloudObstacleTransform::SampleInLineSegment(
    const Eigen::Vector2f& start, const Eigen::Vector2f& end,
    std::vector<Position2D>* points) {
  points->clear();

  const Eigen::Vector2f line = end - start;

  if (std::sqrt(line.x() * line.x() + line.y() * line.y()) < 0.1) {
    points->emplace_back(Position2D(start.x(), start.y()));
    points->emplace_back(Position2D(end.x(), end.y()));
    return;
  }

  const Eigen::Vector2f unit_line_vec = line.normalized();
  float len = line.norm();

  float s = 0.0;
  float ds = 0.1;

  Eigen::Vector2f point;
  while (s < len) {
    point = start + s * unit_line_vec;

    points->emplace_back(Position2D(point.x(), point.y()));

    s += ds;
  }

  points->emplace_back(Position2D(end.x(), end.y()));

  return;
}

void PointCloudObstacleTransform::GenerateLocalObstacle(
    std::shared_ptr<apa_planner::ApaObstacleManager> obs_manager,
    ParkObstacleList& obs_list) {
  if (obs_manager == nullptr) {
    return;
  }

  planning::PointCloudObstacle obs;
  for (auto& pair : obs_manager->GetObstacles()) {
    if (pair.second.GetObsMovementType() ==
        apa_planner::ApaObsMovementType::MOTION) {
      continue;
    }

    obs.points.clear();
    obs.points.reserve(pair.second.GetPtClout2dLocal().size());

    for (const auto& pt : pair.second.GetPtClout2dLocal()) {
      obs.points.emplace_back(
          Position2D(static_cast<float>(pt.x()), static_cast<float>(pt.y())));
    }

    pair.second.GenerateLocalBoundingbox(&obs.box);

    if (pair.second.GetPtClout2dLocal().size() > 0) {
      GeneratePolygonByAABB(&obs.envelop_polygon, obs.box);
    } else {
      obs.envelop_polygon = pair.second.GetPolygon2DLocal();
    }

    obs.obs_type = pair.second.GetObsAttributeType();
    obs_list.point_cloud_list.emplace_back(obs);
  }

  return;
}

void PointCloudObstacleTransform::GenerateLocalObstacle(
    std::shared_ptr<apa_planner::ApaObstacleManager> obs_manager,
    ParkObstacleList& obs_list, cdl::AABB& box) {
  if (obs_manager == nullptr) {
    return;
  }

  planning::PointCloudObstacle obs;
  for (auto& pair : obs_manager->GetObstacles()) {
    if (pair.second.GetObsMovementType() ==
        apa_planner::ApaObsMovementType::MOTION) {
      continue;
    }

    obs.points.clear();
    obs.points.reserve(pair.second.GetPtClout2dLocal().size());

    for (const auto& pt : pair.second.GetPtClout2dLocal()) {
      if (box.contain(cdl::Vector2r(pt.x(), pt.y()))) {
        continue;
      }

      obs.points.emplace_back(Position2D(pt.x(), pt.y()));
    }

    pair.second.GenerateLocalBoundingbox(&obs.box);

    if (pair.second.GetPtClout2dLocal().size() > 0) {
      GeneratePolygonByAABB(&obs.envelop_polygon, obs.box);
    } else {
      obs.envelop_polygon = pair.second.GetPolygon2DLocal();
    }

    obs.obs_type = pair.second.GetObsAttributeType();
    obs_list.point_cloud_list.emplace_back(obs);
  }

  return;
}
}  // namespace planning
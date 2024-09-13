
#include "point_cloud_obstacle.h"

#include "./../../modules/apa_function/src/apa_param_setting.h"
#include "./../collision_detection/gjk2d_interface.h"
#include "polygon_base.h"
#include "pose2d.h"
#include "transform2d.h"
#include "virtual_wall_decider.h"

namespace planning {

const int PointCloudObstacleTransform::GenerateLocalObstacle(
    ParkObstacleList& obs_list, const LocalView* local_view,
    const bool delete_obs_around_ego, const double slot_length,
    const double slot_width, const Pose2D& slot_base_pose,
    const Pose2D& ego_start, const Pose2D& ego_final_goal) {
  Transform2d slot_tf;
  slot_tf.SetBasePose(slot_base_pose);

  // obs
  VirtualWallDecider wall_decider;
  wall_decider.Process(obs_list.virtual_obs, 40.0, 15.0, slot_width,
                       slot_length, ego_start, ego_final_goal);

  // hack: delete obstacle around ego and slot. In the future, it will be
  // retired.
  const apa_planner::ApaParameters& config = apa_param.GetParam();
  Polygon2D ego_local_polygon;
  Polygon2D ego_global_polygon;
  double veh_x_buffer = 0.3;
  double veh_y_buffer = 0.01;

  GenerateUpLeftFrameBox(
      &ego_local_polygon, -config.rear_overhanging - veh_x_buffer,
      -config.max_car_width / 2 - veh_y_buffer,
      config.car_length - config.rear_overhanging + veh_x_buffer,
      config.max_car_width / 2 + veh_y_buffer);
  ULFLocalPolygonToGlobal(&ego_global_polygon, &ego_local_polygon, ego_start);

  // generate local obs
  if (local_view == nullptr) {
    ILOG_ERROR << "local view is null";

    return 0;
  }

  ILOG_INFO
      << "obs, size: " << obs_list.point_cloud_list.size()
      << ", fusion_object_num: "
      << (size_t)(local_view->fusion_objects_info.fusion_object_size)
      << ", ground_lines_size: "
      << static_cast<size_t>(
             local_view->ground_line_perception.ground_lines_size)
      << ", fusion_occupancy_objects_info size: "
      << (size_t)(local_view->fusion_occupancy_objects_info.fusion_object_size);

  uint8_t number =
      local_view->fusion_occupancy_objects_info.fusion_object_size +
      local_view->ground_line_perception.ground_lines_size;

  obs_list.point_cloud_list.resize(number);

  // slot aabb
  cdl::AABB slot_box;

  double slot_x_buffer = 1.0;
  double slot_y_buffer = 0.05;
  double safe_slot_width =
      std::max(slot_width, config.max_car_width + veh_x_buffer) + slot_y_buffer;
  double safe_slot_length =
      std::max(slot_length, config.car_length) + slot_x_buffer;

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

    obs = &obs_list.point_cloud_list[i];
    obs->obs_type = ParkObstacleType::fusion_obj;
    obs->points.clear();
    cdl::AABB box;

    for (uint32 j = 0; j < points.polygon_points_size; j++) {
      global.x = points.polygon_points[j].x;
      global.y = points.polygon_points[j].y;

      slot_tf.GlobalPointToULFLocal(&local, global);

      if (config.enable_delete_fusion_obj_in_slot) {
        is_collision = slot_box.contain(cdl::Vector2r(local.x, local.y));

        if (is_collision) {
          // ILOG_INFO << "xy " << local.x << " " << local.y
          //           << " delete fusion_obj";
          continue;
        }
      }

      // delete by ego
      if (delete_obs_around_ego) {
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

  // ground line, todo: ground line is veh reference frame, so need transform.

  for (uint8 i = 0; i < local_view->ground_line_perception.ground_lines_size;
       i++) {
    const iflyauto::GroundLine& gl =
        local_view->ground_line_perception.ground_lines[i];

    obs = &obs_list.point_cloud_list[i + fusion_obj_number];
    obs->obs_type = ParkObstacleType::ground_line;
    obs->points.clear();
    cdl::AABB box;

    for (uint8 j = 0; j < gl.points_3d_size; j++) {
      global.x = gl.points_3d[j].x;
      global.y = gl.points_3d[j].y;

      slot_tf.GlobalPointToULFLocal(&local, global);

      if (config.enable_delete_fusion_obj_in_slot) {
        is_collision = slot_box.contain(cdl::Vector2r(local.x, local.y));

        if (is_collision) {
          // ILOG_INFO << "xy " << local.x << " " << local.y
          //           << "ego delete fusion_obj";
          continue;
        }
      }

      // delete by ego
      if (delete_obs_around_ego) {
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

  ILOG_INFO << "GenerateFusionPolygon, size "
            << obs_list.point_cloud_list.size();

  return 0;
}

void PointCloudObstacleTransform::GenerateGlobalObstacle(
    ParkObstacleList& obs_list, const LocalView* local_view) {
  // generate local obs
  if (local_view == nullptr) {
    ILOG_ERROR << "local view is null";

    return;
  }

  ILOG_INFO
      << "obs, size: " << obs_list.point_cloud_list.size()
      << ", fusion_object_num: "
      << (size_t)(local_view->fusion_objects_info.fusion_object_size)
      << ", ground_lines_size: "
      << static_cast<size_t>(
             local_view->ground_line_perception.ground_lines_size)
      << ", fusion_occupancy_objects_info size: "
      << (size_t)(local_view->fusion_occupancy_objects_info.fusion_object_size);

  uint8_t number =
      local_view->fusion_occupancy_objects_info.fusion_object_size +
      local_view->ground_line_perception.ground_lines_size;

  obs_list.point_cloud_list.resize(number);

  Position2D global_point;
  planning::PointCloudObstacle* obs;

  // fusion obj
  uint8 fusion_obj_number =
      local_view->fusion_occupancy_objects_info.fusion_object_size;
  for (uint8 i = 0; i < fusion_obj_number; i++) {
    const iflyauto::FusionOccupancyAdditional& points =
        local_view->fusion_occupancy_objects_info.fusion_object[i]
            .additional_occupancy_info;

    obs = &obs_list.point_cloud_list[i];
    obs->obs_type = ParkObstacleType::fusion_obj;
    obs->points.clear();
    cdl::AABB box;

    for (uint32 j = 0; j < points.polygon_points_size; j++) {
      global_point.x = points.polygon_points[j].x;
      global_point.y = points.polygon_points[j].y;

      box.MergePoint(cdl::Vector2r(global_point.x, global_point.y));
      obs->points.emplace_back(Position2D(global_point.x, global_point.y));
    }

    // use box to generate polygon for future safe check
    if (obs->points.size() > 0) {
      GeneratePolygonByAABB(&obs->envelop_polygon, box);
      obs->box = box;
    }
  }

  // ground line, todo: ground line is veh reference frame, so need transform.

  for (uint8 i = 0; i < local_view->ground_line_perception.ground_lines_size;
       i++) {
    const iflyauto::GroundLine& gl =
        local_view->ground_line_perception.ground_lines[i];

    obs = &obs_list.point_cloud_list[i + fusion_obj_number];
    obs->obs_type = ParkObstacleType::ground_line;
    obs->points.clear();
    cdl::AABB box;

    for (uint8 j = 0; j < gl.points_3d_size; j++) {
      global_point.x = gl.points_3d[j].x;
      global_point.y = gl.points_3d[j].y;

      box.MergePoint(cdl::Vector2r(global_point.x, global_point.y));
      obs->points.emplace_back(Position2D(global_point.x, global_point.y));
    }

    if (obs->points.size() > 0) {
      GeneratePolygonByAABB(&obs->envelop_polygon, box);

      obs->box = box;
    }
  }

  ILOG_INFO << "fusion obs, size= " << obs_list.point_cloud_list.size()
            << " ,virtual obs size = " << obs_list.virtual_obs.size();

  return;
}
}  // namespace planning
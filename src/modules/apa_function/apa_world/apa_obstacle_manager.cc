#include "apa_obstacle_manager.h"
#include <bits/stdint-intn.h>

#include <algorithm>
#include <cstddef>
#include <vector>

#include "apa_obstacle.h"
#include "apa_param_config.h"
#include "common_c.h"
#include "common_platform_type_soc.h"
#include "environmental_model.h"
#include "local_view.h"
#include "log_glog.h"
#include "polygon_base.h"
#include "pose2d.h"

namespace planning {
namespace apa_planner {
void ApaObstacleManager::Update(const LocalView* local_view) {
  Reset();
  if (local_view == nullptr) {
    ILOG_ERROR << "Update ApaObstacleManager, local_view_ptr is nullptr";
    return;
  }

  ILOG_INFO << "Update ApaObstacleManager";

  // 读取超声波扇形距离
  const double min_uss_dist = apa_param.GetParam().min_uss_origin_dist;
  if (apa_param.GetParam().is_uss_dist_from_perception) {
    const auto& uss_dis_info_buf =
        local_view->uss_percept_info.dis_from_car_to_obj;

    //  front uss: uss dis need to be transfered from mm to m. order: fl apa, 4
    //  upa, fr apa
    for (const auto& front_uss_idx :
         apa_param.GetParam().uss_wdis_index_front) {
      uss_dis_vec_.emplace_back(
          std::max(min_uss_dist, 0.001 * uss_dis_info_buf[front_uss_idx]));
    }

    // rear uss: uss dis need to be transfered from mm to m. order: rr apa, 4
    // upa, rl apa
    for (const auto& rear_uss_idx : apa_param.GetParam().uss_wdis_index_back) {
      uss_dis_vec_.emplace_back(
          std::max(min_uss_dist, 0.001 * uss_dis_info_buf[rear_uss_idx]));
    }
  } else {
    // load uss dist from uss wave, m id f
    const auto& upa_dis_info_buf = local_view->uss_wave_info.upa_dis_info_buf;

    const std::vector<int> front_wids_idx_vec = {0, 9, 6, 3, 1, 11};
    const std::vector<int> rear_wids_idx_vec = {0, 1, 3, 6, 9, 11};

    const std::vector<std::vector<int>> wids_idx_vec = {front_wids_idx_vec,
                                                        rear_wids_idx_vec};
    for (size_t i = 0; i < wids_idx_vec.size(); i++) {
      for (size_t j = 0; j < wids_idx_vec[i].size(); j++) {
        const auto idx = wids_idx_vec[i][j];
        uss_dis_vec_.emplace_back(std::max(
            min_uss_dist, 1.0 * upa_dis_info_buf[i].wdis[idx].wdis_value[0]));
      }
    }
  }

  // 读取通用障碍物点云
  if (apa_param.GetParam().use_fus_occ_obj) {
    const uint8 fusion_obs_size =
        std::min(local_view->fusion_occupancy_objects_info.fusion_object_size,
                 static_cast<uint8>(FUSION_OCCUPANCY_OBJECTS_MAX_NUM));
    for (uint8 i = 0; i < fusion_obs_size; ++i) {
      // [hack]: need to retire in published version.
      if (local_view->fusion_occupancy_objects_info.fusion_object[i]
              .common_occupancy_info.type == iflyauto::OBJECT_TYPE_OCC_COLUMN) {
        continue;
      }

      const iflyauto::FusionOccupancyAdditional& fusion_occupancy_object =
          local_view->fusion_occupancy_objects_info.fusion_object[i]
              .additional_occupancy_info;
      const uint32 polygon_points_size =
          std::min(fusion_occupancy_object.polygon_points_size,
                   static_cast<uint32>(
                       FUSION_OCCUPANCY_OBJECTS_POLYGON_POINTS_SET_MAX_NUM));
      if (polygon_points_size < 1) {
        continue;
      }
      std::vector<Eigen::Vector2d> fusion_pt_clout_2d;
      fusion_pt_clout_2d.reserve(polygon_points_size);
      Polygon2D polygon;
      cdl::AABB box = cdl::AABB();
      for (uint32 j = 0; j < polygon_points_size; ++j) {
        const Eigen::Vector2d fusion_pt(
            fusion_occupancy_object.polygon_points[j].x,
            fusion_occupancy_object.polygon_points[j].y);
        box.MergePoint(cdl::Vector2r(fusion_pt.x(), fusion_pt.y()));
        fusion_pt_clout_2d.emplace_back(std::move(fusion_pt));
      }

      GeneratePolygonByAABB(&polygon, box);

      const iflyauto::ObjectType obs_type =
          local_view->fusion_occupancy_objects_info.fusion_object[i]
              .common_occupancy_info.type;

      ApaObstacle apa_obs;
      if (apa_param.GetParam().enable_use_dynamic_obs) {
        if (obs_type == iflyauto::OBJECT_TYPE_PEDESTRIAN ||
            obs_type == iflyauto::OBJECT_TYPE_UNKNOWN_MOVABLE ||
            obs_type == iflyauto::OBJECT_TYPE_OCC_PEOPLE ||
            obs_type == iflyauto::OBJECT_TYPE_OCC_GENERAL_DYNAMIC) {
          ILOG_INFO << "there are people or dynamic obs";
          apa_obs.SetObsMovementType(ApaObsMovementType::MOTION);
        }
      }

      apa_obs.SetPtClout2dGlobal(fusion_pt_clout_2d);
      apa_obs.SetObsAttributeType(ApaObsAttributeType::FUSION_POINT_CLOUD);
      apa_obs.SetBoxGlobal(box);
      apa_obs.SetPolygonGlobal(polygon);
      apa_obs.SetId(obs_id_generate_);
      obstacles_[obs_id_generate_] = apa_obs;
      obs_id_generate_++;
    }
  }

  if (apa_param.GetParam().use_object_detect) {
    const uint8 fusion_obs_size =
        std::min(local_view->fusion_objects_info.fusion_object_size,
                 static_cast<uint8>(FUSION_OBJECT_MAX_NUM));
    for (uint8 i = 0; i < fusion_obs_size; ++i) {
      const iflyauto::Obstacle& obs =
          local_view->fusion_objects_info.fusion_object[i].common_info;

      if (!IsConsideredODType(obs.type)) {
        continue;
      }

      Pose2D center_pose(obs.center_position.x, obs.center_position.y,
                  obs.heading_angle);

      std::vector<Eigen::Vector2f> local_box;
      GenerateBoundingBox(obs.shape.length, obs.shape.width,
                          Eigen::Vector2f(0.0f, 0.0f), local_box);

      std::vector<Eigen::Vector2f> global_box;
      LocalPolygonToGlobal(local_box, center_pose, global_box);

      std::vector<Eigen::Vector2d> box_points;
      for (uint8 j = 0; j < global_box.size(); ++j) {
        const Eigen::Vector2d fusion_pt(global_box[j].x(), global_box[j].y());
        box_points.emplace_back(std::move(fusion_pt));
      }

      ApaObstacle apa_obs;
      apa_obs.SetPtClout2dGlobal(box_points);
      apa_obs.SetObsAttributeType(ApaObsAttributeType::FUSION_POLYGON);

      Polygon2D polygon;
      GeneratePolygonByPoints(global_box, &polygon);
      apa_obs.SetPolygonGlobal(polygon);

      cdl::AABB box = cdl::AABB();
      GetBoundingBoxByPolygon(&box, &polygon);
      apa_obs.SetBoxGlobal(box);

      if (apa_param.GetParam().enable_use_dynamic_obs &&
          IsDynamicObjectType(obs.type)) {
        apa_obs.SetObsMovementType(ApaObsMovementType::MOTION);
      }

      apa_obs.SetId(obs_id_generate_);
      obstacles_[obs_id_generate_] = apa_obs;
      obs_id_generate_++;
    }
  }

  // 读取接地线障碍物点云
  if (apa_param.GetParam().use_ground_line) {
    const uint8 ground_lines_size =
        std::min(local_view->ground_line_perception.groundline_size,
                 static_cast<uint8>(FUSION_GROUNDLINE_MAX_NUM));
    for (uint8 i = 0; i < ground_lines_size; ++i) {
      const iflyauto::FusionGroundLine& gl =
          local_view->ground_line_perception.groundline[i];

      if (gl.resource_type == iflyauto::RESOURCE_TYPE_MAP) {
        continue;
      }

      const uint8 points_3d_size =
          std::min(gl.groundline_point_size,
                   static_cast<uint8>(FUSION_GROUNDLINE_POINT_MAX_NUM));
      if (points_3d_size < 1) {
        continue;
      }
      std::vector<Eigen::Vector2d> gl_pt_clout_2d;
      gl_pt_clout_2d.reserve(points_3d_size);
      Polygon2D polygon;
      cdl::AABB box = cdl::AABB();
      for (uint8 j = 0; j < points_3d_size; ++j) {
        const Eigen::Vector2d gl_pt(gl.groundline_point[j].x, gl.groundline_point[j].y);
        box.MergePoint(cdl::Vector2r(gl_pt.x(), gl_pt.y()));
        gl_pt_clout_2d.emplace_back(std::move(gl_pt));
      }

      GeneratePolygonByAABB(&polygon, box);

    ApaObstacle apa_obs;
    apa_obs.SetPtClout2dGlobal(gl_pt_clout_2d);
    apa_obs.SetObsAttributeType(ApaObsAttributeType::GROUND_LINE_POINT_CLOUD);
    apa_obs.SetBoxGlobal(box);
    apa_obs.SetPolygonGlobal(polygon);
    apa_obs.SetId(obs_id_generate_);
    obstacles_[obs_id_generate_] = apa_obs;
    obs_id_generate_++;
    }
  }

  // 读取超声波障碍物点云
  if (apa_param.GetParam().use_uss_pt_clound) {
    const uint8 uss_obs_size = std::min(1, USS_PERCEPTION_OUTLINE_DATAORI_NUM);
    for (uint8 i = 0; i < uss_obs_size; ++i) {
      const iflyauto::ApaSlotOutlineCoordinateDataType& obj_info =
          local_view->uss_percept_info.out_line_dataori[i];
      const uint32 pt_cloud_size = std::min(
          obj_info.obj_pt_cnt,
                 static_cast<uint32>(USS_PERCEPTION_APA_SLOT_OBJ_MAX_NUM));
      if (pt_cloud_size < 1) {
        continue;
      }
      std::vector<Eigen::Vector2d> uss_pt_clout_2d;
      uss_pt_clout_2d.reserve(pt_cloud_size);
      Polygon2D polygon;
      cdl::AABB box = cdl::AABB();
      for (uint32 j = 0; j < pt_cloud_size; ++j) {
        const Eigen::Vector2d uss_pt(obj_info.obj_pt_global[j].x,
                                     obj_info.obj_pt_global[j].y);
        box.MergePoint(cdl::Vector2r(uss_pt.x(), uss_pt.y()));
        uss_pt_clout_2d.emplace_back(std::move(uss_pt));
      }

      GeneratePolygonByAABB(&polygon, box);

      ApaObstacle apa_obs;
      apa_obs.SetPtClout2dGlobal(uss_pt_clout_2d);
      apa_obs.SetObsAttributeType(ApaObsAttributeType::USS_POINT_CLOUD);
      apa_obs.SetBoxGlobal(box);
      apa_obs.SetPolygonGlobal(polygon);
      apa_obs.SetId(obs_id_generate_);
      obstacles_[obs_id_generate_] = apa_obs;
      obs_id_generate_++;
    }
  }

  // todo: 读取限位器信息
}

void ApaObstacleManager::SetObstacle(const size_t id,
                                     const ApaObstacle& apa_obs) {
  obstacles_[id] = apa_obs;
}

const bool ApaObstacleManager::GetObstacle(const size_t id, ApaObstacle* obs) {
  if (obstacles_.count(id) == 0) {
    return false;
  }

  obs = &obstacles_[id];
  return true;
}

const bool ApaObstacleManager::GetObstacle(const ApaObsAttributeType type,
                                           std::vector<ApaObstacle*>& obs_vec) {
  obs_vec.clear();
  for (auto& pair : obstacles_) {
    if (pair.second.GetObsAttributeType() == type) {
      obs_vec.emplace_back(&pair.second);
    }
  }
  return obs_vec.size() > 0;
}

void ApaObstacleManager::TransformCoordFromGlobalToLocal(
    const pnc::geometry_lib::GlobalToLocalTf& g2l_tf) {
  for (auto& pair : obstacles_) {
    if (pair.second.GetObsAttributeType() !=
        ApaObsAttributeType::VIRTUAL_POINT_CLOUD) {
      pair.second.TransformCoordFromGlobalToLocal(g2l_tf);
    }
  }
}

const bool ApaObstacleManager::IsConsideredODType(
    const iflyauto::ObjectType type) {
  if (type == iflyauto::OBJECT_TYPE_PEDESTRIAN ||
      type == iflyauto::OBJECT_TYPE_ANIMAL ||
      type == iflyauto::OBJECT_TYPE_CYCLE_RIDING ||
      type == iflyauto::OBJECT_TYPE_MOTORCYCLE_RIDING ||
      type == iflyauto::OBJECT_TYPE_TRICYCLE_RIDING) {
    return true;
  }

  return false;
}

const bool ApaObstacleManager::IsDynamicObjectType(
    const iflyauto::ObjectType type) {
  if (type == iflyauto::OBJECT_TYPE_PEDESTRIAN ||
      type == iflyauto::OBJECT_TYPE_ANIMAL ||
      type == iflyauto::OBJECT_TYPE_CYCLE_RIDING ||
      type == iflyauto::OBJECT_TYPE_MOTORCYCLE_RIDING ||
      type == iflyauto::OBJECT_TYPE_TRICYCLE_RIDING) {
    return true;
  }

  return false;
}

}  // namespace apa_planner
}  // namespace planning

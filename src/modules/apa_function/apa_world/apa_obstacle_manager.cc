#include "apa_obstacle_manager.h"

#include <algorithm>
#include <cstddef>
#include <vector>

#include "apa_obstacle.h"
#include "apa_param_config.h"
#include "common_platform_type_soc.h"
#include "environmental_model.h"
#include "local_view.h"

namespace planning {
namespace apa_planner {
void ApaObstacleManager::Update(const LocalView* local_view) {
  Reset();
  if (local_view == nullptr) {
    ILOG_ERROR << "Update ApaObstacleManager, local_view_ptr is nullptr";
    return;
  }

  // 读取通用障碍物点云
  if (apa_param.GetParam().use_fus_occ_obj) {
    const uint8 fusion_obs_size =
        std::min(local_view->fusion_occupancy_objects_info.fusion_object_size,
                 static_cast<uint8>(FUSION_OCCUPANCY_OBJECT_MAX_NUM));
    for (uint8 i = 0; i < fusion_obs_size; ++i) {
      const iflyauto::FusionOccupancyAdditional& fusion_occupancy_object =
          local_view->fusion_occupancy_objects_info.fusion_object[i]
              .additional_occupancy_info;
      const uint32 polygon_points_size = std::min(
          fusion_occupancy_object.polygon_points_size,
          static_cast<uint32>(FUSION_OCCUPANCY_OBJECTS_POLYGON_POINTS_SET_NUM));
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

      ApaObstacle apa_obs;
      apa_obs.SetPtClout2dGlobal(fusion_pt_clout_2d);
      apa_obs.SetObsAttributeType(ApaObsAttributeType::FUSION_POINT_CLOUD);
      apa_obs.SetBoxGlobal(box);
      apa_obs.SetPolygonGlobal(polygon);
      apa_obs.SetId(obs_id_generate_);
      obstacles_[obs_id_generate_] = apa_obs;
      obs_id_generate_++;
    }
  } else {
    const uint8 fusion_obs_size =
        std::min(local_view->fusion_objects_info.fusion_object_size,
                 static_cast<uint8>(FUSION_OBJECT_MAX_NUM));
    for (uint8 i = 0; i < fusion_obs_size; ++i) {
      const iflyauto::FusionObjectsAdditional& fusion_object =
          local_view->fusion_objects_info.fusion_object[i].additional_info;
      const uint8 polygon_points_size =
          std::min(fusion_object.polygon_points_size,
                   static_cast<uint8>(FUSION_OBJECTS_POLYGON_POINTS_SET_NUM));
      if (polygon_points_size < 1) {
        continue;
      }
      std::vector<Eigen::Vector2d> fusion_pt_clout_2d;
      fusion_pt_clout_2d.reserve(polygon_points_size);
      Polygon2D polygon;
      cdl::AABB box = cdl::AABB();
      for (uint8 j = 0; j < polygon_points_size; ++j) {
        const Eigen::Vector2d fusion_pt(fusion_object.polygon_points[j].x,
                                        fusion_object.polygon_points[j].y);
        box.MergePoint(cdl::Vector2r(fusion_pt.x(), fusion_pt.y()));
        fusion_pt_clout_2d.emplace_back(std::move(fusion_pt));
      }

      GeneratePolygonByAABB(&polygon, box);

      ApaObstacle apa_obs;
      apa_obs.SetPtClout2dGlobal(fusion_pt_clout_2d);
      apa_obs.SetObsAttributeType(ApaObsAttributeType::FUSION_POINT_CLOUD);
      apa_obs.SetBoxGlobal(box);
      apa_obs.SetPolygonGlobal(polygon);
      apa_obs.SetId(obs_id_generate_);
      obstacles_[obs_id_generate_] = apa_obs;
      obs_id_generate_++;
    }
  }

  // 读取接地线障碍物点云
  const uint8 ground_lines_size =
      std::min(local_view->ground_line_perception.ground_lines_size,
               static_cast<uint8>(GROUND_LINES_NUM));
  for (uint8 i = 0; i < ground_lines_size; ++i) {
    const iflyauto::GroundLine& gl =
        local_view->ground_line_perception.ground_lines[i];
    const uint8 points_3d_size =
        std::min(gl.points_3d_size, static_cast<uint8>(GROUND_LINE_POINTS_NUM));
    if (points_3d_size < 1) {
      continue;
    }
    std::vector<Eigen::Vector2d> gl_pt_clout_2d;
    gl_pt_clout_2d.reserve(points_3d_size);
    Polygon2D polygon;
    cdl::AABB box = cdl::AABB();
    for (uint8 j = 0; j < points_3d_size; ++j) {
      const Eigen::Vector2d gl_pt(gl.points_3d[j].x, gl.points_3d[j].y);
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

  // 读取超声波障碍物点云
  const uint8 uss_obs_size = std::min(1, NUM_OF_OUTLINE_DATAORI);
  for (uint8 i = 0; i < uss_obs_size; ++i) {
    const iflyauto::ApaSlotOutlineCoordinateDataType& obj_info =
        local_view->uss_percept_info.out_line_dataori[i];
    const uint32 pt_cloud_size =
        std::min(obj_info.obj_pt_cnt, static_cast<uint32>(NUM_OF_APA_SLOT_OBJ));
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
    const pnc::geometry_lib::PathPoint& origin_pose) {
  pnc::geometry_lib::GlobalToLocalTf g2l_tf;
  g2l_tf.Init(origin_pose.pos, origin_pose.heading);
  for (auto& pair : obstacles_) {
    if (pair.second.GetObsAttributeType() !=
        ApaObsAttributeType::VIRTUAL_POINT_CLOUD) {
      pair.second.TransformCoordFromGlobalToLocal(g2l_tf);
    }
  }
}

}  // namespace apa_planner
}  // namespace planning

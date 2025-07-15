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
#include "src/library/convex_collision_detection/gjk2d_interface.h"

namespace planning {
namespace apa_planner {
void ApaObstacleManager::Update(
    const LocalView* local_view,
    const std::shared_ptr<ApaStateMachineManager>& state_machine_ptr) {
  Reset();
  if (local_view == nullptr) {
    ILOG_ERROR << "Update ApaObstacleManager, local_view_ptr is nullptr";
    return;
  }

  state_machine_ptr_ = state_machine_ptr;

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
    const auto& upa_dis_info_buf =
        local_view->uss_wave_info.sonar_distance_data;

    const std::vector<int> front_wids_idx_vec = {0, 1, 2, 3, 4, 5};
    const std::vector<int> rear_wids_idx_vec = {6, 7, 8, 9, 10, 11};

    const std::vector<std::vector<int>> wids_idx_vec = {front_wids_idx_vec,
                                                        rear_wids_idx_vec};
    for (size_t i = 0; i < wids_idx_vec.size(); i++) {
      for (size_t j = 0; j < wids_idx_vec[i].size(); j++) {
        const auto idx = wids_idx_vec[i][j];
        uss_dis_vec_.emplace_back(std::max(
            min_uss_dist, 1.0 * upa_dis_info_buf[idx].pas_sonarx_distance));
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
      if (!apa_param.GetParam().use_fus_occ_column) {
        if (local_view->fusion_occupancy_objects_info.fusion_object[i]
                .common_occupancy_info.type ==
            iflyauto::OBJECT_TYPE_OCC_COLUMN) {
          continue;
        }
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

      ApaObsScemanticType scemantic_type = ApaObsScemanticType::UNKNOWN;
      switch (obs_type) {
        case iflyauto::OBJECT_TYPE_OCC_COLUMN:
          scemantic_type = ApaObsScemanticType::COLUMN;
          break;
        case iflyauto::OBJECT_TYPE_OCC_WALL:
          scemantic_type = ApaObsScemanticType::WALL;
          break;
        case iflyauto::OBJECT_TYPE_OCC_CAR:
          scemantic_type = ApaObsScemanticType::CAR;
          break;
        case iflyauto::OBJECT_TYPE_OCC_GROUDING_WIRE:
          scemantic_type = ApaObsScemanticType::CURB;
          break;
        default:
          scemantic_type = ApaObsScemanticType::UNKNOWN;
          break;
      }

      apa_obs.SetObsScemanticType(scemantic_type);
      apa_obs.SetPtClout2dGlobal(fusion_pt_clout_2d);
      apa_obs.SetObsAttributeType(ApaObsAttributeType::FUSION_POINT_CLOUD);
      if (!apa_param.GetParam().enable_multi_height_col_det) {
        apa_obs.SetObsHeightType(ApaObsHeightType::HIGH);
      }
      apa_obs.SetBoxGlobal(box);
      apa_obs.SetPolygonGlobal(polygon);
      apa_obs.SetId(obs_id_generate_);
      apa_obs.ClearDecision();
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

      double speed = std::sqrt(obs.velocity.x * obs.velocity.x +
                               obs.velocity.y * obs.velocity.y);
      if (speed < 0.02) {
        continue;
      }

      Pose2D center_pose(obs.center_position.x, obs.center_position.y,
                         obs.heading_angle);

      std::vector<Eigen::Vector2d> local_box;
      GenerateBoundingBox(obs.shape.length, obs.shape.width,
                          Eigen::Vector2d(0.0f, 0.0f), local_box);

      std::vector<Eigen::Vector2d> global_box;
      LocalPolygonToGlobal(local_box, center_pose, global_box);

      ApaObstacle apa_obs;
      apa_obs.Reset();
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

      pnc::geometry_lib::PathPoint pose(
          Eigen::Vector2d(obs.center_position.x, obs.center_position.y),
          obs.heading_angle);
      apa_obs.SetPose(pose);
      apa_obs.SetSpeed(speed);

      Eigen::Vector2d speed_dir(obs.velocity.y, obs.velocity.x);
      double dot = speed_dir.dot(pose.heading_vec);
      if (dot >= 0.0) {
        apa_obs.SetSpeedHeading(pose.heading_vec);
      } else {
        apa_obs.SetSpeedHeading(
            Eigen::Vector2d(-pose.heading_vec[0], -pose.heading_vec[1]));
      }

      apa_obs.SetObsAttributeType(ApaObsAttributeType::FUSION_POLYGON);
      apa_obs.SetObsScemanticType(ApaObsScemanticType::UNKNOWN);
      if (!apa_param.GetParam().enable_multi_height_col_det) {
        apa_obs.SetObsHeightType(ApaObsHeightType::HIGH);
      }
      apa_obs.SetObsMovementType(ApaObsMovementType::MOTION);
      apa_obs.SetId(obs_id_generate_);
      apa_obs.ClearDecision();
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

      if (!apa_param.GetParam().use_ground_line_wall_column &&
          (gl.type == iflyauto::GROUND_LINE_TYPE_COLUMN ||
           gl.type == iflyauto::GROUND_LINE_TYPE_WALL)) {
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
        const Eigen::Vector2d gl_pt(gl.groundline_point[j].x,
                                    gl.groundline_point[j].y);
        box.MergePoint(cdl::Vector2r(gl_pt.x(), gl_pt.y()));
        gl_pt_clout_2d.emplace_back(std::move(gl_pt));
      }

      GeneratePolygonByAABB(&polygon, box);

      ApaObstacle apa_obs;
      ApaObsScemanticType scemantic_type = ApaObsScemanticType::UNKNOWN;
      switch (gl.type) {
        case iflyauto::GROUND_LINE_TYPE_COLUMN:
          scemantic_type = ApaObsScemanticType::COLUMN;
          break;
        case iflyauto::GROUND_LINE_TYPE_WALL:
          scemantic_type = ApaObsScemanticType::WALL;
          break;
        case iflyauto::GROUND_LINE_TYPE_FENCE:
          scemantic_type = ApaObsScemanticType::FENCE;
          break;
        case iflyauto::GROUND_LINE_TYPE_STEP:
          scemantic_type = ApaObsScemanticType::STEP;
          break;
        case iflyauto::GROUND_LINE_TYPE_CURB:
          scemantic_type = ApaObsScemanticType::CURB;
          break;
        case iflyauto::GROUND_LINE_TYPE_SPECIAL:
          scemantic_type = ApaObsScemanticType::SPECIAL;
          break;
        default:
          scemantic_type = ApaObsScemanticType::UNKNOWN;
          break;
      }

      apa_obs.SetObsScemanticType(scemantic_type);
      apa_obs.SetPtClout2dGlobal(gl_pt_clout_2d);
      apa_obs.SetObsAttributeType(ApaObsAttributeType::GROUND_LINE_POINT_CLOUD);
      if (!apa_param.GetParam().enable_multi_height_col_det) {
        apa_obs.SetObsHeightType(ApaObsHeightType::HIGH);
      }
      apa_obs.SetBoxGlobal(box);
      apa_obs.SetPolygonGlobal(polygon);
      apa_obs.SetId(obs_id_generate_);
      apa_obs.ClearDecision();
      obstacles_[obs_id_generate_] = apa_obs;
      obs_id_generate_++;
    }
  }

  // 读取超声波障碍物点云
  if (apa_param.GetParam().uss_config.use_uss_pt_clound) {
    const iflyauto::IFLYLocalization& localization_info =
        local_view->localization;
    Pose2D ego_pose;
    ego_pose.x = localization_info.position.position_boot.x;
    ego_pose.y = localization_info.position.position_boot.y;
    ego_pose.theta = localization_info.orientation.euler_boot.yaw;
    Transform2d tf;
    tf.SetBasePose(ego_pose);

    if (apa_param.GetParam().uss_config.use_fusion) {
      const apa_planner::ApaParameters& config = apa_param.GetParam();
      Polygon2D ego_local;
      GetUpLeftCoordinatePolygonByParam(
          &ego_local,
          config.rear_overhanging + config.uss_config.point_max_dist,
          config.wheel_base + config.front_overhanging +
              config.uss_config.point_max_dist,
          config.car_width / 2.0 + config.uss_config.point_max_dist);

      Polygon2D ego_global;
      ULFLocalPolygonToGlobal(&ego_global, &ego_local, tf);
      GJK2DInterface gjk_interface;
      bool is_contain = false;

      const uint8 uss_obs_size =
          std::min(1, USS_PERCEPTION_OUTLINE_DATAORI_NUM);
      for (uint8 i = 0; i < uss_obs_size; ++i) {
        const iflyauto::ApaSlotOutlineCoordinateDataType& obj_info =
            local_view->uss_percept_info.out_line_dataori[i];
        const uint32 pt_cloud_size =
            std::min(obj_info.obj_pt_cnt,
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

          gjk_interface.PolygonPointCollisionDetect(
              &ego_global, Eigen::Vector2d(uss_pt[0], uss_pt[1]), &is_contain);
          if (!is_contain) {
            continue;
          }

          box.MergePoint(cdl::Vector2r(uss_pt.x(), uss_pt.y()));
          uss_pt_clout_2d.emplace_back(std::move(uss_pt));
        }

        GeneratePolygonByAABB(&polygon, box);

        ApaObstacle apa_obs;
        apa_obs.SetPtClout2dGlobal(uss_pt_clout_2d);
        apa_obs.SetObsAttributeType(ApaObsAttributeType::USS_POINT_CLOUD);
        apa_obs.SetObsScemanticType(ApaObsScemanticType::UNKNOWN);
        if (!apa_param.GetParam().enable_multi_height_col_det) {
          apa_obs.SetObsHeightType(ApaObsHeightType::HIGH);
        }
        apa_obs.SetObsMovementType(ApaObsMovementType::STATIC);
        apa_obs.SetBoxGlobal(box);
        apa_obs.SetPolygonGlobal(polygon);
        apa_obs.SetId(obs_id_generate_);
        apa_obs.ClearDecision();
        obstacles_[obs_id_generate_] = apa_obs;
        obs_id_generate_++;
      }
    } else {
      Polygon2D polygon;
      cdl::AABB box = cdl::AABB();

      Eigen::Vector2d global;

      const iflyauto::UssPdcPrivPointType& obj_info =
          local_view->uss_wave_info.priv_point_data;
      std::vector<Eigen::Vector2d> uss_pt_clout_2d;
      uss_pt_clout_2d.reserve(USS_WAVE_PDC_PRIV_POINT_NUM);
      for (uint32 j = 0; j < USS_WAVE_PDC_PRIV_POINT_NUM; ++j) {
        if (obj_info.priv_point_data_prop[j].point_valid == 0) {
          continue;
        }

        tf.ULFLocalPointToGlobal(
            global,
            Eigen::Vector2d(obj_info.priv_point_data_prop[j].point_x * 0.01,
                            obj_info.priv_point_data_prop[j].point_y * 0.01));

        box.MergePoint(cdl::Vector2r(global.x(), global.y()));
        uss_pt_clout_2d.emplace_back(Eigen::Vector2d(global.x(), global.y()));
      }

      GeneratePolygonByAABB(&polygon, box);

      ApaObstacle apa_obs;
      apa_obs.SetPtClout2dGlobal(uss_pt_clout_2d);
      apa_obs.SetObsAttributeType(ApaObsAttributeType::USS_POINT_CLOUD);
      apa_obs.SetObsScemanticType(ApaObsScemanticType::UNKNOWN);
      if (!apa_param.GetParam().enable_multi_height_col_det) {
        apa_obs.SetObsHeightType(ApaObsHeightType::HIGH);
      }
      apa_obs.SetObsMovementType(ApaObsMovementType::STATIC);
      apa_obs.SetBoxGlobal(box);
      apa_obs.SetPolygonGlobal(polygon);
      apa_obs.SetId(obs_id_generate_);
      apa_obs.ClearDecision();
      obstacles_[obs_id_generate_] = apa_obs;
      obs_id_generate_++;
    }
  }

  // limiters
  if (apa_param.GetParam().enable_side_pass_limiter &&
      state_machine_ptr_->IsParkingStatus()) {
    std::vector<Eigen::Vector2d> limiter_points;
    limiter_points.clear();
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

        pnc::geometry_lib::SampleInLineSegment(
            Eigen::Vector2d(limiter->end_points[0].x, limiter->end_points[0].y),
            Eigen::Vector2d(limiter->end_points[1].x, limiter->end_points[1].y),
            0.4, limiter_points);
      }
    }

    Polygon2D polygon;
    cdl::AABB box = cdl::AABB();
    for (int j = 0; j < limiter_points.size(); ++j) {
      box.MergePoint(
          cdl::Vector2r(limiter_points[j].x(), limiter_points[j].y()));
    }
    GeneratePolygonByAABB(&polygon, box);

    ApaObstacle apa_obs;
    apa_obs.SetObsScemanticType(ApaObsScemanticType::LIMITER);
    apa_obs.SetPtClout2dGlobal(limiter_points);
    apa_obs.SetObsAttributeType(ApaObsAttributeType::SLOT_LIMITER);
    apa_obs.SetObsHeightType(ApaObsHeightType::LOW);
    if (!apa_param.GetParam().enable_multi_height_col_det) {
      apa_obs.SetObsHeightType(ApaObsHeightType::HIGH);
    }
    apa_obs.SetObsMovementType(ApaObsMovementType::STATIC);
    apa_obs.SetBoxGlobal(box);
    apa_obs.SetPolygonGlobal(polygon);
    apa_obs.SetId(obs_id_generate_);
    apa_obs.ClearDecision();
    obstacles_[obs_id_generate_] = apa_obs;
    obs_id_generate_++;
  }

  return;
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
  if (type >= iflyauto::OBJECT_TYPE_WATER_SAFETY_BARRIER &&
      type <= iflyauto::OBJECT_TYPE_OCC_NUM) {
    return false;
  }

  return true;
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

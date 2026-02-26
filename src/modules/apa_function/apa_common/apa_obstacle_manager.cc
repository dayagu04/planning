#include "apa_obstacle_manager.h"

#include <Eigen/src/Core/Matrix.h>
#include <bits/stdint-intn.h>

#include <algorithm>
#include <cstddef>
#include <vector>

#include "apa_lon_util.h"
#include "apa_obstacle.h"
#include "apa_param_config.h"
#include "apa_utils.h"
#include "common_c.h"
#include "common_platform_type_soc.h"
#include "debug_info_log.h"
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
    const iflyauto::PlanningOutput* planning_output,
    const ApaStateMachineManager& state_machine_manager,
    const size_t ego_slot_id) {
  if (local_view == nullptr || planning_output == nullptr) {
    ILOG_ERROR << "Update ApaObstacleManager, local_view_ptr is nullptr";
    return;
  }

  const auto& param = apa_param.GetParam();

  if (param.smart_fold_mirror_params.locked_obs_slot_with_fold_mirror &&
      planning_output->rear_view_mirror_signal_command.available &&
      planning_output->rear_view_mirror_signal_command.rear_view_mirror_value ==
          iflyauto::REAR_VIEW_MIRROR_FOLD) {
    JSON_DEBUG_VALUE("locked_obs_slot_with_fold_mirror", true)
    return;
  }
  JSON_DEBUG_VALUE("locked_obs_slot_with_fold_mirror", false)

  ResetSingleFrameObs();

  state_machine_manager_ = state_machine_manager;

  ILOG_INFO << "Update ApaObstacleManager";

  double runover_height = param.runover_height - param.height_redundancy;
  double chassis_height =
      std::min(param.front_overhanging_height, param.rear_overhanging_height) -
      param.height_redundancy;
  double mirror_height = param.mirror_height - param.height_redundancy;
  if (!param.enable_multi_height_col_det) {
    runover_height = -1068.8;
    chassis_height = -1068.8;
    mirror_height = -1068.8;
  } else if (!param.enable_runover_obs) {
    runover_height = -1068.8;
  }

  const iflyauto::ParkingFusionInfo* slot_list =
      &local_view->parking_fusion_info;

  ego_slot_is_parallel_ = false;
  parallel_slot_neigbour_objs_heading_ = {-100, -100};
  for (uint8 i = 0; i < slot_list->parking_fusion_slot_lists_size; i++) {
    last_ego_slot_ = &slot_list->parking_fusion_slot_lists[i];
    // For now, do not consider ego slot limiter.
    // Todo: consider all limiters.
    if (last_ego_slot_->type ==
            iflyauto::ParkingSlotType::PARKING_SLOT_TYPE_HORIZONTAL &&
        state_machine_manager_.IsParkOutStatus() && last_ego_slot_->id != 0) {
      if (slot_list->select_slot_id == last_ego_slot_->id ||
          static_cast<uint32>(ego_slot_id) == last_ego_slot_->id) {
        ego_slot_is_parallel_ = true;
        break;
      }
    }
  }

  // ultrasonic sector distance
  const double min_uss_dist = param.min_uss_origin_dist;
  if (param.is_uss_dist_from_perception) {
    const auto& uss_dis_info_buf =
        local_view->uss_percept_info.dis_from_car_to_obj;

    //  front uss: uss dis need to be transfered from mm to m. order: fl apa, 4
    //  upa, fr apa
    for (const auto& front_uss_idx : param.uss_wdis_index_front) {
      uss_dis_vec_.emplace_back(
          std::max(min_uss_dist, 0.001 * uss_dis_info_buf[front_uss_idx]));
    }

    // rear uss: uss dis need to be transfered from mm to m. order: rr apa, 4
    // upa, rl apa
    for (const auto& rear_uss_idx : param.uss_wdis_index_back) {
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

  // occ pt cloud
  if (param.use_fus_occ_obj) {
    const uint8 fusion_obs_size =
        std::min(local_view->fusion_occupancy_objects_info.fusion_object_size,
                 static_cast<uint8>(FUSION_OCCUPANCY_OBJECTS_MAX_NUM));
    for (uint8 i = 0; i < fusion_obs_size; ++i) {
      const iflyauto::Obstacle& fus_occ_common_info =
          local_view->fusion_occupancy_objects_info.fusion_object[i]
              .common_occupancy_info;

      const iflyauto::FusionOccupancyAdditional& fus_occ_add_info =
          local_view->fusion_occupancy_objects_info.fusion_object[i]
              .additional_occupancy_info;

      const iflyauto::ObjectType obs_type = fus_occ_common_info.type;

      // [hack]: need to retire in published version.
      if (!param.use_fus_occ_column &&
          obs_type == iflyauto::OBJECT_TYPE_OCC_COLUMN) {
        continue;
      }

      const uint32 polygon_points_size =
          std::min(fus_occ_add_info.polygon_points_size,
                   static_cast<uint32>(
                       FUSION_OCCUPANCY_OBJECTS_POLYGON_POINTS_SET_MAX_NUM));

      if (polygon_points_size < 1) {
        continue;
      }

      std::vector<Eigen::Vector2d> fusion_pt_clout_lower_chassis;
      std::vector<Eigen::Vector2d> fusion_pt_clout_lower_mirror;
      std::vector<Eigen::Vector2d> fusion_pt_clout_higher_mirror;
      fusion_pt_clout_lower_chassis.reserve(polygon_points_size);
      fusion_pt_clout_lower_mirror.reserve(polygon_points_size);
      fusion_pt_clout_higher_mirror.reserve(polygon_points_size);

      for (uint32 j = 0; j < polygon_points_size; ++j) {
        const auto& fusion_pt = fus_occ_add_info.polygon_points[j];
        const Eigen::Vector2d fusion_pt_2d(fusion_pt.x, fusion_pt.y);
#if HAVE_3D_OBS_INTERFACE
        if (fusion_pt.z < runover_height) {
          continue;
        } else if (fusion_pt.z < chassis_height) {
          fusion_pt_clout_lower_chassis.emplace_back(std::move(fusion_pt_2d));
        } else if (fusion_pt.z < mirror_height) {
          fusion_pt_clout_lower_mirror.emplace_back(std::move(fusion_pt_2d));
        } else {
          fusion_pt_clout_higher_mirror.emplace_back(std::move(fusion_pt_2d));
        }
#else
        fusion_pt_clout_higher_mirror.emplace_back(std::move(fusion_pt_2d));
#endif
      }

      std::unordered_map<ApaObsHeightType, std::vector<Eigen::Vector2d>>
          occ_pt_map;
      occ_pt_map[ApaObsHeightType::LOW] = fusion_pt_clout_lower_chassis;
      occ_pt_map[ApaObsHeightType::MID] = fusion_pt_clout_lower_mirror;
      occ_pt_map[ApaObsHeightType::HIGH] = fusion_pt_clout_higher_mirror;

      ApaObsMovementType obs_move_type = ApaObsMovementType::STATIC;
      if (param.enable_use_dynamic_obs &&
          (obs_type == iflyauto::OBJECT_TYPE_PEDESTRIAN ||
           obs_type == iflyauto::OBJECT_TYPE_UNKNOWN_MOVABLE ||
           obs_type == iflyauto::OBJECT_TYPE_OCC_PEOPLE ||
           obs_type == iflyauto::OBJECT_TYPE_OCC_GENERAL_DYNAMIC)) {
        obs_move_type = ApaObsMovementType::MOTION;
      }

      size_t obs_parent_id_generate = obs_id_generate_;
      for (const auto& pair : occ_pt_map) {
        if (pair.second.empty()) {
          continue;
        }
        ApaObstacle apa_obs;
        Polygon2D polygon;
        cdl::AABB box = cdl::AABB();

        for (const Eigen::Vector2d& pt : pair.second) {
          box.MergePoint(cdl::Vector2r(pt.x(), pt.y()));
        }
        GeneratePolygonByAABB(&polygon, box);

        apa_obs.SetBoxGlobal(box);
        apa_obs.SetPolygonGlobal(polygon);
        apa_obs.SetObsHeightType(pair.first);
        apa_obs.SetPtClout2dGlobal(pair.second);
        apa_obs.SetObsScemanticType(obs_type);
        apa_obs.SetObsAttributeType(ApaObsAttributeType::FUSION_POINT_CLOUD);
        apa_obs.SetObsMovementType(obs_move_type);
        apa_obs.SetId(obs_id_generate_);
        apa_obs.SetParentId(obs_parent_id_generate);
        apa_obs.ClearDecision();
        obstacles_[obs_id_generate_] = apa_obs;
        obs_id_generate_++;
      }
    }
  }

  if (param.use_object_detect) {
    GenerateObsByOD(local_view, apa_param.GetParam().od_config);
    GenerateObsByODTracking(local_view, apa_param.GetParam().od_config);
  }

  // gl pt cloud
  if (param.use_ground_line) {
    const uint8 ground_lines_size =
        std::min(local_view->ground_line_perception.groundline_size,
                 static_cast<uint8>(FUSION_GROUNDLINE_MAX_NUM));
    for (uint8 i = 0; i < ground_lines_size; ++i) {
      const iflyauto::FusionGroundLine& gl =
          local_view->ground_line_perception.groundline[i];

      if (gl.resource_type == iflyauto::RESOURCE_TYPE_MAP) {
        continue;
      }

      if (!param.use_ground_line_wall_column &&
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

      std::vector<Eigen::Vector2d> gl_pt_clout_lower_chassis;
      std::vector<Eigen::Vector2d> gl_pt_clout_lower_mirror;
      std::vector<Eigen::Vector2d> gl_pt_clout_higher_mirror;
      gl_pt_clout_lower_chassis.reserve(points_3d_size);
      gl_pt_clout_lower_mirror.reserve(points_3d_size);
      gl_pt_clout_higher_mirror.reserve(points_3d_size);

      for (uint8 j = 0; j < points_3d_size; ++j) {
        const auto& gl_pt = gl.groundline_point[j];
        const Eigen::Vector2d gl_pt_2d(gl_pt.x, gl_pt.y);
#if HAVE_3D_OBS_INTERFACE
        if (gl_pt.z < runover_height) {
          continue;
        } else if (gl_pt.z < chassis_height) {
          gl_pt_clout_lower_chassis.emplace_back(std::move(gl_pt_2d));
        } else if (gl_pt.z < mirror_height) {
          gl_pt_clout_lower_mirror.emplace_back(std::move(gl_pt_2d));
        } else {
          gl_pt_clout_higher_mirror.emplace_back(std::move(gl_pt_2d));
        }
#else
        gl_pt_clout_higher_mirror.emplace_back(std::move(gl_pt_2d));
#endif
      }

      std::unordered_map<ApaObsHeightType, std::vector<Eigen::Vector2d>>
          gl_pt_map;
      gl_pt_map[ApaObsHeightType::LOW] = gl_pt_clout_lower_chassis;
      gl_pt_map[ApaObsHeightType::MID] = gl_pt_clout_lower_mirror;
      gl_pt_map[ApaObsHeightType::HIGH] = gl_pt_clout_higher_mirror;

      size_t obs_parent_id_generate = obs_id_generate_;
      for (const auto& pair : gl_pt_map) {
        if (pair.second.empty()) {
          continue;
        }
        ApaObstacle apa_obs;
        Polygon2D polygon;
        cdl::AABB box = cdl::AABB();

        for (const Eigen::Vector2d& pt : pair.second) {
          box.MergePoint(cdl::Vector2r(pt.x(), pt.y()));
        }
        GeneratePolygonByAABB(&polygon, box);

        apa_obs.SetBoxGlobal(box);
        apa_obs.SetPolygonGlobal(polygon);
        apa_obs.SetObsHeightType(pair.first);
        apa_obs.SetPtClout2dGlobal(pair.second);
        apa_obs.SetObsScemanticType(gl.type);
        apa_obs.SetObsAttributeType(
            ApaObsAttributeType::GROUND_LINE_POINT_CLOUD);
        apa_obs.SetObsMovementType(ApaObsMovementType::STATIC);
        apa_obs.SetId(obs_id_generate_);
        apa_obs.SetParentId(obs_parent_id_generate);
        apa_obs.ClearDecision();
        obstacles_[obs_id_generate_] = apa_obs;
        obs_id_generate_++;
      }
    }
  }

  // ultrasonic pt cloud
  if (param.uss_config.use_uss_pt_cloud) {
    GenerateUss(local_view);
  }

  // limiters
  if (param.enable_side_pass_limiter) {
    const iflyauto::ParkingFusionInfo* slot_list =
        &local_view->parking_fusion_info;

    for (uint8 i = 0; i < slot_list->parking_fusion_slot_lists_size; i++) {
      const iflyauto::ParkingFusionSlot* slot =
          &slot_list->parking_fusion_slot_lists[i];
      // For now, do not consider ego slot limiter.
      // Todo: consider all limiters.
      if (slot_list->select_slot_id == slot->id ||
          static_cast<uint32>(ego_slot_id) == slot->id) {
        continue;
      }

      std::vector<Eigen::Vector2d> limiter_points;
      limiter_points.clear();
      for (uint8 j = 0; j < slot->limiters_size; j++) {
        const iflyauto::ParkingFusionLimiter* limiter = &slot->limiters[j];
        pnc::geometry_lib::SampleInLineSegment(
            Eigen::Vector2d(limiter->end_points[0].x, limiter->end_points[0].y),
            Eigen::Vector2d(limiter->end_points[1].x, limiter->end_points[1].y),
            0.4, limiter_points);
      }

      if (limiter_points.empty()) {
        continue;
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
      apa_obs.SetObsMovementType(ApaObsMovementType::STATIC);
      apa_obs.SetBoxGlobal(box);
      apa_obs.SetPolygonGlobal(polygon);
      apa_obs.SetId(obs_id_generate_);
      apa_obs.SetParentId(obs_id_generate_);
      apa_obs.ClearDecision();
      obstacles_[obs_id_generate_] = apa_obs;
      obs_id_generate_++;
    }
  }

  PrintUseObsHeightMethod(param.use_obs_height_method);
  ILOG_INFO << "enable obs height method: "
            << param.enable_multi_height_col_det;

  JSON_DEBUG_VALUE("total_obs_size", obstacles_.size())
  ILOG_INFO << "obstacle size: " << obstacles_.size();

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

const bool ApaObstacleManager::IsOccType(const iflyauto::ObjectType type) {
  if (type >= iflyauto::OBJECT_TYPE_OCC_EMPTY &&
      type <= iflyauto::OBJECT_TYPE_OCC_NUM) {
    return true;
  }

  return false;
}

const bool ApaObstacleManager::NeedTrackingObjectType(
    const iflyauto::ObjectType type) {
  bool is_in_first_range = (type >= iflyauto::OBJECT_TYPE_UNKNOWN &&
                            type <= iflyauto::OBJECT_TYPE_ANIMAL);  // 0-13
  bool is_in_second_range =
      (type >= iflyauto::OBJECT_TYPE_CYCLE_RIDING &&
       type <= iflyauto::OBJECT_TYPE_TRICYCLE_RIDING);  // 18-20
  bool is_in_third_range = (type >= iflyauto::OBJECT_TYPE_SPECIAL_VEHICLE &&
                            type <= iflyauto::OBJECT_TYPE_CHILD);  // 39-49

  return is_in_first_range || is_in_second_range || is_in_third_range;
}

void ApaObstacleManager::GenerateObsByOD(
    const LocalView* local_view, const ObjectDetectObsConfig& od_config) {
  const uint8 fusion_obs_size =
      std::min(local_view->fusion_objects_info.fusion_object_size,
               static_cast<uint8>(FUSION_OBJECT_MAX_NUM));
  for (uint8 i = 0; i < fusion_obs_size; ++i) {
    const iflyauto::Obstacle& obs =
        local_view->fusion_objects_info.fusion_object[i].common_info;
    if (IsOccType(obs.type)) {
      continue;
    }

    double speed = std::sqrt(obs.velocity.x * obs.velocity.x +
                             obs.velocity.y * obs.velocity.y);
    if (IsDynamicODVeh(od_config.moving_veh_speed_thresh, speed, obs.type)) {
      if (!od_config.use_dynamic_od_car) {
        continue;
      }
    } else if (IsODSpecificationer(obs.type)) {
      if (!od_config.use_specificationer) {
        continue;
      }
    } else if (obs.type == iflyauto::OBJECT_TYPE_DECELER) {
      if (!od_config.use_speed_bump) {
        continue;
      }
    } else if (IsODLivingThings(obs.type)) {
      if (!od_config.use_living_things) {
        continue;
      }
    } else {
      if (ego_slot_is_parallel_ && obs.type >= iflyauto::OBJECT_TYPE_COUPE &&
          obs.type <= iflyauto::OBJECT_TYPE_TRAILER) {
        const iflyauto::IFLYLocalization& localization_info =
            local_view->localization;
        Eigen::Vector3d ego_pose;
        ego_pose[0] = localization_info.position.position_boot.x;
        ego_pose[1] = localization_info.position.position_boot.y;
        ego_pose[2] = localization_info.orientation.euler_boot.yaw;
        auto obs_ego_line_heading =
            std::atan2(obs.center_position.y - ego_pose[1],
                       obs.center_position.x - ego_pose[0]);
        bool is_front_car =
            std::abs(obs_ego_line_heading - ego_pose[2]) < M_PI / 2;
        if (is_front_car) {
          auto res =
              CheckParaSlotObsAreNeighbour(obs, last_ego_slot_, ego_pose);
          if (res.first == 0) {
            parallel_slot_neigbour_objs_heading_[0] = res.second;
          }
        }
      }
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

    Polygon2D polygon;
    GeneratePolygonByPoints(global_box, &polygon);
    apa_obs.SetPolygonGlobal(polygon);

    // Sometimes OCC won't send specificationer now, sample od.
    if (IsODSpecificationer(obs.type)) {
      std::vector<Eigen::Vector2d> local_pts, global_pts;
      GeneratePtsByBox(obs.shape.length, obs.shape.width,
                       Eigen::Vector2d(0.0f, 0.0f), local_pts, 0.2);
      LocalPolygonToGlobal(local_pts, center_pose, global_pts);
      apa_obs.SetPtClout2dGlobal(global_pts);
    }

    cdl::AABB box = cdl::AABB();
    GetBoundingBoxByPolygon(&box, &polygon);
    apa_obs.SetBoxGlobal(box);

    pnc::geometry_lib::PathPoint pose(
        Eigen::Vector2d(obs.center_position.x, obs.center_position.y),
        obs.heading_angle);
    apa_obs.SetPose(pose);
    apa_obs.SetSpeed(speed);

    if (IsDynamicODVeh(od_config.moving_veh_speed_thresh, speed, obs.type) ||
        IsDynamicLivingThings(speed, obs.type)) {
      Eigen::Vector2d speed_dir(obs.velocity.x, obs.velocity.y);
      apa_obs.SetSpeedHeading(speed_dir.normalized());
      apa_obs.SetObsMovementType(ApaObsMovementType::MOTION);
    } else {
      apa_obs.SetObsMovementType(ApaObsMovementType::STATIC);
    }

    apa_obs.SetObsAttributeType(ApaObsAttributeType::FUSION_POLYGON);
    apa_obs.SetObsScemanticType(obs.type);
    apa_obs.SetObsHeightType(ApaObsHeightType::HIGH);

    apa_obs.SetId(obs_id_generate_);
    apa_obs.SetParentId(obs_id_generate_);
    apa_obs.ClearDecision();
    obstacles_[obs_id_generate_] = apa_obs;
    obs_id_generate_++;
  }

  return;
}

void ApaObstacleManager::GenerateObsByODTracking(
    const LocalView* local_view, const ObjectDetectObsConfig& od_config) {
  std::unordered_set<size_t> current_local_ids;
  std::unordered_set<size_t> current_fusion_ids;
  const uint8 fusion_obs_size =
      std::min(local_view->fusion_objects_info.fusion_object_size,
               static_cast<uint8>(FUSION_OBJECT_MAX_NUM));
  for (uint8 i = 0; i < fusion_obs_size; ++i) {
    const iflyauto::Obstacle& obs =
        local_view->fusion_objects_info.fusion_object[i].common_info;
    if (!NeedTrackingObjectType(obs.type)) {
      continue;
    }

    double speed = std::sqrt(obs.velocity.x * obs.velocity.x +
                             obs.velocity.y * obs.velocity.y);
    if (IsDynamicODVeh(od_config.moving_veh_speed_thresh, speed, obs.type)) {
      if (!od_config.use_dynamic_od_car) {
        continue;
      }
    } else if (IsODLivingThings(obs.type)) {
      if (!od_config.use_living_things) {
        continue;
      }
    } else {
      if (ego_slot_is_parallel_ && obs.type >= iflyauto::OBJECT_TYPE_COUPE &&
          obs.type <= iflyauto::OBJECT_TYPE_TRAILER) {
        const iflyauto::IFLYLocalization& localization_info =
            local_view->localization;
        Eigen::Vector3d ego_pose;
        ego_pose[0] = localization_info.position.position_boot.x;
        ego_pose[1] = localization_info.position.position_boot.y;
        ego_pose[2] = localization_info.orientation.euler_boot.yaw;
        auto obs_ego_line_heading =
            std::atan2(obs.center_position.y - ego_pose[1],
                       obs.center_position.x - ego_pose[0]);
        bool is_front_car =
            std::abs(obs_ego_line_heading - ego_pose[2]) < M_PI / 2;
        if (is_front_car) {
          auto res =
              CheckParaSlotObsAreNeighbour(obs, last_ego_slot_, ego_pose);
          if (res.first == 0) {
            parallel_slot_neigbour_objs_heading_[0] = res.second;
          }
        }
      }
      continue;
    }

    Pose2D center_pose(obs.center_position.x, obs.center_position.y,
                       obs.heading_angle);

    std::vector<Eigen::Vector2d> local_box;
    GenerateBoundingBox(obs.shape.length, obs.shape.width,
                        Eigen::Vector2d(0.0f, 0.0f), local_box);

    std::vector<Eigen::Vector2d> global_box;
    LocalPolygonToGlobal(local_box, center_pose, global_box);

    size_t fusion_id = obs.id;
    current_fusion_ids.insert(fusion_id);

    size_t local_id;
    auto map_it = fusion_id_to_local_id_.find(fusion_id);
    if (map_it == fusion_id_to_local_id_.end()) {
      local_id = obs_tracking_id_generate_++;
      fusion_id_to_local_id_[fusion_id] = local_id;
    } else {
      local_id = map_it->second;
    }

    current_local_ids.insert(local_id);

    auto it = obstacles_od_tracking_.find(local_id);
    bool is_new = false;
    if (it == obstacles_od_tracking_.end()) {
      ApaObstacle new_obs;
      new_obs.SetId(local_id);
      new_obs.SetParentId(local_id);
      it = obstacles_od_tracking_.emplace(local_id, std::move(new_obs)).first;
      is_new = true;
    }
    ApaObstacle& apa_obs = it->second;

    if (is_new) {
      apa_obs.Reset();
    }

    Polygon2D polygon;
    GeneratePolygonByPoints(global_box, &polygon);
    apa_obs.SetPolygonGlobal(polygon);

    // Sometimes OCC won't send specificationer now, sample od.
    if (IsODSpecificationer(obs.type)) {
      std::vector<Eigen::Vector2d> local_pts, global_pts;
      GeneratePtsByBox(obs.shape.length, obs.shape.width,
                       Eigen::Vector2d(0.0f, 0.0f), local_pts, 0.2);
      LocalPolygonToGlobal(local_pts, center_pose, global_pts);
      apa_obs.SetPtClout2dGlobal(global_pts);
    }

    cdl::AABB box = cdl::AABB();
    GetBoundingBoxByPolygon(&box, &polygon);
    apa_obs.SetBoxGlobal(box);

    pnc::geometry_lib::PathPoint pose(
        Eigen::Vector2d(obs.center_position.x, obs.center_position.y),
        obs.heading_angle);
    apa_obs.SetPose(pose);
    apa_obs.SetSpeed(speed);

    if (IsDynamicODVeh(od_config.moving_veh_speed_thresh, speed, obs.type) ||
        IsDynamicLivingThings(speed, obs.type)) {
      Eigen::Vector2d speed_dir(obs.velocity.x, obs.velocity.y);
      apa_obs.SetSpeedHeading(speed_dir.normalized());
      apa_obs.SetObsMovementType(ApaObsMovementType::MOTION);
    } else {
      apa_obs.SetObsMovementType(ApaObsMovementType::STATIC);
    }

    apa_obs.SetObsAttributeType(ApaObsAttributeType::FUSION_POLYGON);
    apa_obs.SetObsScemanticType(obs.type);
    apa_obs.SetObsHeightType(ApaObsHeightType::HIGH);
    apa_obs.ClearDecision();
  }

  UpdateObstacleODLostFrames(current_local_ids);

  return;
}

void ApaObstacleManager::GenerateUss(const LocalView* local_view) {
  const iflyauto::IFLYLocalization& localization_info =
      local_view->localization;
  Eigen::Vector3d ego_pose;
  ego_pose[0] = localization_info.position.position_boot.x;
  ego_pose[1] = localization_info.position.position_boot.y;
  ego_pose[2] = localization_info.orientation.euler_boot.yaw;

  if (IsActiveApaState(local_view->function_state_machine_info.current_state)) {
    localization_path_.Add(ego_pose);
  } else {
    localization_path_.SetFirstPoint(ego_pose);
  }

  const apa_planner::ApaParameters& config = apa_param.GetParam();
  Polygon2D ego_local;
  double ego_v = std::fabs(localization_info.velocity.velocity_body.vx);
  double lon_dist =
      std::max(config.uss_config.uss_lon_attention_dist,
               config.uss_config.uss_dist_coefficient_by_vel * ego_v);
  GetVehPolygonBy4Edge(
      &ego_local, config.rear_overhanging + lon_dist,
      config.wheel_base + config.front_overhanging + lon_dist,
      config.max_car_width / 2.0 + config.uss_config.uss_lat_attention_dist);

  Polygon2D ego_global;
  std::vector<Polygon2D> path;
  path.reserve(localization_path_.Size());
  Transform2d tf;
  for (int32_t q = 0; q < localization_path_.Size(); q++) {
    tf.SetBasePose(localization_path_.path[q]);
    ULFLocalPolygonToGlobal(&ego_global, &ego_local, tf);
    path.emplace_back(ego_global);
  }

  bool is_contain = false;
  GJK2DInterface gjk_interface;

  int low_type = 0;
  if (!config.enable_multi_height_col_det) {
    low_type = -1;
  }

  const uint8 uss_obs_size = std::min(1, USS_PERCEPTION_OUTLINE_DATAORI_NUM);
  for (uint8 i = 0; i < uss_obs_size; ++i) {
    const iflyauto::ApaSlotOutlineCoordinateDataType& obj_info =
        local_view->uss_percept_info.out_line_dataori[i];

    const uint32 pt_cloud_size =
        std::min(obj_info.obj_pt_cnt,
                 static_cast<uint32>(USS_PERCEPTION_APA_SLOT_OBJ_MAX_NUM));

    if (pt_cloud_size < 1) {
      continue;
    }

    std::vector<Eigen::Vector2d> uss_pt_clout_lower_chassis;
    std::vector<Eigen::Vector2d> uss_pt_clout_lower_mirror;
    std::vector<Eigen::Vector2d> uss_pt_clout_higher_mirror;
    uss_pt_clout_lower_chassis.reserve(pt_cloud_size);
    uss_pt_clout_lower_mirror.reserve(pt_cloud_size);
    uss_pt_clout_higher_mirror.reserve(pt_cloud_size);

    for (uint32 j = 0; j < pt_cloud_size; ++j) {
      const auto& uss_pt = obj_info.obj_pt_global[j];
      const Eigen::Vector2d uss_pt_2d(uss_pt.x, uss_pt.y);

      for (int32_t q = 0; q < path.size(); q++) {
        gjk_interface.PolygonPointCollisionDetect(&path[q], uss_pt_2d,
                                                  &is_contain);

        if (is_contain) {
#if HAVE_3D_OBS_INTERFACE
          const int high_type = obj_info.point_high[j];
          if (high_type == low_type) {
            uss_pt_clout_lower_chassis.emplace_back(std::move(uss_pt_2d));
          } else {
            uss_pt_clout_higher_mirror.emplace_back(std::move(uss_pt_2d));
          }
#else
          uss_pt_clout_higher_mirror.emplace_back(std::move(uss_pt_2d));
#endif
          break;
        }
      }
    }

    std::unordered_map<ApaObsHeightType, std::vector<Eigen::Vector2d>>
        uss_pt_map;
    uss_pt_map[ApaObsHeightType::LOW] = uss_pt_clout_lower_chassis;
    uss_pt_map[ApaObsHeightType::MID] = uss_pt_clout_lower_mirror;
    uss_pt_map[ApaObsHeightType::HIGH] = uss_pt_clout_higher_mirror;

    size_t obs_parent_id_generate_ = obs_id_generate_;
    for (const auto& pair : uss_pt_map) {
      if (pair.second.empty()) {
        continue;
      }

      ApaObstacle apa_obs;
      Polygon2D polygon;
      cdl::AABB box = cdl::AABB();

      for (const Eigen::Vector2d& pt : pair.second) {
        box.MergePoint(cdl::Vector2r(pt.x(), pt.y()));
      }

      GeneratePolygonByAABB(&polygon, box);

      apa_obs.SetBoxGlobal(box);
      apa_obs.SetPolygonGlobal(polygon);
      apa_obs.SetObsHeightType(pair.first);
      apa_obs.SetPtClout2dGlobal(pair.second);
      apa_obs.SetObsScemanticType(ApaObsScemanticType::UNKNOWN);
      apa_obs.SetObsAttributeType(ApaObsAttributeType::USS_POINT_CLOUD);
      apa_obs.SetObsMovementType(ApaObsMovementType::STATIC);
      apa_obs.SetId(obs_id_generate_);
      apa_obs.SetParentId(obs_parent_id_generate_);
      apa_obs.ClearDecision();
      obstacles_[obs_id_generate_] = apa_obs;
      obs_id_generate_++;
    }
  }

  return;
}

std::pair<int, float> ApaObstacleManager::CheckParaSlotObsAreNeighbour(
    iflyauto::Obstacle obs, const iflyauto::ParkingFusionSlot* slot,
    Eigen::Vector3d ego_pose) {
  std::array<Eigen::Vector2d, 4> obs_vex_pts_in_ego;
  std::array<Eigen::Vector2d, 4> obs_vex_pts_in_global;
  obs_vex_pts_in_ego[0] =
      Eigen::Vector2d(obs.shape.length / 2.0, obs.shape.width / 2.0);
  obs_vex_pts_in_ego[1] =
      Eigen::Vector2d(obs.shape.length / 2.0, -obs.shape.width / 2.0);
  obs_vex_pts_in_ego[2] =
      Eigen::Vector2d(-obs.shape.length / 2.0, obs.shape.width / 2.0);
  obs_vex_pts_in_ego[3] =
      Eigen::Vector2d(-obs.shape.length / 2.0, -obs.shape.width / 2.0);
  Eigen::Matrix2d rot_mat;
  rot_mat << cos(obs.heading_angle), -sin(obs.heading_angle),
      sin(obs.heading_angle), cos(obs.heading_angle);
  Eigen::Vector2d obs_center_in_global =
      Eigen::Vector2d(obs.center_position.x, obs.center_position.y);
  obs_vex_pts_in_global[0] =
      obs_center_in_global + rot_mat * obs_vex_pts_in_ego[0];
  obs_vex_pts_in_global[1] =
      obs_center_in_global + rot_mat * obs_vex_pts_in_ego[1];
  obs_vex_pts_in_global[2] =
      obs_center_in_global + rot_mat * obs_vex_pts_in_ego[2];
  obs_vex_pts_in_global[3] =
      obs_center_in_global + rot_mat * obs_vex_pts_in_ego[3];

  std::array<double, 4> d_per_edge = {1.0, 2.5, 1.0, 2.5};
  return CheckParaSlotObsPtsAreNeighbour(obs_vex_pts_in_global, slot,
                                         d_per_edge, ego_pose);
}

std::pair<int, float> ApaObstacleManager::CheckParaSlotObsPtsAreNeighbour(
    std::array<Eigen::Vector2d, 4> obs_vex_pts_in_global,
    const iflyauto::ParkingFusionSlot* slot,
    const std::array<double, 4> d_per_edge, Eigen::Vector3d ego_pose) {
  // 0--1
  // |  |
  // 2--3

  const auto convertToCCW = [](const std::array<Eigen::Vector2d, 4>& original) {
    double original_area = 0.0;
    for (int i = 0; i < 4; ++i) {
      int j = (i + 1) % 4;
      original_area +=
          original[i].x() * original[j].y() - original[j].x() * original[i].y();
    }
    // if (std::fabs(original_area) < 1e-8) {
    //   throw std::invalid_argument(
    //       "Invalid quadrilateral: collinear points or zero area!");
    // }
    std::array<Eigen::Vector2d, 4> pts = original;
    if (original_area < 0) {
    }
    std::swap(pts[2], pts[3]);  // 仅翻转后两点

    double new_area = 0.0;
    for (int i = 0; i < 4; ++i) {
      int j = (i + 1) % 4;
      new_area += pts[i].x() * pts[j].y() - pts[j].x() * pts[i].y();
    }
    // if (new_area < 1e-8) {
    //   throw std::runtime_error(
    //       "Only swapping last two points is insufficient for CCW!");
    // }
    return pts;
  };

  const std::function<std::array<Eigen::Vector2d, 4>(
      const std::array<Eigen::Vector2d, 4>&, const std::array<double, 4>&)>
      translateQuadrilateral = [](const std::array<Eigen::Vector2d, 4>& pts,
                                  const std::array<double, 4>& d_per_edge) {
        Eigen::Vector2d M;
        for (int i = 0; i < pts.size(); ++i) {
          M += pts[i];
        }
        M = M / pts.size();
        Eigen::Vector2d MA = pts[0] - M;
        Eigen::Vector2d MB = pts[1] - M;
        Eigen::Vector2d MC = pts[2] - M;
        Eigen::Vector2d MD = pts[3] - M;
        MA = 2 * MA;
        MB = 2 * MB;
        MC = 2 * MC;
        MD = 2 * MD;
        Eigen::Vector2d new_A = M + MA;
        Eigen::Vector2d new_B = M + MB;
        Eigen::Vector2d new_C = M + MC;
        Eigen::Vector2d new_D = M + MD;

        return std::array<Eigen::Vector2d, 4>{new_A, new_B, new_C, new_D};
      };
  std::array<Eigen::Vector2d, 4> slot_global_pts;
  for (int i = 0; i < 4; ++i) {
    slot_global_pts[i] =
        Eigen::Vector2d(slot->corner_points[i].x, slot->corner_points[i].y);
  }

  const auto isPointInCounterClockwiseQuad =
      [](const std::array<Eigen::Vector2d, 4>& quad,
         const Eigen::Vector2d& a) -> bool {
    std::array<Eigen::Vector2d, 4> edges;
    for (int i = 0; i < 4; ++i) {
      edges[i] = quad[(i + 1) % 4] - quad[i];
    }
    std::array<Eigen::Vector2d, 4> pointToVertex;
    for (int i = 0; i < 4; ++i) {
      pointToVertex[i] = quad[i] - a;
    }

    std::array<Eigen::Vector2d, 4> normals;
    for (int i = 0; i < 4; ++i) {
      normals[i] = Eigen::Vector2d(-edges[i].y(), edges[i].x());
    }

    std::array<double, 4> dots;
    for (int i = 0; i < 4; ++i) {
      dots[i] = normals[i].dot(pointToVertex[i]);
    }

    bool allPositive = true;
    bool allNegative = true;

    for (int i = 0; i < 4; ++i) {
      if (dots[i] < 0) allPositive = false;
      if (dots[i] > 0) allNegative = false;
    }

    return allPositive || allNegative;
  };
  slot_global_pts = convertToCCW(slot_global_pts);
  Eigen::Vector2d slot_01 = slot_global_pts[0] - slot_global_pts[1];
  Eigen::Vector2d ego_pose_unit_vec =
      Eigen::Vector2d(cos(ego_pose[2]), sin(ego_pose[2]));
  if (slot_01.dot(ego_pose_unit_vec) < 0) {
    slot_01 = -slot_01;
  }
  auto box_02 = obs_vex_pts_in_global[0] - obs_vex_pts_in_global[2];
  float delta_heading =
      std::atan2(box_02.y(), box_02.x()) - std::atan2(slot_01.y(), slot_01.x());

  auto translated_slot_pts =
      translateQuadrilateral(slot_global_pts, d_per_edge);
  // translated_slot_pts = convertToCCW(translated_slot_pts);

  if (isPointInCounterClockwiseQuad(translated_slot_pts,
                                    obs_vex_pts_in_global[0]) &&
      isPointInCounterClockwiseQuad(translated_slot_pts,
                                    obs_vex_pts_in_global[1])) {
    return std::pair<int, float>(0, delta_heading);
  }
  if (isPointInCounterClockwiseQuad(translated_slot_pts,
                                    obs_vex_pts_in_global[2]) &&
      isPointInCounterClockwiseQuad(translated_slot_pts,
                                    obs_vex_pts_in_global[3])) {
    return std::pair<int, float>(0, delta_heading);
  }
  return std::pair<int, float>(-1, -1);
}

void ApaObstacleManager::UpdateObstacleODLostFrames(
    const std::unordered_set<size_t>& current_ids) {
  constexpr int kMaxLostFrames = 3;

  for (auto it = obstacles_od_tracking_.begin();
       it != obstacles_od_tracking_.end();) {
    size_t local_id = it->first;

    if (current_ids.count(it->first) == 0) {
      it->second.IncreaseLostFrame();
      if (it->second.LostFrameCount() > kMaxLostFrames) {
        for (auto map_it = fusion_id_to_local_id_.begin();
             map_it != fusion_id_to_local_id_.end();) {
          if (map_it->second == local_id) {
            map_it = fusion_id_to_local_id_.erase(map_it);
          } else {
            ++map_it;
          }
        }
        it = obstacles_od_tracking_.erase(it);
        continue;
      }
    } else {
      it->second.ResetLostFrame();
    }
    ++it;
  }
}

}  // namespace apa_planner
}  // namespace planning

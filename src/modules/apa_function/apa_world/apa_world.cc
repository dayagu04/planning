#include "apa_world.h"

#include <cstdint>
#include <cstdio>
#include <vector>

#include "apa_data.h"
#include "apa_param_config.h"
#include "apa_state_machine_manager.h"
#include "common.pb.h"
#include "common_c.h"
#include "debug_info_log.h"
#include "func_state_machine_c.h"
#include "general_planning_context.h"
#include "geometry_math.h"
#include "log_glog.h"
#include "slot_management_info.pb.h"

namespace planning {
namespace apa_planner {
void ApaWorld::Init() {
  apa_data_ptr_ = std::make_shared<ApaData>();
  measure_data_ptr_ = std::make_shared<ApaMeasureDataManager>();
  state_machine_ptr_ = std::make_shared<ApaStateMachineManager>();
  slot_manager_ptr_ = std::make_shared<SlotManager>();
  uss_obstacle_avoider_ptr_ = std::make_shared<UssObstacleAvoidance>();
  collision_detector_ptr_ = std::make_shared<CollisionDetector>();
  lateral_path_optimizer_ptr_ = std::make_shared<LateralPathOptimizer>();
}

void ApaWorld::Reset() {
  apa_data_ptr_->Reset();
  state_machine_ptr_->Reset();
  slot_manager_ptr_->Reset();
  uss_obstacle_avoider_ptr_->Reset();
  collision_detector_ptr_->Reset();
  lateral_path_optimizer_ptr_->Reset();
  local_view_ptr_ = nullptr;
}

void ApaWorld::Preprocess() {
  apa_data_ptr_->uss_wave_info_ptr = &local_view_ptr_->uss_wave_info;
  apa_data_ptr_->uss_percept_info_ptr = &local_view_ptr_->uss_percept_info;
  apa_data_ptr_->localization_ptr = &local_view_ptr_->localization;
  apa_data_ptr_->vehicle_service_info_ptr =
      &local_view_ptr_->vehicle_service_output_info;
  apa_data_ptr_->parking_slot_ptr = &local_view_ptr_->parking_fusion_info;
  apa_data_ptr_->func_state_ptr = &local_view_ptr_->function_state_machine_info;
  apa_data_ptr_->ground_line_perception_info_ptr =
      &local_view_ptr_->ground_line_perception;
  apa_data_ptr_->fusion_objects_info_ptr =
      &local_view_ptr_->fusion_objects_info;
  apa_data_ptr_->fusion_occupancy_objects_info_ptr =
      &local_view_ptr_->fusion_occupancy_objects_info;
  apa_data_ptr_->local_view_ptr_ = local_view_ptr_;
  apa_data_ptr_->control_output_ptr = &local_view_ptr_->control_output;

  measure_data_ptr_->Update(local_view_ptr_);

  state_machine_ptr_->Update(local_view_ptr_);

  UpdateSlots();

  UpdateObstacles();

  UpdateUssDistance();

  UpdateCarPredictTraj();

  UpdateSlots();
}

void ApaWorld::UpdateSlots() {
  if (apa_data_ptr_->parking_slot_ptr == nullptr) {
    ILOG_INFO << "parking_slot_ptr is nullptr";
    return;
  }
  apa_data_ptr_->apa_slots.Reset();
  for (uint8_t i = 0;
       i < apa_data_ptr_->parking_slot_ptr->parking_fusion_slot_lists_size;
       ++i) {
    const auto& fusion_slot =
        apa_data_ptr_->parking_slot_ptr->parking_fusion_slot_lists[i];
    ApaSlot apa_slot;
    apa_slot.slot_id = fusion_slot.id;
    apa_slot.is_release =
        (fusion_slot.allow_parking == iflyauto::ALLOW_PARKING);
    if (!apa_slot.is_release) {
      apa_slot.slot_occupied_reason = SlotOccupiedReason::FUSION;
      apa_data_ptr_->apa_slots.slots_vec.emplace_back(std::move(apa_slot));
      continue;
    }
    switch (fusion_slot.type) {
      case iflyauto::PARKING_SLOT_TYPE_VERTICAL:
        apa_slot.slot_type = SlotType::PERPENDICULAR;
        break;
      case iflyauto::PARKING_SLOT_TYPE_HORIZONTAL:
        apa_slot.slot_type = SlotType::PARALLEL;
        break;
      case iflyauto::PARKING_SLOT_TYPE_SLANTING:
        apa_slot.slot_type = SlotType::SLANT;
        break;
      default:
        apa_slot.slot_type = SlotType::INVALID;
        apa_slot.is_release = false;
        break;
    }
    switch (fusion_slot.fusion_source) {
      case iflyauto::SLOT_SOURCE_TYPE_ONLY_CAMERA:
        apa_slot.slot_source_type = SlotSourceType::CAMERA;
        break;
      case iflyauto::SLOT_SOURCE_TYPE_ONLY_USS:
        apa_slot.slot_source_type = SlotSourceType::USS;
        break;
      case iflyauto::SLOT_SOURCE_TYPE_CAMERA_USS:
        apa_slot.slot_source_type = SlotSourceType::CAMERA_USS;
        break;
      default:
        apa_slot.slot_source_type = SlotSourceType::INVALID;
        apa_slot.is_release = false;
        break;
    }

    // 上游给的 01必须是库口两个角点 且02必须在一边  13必须在一边
    apa_slot.origin_corner_coord_global.pt_0 << fusion_slot.corner_points[0].x,
        fusion_slot.corner_points[0].y;
    apa_slot.origin_corner_coord_global.pt_1 << fusion_slot.corner_points[1].x,
        fusion_slot.corner_points[1].y;
    apa_slot.origin_corner_coord_global.pt_2 << fusion_slot.corner_points[2].x,
        fusion_slot.corner_points[2].y;
    apa_slot.origin_corner_coord_global.pt_3 << fusion_slot.corner_points[3].x,
        fusion_slot.corner_points[3].y;
    apa_slot.origin_corner_coord_global.CalCenter();

    // 处理一下 对于泊车 02在右边 13在左边
    const Eigen::Vector2d mid_pt_01 =
        (apa_slot.origin_corner_coord_global.pt_0 +
         apa_slot.origin_corner_coord_global.pt_1) *
        0.5;
    const Eigen::Vector2d mid_pt_23 =
        (apa_slot.origin_corner_coord_global.pt_2 +
         apa_slot.origin_corner_coord_global.pt_3) *
        0.5;

    const Eigen::Vector2d slot_mid_vec = mid_pt_01 - mid_pt_23;

    const Eigen::Vector2d mid_pt23_to_pt0_vec =
        apa_slot.origin_corner_coord_global.pt_0 - mid_pt_23;

    const double cross = slot_mid_vec.x() * mid_pt23_to_pt0_vec.y() -
                         slot_mid_vec.y() * mid_pt23_to_pt0_vec.x();

    if (cross > 0.0) {
      std::swap(apa_slot.origin_corner_coord_global.pt_0,
                apa_slot.origin_corner_coord_global.pt_1);
      std::swap(apa_slot.origin_corner_coord_global.pt_2,
                apa_slot.origin_corner_coord_global.pt_3);
    }

    apa_slot.processed_corner_coord_global =
        apa_slot.origin_corner_coord_global;
    if (apa_slot.slot_type == SlotType::SLANT) {
      ILOG_INFO << "slant slot, should postprocess corner to perpendicular";

      const Eigen::Vector2d pt_01_vec =
          apa_slot.origin_corner_coord_global.pt_1 -
          apa_slot.origin_corner_coord_global.pt_0;
      const Eigen::Vector2d pt_01_unit_vec = pt_01_vec.normalized();
      const Eigen::Vector2d pt_02_vec =
          apa_slot.origin_corner_coord_global.pt_2 -
          apa_slot.origin_corner_coord_global.pt_0;
      const Eigen::Vector2d pt_02_unit_vec = pt_02_vec.normalized();
      const Eigen::Vector2d pt_13_vec =
          apa_slot.origin_corner_coord_global.pt_3 -
          apa_slot.origin_corner_coord_global.pt_1;
      const Eigen::Vector2d pt_13_unit_vec = pt_13_vec.normalized();

      const double cos_theta = pt_01_unit_vec.dot(pt_02_unit_vec);

      if (cos_theta > 0.0) {
        // toward right
        const double dis_0_0dot = pt_01_vec.dot(pt_02_unit_vec);
        const Eigen::Vector2d pt_0dot =
            apa_slot.origin_corner_coord_global.pt_0 +
            dis_0_0dot * pt_02_unit_vec;
        const double dist_0dot_2 = pt_02_vec.norm() - dis_0_0dot;
        const Eigen::Vector2d pt_3dot =
            apa_slot.origin_corner_coord_global.pt_1 +
            dist_0dot_2 * pt_02_unit_vec;
        apa_slot.processed_corner_coord_global.pt_0 = pt_0dot;
        apa_slot.processed_corner_coord_global.pt_3 = pt_3dot;
      } else {
        // toward left
        const Eigen::Vector2d pt_10_vec = -pt_01_vec;
        const double dist_1_1dot = pt_10_vec.dot(pt_13_unit_vec);
        const Eigen::Vector2d pt_1dot =
            apa_slot.origin_corner_coord_global.pt_1 +
            dist_1_1dot * pt_13_unit_vec;
        const double dist_1dot_3 = pt_13_vec.norm() - dist_1_1dot;
        const Eigen::Vector2d pt_2dot =
            apa_slot.origin_corner_coord_global.pt_0 +
            dist_1dot_3 * pt_13_unit_vec;
        apa_slot.processed_corner_coord_global.pt_1 = pt_1dot;
        apa_slot.processed_corner_coord_global.pt_2 = pt_2dot;
      }
    }
    apa_slot.processed_corner_coord_global.CalCenter();

    if (fusion_slot.limiters_size > 0) {
      apa_slot.limiter.valid = true;
      if (fusion_slot.limiters_size == 1) {
        apa_slot.limiter.start_pt << fusion_slot.limiters[0].end_points[0].x,
            fusion_slot.limiters[0].end_points[0].y;
        apa_slot.limiter.end_pt << fusion_slot.limiters[0].end_points[1].x,
            fusion_slot.limiters[0].end_points[1].y;
      } else if (fusion_slot.limiters_size == 2) {
        apa_slot.limiter.start_pt
            << 0.5 * (fusion_slot.limiters[0].end_points[0].x +
                      fusion_slot.limiters[0].end_points[1].x),
            0.5 * (fusion_slot.limiters[0].end_points[0].y +
                   fusion_slot.limiters[0].end_points[1].y);
        apa_slot.limiter.end_pt
            << 0.5 * (fusion_slot.limiters[1].end_points[0].x +
                      fusion_slot.limiters[1].end_points[1].x),
            0.5 * (fusion_slot.limiters[1].end_points[0].y +
                   fusion_slot.limiters[1].end_points[1].y);
      }
    }
    apa_data_ptr_->apa_slots.slots_vec.emplace_back(std::move(apa_slot));
  }

  apa_data_ptr_->apa_slots.slot_size =
      apa_data_ptr_->apa_slots.slots_vec.size();
}

void ApaWorld::UpdateUssDistance() {}
void ApaWorld::UpdateObstacles() {
  apa_data_ptr_->apa_obs_map.clear();
  UpdateFuisonObs();
  UpdateGroundLineObs();
  UpdateUssObs();
}

void ApaWorld::UpdateFuisonObs() {
  const bool use_fus_occ_obj = apa_param.GetParam().use_fus_occ_obj;
  if (use_fus_occ_obj &&
      apa_data_ptr_->fusion_occupancy_objects_info_ptr == nullptr) {
    ILOG_INFO << "fusion_occ_objects_info_ptr is nullptr";
    return;
  }
  if (!use_fus_occ_obj && apa_data_ptr_->fusion_objects_info_ptr == nullptr) {
    ILOG_INFO << "fusion_objects_info_ptr is nullptr";
    return;
  }

  uint8 fusion_object_num;
  if (use_fus_occ_obj) {
    fusion_object_num =
        apa_data_ptr_->fusion_occupancy_objects_info_ptr->fusion_object_size;
  } else {
    fusion_object_num =
        apa_data_ptr_->fusion_objects_info_ptr->fusion_object_size;
  }

  if (fusion_object_num == 0) {
    ILOG_INFO << "fusion objects is empty";
    return;
  }

  std::vector<Eigen::Vector2d> obs_pt_vec;
  // Assuming an object has a maximum of 66 obstacle points
  obs_pt_vec.reserve(fusion_object_num * 66);

  Eigen::Vector2d fs_pt;
  if (use_fus_occ_obj) {
    iflyauto::FusionOccupancyAdditional fusion_occupancy_object;
    for (uint8 i = 0;
         i < std::min(fusion_object_num,
                      static_cast<uint8>(FUSION_OCCUPANCY_OBJECT_MAX_NUM));
         ++i) {
      fusion_occupancy_object =
          apa_data_ptr_->fusion_occupancy_objects_info_ptr->fusion_object[i]
              .additional_occupancy_info;
      for (uint32 j = 0;
           j < std::min(fusion_occupancy_object.polygon_points_size,
                        static_cast<uint32>(
                            FUSION_OCCUPANCY_OBJECTS_POLYGON_POINTS_SET_NUM));
           ++j) {
        fs_pt << fusion_occupancy_object.polygon_points[j].x,
            fusion_occupancy_object.polygon_points[j].y;
        obs_pt_vec.emplace_back(fs_pt);
      }
    }
  } else {
    iflyauto::FusionObjectsAdditional fusion_object;
    for (uint8 i = 0; i < std::min(fusion_object_num,
                                   static_cast<uint8>(FUSION_OBJECT_MAX_NUM));
         ++i) {
      fusion_object = apa_data_ptr_->fusion_objects_info_ptr->fusion_object[i]
                          .additional_info;
      for (uint32 j = 0;
           j <
           std::min(fusion_object.polygon_points_size,
                    static_cast<uint8>(FUSION_OBJECTS_POLYGON_POINTS_SET_NUM));
           ++j) {
        fs_pt << fusion_object.polygon_points[j].x,
            fusion_object.polygon_points[j].y;
        obs_pt_vec.emplace_back(fs_pt);
      }
    }
  }

  apa_data_ptr_->apa_obs_map[ObstacleType::FUSION] = obs_pt_vec;

  ILOG_INFO << "fusion objects size = " << obs_pt_vec.size();

  return;
}

void ApaWorld::UpdateGroundLineObs() {
  if (apa_data_ptr_->ground_line_perception_info_ptr == nullptr) {
    ILOG_INFO << "ground_line_perception_info_ptr is nullptr";
    return;
  }

  const uint8_t ground_lines_size =
      apa_data_ptr_->ground_line_perception_info_ptr->ground_lines_size;

  if (ground_lines_size == 0) {
    ILOG_INFO << "ground line is empty";
    return;
  }

  std::vector<Eigen::Vector2d> obs_pt_vec;
  // Assuming an object has a maximum of 33 obstacle points
  obs_pt_vec.reserve(ground_lines_size * 66);

  Eigen::Vector2d gl_pt;
  iflyauto::GroundLine gl;
  for (uint8_t i = 0;
       i < std::min(ground_lines_size, static_cast<uint8>(GROUND_LINES_NUM));
       ++i) {
    gl = apa_data_ptr_->ground_line_perception_info_ptr->ground_lines[i];
    for (uint8 j = 0; j < std::min(gl.points_3d_size,
                                   static_cast<uint8>(GROUND_LINE_POINTS_NUM));
         ++j) {
      gl_pt << gl.points_3d[j].x, gl.points_3d[j].y;
      obs_pt_vec.emplace_back(gl_pt);
    }
  }

  apa_data_ptr_->apa_obs_map[ObstacleType::GROUND_LINE] = obs_pt_vec;

  ILOG_INFO << "ground line objects size = " << obs_pt_vec.size();

  return;
}

void ApaWorld::UpdateUssObs() {
  if (apa_data_ptr_->uss_percept_info_ptr == nullptr) {
    ILOG_INFO << "uss_percept_info_ptr is empty";
    return;
  }

  const auto& obj_info_desample =
      apa_data_ptr_->uss_percept_info_ptr
          ->out_line_dataori[0];  // 0 means desample while 1 means raw model
                                  // output

  const uint32 uss_pt_num = obj_info_desample.obj_pt_cnt;

  if (uss_pt_num == 0) {
    ILOG_INFO << "uss obs is empty";
    return;
  }

  std::vector<Eigen::Vector2d> obs_pt_vec;
  obs_pt_vec.reserve(uss_pt_num);

  Eigen::Vector2d uss_pt;
  for (uint32 i = 0;
       i < std::min(uss_pt_num, static_cast<uint32>(NUM_OF_APA_SLOT_OBJ));
       ++i) {
    uss_pt << obj_info_desample.obj_pt_global[i].x,
        obj_info_desample.obj_pt_global[i].y;
    obs_pt_vec.emplace_back(uss_pt);
  }

  apa_data_ptr_->apa_obs_map[ObstacleType::USS] = obs_pt_vec;

  ILOG_INFO << "uss objects size = " << obs_pt_vec.size();

  return;
}

void ApaWorld::UpdateCarPredictTraj() {
  // 填充车辆基于自车方向盘转角或者控制MPC的预测轨迹，
  // 用于后续的碰撞检测和速度规划
  const double predict_distance = 3.0;

  bool use_steer_angle_flag = false;

  CarPredictTraj& car_predict_traj = apa_data_ptr_->car_predict_traj;
  car_predict_traj.Reset();
  std::vector<pnc::geometry_lib::PathPoint>& car_predict_pt_vec =
      car_predict_traj.car_predict_pt_vec;
  // 如果没有planning_output 那么没有预测轨迹
  if (apa_data_ptr_->plan_output_ptr == nullptr) {
    return;
  }
  if (apa_data_ptr_->plan_output_ptr->planning_status.apa_planning_status !=
      iflyauto::APA_IN_PROGRESS) {
    return;
  }
  if (apa_data_ptr_->control_output_ptr == nullptr ||
      apa_data_ptr_->control_output_ptr->control_trajectory
              .control_result_points_size < 1) {
    use_steer_angle_flag = true;
  }

  if (!use_steer_angle_flag) {
    car_predict_pt_vec.emplace_back(pnc::geometry_lib::PathPoint(
        measure_data_ptr_->GetPos(), measure_data_ptr_->GetHeading()));
    pnc::geometry_lib::LocalToGlobalTf l2g_tf;
    l2g_tf.Init(measure_data_ptr_->GetPos(), measure_data_ptr_->GetHeading());
    const size_t control_result_points_size =
        apa_data_ptr_->control_output_ptr->control_trajectory
            .control_result_points_size;
    const auto control_result_points =
        apa_data_ptr_->control_output_ptr->control_trajectory
            .control_result_points;
    for (size_t i = 0; i < control_result_points_size; ++i) {
      const auto& pt = control_result_points[i];
      pnc::geometry_lib::PathPoint car_predict_pt;
      car_predict_pt.pos = l2g_tf.GetPos(Eigen::Vector2d(pt.x, pt.y));
      car_predict_pt.heading =
          pnc::geometry_lib::NormalizeAngle(l2g_tf.GetHeading(pt.z));

      const double ds = std::hypot(
          car_predict_pt.pos.x() - car_predict_pt_vec.back().pos.x(),
          car_predict_pt.pos.y() - car_predict_pt_vec.back().pos.y());

      const double dphi = pnc::geometry_lib::NormalizeAngle(
          car_predict_pt.heading - car_predict_pt_vec.back().heading);

      if (ds > 0.5 || dphi * kRad2Deg > 5.1) {
        ILOG_INFO << "control output is err";
        car_predict_pt_vec.clear();
        break;
      }

      if (ds < 1e-2) {
        continue;
      }

      car_predict_pt.s = ds + car_predict_pt_vec.back().s;
      car_predict_pt_vec.emplace_back(std::move(car_predict_pt));
    }

    if (car_predict_pt_vec.back().s < predict_distance) {
      const pnc::geometry_lib::PathPoint& last_car_predict_pt =
          car_predict_pt_vec.back();
      double min_dist = std::numeric_limits<double>::infinity();
      int min_index = -1;
      for (int i = 0;
           i <
           apa_data_ptr_->plan_output_ptr->trajectory.trajectory_points_size;
           ++i) {
        const iflyauto::TrajectoryPoint& planning_pt =
            apa_data_ptr_->plan_output_ptr->trajectory.trajectory_points[i];
        const double dist =
            std::hypot(planning_pt.x - last_car_predict_pt.pos.x(),
                       planning_pt.y - last_car_predict_pt.pos.y());
        if (dist < min_dist) {
          min_index = i;
          min_dist = dist;
        }
      }

      for (int i = min_index + 1;
           i < apa_data_ptr_->plan_output_ptr->trajectory
                   .trajectory_points_size &&
           car_predict_pt_vec.back().s < predict_distance;
           ++i) {
        pnc::geometry_lib::PathPoint car_predict_pt;
        car_predict_pt.pos.x() =
            apa_data_ptr_->plan_output_ptr->trajectory.trajectory_points[i].x;
        car_predict_pt.pos.y() =
            apa_data_ptr_->plan_output_ptr->trajectory.trajectory_points[i].y;
        car_predict_pt.heading =
            apa_data_ptr_->plan_output_ptr->trajectory.trajectory_points[i]
                .heading_yaw;

        const double ds = std::hypot(
            car_predict_pt.pos.x() - car_predict_pt_vec.back().pos.x(),
            car_predict_pt.pos.y() - car_predict_pt_vec.back().pos.y());

        car_predict_pt.s = ds + car_predict_pt_vec.back().s;
        car_predict_pt_vec.emplace_back(std::move(car_predict_pt));
      }
    }
  }

  if (car_predict_pt_vec.size() < 2) {
    car_predict_traj.Reset();
    const double steer_angle = measure_data_ptr_->GetSteerWheelAngle();
    const uint8_t gear =
        (apa_data_ptr_->plan_output_ptr->gear_command.gear_command_value ==
         iflyauto::GEAR_COMMAND_VALUE_DRIVE)
            ? pnc::geometry_lib::SEG_GEAR_DRIVE
            : pnc::geometry_lib::SEG_GEAR_REVERSE;
    pnc::geometry_lib::PathSegment path_seg;
    if (std::fabs(steer_angle) < 2.68 * kDeg2Rad) {
      pnc::geometry_lib::CalLineFromPt(
          gear, predict_distance,
          pnc::geometry_lib::PathPoint(measure_data_ptr_->GetPos(),
                                       measure_data_ptr_->GetHeading()),
          path_seg);
    } else {
      const auto& front_wheel_angle =
          std::fabs(measure_data_ptr_->GetFrontWheelAngle());
      const double rear_axle_center_turn_radius =
          1.0 / (apa_param.GetParam().c1 * std::tan(front_wheel_angle));
      const uint8_t steer = (steer_angle > 0.0)
                                ? pnc::geometry_lib::SEG_STEER_LEFT
                                : pnc::geometry_lib::SEG_STEER_RIGHT;
      pnc::geometry_lib::CalArcFromPt(
          gear, steer, predict_distance, rear_axle_center_turn_radius,
          pnc::geometry_lib::PathPoint(measure_data_ptr_->GetPos(),
                                       measure_data_ptr_->GetHeading()),
          path_seg);
    }

    pnc::geometry_lib::SamplePointSetInPathSeg(car_predict_pt_vec, path_seg,
                                               0.4);
  }
}

const bool ApaWorld::Update() {
  if (local_view_ptr_ == nullptr) {
    ILOG_INFO << "-- apa_world: local view ptr is nullptr, err ---";
    return false;
  }
  ILOG_INFO << "-- apa_world: run preprocess ---";
  Preprocess();

  ILOG_INFO << "func_state = "
            << static_cast<int>(apa_data_ptr_->func_state_ptr->current_state);

  // only for hack if outter machine no reset, need delete
  if (state_machine_ptr_->IsSeachingStatus()) {
    apa_data_ptr_->is_slot_type_fixed = false;
    apa_data_ptr_->slot_id = 0;
    apa_data_ptr_->slot_type = Common::PARKING_SLOT_TYPE_INVALID;
  }

  // run slot manager
  // currently path planning starts once id is selected in searching state
  ILOG_INFO << "-- apa_world: run slot_management ---";
  if (!slot_manager_ptr_->Update(apa_data_ptr_, state_machine_ptr_,
                                 measure_data_ptr_)) {
    ILOG_INFO << "shouldn't have entered the parking function at that time";
  }

  if (!apa_data_ptr_->is_slot_type_fixed) {
    // TODO: selected slot (slot_type) should be obtained in slot management
    apa_data_ptr_->slot_type = slot_manager_ptr_->GetEgoSlotInfo().slot_type;
    apa_data_ptr_->slot_id = slot_manager_ptr_->GetEgoSlotInfo().select_slot_id;
    apa_data_ptr_->is_slot_type_fixed = true;
  }

  ILOG_INFO << "current slot type in slm ="
            << static_cast<int>(slot_manager_ptr_->GetEgoSlotInfo().slot_type);
  ILOG_INFO << "fixed slot type ="
            << static_cast<int>(apa_data_ptr_->slot_type);

  return true;
}

const bool ApaWorld::Update(const LocalView* local_view,
                            const iflyauto::PlanningOutput& planning_output) {
  local_view_ptr_ = local_view;
  apa_data_ptr_->plan_output_ptr = &planning_output;
  return Update();
}

}  // namespace apa_planner
}  // namespace planning

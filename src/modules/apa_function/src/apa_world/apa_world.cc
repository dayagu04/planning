#include "apa_world.h"

#include <cstdint>
#include <cstdio>
#include <vector>

#include "apa_data.h"
#include "apa_param_setting.h"
#include "common.pb.h"
#include "common_c.h"
#include "debug_info_log.h"
#include "func_state_machine_c.h"
#include "general_planning_context.h"
#include "geometry_math.h"
#include "log_glog.h"
#include "slot_management_info.pb.h"
#include "src/library/hybrid_astar_lib/astar_scheduler.h"

namespace planning {
namespace apa_planner {
void ApaWorld::Init() {
  apa_data_ptr_ = std::make_shared<ApaData>();
  slot_manager_ptr_ = std::make_shared<SlotManagement>();
  uss_obstacle_avoider_ptr_ = std::make_shared<UssObstacleAvoidance>();
  collision_detector_ptr_ = std::make_shared<CollisionDetector>();
  lateral_path_optimizer_ptr_ = std::make_shared<LateralPathOptimizer>();
}

void ApaWorld::Reset() {
  apa_data_ptr_->Reset();
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

  UpdateEgoState();

  UpdateStateMachine();

  UpdateParkOutDirection();

  UpdateSlots();

  UpdateObstacles();

  UpdateUssDistance();
}

void ApaWorld::UpdateEgoState() {
  MeasurementData& measurement_data = apa_data_ptr_->measurement_data;

  measurement_data.steer_wheel_angle =
      apa_data_ptr_->vehicle_service_info_ptr->steering_wheel_angle;

  if (apa_data_ptr_->vehicle_service_info_ptr->brake_pedal_pressed_available &&
      apa_data_ptr_->vehicle_service_info_ptr->brake_pedal_pressed) {
    measurement_data.brake_flag = true;
  } else {
    measurement_data.brake_flag = false;
  }

  const auto& pose = apa_data_ptr_->localization_ptr->position.position_boot;

  const Eigen::Vector2d current_pos(pose.x, pose.y);

  const ApaParameters& param = apa_param.GetParam();
  // calculate standstill time by pos
  const double local_move_dist = (measurement_data.pos - current_pos).norm();

  if (local_move_dist < param.car_static_pos_err_strict) {
    measurement_data.car_static_timer_by_pos_strict += param.plan_time;
  } else {
    measurement_data.car_static_timer_by_pos_strict = 0.0;
  }
  if (local_move_dist < param.car_static_pos_err_normal) {
    measurement_data.car_static_timer_by_pos_normal += param.plan_time;
  } else {
    measurement_data.car_static_timer_by_pos_normal = 0.0;
  }

  measurement_data.pos = current_pos;
  measurement_data.heading =
      apa_data_ptr_->localization_ptr->orientation.euler_boot.yaw;
  measurement_data.heading_vec << std::cos(measurement_data.heading),
      std::sin(measurement_data.heading);

  const Eigen::Vector2d heading_vec_turn_right(
      measurement_data.heading_vec.y(), -measurement_data.heading_vec.x());

  const Eigen::Vector2d heading_vec_turn_left(-measurement_data.heading_vec.y(),
                                              measurement_data.heading_vec.x());

  measurement_data.right_mirror_pos =
      measurement_data.pos +
      param.lon_dist_mirror_to_rear_axle * measurement_data.heading_vec +
      param.lat_dist_mirror_to_center * heading_vec_turn_right;

  measurement_data.left_mirror_pos =
      measurement_data.pos +
      param.lon_dist_mirror_to_rear_axle * measurement_data.heading_vec +
      param.lat_dist_mirror_to_center * heading_vec_turn_left;

  // measurement_data.vel_ego =
  //     local_view_ptr_->vehicle_service_output_info.vehicle_speed();

  measurement_data.vel =
      apa_data_ptr_->localization_ptr->velocity.velocity_body.vx;

  // calculate standstill time by velocity
  if (std::fabs(measurement_data.vel) < param.car_static_velocity_strict) {
    measurement_data.car_static_timer_by_vel_strict += param.plan_time;
  } else {
    measurement_data.car_static_timer_by_vel_strict = 0.0;
  }
  if (std::fabs(measurement_data.vel) < param.car_static_velocity_normal) {
    measurement_data.car_static_timer_by_vel_normal += param.plan_time;
  } else {
    measurement_data.car_static_timer_by_vel_normal = 0.0;
  }

  // static flag
  measurement_data.static_flag =
      (measurement_data.car_static_timer_by_pos_strict >
           param.car_static_keep_time_by_pos_strict ||
       measurement_data.car_static_timer_by_pos_normal >
           param.car_static_keep_time_by_pos_normal) &&
      (measurement_data.car_static_timer_by_vel_strict >
           param.car_static_keep_time_by_vel_strict ||
       measurement_data.car_static_timer_by_vel_normal >
           param.car_static_keep_time_by_vel_normal);

  JSON_DEBUG_VALUE("local_move_dist", local_move_dist)
  JSON_DEBUG_VALUE("local_vel", measurement_data.vel)
  JSON_DEBUG_VALUE("car_static_timer_by_pos_strict",
                   measurement_data.car_static_timer_by_pos_strict)
  JSON_DEBUG_VALUE("car_static_timer_by_pos_normal",
                   measurement_data.car_static_timer_by_pos_normal)
  JSON_DEBUG_VALUE("car_static_timer_by_vel_strict",
                   measurement_data.car_static_timer_by_vel_strict)
  JSON_DEBUG_VALUE("car_static_timer_by_vel_normal",
                   measurement_data.car_static_timer_by_vel_normal)
  JSON_DEBUG_VALUE("static_flag", measurement_data.static_flag)
}

void ApaWorld::UpdateStateMachine() {
  ApaStateMachine& cur_state = apa_data_ptr_->cur_state;
  const uint8_t state = apa_data_ptr_->func_state_ptr->current_state;
  apa_data_ptr_->current_state = state;

  cur_state = ApaStateMachine::INVALID;
  if (state == iflyauto::FunctionalState_PARK_STANDBY) {
    cur_state = ApaStateMachine::INVALID;
  }

  if (state == iflyauto::FunctionalState_PARK_SUSPEND) {
    cur_state = ApaStateMachine::SUSPEND;
  }

  if (state == iflyauto::FunctionalState_PARK_COMPLETED) {
    cur_state = ApaStateMachine::COMPLETE;
  }

  if (state == iflyauto::FunctionalState_PARK_IN_SEARCHING) {
    cur_state = ApaStateMachine::SEARCH_IN;
    apa_data_ptr_->apa_function = ApaFunction::PARK_IN;
  }

  if (state == iflyauto::FunctionalState_PARK_OUT_SEARCHING) {
    cur_state = ApaStateMachine::SEARCH_OUT;
    apa_data_ptr_->apa_function = ApaFunction::PARK_OUT;
  }

  if (state == iflyauto::FunctionalState_PARK_GUIDANCE) {
    if (apa_data_ptr_->apa_function == ApaFunction::PARK_IN) {
      cur_state = ApaStateMachine::ACTIVE_IN;
    } else if (apa_data_ptr_->apa_function == ApaFunction::PARK_OUT) {
      cur_state = ApaStateMachine::ACTIVE_OUT;
    }
  }

  if (state == iflyauto::FunctionalState_PARK_OUT_SEARCHING) {
    cur_state = ApaStateMachine::SEARCH_OUT;
    apa_data_ptr_->apa_function = ApaFunction::PARK_OUT;
  }
}

void ApaWorld::UpdateParkOutDirection() {
  const iflyauto::ApaParkOutDirection park_out_direction =
      apa_data_ptr_->func_state_ptr->parking_req.apa_park_out_direction;

  ApaParkingOutDirection& out_dir = apa_data_ptr_->park_out_direction;

  if (park_out_direction == iflyauto::PRK_OUT_TO_FRONT_LEFT_CROSS ||
      park_out_direction == iflyauto::PRK_OUT_TO_FRONT_LEFT_PARALLEL) {
    out_dir = ApaParkingOutDirection::LEFT_FRONT;
  } else if (park_out_direction == iflyauto::PRK_OUT_TO_FRONT_RIGHT_CROSS ||
             park_out_direction == iflyauto::PRK_OUT_TO_FRONT_RIGHT_PARALLEL) {
    out_dir = ApaParkingOutDirection::RIGHT_FRONT;
  } else if (park_out_direction == iflyauto::PRK_OUT_TO_BACK_LEFT_CROSS) {
    out_dir = ApaParkingOutDirection::LEFT_REAR;
  } else if (park_out_direction == iflyauto::PRK_OUT_TO_BACK_RIGHT_CROSS) {
    out_dir = ApaParkingOutDirection::RIGHT_REAR;
  } else if (park_out_direction == iflyauto::PRK_OUT_TO_FRONT_OUT) {
    out_dir = ApaParkingOutDirection::FRONT;
  } else if (park_out_direction == iflyauto::PRK_OUT_TO_BACK_OUT) {
    out_dir = ApaParkingOutDirection::REAR;
  } else {
    out_dir = ApaParkingOutDirection::INVALID;
  }
}

void ApaWorld::UpdateSlots() {}
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
    for (uint8 i = 0; i < fusion_object_num; ++i) {
      fusion_occupancy_object =
          apa_data_ptr_->fusion_occupancy_objects_info_ptr->fusion_object[i]
              .additional_occupancy_info;
      for (uint32 j = 0; j < fusion_occupancy_object.polygon_points_size; ++j) {
        fs_pt << fusion_occupancy_object.polygon_points[j].x,
            fusion_occupancy_object.polygon_points[j].y;
        obs_pt_vec.emplace_back(fs_pt);
      }
    }
  } else {
    iflyauto::FusionObjectsAdditional fusion_object;
    for (uint8 i = 0; i < fusion_object_num; ++i) {
      fusion_object = apa_data_ptr_->fusion_objects_info_ptr->fusion_object[i]
                          .additional_info;
      for (uint32 j = 0; j < fusion_object.polygon_points_size; ++j) {
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
  for (uint8_t i = 0; i < ground_lines_size; ++i) {
    gl = apa_data_ptr_->ground_line_perception_info_ptr->ground_lines[i];
    for (uint8 j = 0; j < gl.points_3d_size; ++j) {
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
  for (uint32 i = 0; i < uss_pt_num; ++i) {
    uss_pt << obj_info_desample.obj_pt_global[i].x,
        obj_info_desample.obj_pt_global[i].y;
    obs_pt_vec.emplace_back(uss_pt);
  }

  apa_data_ptr_->apa_obs_map[ObstacleType::USS] = obs_pt_vec;

  ILOG_INFO << "uss objects size = " << obs_pt_vec.size();

  return;
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

  PrintApaStateMachine(apa_data_ptr_->cur_state);

  PrintApaParkingOutDirection(apa_data_ptr_->park_out_direction);

  apa_data_ptr_->planner_type = ApaPlannerType::INVALID_PLANNER;

  // only for hack if outter machine no reset
  if (apa_data_ptr_->cur_state == ApaStateMachine::SEARCH_IN ||
      apa_data_ptr_->cur_state == ApaStateMachine::SEARCH_OUT) {
    apa_data_ptr_->is_slot_type_fixed = false;
    apa_data_ptr_->slot_id = 0;
    apa_data_ptr_->slot_type = Common::PARKING_SLOT_TYPE_INVALID;
  }

  // run slot manager
  // currently path planning starts once id is selected in searching state

  ILOG_INFO << "-- apa_world: run slot_management ---";
  if (!slot_manager_ptr_->Update(apa_data_ptr_)) {
    ILOG_INFO << "shouldn't have entered the parking function at that time";
    return false;
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

  if (apa_data_ptr_->cur_state == ApaStateMachine::ACTIVE_WAIT_IN ||
      apa_data_ptr_->cur_state == ApaStateMachine::ACTIVE_IN) {
    if (apa_data_ptr_->slot_type ==
        Common::ParkingSlotType::PARKING_SLOT_TYPE_VERTICAL) {
      if (apa_param.GetParam().path_generator_type ==
          ParkPathGenerationType::GEOMETRY_BASED) {
        if (apa_data_ptr_->apa_parking_direction ==
            ApaParkingDirection::FRONT_END_PARKING_DIRECTION) {
          apa_data_ptr_->planner_type =
              ApaPlannerType::PERPENDICULAR_PARK_HEADING_IN_PLANNER;

          ILOG_INFO << "planner_type = PERPENDICULAR_PARK_HEADING_IN!";
        } else {
          apa_data_ptr_->planner_type =
              ApaPlannerType::PERPENDICULAR_PARK_IN_PLANNER;

          AstarScheduler* astar_scheduler = AstarScheduler::GetAstarScheduler();
          if (astar_scheduler->IsNeedAstarSearch()) {
            apa_data_ptr_->planner_type = ApaPlannerType::HYBRID_ASTAR_PLANNER;
          }

          ILOG_INFO << "planner_type = PERPENDICULAR_PARK_IN!";
        }
      } else {
        apa_data_ptr_->planner_type = ApaPlannerType::HYBRID_ASTAR_PLANNER;
      }
      ILOG_INFO << "path plan method = "
                << static_cast<int>(apa_data_ptr_->planner_type);

    } else if (apa_data_ptr_->slot_type ==
               Common::ParkingSlotType::PARKING_SLOT_TYPE_HORIZONTAL) {
      ILOG_INFO << "planner_type = PARALLEL_PARK_IN!";
      if (apa_param.GetParam().path_generator_type ==
          ParkPathGenerationType::GEOMETRY_BASED) {
        apa_data_ptr_->planner_type = ApaPlannerType::PARALLEL_PARK_IN_PLANNER;

      } else {
        apa_data_ptr_->planner_type = ApaPlannerType::HYBRID_ASTAR_PLANNER;
      }
    } else if (apa_data_ptr_->slot_type ==
               Common::ParkingSlotType::PARKING_SLOT_TYPE_SLANTING) {
      ILOG_INFO << "planner_type = SLANT_PARK_IN!";
      apa_data_ptr_->planner_type = ApaPlannerType::SLANT_PARK_IN_PLANNER;
    } else {
      ILOG_INFO << "current slot type is not supported now!";
      return false;
    }
  } else if (apa_data_ptr_->cur_state == ApaStateMachine::ACTIVE_OUT) {
    ILOG_INFO << "planner_type = PERPENDICULAR_PARK_OUT!";
    apa_data_ptr_->planner_type =
        ApaPlannerType::PERPENDICULAR_PARK_OUT_PLANNER;
  }

  PrintApaPlannerType(apa_data_ptr_->planner_type);

  return true;
}

const bool ApaWorld::Update(const LocalView* local_view) {
  local_view_ptr_ = local_view;
  return Update();
}

}  // namespace apa_planner
}  // namespace planning

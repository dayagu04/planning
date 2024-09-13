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

void ApaWorld::UpdateSlots() {}
void ApaWorld::UpdateUssDistance() {}
void ApaWorld::UpdateObstacles() {}

const bool ApaWorld::Update() {
  if (local_view_ptr_ == nullptr) {
    DEBUG_PRINT("-- apa_world: local view ptr is nullptr, err ---");
    return false;
  }
  DEBUG_PRINT("-- apa_world: run preprocess ---");
  Preprocess();

  DEBUG_PRINT("func_state = " << static_cast<int>(
                  apa_data_ptr_->func_state_ptr->current_state));

  PrintApaStateMachine(apa_data_ptr_->cur_state);

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

  DEBUG_PRINT("-- apa_world: run slot_management ---");
  if (!slot_manager_ptr_->Update(apa_data_ptr_)) {
    DEBUG_PRINT("shouldn't have entered the parking function at that time");
    return false;
  }

  if (!apa_data_ptr_->is_slot_type_fixed) {
    // TODO: selected slot (slot_type) should be obtained in slot management
    apa_data_ptr_->slot_type = slot_manager_ptr_->GetEgoSlotInfo().slot_type;
    apa_data_ptr_->slot_id = slot_manager_ptr_->GetEgoSlotInfo().select_slot_id;
    apa_data_ptr_->is_slot_type_fixed = true;
  }

  DEBUG_PRINT("current slot type in slm =" << static_cast<int>(
                  slot_manager_ptr_->GetEgoSlotInfo().slot_type));
  DEBUG_PRINT(
      "fixed slot type =" << static_cast<int>(apa_data_ptr_->slot_type));

  if (apa_data_ptr_->cur_state == ApaStateMachine::ACTIVE_WAIT_IN ||
      apa_data_ptr_->cur_state == ApaStateMachine::ACTIVE_IN) {
    if (apa_data_ptr_->slot_type ==
        Common::ParkingSlotType::PARKING_SLOT_TYPE_VERTICAL) {
      DEBUG_PRINT("planner_type = PERPENDICULAR_PARK_IN!");

      if (apa_param.GetParam().path_generator_type ==
          ParkPathGenerationType::GEOMETRY_BASED) {
        apa_data_ptr_->planner_type =
            ApaPlannerType::PERPENDICULAR_PARK_IN_PLANNER;
      } else {
        apa_data_ptr_->planner_type = ApaPlannerType::HYBRID_ASTAR_PLANNER;
      }
      ILOG_INFO << "path plan method = "
                << static_cast<int>(apa_data_ptr_->planner_type);
    } else if (apa_data_ptr_->slot_type ==
               Common::ParkingSlotType::PARKING_SLOT_TYPE_HORIZONTAL) {
      DEBUG_PRINT("planner_type = PARALLEL_PARK_IN!");
      apa_data_ptr_->planner_type = ApaPlannerType::PARALLEL_PARK_IN_PLANNER;
    } else if (apa_data_ptr_->slot_type ==
               Common::ParkingSlotType::PARKING_SLOT_TYPE_SLANTING) {
      DEBUG_PRINT("planner_type = SLANT_PARK_IN!");
      apa_data_ptr_->planner_type = ApaPlannerType::SLANT_PARK_IN_PLANNER;
    } else {
      DEBUG_PRINT("current slot type is not supported now!");
      return false;
    }
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

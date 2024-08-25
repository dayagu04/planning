#include "apa_world.h"

#include <cstdint>
#include <vector>

#include "apa_param_setting.h"
#include "common.pb.h"
#include "common_c.h"
#include "debug_info_log.h"
#include "func_state_machine_c.h"
#include "general_planning_context.h"
#include "geometry_math.h"
#include "slot_management_info.pb.h"

namespace planning {
namespace apa_planner {
void ApaWorld::Init() {
  measures_ptr_ = std::make_shared<Measurements>();
  slot_manager_ptr_ = std::make_shared<SlotManagement>();
  uss_obstacle_avoider_ptr_ = std::make_shared<UssObstacleAvoidance>();
  collision_detector_ptr_ = std::make_shared<CollisionDetector>();
}

void ApaWorld::Reset() {
  measures_ptr_->Reset();
  slot_manager_ptr_->Reset();
  uss_obstacle_avoider_ptr_->Reset();
  collision_detector_ptr_->Reset();
  local_view_ptr_ = nullptr;
}

void ApaWorld::Preprocess() {
  // update ego info
  UpdateEgoState();
}

void ApaWorld::UpdateEgoState() {
  measures_ptr_->current_state =
      local_view_ptr_->function_state_machine_info.current_state;

  const auto& pos = local_view_ptr_->localization.position.position_boot;
  const auto& heading =
      local_view_ptr_->localization.orientation.euler_boot.yaw;

  const Eigen::Vector2d current_pos(pos.x, pos.y);

  const ApaParameters& param = apa_param.GetParam();
  // calculate standstill time by pos
  const double local_move_dist = (measures_ptr_->pos_ego - current_pos).norm();
  if (local_move_dist < param.car_static_pos_err_strict) {
    measures_ptr_->car_static_timer_by_pos_strict += param.plan_time;
  } else {
    measures_ptr_->car_static_timer_by_pos_strict = 0.0;
  }
  if (local_move_dist < param.car_static_pos_err_normal) {
    measures_ptr_->car_static_timer_by_pos_normal += param.plan_time;
  } else {
    measures_ptr_->car_static_timer_by_pos_normal = 0.0;
  }

  measures_ptr_->pos_ego = current_pos;
  measures_ptr_->heading_ego = heading;
  measures_ptr_->heading_ego_vec << std::cos(heading), std::sin(heading);

  // measures_ptr_->vel_ego =
  //     local_view_ptr_->vehicle_service_output_info.vehicle_speed();

  measures_ptr_->vel_ego =
      local_view_ptr_->localization.velocity.velocity_body.vx;

  // calculate standstill time by velocity
  if (std::fabs(measures_ptr_->vel_ego) < param.car_static_velocity_strict) {
    measures_ptr_->car_static_timer_by_vel_strict += param.plan_time;
  } else {
    measures_ptr_->car_static_timer_by_vel_strict = 0.0;
  }
  if (std::fabs(measures_ptr_->vel_ego) < param.car_static_velocity_normal) {
    measures_ptr_->car_static_timer_by_vel_normal += param.plan_time;
  } else {
    measures_ptr_->car_static_timer_by_vel_normal = 0.0;
  }

  // static flag
  measures_ptr_->static_flag = (measures_ptr_->car_static_timer_by_pos_strict >
                                    param.car_static_keep_time_by_pos_strict ||
                                measures_ptr_->car_static_timer_by_pos_normal >
                                    param.car_static_keep_time_by_pos_normal) &&
                               (measures_ptr_->car_static_timer_by_vel_strict >
                                    param.car_static_keep_time_by_vel_strict ||
                                measures_ptr_->car_static_timer_by_vel_normal >
                                    param.car_static_keep_time_by_vel_normal);

  JSON_DEBUG_VALUE("local_move_dist", local_move_dist)
  JSON_DEBUG_VALUE("local_vel", measures_ptr_->vel_ego)
  JSON_DEBUG_VALUE("car_static_timer_by_pos_strict",
                   measures_ptr_->car_static_timer_by_pos_strict)
  JSON_DEBUG_VALUE("car_static_timer_by_pos_normal",
                   measures_ptr_->car_static_timer_by_pos_normal)
  JSON_DEBUG_VALUE("car_static_timer_by_vel_strict",
                   measures_ptr_->car_static_timer_by_vel_strict)
  JSON_DEBUG_VALUE("car_static_timer_by_vel_normal",
                   measures_ptr_->car_static_timer_by_vel_normal)
  JSON_DEBUG_VALUE("static_flag", measures_ptr_->static_flag)
}

const bool ApaWorld::CheckSelectedSlot() const {
  // check selected slot id
  if (local_view_ptr_->parking_fusion_info.select_slot_id == 0) {
    DEBUG_PRINT("Error: no selected id");
    return false;
  }

  // check slot size in slot management
  const size_t slots_size =
      slot_manager_ptr_->GetOutputPtr()->slot_info_vec_size();
  if (slots_size == 0) {
    DEBUG_PRINT("empty managed slot!");
    return false;
  }

  const size_t selected_slot_id =
      local_view_ptr_->parking_fusion_info.select_slot_id;
  DEBUG_PRINT("selected_slot_id:" << selected_slot_id);

  // check slot type
  const auto& fusion_slots =
      local_view_ptr_->parking_fusion_info.parking_fusion_slot_lists;

  bool valid_selected_slot = false;
  const int fusion_slot_lists_size =
      local_view_ptr_->parking_fusion_info.parking_fusion_slot_lists_size;
  for (int i = 0; i < fusion_slot_lists_size; ++i) {
    if (selected_slot_id == fusion_slots[i].id) {
      const auto& slot_type = fusion_slots[i].type;

      if (slot_type == iflyauto::PARKING_SLOT_TYPE_VERTICAL) {
        DEBUG_PRINT("perpendicular slot selected in fusion");
      } else if (slot_type == iflyauto::PARKING_SLOT_TYPE_HORIZONTAL) {
        DEBUG_PRINT("parallel slot selected in fusion");
      }

      measures_ptr_->slot_type = slot_type;
      valid_selected_slot = true;
      break;
    }
  }

  if (!valid_selected_slot) {
    DEBUG_PRINT("selected slot is invalid!");
    return false;
  }

  // check if selected slot released in slot management
  common::SlotInfo selected_slot_slm;

  if (!slot_manager_ptr_->GetSelectedSlot(selected_slot_slm,
                                          static_cast<int>(selected_slot_id))) {
    DEBUG_PRINT("selected slot is not found in slot management!");
    return false;
  }

  // DEBUG_PRINT("selected_slot_slm = " << selected_slot_slm.DebugString()
  //          );

  if (!selected_slot_slm.is_release()) {
    DEBUG_PRINT("selected slot is not released!");
    return false;
  }

  measures_ptr_->target_managed_slot.CopyFrom(selected_slot_slm);

  return true;
}

const bool ApaWorld::CheckParkInState() const {
  const bool park_in_search_stm =
      local_view_ptr_->function_state_machine_info.current_state ==
      iflyauto::FunctionalState_PARK_IN_SEARCHING;

  const bool park_in_activated_stm =
      (local_view_ptr_->function_state_machine_info.current_state >=
           iflyauto::FunctionalState_PARK_GUIDANCE &&
       local_view_ptr_->function_state_machine_info.current_state <=
           iflyauto::FunctionalState_PARK_COMPLETED) &&
      measures_ptr_->history_apa_function ==
          GeneralApaFunction::PARK_IN_FUNCTION;

  return park_in_search_stm || park_in_activated_stm;
}

const bool ApaWorld::CheckParkInActivated() const {
  return local_view_ptr_->function_state_machine_info.current_state >=
             iflyauto::FunctionalState_PARK_GUIDANCE &&
         local_view_ptr_->function_state_machine_info.current_state <=
             iflyauto::FunctionalState_PARK_COMPLETED &&
         measures_ptr_->history_apa_function ==
             GeneralApaFunction::PARK_IN_FUNCTION;
}

const bool ApaWorld::CheckParkOutState() const {
  const bool park_out_search_stm =
      local_view_ptr_->function_state_machine_info.current_state ==
      iflyauto::FunctionalState_PARK_OUT_SEARCHING;

  const bool park_out_activated_stm =
      (local_view_ptr_->function_state_machine_info.current_state >=
           iflyauto::FunctionalState_PARK_GUIDANCE &&
       local_view_ptr_->function_state_machine_info.current_state <=
           iflyauto::FunctionalState_PARK_COMPLETED) &&
      measures_ptr_->history_apa_function ==
          GeneralApaFunction::PARK_OUT_FUNCTION;

  return park_out_search_stm || park_out_activated_stm;
}

const bool ApaWorld::Update() {
  // preprocess measurements
  DEBUG_PRINT("-- apa_world: run preprocess ---");
  Preprocess();
  measures_ptr_->planner_type = ApaPlannerType::NONE_PLANNER;

  DEBUG_PRINT(
      "current_state = " << static_cast<int>(measures_ptr_->current_state));

  if (measures_ptr_->current_state == iflyauto::FunctionalState_PARK_STANDBY) {
    measures_ptr_->history_apa_function = GeneralApaFunction::NONE_FUNCTION;
  } else if (measures_ptr_->current_state ==
             iflyauto::FunctionalState_PARK_IN_SEARCHING) {
    measures_ptr_->history_apa_function = GeneralApaFunction::PARK_IN_FUNCTION;
  } else if (measures_ptr_->current_state ==
             iflyauto::FunctionalState_PARK_OUT_SEARCHING) {
    measures_ptr_->history_apa_function = GeneralApaFunction::PARK_OUT_FUNCTION;
  }

  // check parking scenarios
  if (CheckParkInState()) {
    measures_ptr_->general_apa_function = GeneralApaFunction::PARK_IN_FUNCTION;
    DEBUG_PRINT("current parking in is activated now!");
  } else if (CheckParkOutState()) {
    measures_ptr_->general_apa_function = GeneralApaFunction::PARK_OUT_FUNCTION;
    DEBUG_PRINT("current parking out is not supported now!");
    return false;
  } else {
    measures_ptr_->general_apa_function = GeneralApaFunction::NONE_FUNCTION;
    DEBUG_PRINT("current parking scenarios is not supported now!");
    return false;
  }

  // run slot manager
  // currently path planning starts once id is selected in searching state

  DEBUG_PRINT("-- apa_world: run slot_management ---");
  if (!slot_manager_ptr_->Update(local_view_ptr_)) {
    DEBUG_PRINT("shouldn't have entered the parking function at that time");
    return false;
  }

  measures_ptr_->slot_type = slot_manager_ptr_->GetEgoSlotInfo().slot_type;

  // check park in planner
  if (CheckParkInActivated()) {
    if (measures_ptr_->slot_type ==
        Common::ParkingSlotType::PARKING_SLOT_TYPE_VERTICAL) {
      DEBUG_PRINT("planner_type = PERPENDICULAR_PARK_IN!");
      measures_ptr_->planner_type =
          ApaPlannerType::PERPENDICULAR_PARK_IN_PLANNER;
    } else if (measures_ptr_->slot_type ==
               Common::ParkingSlotType::PARKING_SLOT_TYPE_HORIZONTAL) {
      DEBUG_PRINT("planner_type = PARALLEL_PARK_IN!");
      measures_ptr_->planner_type = ApaPlannerType::PARALLEL_PARK_IN_PLANNER;
    } else if (measures_ptr_->slot_type ==
               Common::ParkingSlotType::PARKING_SLOT_TYPE_SLANTING) {
      DEBUG_PRINT("planner_type = SLANT_PARK_IN!");
      measures_ptr_->planner_type =
          ApaPlannerType::PERPENDICULAR_PARK_IN_PLANNER;
    } else {
      DEBUG_PRINT("current slot type is not supported now!");
      return false;
    }
  }

  // DEBUG_PRINT("planner_type = "
  //           << static_cast<int>(measures_ptr_->planner_type));

  return true;
}

const bool ApaWorld::Update(const LocalView* local_view) {
  local_view_ptr_ = local_view;
  return Update();
}

}  // namespace apa_planner
}  // namespace planning

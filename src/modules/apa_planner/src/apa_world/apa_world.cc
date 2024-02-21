#include "apa_world.h"

#include <cstdint>
#include <vector>

#include "apa_param_setting.h"
#include "common.pb.h"
#include "func_state_machine.pb.h"
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
      local_view_ptr_->function_state_machine_info.current_state();

  const auto& pose = local_view_ptr_->localization_estimate.pose();

  const Eigen::Vector2d current_pos(pose.local_position().x(),
                                    pose.local_position().y());

  // calculate standstill time by pos
  if ((measures_ptr_->pos_ego - current_pos).norm() <
      apa_param.GetParam().car_static_pos_err) {
    measures_ptr_->car_static_timer_by_pos += apa_param.GetParam().plan_time;
  } else {
    measures_ptr_->car_static_timer_by_pos = 0.0;
  }

  measures_ptr_->pos_ego = current_pos;
  measures_ptr_->heading_ego = pose.heading();
  measures_ptr_->heading_ego_vec << std::cos(pose.heading()),
      std::sin(pose.heading());

  // measures_ptr_->vel_ego =
  //     local_view_ptr_->vehicle_service_output_info.vehicle_speed();

  measures_ptr_->vel_ego = local_view_ptr_->localization_estimate.pose()
                               .linear_velocity_from_wheel();

  // calculate standstill time by velocity
  if (std::fabs(measures_ptr_->vel_ego) <
      apa_param.GetParam().car_static_velocity) {
    measures_ptr_->car_static_timer_by_vel += apa_param.GetParam().plan_time;
  } else {
    measures_ptr_->car_static_timer_by_vel = 0.0;
  }

  // static flag
  measures_ptr_->static_flag =
      (measures_ptr_->car_static_timer_by_vel >=
           apa_param.GetParam().car_static_keep_time_by_vel &&
       measures_ptr_->car_static_timer_by_pos >
           apa_param.GetParam().car_static_keep_time_by_vel) ||
      (measures_ptr_->car_static_timer_by_pos >
       apa_param.GetParam().car_static_keep_time_by_pos);
}

const bool ApaWorld::CheckSelectedSlot() const {
  // check selected slot id
  if (!local_view_ptr_->parking_fusion_info.has_select_slot_id()) {
    std::cout << "Error: no selected id" << std::endl;
    return false;
  }

  // check slot size in slot management
  const size_t slots_size =
      slot_manager_ptr_->GetOutputPtr()->slot_info_vec_size();
  if (slots_size == 0) {
    std::cout << "empty managed slot!" << std::endl;
    return false;
  }

  const size_t selected_slot_id =
      local_view_ptr_->parking_fusion_info.select_slot_id();
  std::cout << "selected_slot_id:" << selected_slot_id << std::endl;

  // check slot type
  const auto& fusion_slots =
      local_view_ptr_->parking_fusion_info.parking_fusion_slot_lists();

  bool valid_selected_slot = false;
  for (int i = 0; i < fusion_slots.size(); ++i) {
    if (selected_slot_id == fusion_slots[i].id()) {
      const auto& slot_type = fusion_slots[i].type();

      if (slot_type == Common::ParkingSlotType::PARKING_SLOT_TYPE_VERTICAL) {
        std::cout << "perpendicular slot selected in fusion" << std::endl;
      } else if (slot_type ==
                 Common::ParkingSlotType::PARKING_SLOT_TYPE_HORIZONTAL) {
        std::cout << "parallel slot selected in fusion" << std::endl;
      }

      measures_ptr_->slot_type = slot_type;
      valid_selected_slot = true;
      break;
    }
  }

  if (!valid_selected_slot) {
    std::cout << "selected slot is invalid!" << std::endl;
    return false;
  }

  // check if selected slot released in slot management
  common::SlotInfo selected_slot_slm;

  if (!slot_manager_ptr_->GetSelectedSlot(selected_slot_slm,
                                          static_cast<int>(selected_slot_id))) {
    std::cout << "selected slot is not found in slot management!" << std::endl;
    return false;
  }

  // std::cout << "selected_slot_slm = " << selected_slot_slm.DebugString()
  //           << std::endl;

  if (!selected_slot_slm.is_release()) {
    std::cout << "selected slot is not released!" << std::endl;
    return false;
  }

  measures_ptr_->target_managed_slot.CopyFrom(selected_slot_slm);

  return true;
}

const bool ApaWorld::CheckParkInState() const {
  return (local_view_ptr_->function_state_machine_info.current_state() >=
              FuncStateMachine::PARK_IN_APA_IN &&
          local_view_ptr_->function_state_machine_info.current_state() <=
              FuncStateMachine::PARK_IN_COMPLETED);
}

const bool ApaWorld::CheckParkInActivated() const {
  return (local_view_ptr_->function_state_machine_info.current_state() >=
              FuncStateMachine::PARK_IN_ACTIVATE_WAIT &&
          local_view_ptr_->function_state_machine_info.current_state() <=
              FuncStateMachine::PARK_IN_COMPLETED);
}

const bool ApaWorld::CheckParkOutState() const {
  return (local_view_ptr_->function_state_machine_info.current_state() >=
              FuncStateMachine::PARK_OUT_SEARCHING &&
          local_view_ptr_->function_state_machine_info.current_state() <=
              FuncStateMachine::PARK_OUT_COMPLETED);
}

const bool ApaWorld::CheckParkOutActivated() const {
  return (local_view_ptr_->function_state_machine_info.current_state() >=
              FuncStateMachine::PARK_OUT_ACTIVATE &&
          local_view_ptr_->function_state_machine_info.current_state() <=
              FuncStateMachine::PARK_OUT_COMPLETED);
}
const bool ApaWorld::Update() {
  // preprocess measurements
  std::cout << "-- apa_world: run preprocess ---\n";
  Preprocess();

  std::cout << "current_state = "
            << static_cast<int>(measures_ptr_->current_state) << std::endl;

  // check parking scenarios
  if (CheckParkInState()) {
    measures_ptr_->general_apa_function = GeneralApaFunction::PARK_IN_FUNCTION;
    std::cout << "current parking in is activated now!" << std::endl;
  } else if (CheckParkOutState()) {
    measures_ptr_->general_apa_function = GeneralApaFunction::PARK_OUT_FUNCTION;
    std::cout << "current parking out is not supported now!" << std::endl;
    return false;
  } else {
    measures_ptr_->general_apa_function = GeneralApaFunction::NONE_FUNCTION;
    std::cout << "current parking scenarios is not supported now!" << std::endl;
    return false;
  }

  // run slot manager
  // path planning only starts when the current state machines are 29 and 30
  std::cout << "-- apa_world: run slot_management ---" << std::endl;
  if (!slot_manager_ptr_->Update(local_view_ptr_)) {
    std::cout << "shouldn't have entered the parking function at that time\n";
    return false;
  }

  measures_ptr_->slot_type = slot_manager_ptr_->GetEgoSlotInfo().slot_type;
  // TODO: selected slot (slot_type) should be obtained in slot management
  measures_ptr_->planner_type = ApaPlannerType::NONE_PLANNER;

  // check park in planner
  if (CheckParkInActivated()) {
    if (measures_ptr_->slot_type ==
        Common::ParkingSlotType::PARKING_SLOT_TYPE_VERTICAL) {
      std::cout << "planner_type = PERPENDICULAR_PARK_IN!" << std::endl;
      measures_ptr_->planner_type =
          ApaPlannerType::PERPENDICULAR_PARK_IN_PLANNER;
    } else if (measures_ptr_->slot_type ==
               Common::ParkingSlotType::PARKING_SLOT_TYPE_HORIZONTAL) {
      std::cout << "planner_type = PARALLEL_PARK_IN!" << std::endl;
      measures_ptr_->planner_type = ApaPlannerType::PARALLEL_PARK_IN_PLANNER;
    } else {
      std::cout << "current slot type is not supported now!" << std::endl;
      return false;
    }
  }

  std::cout << "planner_type = "
            << static_cast<int>(measures_ptr_->planner_type) << std::endl;

  return true;
}

const bool ApaWorld::Update(const LocalView* local_view_ptr) {
  local_view_ptr_ = local_view_ptr;
  return Update();
}

}  // namespace apa_planner
}  // namespace planning

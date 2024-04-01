#include "plan_scheduler.h"

#include <cstddef>
#include <cstdint>
#include <memory>

#include "apa_plan_interface.h"
#include "apa_world.h"
#include "debug_info_log.h"
#include "ifly_time.h"
#include "plan_data.h"
#include "planning_plan.pb.h"

namespace planning {
namespace plan_scheduler {

void PlanScheduler::Init(const std::shared_ptr<LocalView> local_view_ptr) {
  // init plan data
  plan_data_ptr_ = std::make_shared<plan_interface::PlanData>();

  // init local_view_ptr_
  local_view_ptr_ = local_view_ptr;

  // init planner interface stack
  planner_interface_stack_.resize(plan_interface::PlannerInterfaceType::COUNT);

  // init apa planner interface
  planner_interface_stack_[plan_interface::PlannerInterfaceType::APA_PLANNER] =
      std::make_shared<apa_planner::ApaPlanInterface>();

  planner_interface_stack_[plan_interface::PlannerInterfaceType::APA_PLANNER]
      ->Init(plan_data_ptr_);

  // init scc planner interface
}

const bool PlanScheduler::IsApa() const {
  const auto current_state =
      local_view_ptr_->function_state_machine_info.current_state();

  return (current_state >= FuncStateMachine::PARK_IN_APA_IN &&
          current_state <= FuncStateMachine::PARK_IN_COMPLETED);
}

void PlanScheduler::Reset() {
  // reset apa planner
  planner_interface_stack_[plan_interface::PlannerInterfaceType::APA_PLANNER]
      ->Reset();
}

const bool PlanScheduler::Update() {
  const double start_timestamp_ms = IflyTime::Now_ms();

  auto &state_data = plan_data_ptr_->MutableData().state_data;

  auto &debug_info =
      plan_data_ptr_->MutableData().output_data.planning_debug_info;

  state_data.frame_num++;

  // intersection with two planner interface (such as hpp and apa) should be
  // considered later
  const uint8_t last_planner_interface_type = state_data.planner_interface_type;

  uint8_t planner_interface_type =
      plan_interface::PlannerInterfaceType::NONE_PLANNER;

  if (IsApa()) {
    planner_interface_type = plan_interface::PlannerInterfaceType::APA_PLANNER;
    state_data.scene_type = planning::common::SceneType::PARKING_APA;
  } else {
    planner_interface_type = plan_interface::PlannerInterfaceType::NONE_PLANNER;
    state_data.scene_type = planning::common::SceneType::NOT_DEFINED;
  }

  state_data.planner_interface_type = planner_interface_type;

  const auto sync_param_flag =
      (last_planner_interface_type != state_data.planner_interface_type);

  auto &planning_output =
      plan_data_ptr_->MutableData().output_data.planning_output;

  bool success = false;
  for (size_t i = 0; i < plan_interface::PlannerInterfaceType::COUNT; ++i) {
    if (i == state_data.planner_interface_type) {
      // run planner
      success = planner_interface_stack_[i]->Update(local_view_ptr_);
      // set planning output
      planning_output.CopyFrom(planner_interface_stack_[i]->GetPlaningOutput());

      // sync param
      if (sync_param_flag) {
        planner_interface_stack_[i]->SyncParameters();
      }
    } else {
      // reset planner
      planner_interface_stack_[i]->Reset();
    }
  }

  auto end_timestamp_ms = IflyTime::Now_ms();
  const auto frame_duration = end_timestamp_ms - start_timestamp_ms;

  // set frame info
  auto frame_info = debug_info.mutable_frame_info();

  frame_info->set_frame_num(state_data.frame_num);
  frame_info->set_frame_duration_ms(frame_duration);
  frame_info->set_planning_succ(success);

  frame_info->set_scene_type(common::SceneType_Name(
      static_cast<common::SceneType>(state_data.scene_type)));

  DEBUG_PRINT("time_consumption = " << frame_duration << "ms");

  return true;
}

}  // namespace plan_scheduler
}  // namespace planning
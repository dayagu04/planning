#include "parking_scenario_manager.h"

#include <algorithm>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "apa_data.h"
#include "apa_param_config.h"
#include "apa_slot.h"
#include "apa_state_machine_manager.h"
#include "debug_info_log.h"
#include "log_glog.h"
#include "narrow_space/narrow_space_scenario.h"
#include "narrow_space_decider.h"
#include "narrow_space_scenario.h"
#include "parallel_park_in_scenario.h"
#include "parking_scenario.h"
#include "perpendicular_head_in_scenario.h"
#include "perpendicular_head_out_scenario.h"
#include "perpendicular_park_scenario.h"
#include "perpendicular_tail_in_scenario.h"
#include "planning_plan_c.h"

namespace planning {
namespace apa_planner {

bool ParkingScenarioManager::Init(
    const std::shared_ptr<apa_planner::ApaWorld> &apa_world) {
  if (init_) {
    return true;
  }

  // init planners
  scenario_list_[ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN] =
      std::make_shared<PerpendicularTailInScenario>(apa_world);

  scenario_list_[ParkingScenarioType::SCENARIO_SLANT_TAIL_IN] =
      std::make_shared<PerpendicularTailInScenario>(apa_world);

  scenario_list_[ParkingScenarioType::SCENARIO_PARALLEL_IN] =
      std::make_shared<ParallelParkInScenario>(apa_world);

  scenario_list_[ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_OUT] =
      std::make_shared<PerpendicularHeadOutScenario>(apa_world);

  scenario_list_[ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN] =
      std::make_shared<PerpendicularHeadInScenario>(apa_world);

  scenario_list_[ParkingScenarioType::SCENARIO_NARROW_SPACE] =
      std::make_shared<NarrowSpaceScenario>(apa_world);

  current_scenario_ = nullptr;
  init_ = true;
  scenario_type_ = ParkingScenarioType::SCENARIO_UNKNOWN;
  scenario_status_ = ParkingScenarioStatus::STATUS_UNKNOWN;

  apa_world_ = apa_world;
  return true;
}

void ParkingScenarioManager::Excute() {
  scenario_status_ = ParkingScenarioStatus::STATUS_UNKNOWN;
  scenario_type_ = ParkingScenarioType::SCENARIO_UNKNOWN;

  if (apa_world_->GetStateMachineManagerPtr()->IsSeachingStatus()) {
    Reset();
  }

  const auto &cur_state =
      apa_world_->GetStateMachineManagerPtr()->GetStateMachine();

  if (cur_state == ApaStateMachine::SEARCH_IN_SELECTED_CAR_REAR ||
      cur_state == ApaStateMachine::ACTIVE_IN_CAR_REAR) {
    if (apa_world_->GetApaDataPtr()->slot_type ==
        Common::ParkingSlotType::PARKING_SLOT_TYPE_VERTICAL) {
      if (apa_param.GetParam().path_generator_type ==
          ParkPathGenerationType::GEOMETRY_BASED) {
        scenario_type_ = ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN;
        // check is narrow space or not
        if (IsSlotReleaseByHybridAstar()) {
          scenario_type_ = ParkingScenarioType::SCENARIO_NARROW_SPACE;
        }
      } else {
        // only use astar
        scenario_type_ = ParkingScenarioType::SCENARIO_NARROW_SPACE;
      }
    } else if (apa_world_->GetApaDataPtr()->slot_type ==
               Common::ParkingSlotType::PARKING_SLOT_TYPE_SLANTING) {
      scenario_type_ = ParkingScenarioType::SCENARIO_SLANT_TAIL_IN;
    } else if (apa_world_->GetApaDataPtr()->slot_type ==
               Common::ParkingSlotType::PARKING_SLOT_TYPE_HORIZONTAL) {
      if (apa_param.GetParam().path_generator_type ==
          ParkPathGenerationType::GEOMETRY_BASED) {
        scenario_type_ = ParkingScenarioType::SCENARIO_PARALLEL_IN;
      } else {
        scenario_type_ = ParkingScenarioType::SCENARIO_NARROW_SPACE;
      }
    }
  } else if (cur_state == ApaStateMachine::SEARCH_IN_SELECTED_CAR_FRONT ||
             cur_state == ApaStateMachine::ACTIVE_IN_CAR_FRONT) {
    if (apa_world_->GetApaDataPtr()->slot_type ==
        Common::ParkingSlotType::PARKING_SLOT_TYPE_VERTICAL) {
      scenario_type_ = ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN;

      if (apa_param.GetParam().path_generator_type ==
          ParkPathGenerationType::SEARCH_BASED) {
        scenario_type_ = ParkingScenarioType::SCENARIO_NARROW_SPACE;
      }
    }
  } else if (cur_state == ApaStateMachine::SEARCH_OUT_NO_SELECTED ||
             cur_state == ApaStateMachine::SEARCH_OUT_SELECTED_CAR_FRONT ||
             cur_state == ApaStateMachine::ACTIVE_OUT_CAR_FRONT) {
    if (apa_world_->GetApaDataPtr()->slot_type ==
        Common::ParkingSlotType::PARKING_SLOT_TYPE_VERTICAL) {
      scenario_type_ = ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_OUT;
    }
  }

  if (apa_world_->GetStateMachineManagerPtr()->IsParkingStatus()) {
    scenario_status_ = ParkingScenarioStatus::STATUS_RUNNING;
  } else if (apa_world_->GetStateMachineManagerPtr()->IsSeachingStatus()) {
    scenario_status_ = ParkingScenarioStatus::STATUS_TRY;
  }

  current_scenario_ = GetScenarioByType(scenario_type_);
}

void ParkingScenarioManager::Process() {
  if (scenario_status_ == ParkingScenarioStatus::STATUS_RUNNING) {
    ScenarioRunning();
  } else if (scenario_status_ == ParkingScenarioStatus::STATUS_TRY) {
    ScenarioTry();
  }
}

void ParkingScenarioManager::Reset() {
  memset(&planning_output_, 0, sizeof(planning_output_));
  memset(&apa_hmi_data_, 0, sizeof(apa_hmi_data_));

  scenario_type_ = ParkingScenarioType::SCENARIO_UNKNOWN;
  scenario_status_ = ParkingScenarioStatus::STATUS_UNKNOWN;

  // reset all planner
  for (const auto &scene : scenario_list_) {
    scene.second->Reset();
  }

  return;
}

std::shared_ptr<ParkingScenario> ParkingScenarioManager::GetScenarioByType(
    const ParkingScenarioType type) {
  auto it = scenario_list_.find(type);
  if (it != scenario_list_.end()) {
    return scenario_list_[type];
  }

  ILOG_INFO << "invalid index";
  return nullptr;
}

void ParkingScenarioManager::ScenarioRunning() {
  if (current_scenario_ == nullptr) {
    return;
  }
  current_scenario_->ScenarioRunning();

  planning_output_ = current_scenario_->GetOutput();

  apa_hmi_data_ = current_scenario_->GetAPAHmi();

  ILOG_INFO << "scenario running";
}

void ParkingScenarioManager::ScenarioTry() {
  if (current_scenario_ == nullptr) {
    return;
  }

  const auto &cur_state =
      apa_world_->GetStateMachineManagerPtr()->GetStateMachine();

  if (cur_state == ApaStateMachine::SEARCH_IN_SELECTED_CAR_REAR) {
    // 车尾泊入功能
    if (apa_world_->GetApaDataPtr()->slot_type ==
        Common::ParkingSlotType::PARKING_SLOT_TYPE_VERTICAL) {
      std::shared_ptr<ParkingScenario> temp_narrow_scenario =
          scenario_list_[ParkingScenarioType::SCENARIO_NARROW_SPACE];

      if (apa_param.GetParam().path_generator_type ==
          ParkPathGenerationType::GEOMETRY_BASED) {
        // 先用几何尝试
        std::shared_ptr<ParkingScenario> temp_geometry_scenario =
            scenario_list_[ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN];
        temp_geometry_scenario->ScenarioTry();
      }

      if (apa_world_->GetSlotManagerPtr()
              ->GetEgoSlotInfo()
              .release_info.release_state[GEOMETRY_PLANNING_RELEASE] ==
          SlotReleaseState::RELEASE) {
        ILOG_INFO << "scenario geometry path try success, clear astar";

        // A星跑了一半，也要将其清空.
        temp_narrow_scenario->ThreadClear();
      } else {
        ILOG_INFO << "scenario geometry path try fail, try astar";
        temp_narrow_scenario->ScenarioTry();
      }
    } else if (apa_world_->GetApaDataPtr()->slot_type ==
               Common::ParkingSlotType::PARKING_SLOT_TYPE_SLANTING) {
      // todo: 看是否要A星介入
      current_scenario_->ScenarioTry();
    } else {
      // todo: 平行车位
    }
  } else {
    // todo: 其他功能
  }

  return;
}

const bool ParkingScenarioManager::IsSlotReleaseByHybridAstar() {
  SlotReleaseState astar_path_release =
      apa_world_->GetSlotManagerPtr()
          ->GetEgoSlotInfo()
          .release_info.release_state[ASTAR_PLANNING_RELEASE];

  SlotReleaseState geometry_path_release =
      apa_world_->GetSlotManagerPtr()
          ->GetEgoSlotInfo()
          .release_info.release_state[GEOMETRY_PLANNING_RELEASE];

  if (planning_output_.planning_status.apa_planning_status ==
      iflyauto::APA_IN_PROGRESS) {
    JSON_DEBUG_VALUE("geometry_path_release",
                     geometry_path_release == SlotReleaseState::RELEASE)
  }

  if (geometry_path_release == SlotReleaseState::NOT_RELEASE &&
      astar_path_release == SlotReleaseState::RELEASE) {
    return true;
  }
  return false;

  // switch (astar_path_release) {
  //   case SlotReleaseState::RELEASE:
  //     return true;
  //   case SlotReleaseState::UNKOWN:
  //   case SlotReleaseState::NOT_RELEASE:
  //   default:
  //     break;
  // }

  // return false;
}

}  // namespace apa_planner
}  // namespace planning

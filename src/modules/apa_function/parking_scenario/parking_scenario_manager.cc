#include "parking_scenario_manager.h"

#include <algorithm>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "apa_param_config.h"
#include "apa_slot.h"
#include "apa_state_machine_manager.h"
#include "debug_info_log.h"
#include "log_glog.h"
#include "narrow_space/narrow_space_scenario.h"
#include "narrow_space_decider.h"
#include "narrow_space_scenario.h"
#include "parallel_park_in_scenario.h"
#include "parallel_park_out_scenario.h"
#include "parking_scenario.h"
#include "perpendicular_head_in_scenario.h"
#include "perpendicular_head_out_scenario.h"
#include "perpendicular_park_scenario.h"
#include "perpendicular_tail_in_scenario.h"
#include "planning_plan_c.h"
#include "park_hmi_state.h"

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

  scenario_list_[ParkingScenarioType::SCENARIO_PARALLEL_OUT] =
      std::make_shared<ParallelParkOutScenario>(apa_world);

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

void ParkingScenarioManager::UpdateScenarioType() {
  ILOG_INFO << "-------------------- ParkingScenarioManager  Excute "
               "--------------------";
  scenario_status_ = ParkingScenarioStatus::STATUS_UNKNOWN;
  scenario_type_ = ParkingScenarioType::SCENARIO_UNKNOWN;

  if (apa_world_->GetStateMachineManagerPtr()->IsSeachingStatus()) {
    Reset();
  }

  const ApaStateMachine &cur_state =
      apa_world_->GetStateMachineManagerPtr()->GetStateMachine();

  const EgoInfoUnderSlot &ego_info_under_slot =
      apa_world_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  if (cur_state == ApaStateMachine::SEARCH_IN_SELECTED_CAR_REAR ||
      cur_state == ApaStateMachine::ACTIVE_IN_CAR_REAR) {
    if (ego_info_under_slot.slot_type == SlotType::PERPENDICULAR) {
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
    } else if (ego_info_under_slot.slot_type == SlotType::SLANT) {
      scenario_type_ = ParkingScenarioType::SCENARIO_SLANT_TAIL_IN;
    } else if (ego_info_under_slot.slot_type == SlotType::PARALLEL) {
      if (apa_param.GetParam().path_generator_type ==
          ParkPathGenerationType::GEOMETRY_BASED) {
        scenario_type_ = ParkingScenarioType::SCENARIO_PARALLEL_IN;
      } else {
        scenario_type_ = ParkingScenarioType::SCENARIO_NARROW_SPACE;
      }
    }
  } else if (cur_state == ApaStateMachine::SEARCH_IN_SELECTED_CAR_FRONT ||
             cur_state == ApaStateMachine::ACTIVE_IN_CAR_FRONT) {
    if (ego_info_under_slot.slot_type == SlotType::PERPENDICULAR ||
        ego_info_under_slot.slot_type == SlotType::SLANT) {
      scenario_type_ = ParkingScenarioType::SCENARIO_NARROW_SPACE;
    }
  } else if (cur_state == ApaStateMachine::SEARCH_OUT_NO_SELECTED ||
             cur_state == ApaStateMachine::SEARCH_OUT_SELECTED_CAR_FRONT ||
             cur_state == ApaStateMachine::ACTIVE_OUT_CAR_FRONT) {
    if (ego_info_under_slot.slot_type == SlotType::PERPENDICULAR) {
      scenario_type_ = ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_OUT;
    } else if (ego_info_under_slot.slot_type == SlotType::PARALLEL) {
      scenario_type_ = ParkingScenarioType::SCENARIO_PARALLEL_OUT;
    }
  }

  if (apa_world_->GetStateMachineManagerPtr()->IsParkingStatus()) {
    scenario_status_ = ParkingScenarioStatus::STATUS_RUNNING;
  } else if (apa_world_->GetStateMachineManagerPtr()->IsSeachingStatus()) {
    scenario_status_ = ParkingScenarioStatus::STATUS_TRY;
  }

  current_scenario_ = GetScenarioByType(scenario_type_);

  PrintApaScenarioType(scenario_type_);
  PrintApaScenarioStatus(scenario_status_);
}

void ParkingScenarioManager::Process() {
  if (scenario_status_ == ParkingScenarioStatus::STATUS_RUNNING) {
    ScenarioRunning();
  } else if (scenario_status_ == ParkingScenarioStatus::STATUS_TRY) {
    ScenarioTry();
  }

  return;
}

void ParkingScenarioManager::Reset() {
  memset(&planning_output_, 0, sizeof(planning_output_));
  memset(&apa_hmi_data_, 0, sizeof(apa_hmi_data_));

  scenario_type_ = ParkingScenarioType::SCENARIO_UNKNOWN;
  scenario_status_ = ParkingScenarioStatus::STATUS_UNKNOWN;

  apa_world_->GetParkingTaskInterfacePtr()->Reset();

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
    Reset();
    planning_output_.planning_status.apa_planning_status = iflyauto::APA_FAILED;
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

  if (!apa_world_->GetSlotManagerPtr()->IsTargetSlotReleaseByRule()) {
    ILOG_INFO << "not release by rule";
    return;
  }

  const ApaStateMachine &cur_state =
      apa_world_->GetStateMachineManagerPtr()->GetStateMachine();

  const EgoInfoUnderSlot &ego_info_under_slot =
      apa_world_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  if (cur_state == ApaStateMachine::SEARCH_IN_SELECTED_CAR_REAR) {
    // 车尾泊入功能
    if (ego_info_under_slot.slot_type == SlotType::PERPENDICULAR) {
      std::shared_ptr<ParkingScenario> temp_narrow_scenario =
          scenario_list_[ParkingScenarioType::SCENARIO_NARROW_SPACE];

      if (apa_param.GetParam().path_generator_type ==
          ParkPathGenerationType::GEOMETRY_BASED) {
        // 先用几何尝试
        std::shared_ptr<ParkingScenario> temp_geometry_scenario =
            scenario_list_[ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN];
        temp_geometry_scenario->ScenarioTry();
      }

      if (ego_info_under_slot.slot.release_info_
              .release_state[GEOMETRY_PLANNING_RELEASE] ==
          SlotReleaseState::RELEASE) {
        ILOG_INFO << "scenario geometry path try success, clear astar";

        // A星跑了一半，也要将其清空.
        temp_narrow_scenario->ThreadClear();
      } else {
        ILOG_INFO << "scenario geometry path try fail, try astar";
        temp_narrow_scenario->ScenarioTry();
      }
    } else if (ego_info_under_slot.slot_type == SlotType::SLANT) {
      // todo: 看是否要A星介入
      current_scenario_->ScenarioTry();
    } else {
      // todo: 平行车位
      current_scenario_->ScenarioTry();
    }
  } else {
    // todo: 其他功能
    current_scenario_->ScenarioTry();
  }

  ILOG_INFO << "GEOMETRY RELEASE = "
            << GetSlotReleaseStateString(
                   ego_info_under_slot.slot.release_info_
                       .release_state[GEOMETRY_PLANNING_RELEASE])
            << ", ASTAR RELEASE = "
            << GetSlotReleaseStateString(
                   ego_info_under_slot.slot.release_info_
                       .release_state[ASTAR_PLANNING_RELEASE]);
  return;
}

const bool ParkingScenarioManager::IsSlotReleaseByHybridAstar() {
  const SlotReleaseState astar_path_release =
      apa_world_->GetSlotManagerPtr()
          ->GetEgoInfoUnderSlot()
          .slot.release_info_.release_state[ASTAR_PLANNING_RELEASE];

  const SlotReleaseState geometry_path_release =
      apa_world_->GetSlotManagerPtr()
          ->GetEgoInfoUnderSlot()
          .slot.release_info_.release_state[GEOMETRY_PLANNING_RELEASE];

  if (planning_output_.planning_status.apa_planning_status ==
          iflyauto::APA_IN_PROGRESS ||
      planning_output_.planning_status.hpp_planning_status ==
          iflyauto::HPP_RUNNING) {
    JSON_DEBUG_VALUE("geometry_path_release",
                     geometry_path_release == SlotReleaseState::RELEASE)
  }

  if (geometry_path_release == SlotReleaseState::NOT_RELEASE &&
      astar_path_release == SlotReleaseState::RELEASE) {
    ILOG_INFO << "use astar plan";
    return true;
  }
  ILOG_INFO << "use geometry plan";
  return false;
}

void ParkingScenarioManager::GenerateHmiSlotReleaseState() {
  SlotReleaseState state =
      apa_world_->GetSlotManagerPtr()->GetSlotReleaseState();
  switch (state) {
    case SlotReleaseState::NOT_RELEASE:
      apa_hmi_data_.prepare_plan_state = iflyauto::PREPARE_PLANNING_FAILED;
      break;
    case SlotReleaseState::RELEASE:
      apa_hmi_data_.prepare_plan_state = iflyauto::PREPARE_PLANNING_SUCCESS;
      break;
    case SlotReleaseState::UNKOWN:
      apa_hmi_data_.prepare_plan_state = iflyauto::PREPARE_PLANNING_NONE;
      break;
    default:
      apa_hmi_data_.prepare_plan_state = iflyauto::PREPARE_PLANNING_COMPUTING;
      break;
  }

  ILOG_INFO << "release state = " << apa_hmi_data_.prepare_plan_state;

  return;
}

void ParkingScenarioManager::RecommendParkingDirection() {
  ApaDirectionGenerator generator;
  generator.ClearRecommendationDirectionFlag(apa_hmi_data_);
  generator.SetRecommendationDirectionFlag(apa_hmi_data_, ParityBit);

  generator.SetRecommendationDirectionFlag(apa_hmi_data_, VerticalFrontLeft);
  generator.SetRecommendationDirectionFlag(apa_hmi_data_, ParallelFrontLeft);
  generator.SetRecommendationDirectionFlag(apa_hmi_data_, VerticalFront);
  generator.SetRecommendationDirectionFlag(apa_hmi_data_, VerticalFrontRight);
  generator.SetRecommendationDirectionFlag(apa_hmi_data_, ParallelFrontRight);
  generator.SetRecommendationDirectionFlag(apa_hmi_data_, VerticalBack);
  generator.SetRecommendationDirectionFlag(apa_hmi_data_, VerticalBackLeft);
  generator.SetRecommendationDirectionFlag(apa_hmi_data_, VerticalBackRight);
  generator.SetRecommendationDirectionFlag(apa_hmi_data_, VerticalHeadIn);
  generator.SetRecommendationDirectionFlag(apa_hmi_data_, VerticalTailIn);

  ILOG_INFO << "dir = " << apa_hmi_data_.planning_park_dir;

  return;
}

}  // namespace apa_planner
}  // namespace planning

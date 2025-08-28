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
#include "park_hmi_state.h"
#include "parking_scenario.h"
#include "perpendicular_head_in_scenario.h"
#include "perpendicular_head_out_scenario.h"
#include "perpendicular_park_scenario.h"
#include "perpendicular_tail_in_scenario.h"
#include "planning_plan_c.h"
#include "src/modules/apa_function/util/apa_utils.h"

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
  ILOG_INFO << "UpdateScenarioType";
  if (apa_world_->GetStateMachineManagerPtr()->IsSeachingStatus() ||
      apa_world_->GetStateMachineManagerPtr()->GetStateMachine() ==
          ApaStateMachine::STANDBY ||
      apa_world_->GetStateMachineManagerPtr()->GetStateMachine() ==
          ApaStateMachine::ERROR) {
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
        JSON_DEBUG_VALUE("geometry_path_release", false);
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
    } else if (ego_info_under_slot.slot_type == SlotType::PARALLEL) {
      if (apa_param.GetParam().path_generator_type ==
          ParkPathGenerationType::GEOMETRY_BASED) {
        scenario_type_ = ParkingScenarioType::SCENARIO_PARALLEL_IN;
      } else {
        scenario_type_ = ParkingScenarioType::SCENARIO_NARROW_SPACE;
      }
    }
  } else if (cur_state == ApaStateMachine::SEARCH_OUT_NO_SELECTED ||
             cur_state == ApaStateMachine::SEARCH_OUT_SELECTED_CAR_FRONT ||
             cur_state == ApaStateMachine::ACTIVE_OUT_CAR_FRONT ||
             cur_state == ApaStateMachine::SEARCH_OUT_SELECTED_CAR_REAR ||
             cur_state == ApaStateMachine::ACTIVE_OUT_CAR_REAR) {
    if (ego_info_under_slot.slot_type == SlotType::PERPENDICULAR ||
        ego_info_under_slot.slot_type == SlotType::SLANT) {
      if (apa_param.GetParam().use_geometry_path_head_out) {
        scenario_type_ = ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_OUT;
      } else {
        // 垂直泊出功能不使用几何规划时，设置 path_generator_type 为
        // SEARCH_BASED ，进行 hybrid a*；
        scenario_type_ = ParkingScenarioType::SCENARIO_NARROW_SPACE;
      }
    } else if (ego_info_under_slot.slot_type == SlotType::PARALLEL) {
      if (apa_param.GetParam().path_generator_type ==
          ParkPathGenerationType::GEOMETRY_BASED) {
        scenario_type_ = ParkingScenarioType::SCENARIO_PARALLEL_OUT;
      } else {
        scenario_type_ = ParkingScenarioType::SCENARIO_NARROW_SPACE;
      }
    }
  }

  if (apa_world_->GetStateMachineManagerPtr()->IsParkingStatus()) {
    scenario_status_ = ParkingScenarioStatus::STATUS_RUNNING;
  } else if (apa_world_->GetStateMachineManagerPtr()->IsSeachingStatus() &&
             ego_info_under_slot.id != 0) {
    scenario_status_ = ParkingScenarioStatus::STATUS_TRY;
  } else if (apa_world_->GetStateMachineManagerPtr()->IsParkSuspendStatus()) {
    scenario_status_ = ParkingScenarioStatus::STATUS_SUSPEND;
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
  } else if (scenario_status_ == ParkingScenarioStatus::STATUS_SUSPEND) {
    ScenarioSuspend();
  }

  return;
}

void ParkingScenarioManager::Reset() {
  memset(&apa_hmi_data_, 0, sizeof(apa_hmi_data_));

  scenario_type_ = ParkingScenarioType::SCENARIO_UNKNOWN;
  scenario_status_ = ParkingScenarioStatus::STATUS_UNKNOWN;

  apa_world_->GetParkingTaskInterfacePtr()->Reset();
  ClearPlanningOutput();
  planning_output_.planning_status.apa_planning_status =
      iflyauto::ApaPlanningStatus::APA_NONE;

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
  return;
}

void ParkingScenarioManager::ScenarioTry() {
  if (current_scenario_ == nullptr) {
    return;
  }

  if (!apa_world_->GetSlotManagerPtr()->IsTargetSlotReleaseByRule()) {
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
        temp_narrow_scenario->ThreadClearState();
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
    // todo: head in, head out, tail out.
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
                     !(astar_path_release == SlotReleaseState::RELEASE));
  }

  if (geometry_path_release == SlotReleaseState::NOT_RELEASE &&
      astar_path_release == SlotReleaseState::RELEASE) {
    ILOG_INFO << "use astar plan";
    return true;
  }
  ILOG_INFO << "use geometry plan";
  return false;
}

void ParkingScenarioManager::PubPreparePlanState() {
  if (!apa_world_->GetStateMachineManagerPtr()->IsSeachingStatus()) {
    apa_hmi_data_.prepare_plan_state = iflyauto::PREPARE_PLANNING_NONE;
    return;
  }

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

  if (PubPreparePathByStableStrategy()) {
    planning_output_ = current_scenario_->GetOutput();
  }

  ILOG_INFO << "release state = " << apa_hmi_data_.prepare_plan_state;

  return;
}

void ParkingScenarioManager::RecommendParkingDirection() {
  if (current_scenario_ == nullptr) {
    ILOG_ERROR << "current_scenario_ is nullptr, cannot get APA HMI data";
    return;
  }

  apa_hmi_data_.planning_park_dir =
      current_scenario_->GetAPAHmi().planning_park_dir;

  ILOG_INFO << "dir = " << apa_hmi_data_.planning_park_dir;

  return;
}

void ParkingScenarioManager::ClearPlanningOutput() {
  memset(&planning_output_, 0, sizeof(planning_output_));
  return;
}

const bool ParkingScenarioManager::PubPreparePathByStableStrategy() {
  if (current_scenario_ == nullptr) {
    ClearPlanningOutput();
    return false;
  }

  current_scenario_->RecordDebugPath();

  if (!apa_param.GetParam().prepare_plan_config.enable_stable_prepare_route) {
    return true;
  }

  auto &history_path = planning_output_.trajectory;
  auto &new_path = current_scenario_->GetOutput().trajectory;

  // If scenario planning fail, update path
  if (!IsTrajValid(new_path)) {
    return true;
  }

  // If history is invalid, update path
  if (!IsTrajValid(history_path)) {
    return true;
  }

  // If distance is big, update path.
  double dist = DistanceTo(history_path.trajectory_points[0],
                           new_path.trajectory_points[0]);
  if (dist > apa_param.GetParam().prepare_plan_config.start_point_error) {
    ILOG_INFO << "start point error is big" << dist;
    return true;
  }

  dist = DistanceTo(
      history_path.trajectory_points[history_path.trajectory_points_size - 1],
      new_path.trajectory_points[new_path.trajectory_points_size - 1]);
  if (dist > apa_param.GetParam().prepare_plan_config.terminal_point_error) {
    ILOG_INFO << "terminal error is big" << dist;
    return true;
  }

  return false;
}

void ParkingScenarioManager::PubStopReason() {
  apa_hmi_data_.parking_pause_reason = iflyauto::PARKING_PAUSE_OTHER;
  apa_hmi_data_.is_parking_pause = false;

  if (apa_world_->GetStateMachineManagerPtr()->IsSeachingStatus()) {
    return;
  }
  if (apa_world_->GetStateMachineManagerPtr()->IsParkSuspendStatus()) {
    return;
  }

  if (current_scenario_ == nullptr) {
    return;
  }

  if (current_scenario_->IsStopByDynamicObs()) {
    apa_hmi_data_.parking_pause_reason = iflyauto::PARKING_PAUSE_OTHER;
    apa_hmi_data_.is_parking_pause = true;
  } else if (current_scenario_->IsStopByStaticMovableObs()) {
    apa_hmi_data_.parking_pause_reason = iflyauto::PARKING_PAUSE_OTHER;
    apa_hmi_data_.is_parking_pause = true;
  }

  return;
}

void ParkingScenarioManager::ScenarioSuspend() {
  if (current_scenario_ == nullptr) {
    return;
  }
  current_scenario_->ScenarioSuspend();

  return;
}

}  // namespace apa_planner
}  // namespace planning

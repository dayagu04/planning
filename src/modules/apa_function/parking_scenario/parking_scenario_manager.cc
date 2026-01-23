#include "parking_scenario_manager.h"

#include <algorithm>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "apa_context.h"
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
      apa_world_->GetStateMachineManagerPtr()->IsParkInvalidStatus()) {
    Reset();
  }

  const ApaStateMachine &cur_state =
      apa_world_->GetStateMachineManagerPtr()->GetStateMachine();

  const EgoInfoUnderSlot &ego_info_under_slot =
      apa_world_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const ApaParameters &param = apa_param.GetParam();

  const SlotType slot_type = ego_info_under_slot.slot_type;

  if (cur_state == ApaStateMachine::SEARCH_IN_SELECTED_CAR_REAR ||
      cur_state == ApaStateMachine::ACTIVE_IN_CAR_REAR) {
    if (slot_type == SlotType::PERPENDICULAR || slot_type == SlotType::SLANT) {
      if (param.park_path_plan_type == ParkPathPlanType::HYBRID_ASTAR_THREAD) {
        if (slot_type == SlotType::PERPENDICULAR) {
          scenario_type_ = ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN;
        } else if (slot_type == SlotType::SLANT) {
          scenario_type_ = ParkingScenarioType::SCENARIO_SLANT_TAIL_IN;
        }
      } else {
        if (param.path_generator_type ==
            ParkPathGenerationType::GEOMETRY_BASED) {
          // check is narrow space or not
          if (IsSlotReleaseByHybridAstar()) {
            scenario_type_ = ParkingScenarioType::SCENARIO_NARROW_SPACE;
          } else {
            if (slot_type == SlotType::PERPENDICULAR) {
              scenario_type_ =
                  ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN;
            } else if (slot_type == SlotType::SLANT) {
              scenario_type_ = ParkingScenarioType::SCENARIO_SLANT_TAIL_IN;
            }
          }
        } else {
          // only use astar
          scenario_type_ = ParkingScenarioType::SCENARIO_NARROW_SPACE;
        }
      }
    } else if (slot_type == SlotType::PARALLEL) {
      if (param.path_generator_type == ParkPathGenerationType::GEOMETRY_BASED) {
        scenario_type_ = ParkingScenarioType::SCENARIO_PARALLEL_IN;
      } else {
        scenario_type_ = ParkingScenarioType::SCENARIO_NARROW_SPACE;
      }
    }
  } else if (cur_state == ApaStateMachine::SEARCH_IN_SELECTED_CAR_FRONT ||
             cur_state == ApaStateMachine::ACTIVE_IN_CAR_FRONT) {
    if (slot_type == SlotType::PERPENDICULAR || slot_type == SlotType::SLANT) {
      scenario_type_ = ParkingScenarioType::SCENARIO_NARROW_SPACE;
    } else if (slot_type == SlotType::PARALLEL) {
      if (param.path_generator_type == ParkPathGenerationType::GEOMETRY_BASED) {
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
    if (slot_type == SlotType::PERPENDICULAR || slot_type == SlotType::SLANT) {
      if (param.use_geometry_path_head_out) {
        scenario_type_ = ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_OUT;
      } else {
        // 垂直泊出功能不使用几何规划时，设置 path_generator_type 为
        // SEARCH_BASED ，进行 hybrid a*；
        scenario_type_ = ParkingScenarioType::SCENARIO_NARROW_SPACE;
      }
    } else if (slot_type == SlotType::PARALLEL) {
      if (param.path_generator_type == ParkPathGenerationType::GEOMETRY_BASED) {
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
  } else if (apa_world_->GetStateMachineManagerPtr()->GetStateMachine() ==
             ApaStateMachine::MANUAL_PARKING) {
    scenario_status_ = ParkingScenarioStatus::STATUS_MANUAL;
  }

  current_scenario_ = GetScenarioByType(scenario_type_);

  PrintApaScenarioType(scenario_type_);
  PrintApaScenarioStatus(scenario_status_);
}

bool ParkingScenarioManager::Process() {
  bool planning_success = false;
  if (scenario_status_ == ParkingScenarioStatus::STATUS_RUNNING) {
    planning_success = ScenarioRunning();
  } else if (scenario_status_ == ParkingScenarioStatus::STATUS_TRY) {
    planning_success = ScenarioTry();
  } else if (scenario_status_ == ParkingScenarioStatus::STATUS_SUSPEND) {
    planning_success = ScenarioSuspend();
  } else if (scenario_status_ == ParkingScenarioStatus::STATUS_MANUAL) {
    planning_success = SetFunctionRecommendPark();
  }

  return planning_success;
}

void ParkingScenarioManager::Reset() {
  memset(&planning_output_, 0, sizeof(planning_output_));
  memset(&apa_hmi_data_, 0, sizeof(apa_hmi_data_));

  scenario_type_ = ParkingScenarioType::SCENARIO_UNKNOWN;
  scenario_status_ = ParkingScenarioStatus::STATUS_UNKNOWN;
  is_last_parking_pause_ = false;

  // reset all planner
  for (const auto &scene : scenario_list_) {
    scene.second->Reset();
  }

  return;
}

void ParkingScenarioManager::ClearHybridResponse() {
  for (const auto &scene : scenario_list_) {
    scene.second->ClearHybridResponse();
  }
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

bool ParkingScenarioManager::ScenarioRunning() {
  if (current_scenario_ == nullptr) {
    Reset();
    planning_output_.planning_status.apa_planning_status = iflyauto::APA_FAILED;
    return false;
  }
  current_scenario_->ScenarioRunning();

  planning_output_ = current_scenario_->GetOutput();

  apa_hmi_data_ = current_scenario_->GetAPAHmi();

  PubStopReason();

  if (scenario_type_ == ParkingScenarioType::SCENARIO_NARROW_SPACE) {
    JSON_DEBUG_VALUE("geometry_path_release", false);
  } else {
    JSON_DEBUG_VALUE("geometry_path_release", true);
  }

  const auto& parking_frame = current_scenario_->GetFrame();
  ILOG_INFO << "scenario running";
  return parking_frame.plan_stm.path_plan_success;
}

bool ParkingScenarioManager::ScenarioTry() {
  if (current_scenario_ == nullptr) {
    return false;
  }

  if (!apa_world_->GetSlotManagerPtr()->IsTargetSlotReleaseByRule()) {
    ILOG_INFO << "Exit IsTargetSlotReleaseByRule prepare_plan_state: "
              << apa_hmi_data_.prepare_plan_state;
    if (apa_world_->GetStateMachineManagerPtr()->IsSAPAMode()) {
      PubPreparePlanStateFreeSlot();
    }
    return false;
  }

  const ApaStateMachine &cur_state =
      apa_world_->GetStateMachineManagerPtr()->GetStateMachine();

  const EgoInfoUnderSlot &ego_info_under_slot =
      apa_world_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const ApaParameters &param = apa_param.GetParam();
  if (param.park_path_plan_type == ParkPathPlanType::HYBRID_ASTAR_THREAD) {
    current_scenario_->ScenarioTry();
  }

  else {
    if (cur_state == ApaStateMachine::SEARCH_IN_SELECTED_CAR_REAR) {
      // 车尾泊入功能
      if (ego_info_under_slot.slot_type == SlotType::PERPENDICULAR ||
          ego_info_under_slot.slot_type == SlotType::SLANT) {
        std::shared_ptr<ParkingScenario> temp_narrow_scenario =
            scenario_list_[ParkingScenarioType::SCENARIO_NARROW_SPACE];

        std::shared_ptr<ParkingScenario> temp_perpendicular_scenario =
            scenario_list_[ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN];

        std::shared_ptr<ParkingScenario> temp_slant_scenario =
            scenario_list_[ParkingScenarioType::SCENARIO_SLANT_TAIL_IN];

        if (apa_param.GetParam().path_generator_type ==
            ParkPathGenerationType::GEOMETRY_BASED) {
          // 先用几何尝试
          if (ego_info_under_slot.slot_type == SlotType::PERPENDICULAR) {
            temp_perpendicular_scenario->ScenarioTry();
          } else if (ego_info_under_slot.slot_type == SlotType::SLANT) {
            temp_slant_scenario->ScenarioTry();
          }
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
  }

  planning_output_ = current_scenario_->GetOutput();

  apa_hmi_data_ = current_scenario_->GetAPAHmi();

  PublishPreparePlanInfo();

  ILOG_INFO << "GEOMETRY RELEASE = "
            << GetSlotReleaseStateString(
                   ego_info_under_slot.slot.release_info_
                       .release_state[GEOMETRY_PLANNING_RELEASE])
            << ", ASTAR RELEASE = "
            << GetSlotReleaseStateString(
                   ego_info_under_slot.slot.release_info_
                       .release_state[ASTAR_PLANNING_RELEASE]);
  return true;
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

  if (geometry_path_release == SlotReleaseState::RELEASE) {
    return false;
  }

  return astar_path_release != SlotReleaseState::UNKNOWN;
}

void ParkingScenarioManager::PublishPreparePlanInfo() {
  if (!apa_world_->GetStateMachineManagerPtr()->IsSeachingStatus()) {
    return;
  }

  // fill prepare plan traj
  // if true, use current scenario traj and update history prepare plan traj
  // if false, use history prepare plan traj to keep stable
  if (!PubPreparePathByStableStrategy()) {
    ILOG_INFO << "use history prepare plan traj";
    planning_output_.trajectory = history_prepare_plan_traj_;
  } else {
    ILOG_INFO << "use current scenario traj";
    history_prepare_plan_traj_ = planning_output_.trajectory;
  }

  ILOG_INFO << "pre traj pt size = "
            << static_cast<int>(
                   planning_output_.trajectory.trajectory_points_size);

  // fill prepare plan state
  switch (apa_world_->GetSlotManagerPtr()->GetSlotReleaseState()) {
    case SlotReleaseState::NOT_RELEASE:
      apa_hmi_data_.prepare_plan_state = iflyauto::PREPARE_PLANNING_FAILED;
      break;
    case SlotReleaseState::RELEASE:
      apa_hmi_data_.prepare_plan_state = iflyauto::PREPARE_PLANNING_SUCCESS;
      break;
    case SlotReleaseState::UNKNOWN:
      apa_hmi_data_.prepare_plan_state = iflyauto::PREPARE_PLANNING_NONE;
      break;
    default:
      apa_hmi_data_.prepare_plan_state = iflyauto::PREPARE_PLANNING_COMPUTING;
      break;
  }

  ILOG_INFO << "prepare_plan_state = " << apa_hmi_data_.prepare_plan_state;

  // fill prepare plan recommend park direction
  apa_hmi_data_.planning_park_dir =
      current_scenario_->GetAPAHmi().planning_park_dir;
  apa_hmi_data_.planning_recommend_park_dir =
      current_scenario_->GetAPAHmi().planning_recommend_park_dir;
  ILOG_INFO << "release park dir = " << apa_hmi_data_.planning_park_dir
            << ", recommend park dir = "
            << apa_hmi_data_.planning_recommend_park_dir;
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
    case SlotReleaseState::UNKNOWN:
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

void ParkingScenarioManager::PubPreparePlanStateFreeSlot() {
  if (!apa_world_->GetStateMachineManagerPtr()->IsSeachingStatus()) {
    apa_hmi_data_.prepare_plan_state = iflyauto::PREPARE_PLANNING_NONE;
    return;
  }

  SlotReleaseState state =
      apa_world_->GetSlotManagerPtr()->GetSlotReleaseStateFreeSlot();
  switch (state) {
    case SlotReleaseState::NOT_RELEASE:
      apa_hmi_data_.prepare_plan_state = iflyauto::PREPARE_PLANNING_FAILED;
      break;
    case SlotReleaseState::RELEASE:
      apa_hmi_data_.prepare_plan_state = iflyauto::PREPARE_PLANNING_SUCCESS;
      break;
    case SlotReleaseState::UNKNOWN:
      if (apa_world_->GetStateMachineManagerPtr()->IsSAPAMode()) {
        apa_hmi_data_.prepare_plan_state = iflyauto::PREPARE_PLANNING_FAILED;
      } else {
        apa_hmi_data_.prepare_plan_state = iflyauto::PREPARE_PLANNING_NONE;
      }
      break;
    default:
      apa_hmi_data_.prepare_plan_state = iflyauto::PREPARE_PLANNING_COMPUTING;
      break;
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

  apa_hmi_data_.planning_recommend_park_dir =
      current_scenario_->GetAPAHmi().planning_recommend_park_dir;

  ILOG_INFO << "release dir = " << apa_hmi_data_.planning_park_dir
            << ", recommend park dir = "
            << apa_hmi_data_.planning_recommend_park_dir;

  return;
}

void ParkingScenarioManager::ClearPlanningOutput() {
  memset(&planning_output_, 0, sizeof(planning_output_));
  return;
}

const bool ParkingScenarioManager::PubPreparePathByStableStrategy() {
  if (current_scenario_ == nullptr) {
    return false;
  }

  current_scenario_->RecordDebugPath();

  if (!apa_param.GetParam().prepare_plan_config.enable_stable_prepare_route) {
    return true;
  }

  const auto &history_path = history_prepare_plan_traj_;
  const auto &new_path = current_scenario_->GetOutput().trajectory;

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
  apa_hmi_data_.parking_pause_reason = iflyauto::PARKING_PAUSE_NONE;
  apa_hmi_data_.is_parking_pause = false;

  if (apa_world_->GetStateMachineManagerPtr()->IsParkSuspendStatus()) {
    if (is_last_parking_pause_) {
      // The current frame is paused actively
    } else {
      // The current frame is paused passively
      return;
    }
  }

  is_last_parking_pause_ = false;

  if (!apa_world_->GetMeasureDataManagerPtr()->GetStaticFlag()) {
    return;
  }

  if (apa_world_->GetStateMachineManagerPtr()->IsSeachingStatus()) {
    return;
  }
  if (current_scenario_ == nullptr) {
    return;
  }

  if (current_scenario_->IsStopByDynamicObs()) {
    apa_hmi_data_.parking_pause_reason = iflyauto::PARKING_PAUSE_BY_DYNAMIC_OBS;
    apa_hmi_data_.is_parking_pause = true;
  } else if (current_scenario_->IsStopByStaticMovableObs()) {
    apa_hmi_data_.parking_pause_reason = iflyauto::PARKING_PAUSE_FOR_STATIC_OBS;
    apa_hmi_data_.is_parking_pause = true;
  }

  is_last_parking_pause_ = apa_hmi_data_.is_parking_pause;
  ILOG_INFO << "stop reason = " << apa_hmi_data_.parking_pause_reason;

  return;
}

bool ParkingScenarioManager::ScenarioSuspend() {
  if (current_scenario_ == nullptr) {
    return false;
  }
  current_scenario_->ScenarioSuspend();

  PubStopReason();

  return true;
}

bool ParkingScenarioManager::SetFunctionRecommendPark() {
  apa_hmi_data_.recommend_park_out =
      apa_world_->GetSlotManagerPtr()->GetRecommendParkOut() ? TRUE : FALSE;
  return true;
}

}  // namespace apa_planner
}  // namespace planning

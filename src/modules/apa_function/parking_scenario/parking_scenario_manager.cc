#include "parking_scenario_manager.h"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "log_glog.h"
#include "narrow_space/narrow_space_scenario.h"
#include "narrow_space_decider.h"
#include "narrow_space_scenario.h"
#include "parallel_park_in_scenario.h"
#include "perpendicular_head_in_scenario.h"
#include "perpendicular_head_out_scenario.h"
#include "perpendicular_park_scenario.h"
#include "perpendicular_tail_in_scenario.h"


namespace planning {
namespace apa_planner {

bool ParkingScenarioManager::Init(
    const std::shared_ptr<apa_planner::ApaWorld> &world) {
  if (init_) {
    return true;
  }

  // init planners
  scenario_list_[ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN] =
      std::make_shared<PerpendicularTailInScenario>(world);

  scenario_list_[ParkingScenarioType::SCENARIO_PARALLEL_IN] =
      std::make_shared<ParallelParkInScenario>(world);

  scenario_list_[ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_OUT] =
      std::make_shared<PerpendicularHeadOutScenario>(world);

  scenario_list_[ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN] =
      std::make_shared<PerpendicularHeadInScenario>(world);

  scenario_list_[ParkingScenarioType::SCENARIO_NARROW_SPACE] =
      std::make_shared<NarrowSpaceScenario>(world);

  default_scenario_ =
      scenario_list_[ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN];

  current_scenario_ = default_scenario_;
  init_ = true;
  type_ = ParkingScenarioType::SCENARIO_UNKNOWN;
  return true;
}

ParkingScenarioStatus ParkingScenarioManager::Excute(
    std::shared_ptr<apa_planner::ApaData> apa_data) {
  ParkingScenarioStatus scenario_status = ParkingScenarioStatus::STATUS_UNKNOWN;

  if (apa_data->cur_state == ApaStateMachine::ACTIVE_WAIT_IN ||
      apa_data->cur_state == ApaStateMachine::ACTIVE_IN ||
      apa_data->cur_state == ApaStateMachine::SEARCH_IN) {
    if (apa_data->slot_type ==
        Common::ParkingSlotType::PARKING_SLOT_TYPE_VERTICAL) {
      if (apa_param.GetParam().path_generator_type ==
          ParkPathGenerationType::GEOMETRY_BASED) {
        if (apa_param.GetParam().is_heading_in) {
          apa_data->scenario_type =
              ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN;

          ILOG_INFO << "planner_type = PERPENDICULAR_PARK_HEADING_IN!";
        } else {
          apa_data->scenario_type =
              ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN;

          NarrowScenarioDecider *narrow_scenario_decider =
              NarrowScenarioDecider::GetNarrowScenarioDecider();
          if (narrow_scenario_decider->IsNeedAstar()) {
            apa_data->scenario_type =
                ParkingScenarioType::SCENARIO_NARROW_SPACE;
          }

          ILOG_INFO << "planner_type = PERPENDICULAR_PARK_IN!";
        }
      } else {
        apa_data->scenario_type = ParkingScenarioType::SCENARIO_NARROW_SPACE;
      }
      ILOG_INFO << "path plan method = "
                << static_cast<int>(apa_data->scenario_type);

    } else if (apa_data->slot_type ==
               Common::ParkingSlotType::PARKING_SLOT_TYPE_HORIZONTAL) {
      ILOG_INFO << "planner_type = PARALLEL_PARK_IN!";
      if (apa_param.GetParam().path_generator_type ==
          ParkPathGenerationType::GEOMETRY_BASED) {
        apa_data->scenario_type = ParkingScenarioType::SCENARIO_PARALLEL_IN;

      } else {
        apa_data->scenario_type = ParkingScenarioType::SCENARIO_NARROW_SPACE;
      }
    } else if (apa_data->slot_type ==
               Common::ParkingSlotType::PARKING_SLOT_TYPE_SLANTING) {
      ILOG_INFO << "planner_type = SLANT_PARK_IN!";
      apa_data->scenario_type = ParkingScenarioType::SCENARIO_SLANT_TAIL_IN;
    } else {
      ILOG_INFO << "current slot type is not supported now!";
    }
  } else if (apa_data->cur_state == ApaStateMachine::ACTIVE_OUT) {
    DEBUG_PRINT("planner_type = PERPENDICULAR_PARK_OUT!");
    apa_data->scenario_type =
        ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_OUT;
  }

  if (apa_data->cur_state == ApaStateMachine::ACTIVE_WAIT_IN ||
      apa_data->cur_state == ApaStateMachine::ACTIVE_IN) {
    scenario_status = ParkingScenarioStatus::STATUS_RUNNING;
  } else if (apa_data->cur_state == ApaStateMachine::SEARCH_IN &&
             apa_data->slot_type !=
                 Common::ParkingSlotType::PARKING_SLOT_TYPE_INVALID &&
             apa_data->slot_id != 0) {
    scenario_status = ParkingScenarioStatus::STATUS_TRY;
  }

  for (const auto &scene : scenario_list_) {
    if (scene.first == apa_data->scenario_type) {
      current_scenario_ = scenario_list_[apa_data->scenario_type];
      ILOG_INFO << static_cast<int>(apa_data->scenario_type) << " update.";

      if (current_scenario_->GetStatus() !=
          ParkingScenarioStatus::STATUS_RUNNING) {
        current_scenario_->Reset();
        current_scenario_->Enter(scenario_status);
      }

      type_ = apa_data->scenario_type;
      return ParkingScenarioStatus::STATUS_RUNNING;
    }
  }
  ILOG_INFO << "scenario type error!";

  return ParkingScenarioStatus::STATUS_UNKNOWN;
}

void ParkingScenarioManager::Reset() {
  if (current_scenario_) {
    current_scenario_->Exit();
  }
  type_ = ParkingScenarioType::SCENARIO_UNKNOWN;
  ILOG_INFO << "scenario=" << static_cast<int>(type_);

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

  ILOG_ERROR << "invalid index";
  return nullptr;
}
}  // namespace apa_planner
}  // namespace planning

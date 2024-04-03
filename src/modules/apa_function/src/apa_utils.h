#pragma once

#include "dubins_lib.h"
#include "func_state_machine_c.h"
#include "fusion_parking_slot_c.h"
#include "localization_c.h"
#include "math/polygon2d.h"
#include "planning_plan_c.h"
#include "session.h"

namespace planning {

bool IsSlotSelected(framework::Session* session);

bool IsReplanEachFrame(const iflyauto::FuncStateMachine& func_state_machine);

bool IsReplanNecessary(const iflyauto::FuncStateMachine& func_state_machine);

bool IsSlotLineCrossable(
    const iflyauto::ParkingFusionSlot& parking_fusion_slot);

void SetStoppingPlanningOutput(iflyauto::PlanningOutput& planning_output,
                               const pnc::geometry_lib::PathPoint& ego_pose);

void SetFinishedPlanningOutput(iflyauto::PlanningOutput& planning_output,
                               const pnc::geometry_lib::PathPoint& ego_pose);

void SetFailedPlanningOutput(iflyauto::PlanningOutput& planning_output,
                             const pnc::geometry_lib::PathPoint& ego_pose);

void SetIdlePlanningOutput(iflyauto::PlanningOutput& planning_output,
                           const pnc::geometry_lib::PathPoint& ego_pose);

bool IsValidParkingState(const iflyauto::FunctionalState& func_state_machine);
}  // namespace  planning

#pragma once

#include "dubins_lib.h"
#include "frame.h"
#include "func_state_machine.pb.h"
#include "localization.pb.h"
#include "math/polygon2d.h"
#include "parking_fusion.pb.h"
#include "planning_plan.pb.h"

namespace planning {

bool IsSlotSelected(framework::Frame* const frame);

bool IsReplanEachFrame(
    const FuncStateMachine::FuncStateMachine& func_state_machine);

bool IsReplanNecessary(
    const FuncStateMachine::FuncStateMachine& func_state_machine);

bool IsSlotLineCrossable(
    const ParkingFusion::ParkingFusionSlot& parking_fusion_slot);

void SetStoppingPlanningOutput(
    PlanningOutput::PlanningOutput& planning_output,
    const pnc::geometry_lib::PathPoint& ego_pose);

void SetFinishedPlanningOutput(
    PlanningOutput::PlanningOutput& planning_output,
    const pnc::geometry_lib::PathPoint& ego_pose);

void SetFailedPlanningOutput(
    PlanningOutput::PlanningOutput& planning_output,
    const pnc::geometry_lib::PathPoint& ego_pose);

void SetIdlePlanningOutput(
    PlanningOutput::PlanningOutput& planning_output,
    const pnc::geometry_lib::PathPoint& ego_pose);

bool IsValidParkingState(
    const ::FuncStateMachine::FunctionalState& func_state_machine);
}  // namespace  planning

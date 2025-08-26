#pragma once

#include "dubins_lib.h"
#include "func_state_machine_c.h"
#include "fusion_parking_slot_c.h"
#include "math/polygon2d.h"
#include "planning_plan_c.h"
#include "session.h"
#include "src/library/hybrid_astar_lib/hybrid_astar_common.h"

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

// if in apa function: can switch to apa;
// if in hpp searching or parking, can switch to apa;
bool IsSwitchApaState(const iflyauto::FunctionalState& func_state_machine);

bool IsHppSlotSearchingStage(const iflyauto::FunctionalState& current_state);

bool IsHppParkingStage(const iflyauto::FunctionalState& current_state);

bool IsValidApaState(const iflyauto::FunctionalState& current_state);

// check need slot searching, or need parking in apa or hpp.
bool IsSlotSearchingOrParking(const iflyauto::FunctionalState& current_state);

const bool IsTrajValid(const iflyauto::Trajectory& traj);

const ParkingVehDirection GetParkDir(const int dir);

const bool IsODVeh(const iflyauto::ObjectType type);

const bool IsODSpecificationer(const iflyauto::ObjectType type);

const bool IsDynamicOD(const double v, const iflyauto::ObjectType type);

const bool IsMovableStaticOD(const double v, const iflyauto::ObjectType type);

}  // namespace  planning

#pragma once

#include "func_state_machine.pb.h"
#include "parking_fusion.pb.h"
#include "planning_plan.pb.h"

#include "common/geometry_planning_io.h"
#include "frame.h"
#include "math/polygon2d.h"

namespace planning {

planning::planning_math::Polygon2d ConstructVehiclePolygon(
    const PlanningPoint& rear_center, const double half_width,
    const double front_edge_to_rear_center,
    const double rear_edge_to_rear_center, const double front_shrink_dis,
    const double front_side_shrink_dis, const double rear_shrink_dis,
    const double rear_side_shrink_dis);

planning_math::Polygon2d ConstructVehiclePolygonWithBuffer(
    const PlanningPoint& veh_point, const double front_buffer,
    const double rear_buffer, const double lat_buffer);

bool IsSlotSelected(framework::Frame* const frame);

bool IsRoughCalc(framework::Frame* const frame);

bool IsReplanEachFrame(
    const FuncStateMachine::FuncStateMachine& func_state_machine);

bool IsReplanNecessary(
    const FuncStateMachine::FuncStateMachine& func_state_machine);

bool IsSlotLineCrossable(
    const ParkingFusion::ParkingFusionSlot& parking_fusion_slot);

void SetStoppingPlanningOutput(framework::Frame* const frame);

void SetFinishedPlanningOutput(framework::Frame* const frame);

void SetFailedPlanningOutput(framework::Frame* const frame);

bool IsValidParkingState(
    const FuncStateMachine::FuncStateMachine& func_state_machine);
}  // namespace  planning

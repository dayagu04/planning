#pragma once
#include "planning_plan.pb.h"
#include "control_command.pb.h"
#include "localization.pb.h"
#include "func_state_machine.pb.h"

namespace planning
{
// LocalView contains all necessary received data in one frame.
struct VirtualChassisLocalView
{
    PlanningOutput::PlanningOutput planning_;
    ControlCommand::ControlOutput control_;
    FuncStateMachine::FuncStateMachine fsm_;
};
}  // namespace apollo
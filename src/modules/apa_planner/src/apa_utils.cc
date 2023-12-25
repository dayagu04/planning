#include "apa_utils.h"

#include <algorithm>
#include <cstdint>
#include <string>

#include "environmental_model.h"
#include "planning_plan.pb.h"
#include "session.h"

namespace planning {

using ::FuncStateMachine::FuncStateMachine;
using ::FuncStateMachine::FunctionalState;
using framework::Frame;

bool IsSlotSelected(Frame* const frame) {
  const auto& func_state_machine = frame->session()
                                       ->environmental_model()
                                       .get_local_view()
                                       .function_state_machine_info;

  if (!func_state_machine.has_current_state()) {
    AERROR << "func_state_machine is invalid";
    return false;
  }

  if (func_state_machine.current_state() >= FunctionalState::PARK_IN_SELECT &&
      func_state_machine.current_state() <= FunctionalState::PARK_OUT_SECURE) {
    return true;
  }

  return false;
}

bool IsReplanEachFrame(const FuncStateMachine& func_state_machine) {
  if (!func_state_machine.has_current_state()) {
    AERROR << "func_state_machine is invalid";
    return false;
  }

  if (func_state_machine.current_state() >=
          FunctionalState::PARK_IN_SEARCHING &&
      func_state_machine.current_state() <=
          FunctionalState::PARK_IN_ACTIVATE_WAIT) {
    return true;
  }

  return false;
}

bool IsReplanNecessary(const FuncStateMachine& func_state_machine) {
  if (!func_state_machine.has_current_state()) {
    AERROR << "func_state_machine is invalid";
    return false;
  }

  return IsReplanEachFrame(func_state_machine) ||
         func_state_machine.current_state() ==
             FunctionalState::PARK_IN_ACTIVATE_CONTROL;
}

void SetStoppingPlanningOutput(PlanningOutput::PlanningOutput& planning_output,
                               const pnc::geometry_lib::PathPoint& ego_pose) {
  auto trajectory = planning_output.mutable_trajectory();

  trajectory->set_available(true);
  trajectory->set_trajectory_type(
      Common::TrajectoryType::TRAJECTORY_TYPE_TRAJECTORY_POINTS);

  auto gear_command = planning_output.mutable_gear_command();
  gear_command->set_available(true);
  gear_command->set_gear_command_value(
      Common::GearCommandValue::GEAR_COMMAND_VALUE_PARKING);

  trajectory->mutable_trajectory_points()->Clear();

  ::PlanningOutput::TrajectoryPoint* trajectory_point =
      trajectory->add_trajectory_points();
  trajectory_point->set_x(ego_pose.pos.x());
  trajectory_point->set_y(ego_pose.pos.y());
  trajectory_point->set_heading_yaw(ego_pose.heading);
  trajectory_point->set_curvature(0.0);
  trajectory_point->set_t(0.0);
  trajectory_point->set_v(0.0);
  trajectory_point->set_a(0.0);
  trajectory_point->set_distance(0.0);
  trajectory_point->set_jerk(0.0);
}

void SetFinishedPlanningOutput(PlanningOutput::PlanningOutput& planning_output,
                               const pnc::geometry_lib::PathPoint& ego_pose) {
  planning_output.mutable_planning_status()->set_apa_planning_status(
      ::PlanningOutput::ApaPlanningStatus::FINISHED);

  SetStoppingPlanningOutput(planning_output, ego_pose);
  std::cout << "set finished planning output" << std::endl;
}

void SetFailedPlanningOutput(PlanningOutput::PlanningOutput& planning_output,
                             const pnc::geometry_lib::PathPoint& ego_pose) {
  planning_output.mutable_planning_status()->set_apa_planning_status(
      ::PlanningOutput::ApaPlanningStatus::FAILED);

  SetStoppingPlanningOutput(planning_output, ego_pose);
  std::cout << "set failed planning output" << std::endl;
}

void SetIdlePlanningOutput(PlanningOutput::PlanningOutput& planning_output,
                           const pnc::geometry_lib::PathPoint& ego_pose) {
  planning_output.mutable_planning_status()->set_apa_planning_status(
      ::PlanningOutput::ApaPlanningStatus::NONE);

  SetStoppingPlanningOutput(planning_output, ego_pose);
  std::cout << "set idle planning output" << std::endl;
}

bool IsValidParkingState(
    const ::FuncStateMachine::FunctionalState& current_state) {
  std::cout << "current_state:" << current_state << std::endl;

  if (current_state >= FunctionalState::PARK_IN_APA_IN &&
      current_state <= FunctionalState::PARK_OUT_COMPLETED) {
    return true;
  }
  return false;
}

}  // namespace  planning

#include "apa_utils.h"

#include <algorithm>
#include <cstdint>
#include <string>

#include "environmental_model.h"
#include "planning_plan_c.h"
#include "session.h"

namespace planning {

bool IsSlotSelected(framework::Session* session) {
  const auto& func_state_machine = session->environmental_model()
                                       .get_local_view()
                                       .function_state_machine_info;

  if (func_state_machine.current_state >=
          iflyauto::FunctionalState_PARK_GUIDANCE &&
      func_state_machine.current_state <=
          iflyauto::FunctionalState_PARK_COMPLETED) {
    return true;
  }

  return false;
}

bool IsReplanEachFrame(const iflyauto::FuncStateMachine& func_state_machine) {
  if (func_state_machine.current_state >=
      iflyauto::FunctionalState_PARK_IN_SEARCHING
      //     &&
      // func_state_machine.current_state <=
      //     iflyauto::FunctionalState_PARK_IN_ACTIVATE_WAIT
  ) {
    return true;
  }

  return false;
}

bool IsReplanNecessary(const iflyauto::FuncStateMachine& func_state_machine) {
  return IsReplanEachFrame(func_state_machine) ||
         func_state_machine.current_state ==
             iflyauto::FunctionalState_PARK_GUIDANCE;
}

void SetStoppingPlanningOutput(iflyauto::PlanningOutput& planning_output,
                               const pnc::geometry_lib::PathPoint& ego_pose) {
  auto trajectory = &(planning_output.trajectory);

  trajectory->available = true;
  trajectory->trajectory_type = iflyauto::TRAJECTORY_TYPE_TRAJECTORY_POINTS;

  auto gear_command = &(planning_output.gear_command);
  gear_command->available = true;
  gear_command->gear_command_value = iflyauto::GEAR_COMMAND_VALUE_PARKING;

  trajectory->trajectory_points_size = 1;
  auto trajectory_point = &(trajectory->trajectory_points[0]);
  trajectory_point->x = ego_pose.pos.x();
  trajectory_point->y = ego_pose.pos.y();
  trajectory_point->heading_yaw = ego_pose.heading;
  trajectory_point->curvature = 0.0;
  trajectory_point->t = 0.0;
  trajectory_point->v = 0.0;
  trajectory_point->a = 0.0;
  trajectory_point->distance = 0.0;
  trajectory_point->jerk = 0.0;
}

void SetFinishedPlanningOutput(iflyauto::PlanningOutput& planning_output,
                               const pnc::geometry_lib::PathPoint& ego_pose) {
  planning_output.planning_status.apa_planning_status = iflyauto::APA_FINISHED;

  SetStoppingPlanningOutput(planning_output, ego_pose);
  std::cout << "set finished planning output" << std::endl;
}

void SetFailedPlanningOutput(iflyauto::PlanningOutput& planning_output,
                             const pnc::geometry_lib::PathPoint& ego_pose) {
  planning_output.planning_status.apa_planning_status = iflyauto::APA_FAILED;

  SetStoppingPlanningOutput(planning_output, ego_pose);
  std::cout << "set failed planning output" << std::endl;
}

void SetIdlePlanningOutput(iflyauto::PlanningOutput& planning_output,
                           const pnc::geometry_lib::PathPoint& ego_pose) {
  planning_output.planning_status.apa_planning_status = iflyauto::APA_NONE;

  SetStoppingPlanningOutput(planning_output, ego_pose);
  std::cout << "set idle planning output" << std::endl;
}

bool IsValidParkingState(const iflyauto::FunctionalState& current_state) {
  std::cout << "current_state:" << current_state << std::endl;

  if (current_state >= iflyauto::FunctionalState_PARK_IN_SEARCHING &&
      current_state <= iflyauto::FunctionalState_PARK_COMPLETED) {
    return true;
  }
  return false;
}

}  // namespace  planning

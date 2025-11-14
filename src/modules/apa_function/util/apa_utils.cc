#include "apa_utils.h"

#include <algorithm>
#include <cstdint>
#include <string>

#include "environmental_model.h"
#include "planning_plan_c.h"
#include "session.h"
#include "src/modules/apa_function/apa_param_config.h"

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
  planning_output.planning_status.hpp_planning_status = iflyauto::HPP_COMPLETED;
  planning_output.planning_status.apa_planning_status = iflyauto::APA_FINISHED;

  SetStoppingPlanningOutput(planning_output, ego_pose);
  // std::cout << "set finished planning output" << std::endl;
}

void SetFailedPlanningOutput(iflyauto::PlanningOutput& planning_output,
                             const pnc::geometry_lib::PathPoint& ego_pose) {
  planning_output.planning_status.hpp_planning_status =
      iflyauto::HPP_RUNNING_FAILED;
  planning_output.planning_status.apa_planning_status = iflyauto::APA_FAILED;

  SetStoppingPlanningOutput(planning_output, ego_pose);
  // std::cout << "set failed planning output" << std::endl;
}

void SetIdlePlanningOutput(iflyauto::PlanningOutput& planning_output,
                           const pnc::geometry_lib::PathPoint& ego_pose) {
  planning_output.planning_status.hpp_planning_status = iflyauto::HPP_UNKNOWN;
  planning_output.planning_status.apa_planning_status = iflyauto::APA_NONE;

  SetStoppingPlanningOutput(planning_output, ego_pose);
  // std::cout << "set idle planning output" << std::endl;
}

bool IsValidApaState(const iflyauto::FunctionalState& current_state) {
  if ((current_state >= iflyauto::FunctionalState_PARK_STANDBY &&
       current_state <= iflyauto::FunctionalState_PARK_PRE_ACTIVE) ||
      current_state == iflyauto::FunctionalState_PARK_ERROR) {
    return true;
  }
  return false;
}

bool IsSwitchApaState(const iflyauto::FunctionalState& current_state) {
  // std::cout << "current_state:" << current_state << std::endl;

  if (IsValidApaState(current_state) || IsHppParkingStage(current_state)) {
    return true;
  }
  return false;
}

bool IsHppSlotSearchingStage(const iflyauto::FunctionalState& current_state) {
  if (current_state == iflyauto::FunctionalState_HPP_CRUISE_ROUTING ||
      current_state == iflyauto::FunctionalState_HPP_CRUISE_SEARCHING) {
    return true;
  }

  return false;
}

bool IsHppParkingStage(const iflyauto::FunctionalState& current_state) {
  if (current_state == iflyauto::FunctionalState_HPP_PARKING_OUT ||
      current_state == iflyauto::FunctionalState_HPP_PARKING_IN ||
      current_state == iflyauto::FunctionalState_HPP_SUSPEND) {
    return true;
  }

  return false;
}

bool IsSlotSearchingOrParking(const iflyauto::FunctionalState& current_state) {
  if (IsSwitchApaState(current_state) ||
      IsHppSlotSearchingStage(current_state)) {
    return true;
  }

  return false;
}

const bool IsTrajValid(const iflyauto::Trajectory& traj) {
  if (traj.trajectory_points_size <= 0 ||
      traj.trajectory_points_size > PLANNING_TRAJ_POINTS_MAX_NUM) {
    return false;
  }
  return true;
}

bool IsActiveApaState(const iflyauto::FunctionalState& fsm) {
  if (fsm == iflyauto::FunctionalState_PARK_GUIDANCE ||
      fsm == iflyauto::FunctionalState_PARK_SUSPEND ||
      fsm == iflyauto::FunctionalState_PARK_PRE_ACTIVE ||
      IsHppParkingStage(fsm)) {
    return true;
  }
  return false;
}

void UpdateVehicleParam(VehicleParam& vehicle_param) {
  const apa_planner::ApaParameters& params = apa_param.GetParam();

  // read vehicle params
  vehicle_param.length = params.car_length;
  vehicle_param.width = params.car_width;
  vehicle_param.max_width = params.max_car_width;
  vehicle_param.front_overhanging =
      params.car_length - params.rear_overhanging - params.wheel_base;
  vehicle_param.steer_ratio = params.steer_ratio;
  vehicle_param.wheel_base = params.wheel_base;
  vehicle_param.min_turn_radius = params.min_turn_radius + 0.01;
  vehicle_param.mirror_width = (params.max_car_width - params.car_width) * 0.5;
  vehicle_param.rear_edge_to_rear_axle = params.rear_overhanging;
  vehicle_param.front_edge_to_rear_axle =
      params.car_length - params.rear_overhanging;
  double front_wheel = std::atan(vehicle_param.wheel_base /
                                std::max(0.001, vehicle_param.min_turn_radius));
  vehicle_param.max_steer_angle =
      std::fabs(front_wheel * vehicle_param.steer_ratio);

  vehicle_param.mirror_lat_dist_to_center = params.lat_dist_mirror_to_center;
  vehicle_param.mirror_lon_dist_to_rear_axle =
      params.lon_dist_mirror_to_rear_axle;

  return;
}

}  // namespace  planning

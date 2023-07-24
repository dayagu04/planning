#include "common/apa_utils.h"

#include <algorithm>
#include <cstdint>
#include <string>

#include "common/apa_cos_sin.h"
#include "common/planning_log_helper.h"
#include "common/vehicle_param_helper.h"
#include "environmental_model.h"
#include "log_glog.h"
#include "planning_output_context.h"
#include "session.h"

namespace planning {

using ::FuncStateMachine::FuncStateMachine;
using ::FuncStateMachine::FunctionalState;
using ::ParkingFusion::ParkingFusionSlot;
using ::PlanningOutput::PlanningOutput;
using framework::Frame;
using planning::planning_math::Polygon2d;
using planning::planning_math::Vec2d;

Polygon2d ConstructVehiclePolygon(const PlanningPoint& rear_center,
                                  const double half_width,
                                  const double front_edge_to_rear_center,
                                  const double rear_edge_to_rear_center,
                                  const double front_shrink_dis,
                                  const double front_side_shrink_dis,
                                  const double rear_shrink_dis,
                                  const double rear_side_shrink_dis) {
  std::vector<Vec2d> points;
  points.reserve(8);
  const double sin_heading = apa_sin(rear_center.theta);
  const double cos_heading = apa_cos(rear_center.theta);
  const double dx1 = cos_heading * front_edge_to_rear_center;
  const double dy1 = sin_heading * front_edge_to_rear_center;
  const double dx2 =
      cos_heading * (front_edge_to_rear_center - front_side_shrink_dis);
  const double dy2 =
      sin_heading * (front_edge_to_rear_center - front_side_shrink_dis);

  const double dx3 = sin_heading * half_width;
  const double dy3 = cos_heading * half_width;
  const double dx4 = sin_heading * (half_width - front_shrink_dis);
  const double dy4 = cos_heading * (half_width - front_shrink_dis);

  const double dx5 = cos_heading * rear_edge_to_rear_center;
  const double dy5 = sin_heading * rear_edge_to_rear_center;
  const double dx6 =
      cos_heading * (rear_edge_to_rear_center - rear_side_shrink_dis);
  const double dy6 =
      sin_heading * (rear_edge_to_rear_center - rear_side_shrink_dis);

  const double dx7 = sin_heading * (half_width - rear_shrink_dis);
  const double dy7 = cos_heading * (half_width - rear_shrink_dis);

  points.emplace_back(rear_center.x + dx2 + dx3, rear_center.y + dy2 - dy3);
  points.emplace_back(rear_center.x + dx1 + dx4, rear_center.y + dy1 - dy4);
  points.emplace_back(rear_center.x + dx1 - dx4, rear_center.y + dy1 + dy4);
  points.emplace_back(rear_center.x + dx2 - dx3, rear_center.y + dy2 + dy3);
  points.emplace_back(rear_center.x - dx6 - dx3, rear_center.y - dy6 + dy3);
  points.emplace_back(rear_center.x - dx5 - dx7, rear_center.y - dy5 + dy7);
  points.emplace_back(rear_center.x - dx5 + dx7, rear_center.y - dy5 - dy7);
  points.emplace_back(rear_center.x - dx6 + dx3, rear_center.y - dy6 - dy3);

  return Polygon2d(points);
}

Polygon2d ConstructVehiclePolygonWithBuffer(const PlanningPoint& veh_point,
                                            const double front_buffer,
                                            const double rear_buffer,
                                            const double lat_buffer) {
  const double half_width_veh =
      VehicleParamHelper::Instance()->GetParam().width() * 0.5;
  const double front_edge_to_rear_axle =
      VehicleParamHelper::Instance()->GetParam().front_edge_to_rear_axle();
  const double rear_edge_to_rear_axle =
      VehicleParamHelper::Instance()->GetParam().rear_edge_to_rear_axle();
  const double front_shrink_dis =
      VehicleParamHelper::Instance()->GetParam().front_shrink_dis();
  const double front_side_shrink_dis =
      VehicleParamHelper::Instance()->GetParam().front_side_shrink_dis();
  const double rear_shrink_dis =
      VehicleParamHelper::Instance()->GetParam().rear_shrink_dis();
  const double rear_side_shrink_dis =
      VehicleParamHelper::Instance()->GetParam().rear_side_shrink_dis();

  const double front_edge_to_rear_axle_with_safe_dst =
      front_edge_to_rear_axle + front_buffer;
  const double rear_edge_to_rear_axle_with_safe_dst =
      rear_edge_to_rear_axle + rear_buffer;
  const double half_width_with_safe_dis = half_width_veh + lat_buffer;
  return ConstructVehiclePolygon(veh_point, half_width_with_safe_dis,
                                 front_edge_to_rear_axle_with_safe_dst,
                                 rear_edge_to_rear_axle_with_safe_dst,
                                 front_shrink_dis, front_side_shrink_dis,
                                 rear_shrink_dis, rear_side_shrink_dis);
}

bool IsSlotSelected(Frame* const frame) {
  const auto& func_state_machine = frame->session()
                                       ->environmental_model()
                                       .get_local_view()
                                       .function_state_machine_info;

  if (!func_state_machine.has_current_state()) {
    AERROR << "func_state_machine is invalid";
    return false;
  }

  if (func_state_machine.current_state() == FunctionalState::PARK_IN_READY ||
      func_state_machine.current_state() ==
          FunctionalState::PARK_IN_ACTIVATE_WAIT ||
      func_state_machine.current_state() ==
          FunctionalState::PARK_IN_ACTIVATE_CONTROL ||
      func_state_machine.current_state() ==
          FunctionalState::PARK_IN_SUSPEND_ACTIVATE ||
      func_state_machine.current_state() ==
          FunctionalState::PARK_IN_SUSPEND_CLOSE ||
      func_state_machine.current_state() == FunctionalState::PARK_IN_SECURE) {
    return true;
  }

  return false;
}

bool IsRoughCalc(Frame* const frame) {
  const auto& func_state_machine = frame->session()
                                       ->environmental_model()
                                       .get_local_view()
                                       .function_state_machine_info;

  if (!func_state_machine.has_current_state()) {
    AERROR << "func_state_machine is invalid";
    return false;
  }

  if (func_state_machine.current_state() ==
          FunctionalState::PARK_IN_SEARCHING ||
      func_state_machine.current_state() == FunctionalState::PARK_IN_NO_READY ||
      func_state_machine.current_state() == FunctionalState::PARK_IN_READY ||
      func_state_machine.current_state() ==
          FunctionalState::PARK_IN_ACTIVATE_WAIT) {
    return true;
  }

  return false;
}

bool IsReplanEachFrame(const FuncStateMachine& func_state_machine) {
  if (!func_state_machine.has_current_state()) {
    AERROR << "func_state_machine is invalid";
    return false;
  }

  if (func_state_machine.current_state() ==
          FunctionalState::PARK_IN_SEARCHING ||
      func_state_machine.current_state() == FunctionalState::PARK_IN_NO_READY ||
      func_state_machine.current_state() == FunctionalState::PARK_IN_READY ||
      func_state_machine.current_state() ==
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

bool IsSlotLineCrossable(const ParkingFusionSlot& parking_fusion_slot) {
  if (!parking_fusion_slot.has_fusion_source()) {
    return false;
  }

  // 0 means invalid source, 1 means camera only, 2 means uss only,
  // 3 means both camera and uss
  if (parking_fusion_slot.fusion_source() == 1) {
    return true;
  }

  return false;
}

void SetStoppingPlanningOutput(Frame* const frame) {
  auto planning_output_context =
      frame->mutable_session()->mutable_planning_output_context();
  auto planning_output = &(planning_output_context->mutable_planning_status()
                               ->planning_result.planning_output);

  auto trajectory = planning_output->mutable_trajectory();
  trajectory->set_available(true);
  trajectory->set_trajectory_type(
      Common::TrajectoryType::TRAJECTORY_TYPE_TRAJECTORY_POINTS);
  trajectory->mutable_trajectory_points()->Clear();
  auto gear_command = planning_output->mutable_gear_command();
  gear_command->set_available(true);
  gear_command->set_gear_command_value(
      Common::GearCommandValue::GEAR_COMMAND_VALUE_PARKING);
  const auto& pose = frame->session()
                         ->environmental_model()
                         .get_local_view()
                         .localization_estimate.pose();
  ::PlanningOutput::TrajectoryPoint* trajectory_point =
      trajectory->add_trajectory_points();
  trajectory_point->set_x(pose.local_position().x());
  trajectory_point->set_y(pose.local_position().y());
  trajectory_point->set_heading_yaw(pose.euler_angles().yaw());
  trajectory_point->set_curvature(0.0);
  trajectory_point->set_t(0.0);
  trajectory_point->set_v(0.0);
  trajectory_point->set_a(0.0);
  trajectory_point->set_distance(0.0);
  trajectory_point->set_jerk(0.0);
}

void SetFinishedPlanningOutput(Frame* const frame) {
  auto planning_output_context =
      frame->mutable_session()->mutable_planning_output_context();
  auto planning_output = &(planning_output_context->mutable_planning_status()
                               ->planning_result.planning_output);
  planning_output->mutable_planning_status()->set_apa_planning_status(
      ::PlanningOutput::ApaPlanningStatus::FINISHED);
  SetStoppingPlanningOutput(frame);
}

void SetFailedPlanningOutput(Frame* const frame) {
  auto planning_output_context =
      frame->mutable_session()->mutable_planning_output_context();
  auto planning_output = &(planning_output_context->mutable_planning_status()
                               ->planning_result.planning_output);
  planning_output->mutable_planning_status()->set_apa_planning_status(
      ::PlanningOutput::ApaPlanningStatus::FAILED);
  SetStoppingPlanningOutput(frame);
}

bool IsValidParkingState(
    const ::FuncStateMachine::FunctionalState& current_state) {
  AINFO << "current_state:" << current_state;

  if (current_state == FunctionalState::PARK_IN_SEARCHING ||
      current_state == FunctionalState::PARK_IN_NO_READY ||
      current_state == FunctionalState::PARK_IN_READY ||
      current_state == FunctionalState::PARK_IN_ACTIVATE_WAIT ||
      current_state == FunctionalState::PARK_IN_ACTIVATE_CONTROL ||
      current_state == FunctionalState::PARK_IN_SUSPEND_ACTIVATE ||
      current_state == FunctionalState::PARK_IN_SUSPEND_CLOSE ||
      current_state == FunctionalState::PARK_IN_SECURE ||
      current_state == FunctionalState::PARK_IN_COMPLETED) {
    return true;
  }
  return false;
}

}  // namespace  planning

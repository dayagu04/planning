#pragma once

#include "general_planning.h"

#include <string.h>

#include <iostream>

#ifdef CYBER_ENV
#include "autoplt/include/ADSTime.h"
#endif
#include "common/config_context.h"
#include "context/environmental_model.h"

namespace planning {

GeneralPlanning::GeneralPlanning() { Init(); }

GeneralPlanning::~GeneralPlanning() {}

void GeneralPlanning::Init() {
  session_.Init();
  scheduler_.Init(&session_);
  VehicleParam vehicle_param;
  // session->mutable_vehicle_config_context()->load_vehicle_param();
  session_.mutable_vehicle_config_context()->set_vehicle_param(vehicle_param);
  EnvironmentalModel *environmental_model =
      session_.mutable_environmental_model();
  environmental_model->set_vehicle_param(
      session_.vehicle_config_context().get_vehicle_param());
}

bool GeneralPlanning::RunOnce(
    const LocalView &local_view,
    PlanningOutput::PlanningOutput *const planning_output,
    DebugOutput &debug_info,
    PlanningHMI::PlanningHMIOutputInfoStr &planning_hmi_Info) {
  // using namespace FeiMa::SystemFunc;
  LOG_ERROR("GeneralPlanning::RunOnce \n");
  local_view_ = local_view;
  double start_timestamp = IflyTime::Now_ms();
  EnvironmentalModel *environmental_model =
      session_.mutable_environmental_model();
  environmental_model->feed_local_view(local_view);  // todo
  auto pre_planning_status = session_.mutable_planning_output_context()
                                 ->mutable_prev_planning_status();
  *pre_planning_status =
      session_.mutable_planning_output_context()->planning_status();
  auto *planning_status =
      session_.mutable_planning_output_context()->mutable_planning_status();
  planning_status->pre_planning_result = planning_status->planning_result;
  planning_status->planning_result.next_timestamp = start_timestamp;

  printf("VERSION: 2023-03-31 \n");
  // 1.校验输入 TBD

  if (reset_pnc_) {
    // reset dbw when hdmap_valid is changed
    environmental_model->UpdateVehicleDbwStatus(false);
    reset_pnc_ = false;
  }

  // 开始执行规划部分
  scheduler_.RunOnce();

  bool planning_success = session_.planning_context().planning_success();

  planning_status->planning_success = planning_success;
  auto end_timestamp = IflyTime::Now_ms();
  planning_status->time_consumption = end_timestamp - start_timestamp;
  LOG_DEBUG("general planning: planning time cost %f\n",
            planning_status->time_consumption);
  planning_status->planning_result.timestamp =
      planning_status->planning_result.next_timestamp;

  // when planning succeed, update the planning_output
  if (planning_status->planning_success) {
    FillPlanningTrajectory(start_timestamp, planning_output);
    FillPlanningDebugInfo(start_timestamp, debug_info);
    FillPlanningHmiInfo(start_timestamp, planning_hmi_Info);
    std::cout << "The RunOnce is successed !!!!:" << std::endl;
    return true;
  } else {
    std::cout << "The RunOnce is failed !!!!:" << std::endl;
    LOG_DEBUG("RunOnce failed !!!! \n");
    GenerateStopTrajectory(start_timestamp, planning_output);
  }

  double end_time = IflyTime::Now_ms();
  LOG_DEBUG("The time cost of RunOnce is: %f \n", end_time - start_timestamp);
  return false;
}

void GeneralPlanning::FillPlanningTrajectory(
    double start_time, PlanningOutput::PlanningOutput *const planning_output) {
  // 获取计算结果
  const auto &lateral_output =
      session_.planning_context().lateral_behavior_planner_output();
  const auto &vision_only_longitudinal_outputs =
      session_.planning_context().vision_longitudinal_behavior_planner_output();
  auto &planning_result = session_.planning_context().planning_result();

  // 更新输出
  auto time_stamp_us = IflyTime::Now_us();
  // 1.Meta
  planning_output->mutable_meta()->set_plan_timestamp_us(time_stamp_us);
  auto header = planning_output->mutable_meta()->mutable_header();
  header->set_timestamp(time_stamp_us);
  header->set_version("TEST");

  // 2.Trajectory
  auto trajectory = planning_output->mutable_trajectory();
  if (hdmap_valid_) {
    trajectory->set_trajectory_type(
        Common::TrajectoryType::TRAJECTORY_TYPE_TRAJECTORY_POINTS);
    trajectory->mutable_trajectory_points()->Clear();
    trajectory->mutable_target_reference()->Clear();
    for (size_t i = 0; i < planning_result.traj_points.size(); i++) {
      auto path_point = trajectory->add_trajectory_points();
      path_point->set_x(planning_result.traj_points[i].x);
      path_point->set_y(planning_result.traj_points[i].y);
      path_point->set_heading_yaw(planning_result.traj_points[i].heading_angle);
      path_point->set_curvature(planning_result.traj_points[i].curvature);
      path_point->set_t(planning_result.traj_points[i].t);
      path_point->set_v(planning_result.traj_points[i].v);
      path_point->set_a(planning_result.traj_points[i].a);
      path_point->set_distance(planning_result.traj_points[i].s);
      path_point->set_jerk(0.0);  // TBD
    }
  } else {
    // set vision_only_longitudinal_outputs if hdmpa valid is false
    trajectory->set_trajectory_type(
        Common::TrajectoryType::TRAJECTORY_TYPE_TARGET_REFERENCE);
    trajectory->mutable_trajectory_points()->Clear();
    trajectory->mutable_target_reference()->Clear();

    auto target_ref = trajectory->mutable_target_reference();
    // add polynomial
    for (size_t i = 0; i < lateral_output.d_poly.size(); i++) {
      target_ref->add_polynomial(lateral_output.d_poly[i]);
    }

    target_ref->set_target_velocity(
        vision_only_longitudinal_outputs.velocity_target);

    auto acceleration_range_limit =
        target_ref->mutable_acceleration_range_limit();
    acceleration_range_limit->set_min_a(
        vision_only_longitudinal_outputs.a_target_min);
    acceleration_range_limit->set_max_a(
        vision_only_longitudinal_outputs.a_target_max);
  }
  // 3.Turn signal
  auto turn_signal = planning_output->mutable_turn_signal_command();
  turn_signal->set_available(true);
  if (planning_result.turn_signal == NO_CHANGE) {
    turn_signal->set_turn_signal_value(Common::TurnSignalType::TURN_SIGNAL_TYPE_NONE);
  } else if (planning_result.turn_signal == LEFT_CHANGE) {
    turn_signal->set_turn_signal_value(Common::TurnSignalType::TURN_SIGNAL_TYPE_LEFT);
  } else {
    turn_signal->set_turn_signal_value(Common::TurnSignalType::TURN_SIGNAL_TYPE_RIGHT);
  }
  // 4.Light signal

  // 5.Horn signal

  // 6.Gear signal

  // 7.Open loop steering command

  // 8.Planning status
}

void GeneralPlanning::GenerateStopTrajectory(
    double start_time, PlanningOutput::PlanningOutput *const planning_output) {
  // 更新输出
  planning_output->mutable_meta()->set_plan_timestamp_us(IflyTime::Now_ms());

  auto trajectory = planning_output->mutable_trajectory();
  // Hack: 长时规划
  trajectory->set_trajectory_type(
      Common::TrajectoryType::TRAJECTORY_TYPE_TRAJECTORY_POINTS);
  trajectory->mutable_trajectory_points()->Clear();
  trajectory->mutable_target_reference()->Clear();
  double t = 0.0;
  for (size_t i = 0; i < 21; i++) {
    t = 0.1 * i;
    auto path_point = trajectory->add_trajectory_points();
    path_point->set_x(0.0);
    path_point->set_y(0.0);
    path_point->set_heading_yaw(0.0);
    path_point->set_curvature(0.0);
    path_point->set_t(t);
    path_point->set_v(0.0);
    path_point->set_a(0.0);
    path_point->set_distance(0.0);
    path_point->set_jerk(0.0);  // TBD
  }
}

void GeneralPlanning::FillPlanningDebugInfo(double start_time,
                                            DebugOutput &debug_info) {}

void GeneralPlanning::FillPlanningHmiInfo(
    double start_timestamp,
    PlanningHMI::PlanningHMIOutputInfoStr &planning_hmi_Info) {}
}  // namespace planning
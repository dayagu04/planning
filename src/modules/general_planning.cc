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
  planning_output->mutable_meta()->set_plan_timestamp_us(
      planning_result.timestamp);

  auto trajectory = planning_output->mutable_trajectory();
  // Hack: 长时规划
  if (hdmap_valid_) {
    trajectory->trajectory_type(
        common::Trajectory::TRAJECTORY_TYPE_TRAJECTORY_POINTS);
    trajectory->mutable_trajectory_points()->Clear();
    trajectory->mutable_target_reference()->Clear();
    for (size_t i = 0; i < planning_result.traj_points.size(); i++) {
      auto path_point = trajectory->add_trajectory_points();
      path_point->set_x(planning_result.traj_points[i].x);
      path_point->set_y(planning_result.traj_points[i].y);
      path_point->set_heading_angle(planning_result.traj_points[i].heading_yaw);
      path_point->set_curvature(planning_result.traj_points[i].curvature);

      path_point->set_t(planning_result.traj_points[i].t);
      path_point->set_v(planning_result.traj_points[i].v);
      path_point->set_a(planning_result.traj_points[i].a);
      path_point->set_distance(planning_result.traj_points[i].s);
      path_point->set_jerk(planning_result.traj_points[i].curvature);  // TBD
    }
  } else {
    // set vision_only_longitudinal_outputs if hdmpa valid is false
    trajectory->trajectory_type(
        common::Trajectory::TRAJECTORY_TYPE_TARGET_REFERENCE);
    trajectory->mutable_trajectory_points()->Clear();
    trajectory->mutable_target_reference()->Clear();

    // auto velocity = planning_output->mutable_velocity();
    // velocity->set_type(FeiMa::Planning::Velocity::VELOCITY_TARGET_VALUE);
    // velocity->set_target_value(
    //     vision_only_longitudinal_outputs.velocity_target);

    // auto acceleration = planning_output->mutable_acceleration();
    // acceleration->set_type(
    //     FeiMa::Planning::Acceleration::ACCELERATION_RANGE_LIMIT);
    // acceleration->mutable_range_limit()->set_min(
    //     vision_only_longitudinal_outputs.a_target_min);
    // acceleration->mutable_range_limit()->set_max(
    //     vision_only_longitudinal_outputs.a_target_max);
  }

  // HMI结果
}

void GeneralPlanning::GenerateStopTrajectory(
    double start_time, PlanningOutput::PlanningOutput *const planning_output) {}

void GeneralPlanning::FillPlanningDebugInfo(double start_time,
                                            DebugOutput &debug_info) {}

void GeneralPlanning::FillPlanningHmiInfo(
    double start_timestamp,
    PlanningHMI::PlanningHMIOutputInfoStr &planning_hmi_Info) {}
}  // namespace planning
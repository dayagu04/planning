#pragma once

#include "general_planning.h"

#include <string.h>

#include <iostream>

#ifdef  CYBER_ENV
#include "autoplt/include/ADSTime.h"
#endif
#include "modules/context/environmental_model.h"
#include "modules/common/config_context.h"

namespace planning {

GeneralPlanning::GeneralPlanning() { Init(); }

GeneralPlanning::~GeneralPlanning() {}

void GeneralPlanning::Init() {
  session_.Init();
  scheduler_.Init(&session_);
}

bool GeneralPlanning::RunOnce(const LocalView& local_view,
               PlanningOutput::PlanningOutput* const planning_output,
               DebugOutput &debug_info) {
  // using namespace FeiMa::SystemFunc;
  LOG_ERROR("GeneralPlanning::RunOnce \n");
  local_view_ = local_view;
  double start_timestamp = IflyTime::Now_ms();
  EnvironmentalModel *environmental_model = session_.mutable_environmental_model();
  environmental_model->feed_local_view(local_view);//todo
  auto pre_planning_status =
      session_.mutable_planning_output_context()->mutable_prev_planning_status();
  *pre_planning_status =
      session_.mutable_planning_output_context()->planning_status();
  auto *planning_status =
      session_.mutable_planning_output_context()->mutable_planning_status();
  planning_status->pre_planning_result = planning_status->planning_result;
  session_.mutable_planning_output_context()
      ->mutable_planning_status()
      ->planning_result.next_timestamp = start_timestamp;

  VehicleParam vehicle_param;
  // session->mutable_vehicel_config_context()->load_vehicle_param();
  session_.mutable_vehicel_config_context()->set_vehicle_param(vehicle_param);
  environmental_model->set_vehicle_param(session_.vehicel_config_context().get_vehicle_param());


  printf("VERSION: 2023-03-31 \n");

  // 1.校验输入 TBD

  // 2.update world module from local_view_
  double current_time = IflyTime::Now_ms();
  UpdateRawEnvironmentalModel(current_time);

  LOG_DEBUG("update_raw_environmental_model is finished !!!! \n");
  std::string status_msg;
  if (!InputReady(current_time, status_msg)) {
    LOG_ERROR("run_once is failed !!!! \n");
    return false;
  }

  if (reset_pnc_) {
    // reset dbw when hdmap_valid is changed
    environmental_model->UpdateVehicleDbwStatus(false);
    reset_pnc_ = false;
  }

  if (!environmental_model->Update()) {
    LOG_ERROR("(%s)environmental model update error", __FUNCTION__, "\n");
    status_msg = "planning environmental model update error.";
    return false;
  }

  // 开始执行规划部分
  scheduler_.RunOnce();

  bool planning_success =
      session_.planning_context().planning_success();

  planning_status->planning_success = planning_success;
  auto end_timestamp = IflyTime::Now_ms();
  planning_status->time_consumption = end_timestamp - start_timestamp;
  LOG_DEBUG("general planning: planning time cost %f", planning_status->time_consumption);
  planning_status->planning_result.timestamp =
      planning_status->planning_result.next_timestamp;

  // when planning succeed, update the planning_output
  if (planning_status->planning_success) {
    FillPlanningTrajectory(start_timestamp, planning_output);
    FillPlanningDebugInfo(start_timestamp, debug_info);
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

// void GeneralPlanning::UpdateRawEnvironmentalModel(double current_time) {
//   LOG_DEBUG("GeneralPlanning::UpdateRawEnvironmentalmodel \n");

//   // 1.判断车辆是否处于自动状态
//   bool status{true};
//   // 这里采用hack为true
//   LOG_DEBUG("The status is: %i\n", status);
//   // status = true;
//   // LOG_DEBUG("The status is hacked: %i\n", status);

//   environmental_model->UpdateVehicleDbwStatus(status);
//   last_feed_time_[FEED_VEHICLE_DBW_STATUS] = current_time;

//   UpdateFusionObjectInfo(current_time);

//   UpdatePredictionInfo(current_time);

//   UpdateVehicleStatus(current_time);
// }

bool InputReady(double current_time, std::string &error_msg) {
  return true;
}

}  // namespace planning
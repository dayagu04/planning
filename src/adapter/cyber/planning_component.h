#pragma once

#include <stdio.h>
#include <string.h>

#include <iostream>
#include <mutex>

#include "vehicle_service.pb.h"
#include "fusion_objects.pb.h"
#include "localization.pb.h"
#include "planning_plan.pb.h"
#include "control_command.pb.h"
#include "parking_fusion.pb.h"
#include "parking_slot_list.pb.h"
#include "parking_slot_select.pb.h"
#include "prediction.pb.h"
#include "fusion_road.pb.h"
#include "radar_perception_objects.pb.h"
#include "planning_hmi.pb.h"
#include "func_state_machine.pb.h"

#include "autoplt/include/ADSComponent.h"
#include "autoplt/include/ADSNode.h"
#include "ifly_time.h"
#include "general_planning.h"

namespace planning {

using autoplt::ADSNode;

// 黑芝麻头文件中拼写错误
class PlanningComponent final : public autoplt::ADSTimerCoponent {
 public:
  PlanningComponent() = default;

  ~PlanningComponent() = default;

 public:
  bool Init() override;
  bool Proc() override;
  DebugOutput GetDebugInfo();

 private:
  std::mutex msg_mutex_;
  // input signals
  Prediction::PredictionResult prediction_result_msg_;
  FusionRoad::RoadInfo road_info_msg_;
  LocalizationOutput::LocalizationEstimate localization_estimate_msg_;
  FusionObjects::FusionObjectsInfo fusion_objects_info_msg_;
  VehicleService::VehicleServiceOutputInfo vehicel_service_output_info_msg_;
  RadarPerceptionObjects::RadarPerceptionObjectsInfo radar_perception_objects_info_msg_;
  ControlCommand::ControlOutput control_output_msg_;
  HmiMcuInner::HmiMcuInner hmi_mcu_inner_info_msg_;
  ParkingFusion::ParkingFusionInfo parking_fusion_info_msg_;
  FuncStateMachine::FuncStateMachine func_state_machine_msg_;
  LocalView local_view_;
  DebugOutput debug_info_;
//   Session session_;
//   Scheduler scheduler_;

  std::shared_ptr<ADSNode> planning_node_ = nullptr;
  std::shared_ptr<Writer<PlanningOutput::PlanningOutput>> planning_writer_ =
      nullptr;
  std::shared_ptr<Writer<planning::common::PlanningDebugInfo>> planning_debug_writer_ =
      nullptr;
  std::shared_ptr<Writer<PlanningHMI::PlanningHMIOutputInfoStr>> planning_hmi_Info_writer_ =
      nullptr;

  std::unique_ptr<GeneralPlanning> planning_base_ = nullptr;
};

// register planning component
AUTOPLT_REGISTER_COMPONENT(PlanningComponent)

}  // namespace planning
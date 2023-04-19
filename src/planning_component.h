#pragma once

#include <stdio.h>
#include <string.h>

#include <iostream>
#include <mutex>

#include "../res/include/proto/vehicle_service.pb.h"
#include "../res/include/proto/fusion_objects.pb.h"
#include "../res/include/proto/localization.pb.h"
#include "../res/include/proto/planning_plan.pb.h"
#include "../res/include/proto/control_command.pb.h"
#include "../res/include/proto/parking_fusion.pb.h"
#include "../res/include/proto/parking_slot_list.pb.h"
#include "../res/include/proto/parking_slot_select.pb.h"
#include "../res/include/proto/prediction.pb.h"
#include "../res/include/proto/fusion_road.pb.h"
#include "../res/include/proto/radar_perception_objects.pb.h"


#include "autoplt/include/ADSComponent.h"
#include "autoplt/include/ADSNode.h"
#include "autoplt/include/ADSTime.h"
#include "common/ifly_time.h"
#include "src/modules/general_planning.h"

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
  HimMcuInner::HmiMcuInner hmi_mcu_inner_info_msg_;
  LocalView local_view_;
  DebugOutput debug_info_;
//   Session session_;
//   Scheduler scheduler_;

  std::shared_ptr<ADSNode> planning_node_ = nullptr;
  std::shared_ptr<Writer<PlanningOutput::PlanningOutput>> planning_writer_ =
      nullptr;
  std::shared_ptr<Writer<planning::common::PlanningDebugInfo>> planning_debug_writer_ =
      nullptr;

  std::unique_ptr<GeneralPlanning> planning_base_ = nullptr;
};

// register planning component
AUTOPLT_REGISTER_COMPONENT(PlanningComponent)

}  // namespace planning
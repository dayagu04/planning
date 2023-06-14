#include "planning_component.h"

#include "Platform_Types.h"
#include "common/config_context.h"
#include "debug_info_log.h"
#include "general_planning.h"

// This file compile modules register map and constructor and link to .so
// When .so loads, static vairiables will init and module constructors will register.
#include "modules_register.h"

namespace planning {

bool PlanningComponent::Init() {
  planning_adapter_ = std::make_unique<PlanningAdapter>();
  planning_adapter_->Init();

  // 1.定义cyber node
  ADSNode::Init("planning_node");
  planning_node_ = std::make_shared<ADSNode>("planning_node");

  // 2.定义收发topics
  // -------------- reader topics --------------
  auto fusion_objects_reader_ = planning_node_->CreateReader<FusionObjects::FusionObjectsInfo>(
      "/iflytek/fusion/objects",
      [this](const std::shared_ptr<FusionObjects::FusionObjectsInfo> &fusion_objects_info_msg) {
        planning_adapter_->FeedFusionObjects(fusion_objects_info_msg);
      });

  auto fusion_road_reader_ = planning_node_->CreateReader<FusionRoad::RoadInfo>(
      "/iflytek/fusion/road_fusion", [this](const std::shared_ptr<FusionRoad::RoadInfo> &road_info_msg) {
        planning_adapter_->FeedFusionRoad(road_info_msg);
      });

  auto localization_reader_ = planning_node_->CreateReader<LocalizationOutput::LocalizationEstimate>(
      "/iflytek/localization/ego_pose",
      [this](const std::shared_ptr<LocalizationOutput::LocalizationEstimate> &localization_estimate_msg) {
        planning_adapter_->FeedLocalizationOutput(localization_estimate_msg);
      });

  auto prediction_reader_ = planning_node_->CreateReader<Prediction::PredictionResult>(
      "/iflytek/prediction/prediction_result",
      [this](const std::shared_ptr<Prediction::PredictionResult> &prediction_result_msg) {
        planning_adapter_->FeedPredictionResult(prediction_result_msg);
      });

  auto vehicel_service_reader_ = planning_node_->CreateReader<VehicleService::VehicleServiceOutputInfo>(
      "/iflytek/vehicle_service",
      [this](const std::shared_ptr<VehicleService::VehicleServiceOutputInfo> &vehicel_service_output_info_msg) {
        planning_adapter_->FeedVehicleService(vehicel_service_output_info_msg);
      });

  auto radar_perception_objects_reader_ =
      planning_node_->CreateReader<RadarPerceptionObjects::RadarPerceptionObjectsInfo>(
          "/iflytek/radar_perception_info",
          [this](const std::shared_ptr<RadarPerceptionObjects::RadarPerceptionObjectsInfo>
                     &radar_perception_objects_info_msg) {
            planning_adapter_->FeedRadarPerceptionObjects(radar_perception_objects_info_msg);
          });

  auto control_output_reader_ = planning_node_->CreateReader<ControlCommand::ControlOutput>(
      "/iflytek/control/control_command",
      [this](const std::shared_ptr<ControlCommand::ControlOutput> &control_output_msg) {
        planning_adapter_->FeedControlCommand(control_output_msg);
      });

  auto hmi_reader_ = planning_node_->CreateReader<HmiMcuInner::HmiMcuInner>(
      "/iflytek/hmi/mcu_inner", [this](const std::shared_ptr<HmiMcuInner::HmiMcuInner> &hmi_mcu_inner_info_msg) {
        planning_adapter_->FeedHmiMcuInner(hmi_mcu_inner_info_msg);
      });

  auto parking_fusion_info_reader_ = planning_node_->CreateReader<ParkingFusion::ParkingFusionInfo>(
      "/iflytek/fusion/parking_slot",
      [this](const std::shared_ptr<ParkingFusion::ParkingFusionInfo> &parking_fusion_info_msg) {
        planning_adapter_->FeedParkingFusion(parking_fusion_info_msg);
      });

  auto func_state_machine_reader_ = planning_node_->CreateReader<FuncStateMachine::FuncStateMachine>(
      "/iflytek/system_state/soc_state",
      [this](const std::shared_ptr<FuncStateMachine::FuncStateMachine> &func_state_machine_msg) {
        planning_adapter_->FeedFuncStateMachine(func_state_machine_msg);
      });

  // -------------- writter topics --------------
  planning_writer_ = planning_node_->CreateWriter<PlanningOutput::PlanningOutput>("/iflytek/planning/plan");
  planning_adapter_->RegisterOutputWriter(
      [this](const PlanningOutput::PlanningOutput &planning_output) { planning_writer_->Write(planning_output); });

  planning_debug_writer_ =
      planning_node_->CreateWriter<planning::common::PlanningDebugInfo>("/iflytek/planning/debug_info");
  planning_adapter_->RegisterDebugInfoWriter([this](const planning::common::PlanningDebugInfo &planning_debug_info) {
    planning_debug_writer_->Write(planning_debug_info);
  });

  planning_hmi_Info_writer_ =
      planning_node_->CreateWriter<PlanningHMI::PlanningHMIOutputInfoStr>("/iflytek/planning/hmi");
  planning_adapter_->RegisterHMIOutputInfoWriter(
      [this](const PlanningHMI::PlanningHMIOutputInfoStr &planning_hmi_ouput_info) {
        planning_hmi_Info_writer_->Write(planning_hmi_ouput_info);
      });

  return true;
}

bool PlanningComponent::Proc() {
  std::cout << "==============The planning component running=============" << std::endl;
  planning_adapter_->Proc();
  return true;
}

}  // namespace planning

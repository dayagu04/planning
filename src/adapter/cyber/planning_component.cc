#include "planning_component.h"

#include "gflags/gflags.h"

#include "cyber/scheduler/scheduler.h"

#include "../common/cyber/logger/async_logger.h"
#include "../common/log_glog.h"
#include "common/config_context.h"

// This file compile modules register map and constructor and link to .so
// When .so loads, static vairiables will init and module constructors will
// register.
#include "modules_register.h"

// This file add version string to .so
#include "version.h"

namespace planning {

logger::AsyncLogger *async_logger = nullptr;

PlanningComponent::~PlanningComponent() { delete async_logger; }

bool PlanningComponent::Init() {
  InitGflags();
  (void)common::ConfigurationContext::Instance();
  planning_adapter_ = std::make_unique<PlanningAdapter>();
  planning_adapter_->Init();

  InitLogger();

  // 1.定义cyber node
  ADSNode::Init("planning_node");
  planning_node_ = std::make_shared<ADSNode>("planning_node");

  // 2.定义收发topics
  // -------------- reader topics --------------
  auto fusion_objects_reader_ =
      planning_node_->CreateReader<FusionObjects::FusionObjectsInfo>(
          "/iflytek/fusion/objects",
          [this](const std::shared_ptr<FusionObjects::FusionObjectsInfo>
                     &fusion_objects_info_msg) {
            planning_adapter_->FeedFusionObjects(fusion_objects_info_msg);
          });

  auto fusion_road_reader_ = planning_node_->CreateReader<FusionRoad::RoadInfo>(
      "/iflytek/fusion/road_fusion",
      [this](const std::shared_ptr<FusionRoad::RoadInfo> &road_info_msg) {
        planning_adapter_->FeedFusionRoad(road_info_msg);
      });

  auto localization_reader_ =
      planning_node_->CreateReader<LocalizationOutput::LocalizationEstimate>(
          "/iflytek/localization/ego_pose",
          [this](const std::shared_ptr<LocalizationOutput::LocalizationEstimate>
                     &localization_estimate_msg) {
            planning_adapter_->FeedLocalizationOutput(
                localization_estimate_msg);
          });

  auto prediction_reader_ =
      planning_node_->CreateReader<Prediction::PredictionResult>(
          "/iflytek/prediction/prediction_result",
          [this](const std::shared_ptr<Prediction::PredictionResult>
                     &prediction_result_msg) {
            planning_adapter_->FeedPredictionResult(prediction_result_msg);
          });

  auto vehicel_service_reader_ =
      planning_node_->CreateReader<VehicleService::VehicleServiceOutputInfo>(
          "/iflytek/vehicle_service",
          [this](const std::shared_ptr<VehicleService::VehicleServiceOutputInfo>
                     &vehicel_service_output_info_msg) {
            planning_adapter_->FeedVehicleService(
                vehicel_service_output_info_msg);
          });

  auto control_output_reader_ =
      planning_node_->CreateReader<ControlCommand::ControlOutput>(
          "/iflytek/control/control_command",
          [this](const std::shared_ptr<ControlCommand::ControlOutput>
                     &control_output_msg) {
            planning_adapter_->FeedControlCommand(control_output_msg);
          });

  auto hmi_reader_ = planning_node_->CreateReader<HmiMcuInner::HmiMcuInner>(
      "/iflytek/hmi/mcu_inner",
      [this](const std::shared_ptr<HmiMcuInner::HmiMcuInner>
                 &hmi_mcu_inner_info_msg) {
        planning_adapter_->FeedHmiMcuInner(hmi_mcu_inner_info_msg);
      });

  auto parking_fusion_info_reader_ =
      planning_node_->CreateReader<ParkingFusion::ParkingFusionInfo>(
          "/iflytek/fusion/parking_slot",
          [this](const std::shared_ptr<ParkingFusion::ParkingFusionInfo>
                     &parking_fusion_info_msg) {
            planning_adapter_->FeedParkingFusion(parking_fusion_info_msg);
          });

  auto func_state_machine_reader_ =
      planning_node_->CreateReader<FuncStateMachine::FuncStateMachine>(
          "/iflytek/system_state/soc_state",
          [this](const std::shared_ptr<FuncStateMachine::FuncStateMachine>
                     &func_state_machine_msg) {
            planning_adapter_->FeedFuncStateMachine(func_state_machine_msg);
          });

  // -------------- writter topics --------------
  planning_writer_ =
      planning_node_->CreateWriter<PlanningOutput::PlanningOutput>(
          "/iflytek/planning/plan");
  planning_adapter_->RegisterOutputWriter(
      [this](const PlanningOutput::PlanningOutput &planning_output) {
        planning_writer_->Write(planning_output);
      });

  planning_debug_writer_ =
      planning_node_->CreateWriter<planning::common::PlanningDebugInfo>(
          "/iflytek/planning/debug_info");
  planning_adapter_->RegisterDebugInfoWriter(
      [this](const planning::common::PlanningDebugInfo &planning_debug_info) {
        planning_debug_writer_->Write(planning_debug_info);
      });

  planning_hmi_info_writer_ =
      planning_node_->CreateWriter<PlanningHMI::PlanningHMIOutputInfoStr>(
          "/iflytek/planning/hmi");
  planning_adapter_->RegisterHMIOutputInfoWriter(
      [this](const PlanningHMI::PlanningHMIOutputInfoStr
                 &planning_hmi_ouput_info) {
        planning_hmi_info_writer_->Write(planning_hmi_ouput_info);
      });

  return true;
}

bool PlanningComponent::Proc() {
  std::cout << "==============The planning component running============="
            << std::endl;
  planning_adapter_->Proc();
  return true;
}

void PlanningComponent::InitLogger() {
  FLAGS_log_dir = "/asw/planning/log";
  FLAGS_alsologtostderr = false;
  FLAGS_colorlogtostderr = true;
  FLAGS_max_log_size = 500;
  FLAGS_minloglevel = 0;
  FLAGS_v = 0;

  // Init glog
  google::InitGoogleLogging("");
  google::SetLogDestination(google::ERROR, "");
  google::SetLogDestination(google::WARNING, "");
  google::SetLogDestination(google::FATAL, "");

  // Init async logger
  async_logger = new ::apollo::cyber::logger::AsyncLogger(
      google::base::GetLogger(FLAGS_minloglevel));
  google::base::SetLogger(FLAGS_minloglevel, async_logger);
  async_logger->Start();

  auto thread = const_cast<std::thread *>(async_logger->LogThread());
  scheduler::Instance()->SetInnerThreadAttr("async_log", thread);
}

void PlanningComponent::InitGflags() const {
  const std::string flag_file_path = "/asw/planning/res/conf/planning_gflags.conf";
  google::SetCommandLineOption("flagfile", flag_file_path.c_str());
}

}  // namespace planning

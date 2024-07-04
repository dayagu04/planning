#include "planning_component.h"

// #include "logger/async_logger.h"
#include "common/config_context.h"
#include "cyber/scheduler/scheduler.h"
#include "ehr_sdmap.pb.h"
#include "gflags/gflags.h"
#include "log.h"
#include "struct_container.pb.h"

namespace planning {

// logger::AsyncLogger *async_logger = nullptr;

// PlanningComponent::~PlanningComponent() { delete async_logger; }

bool PlanningComponent::Init() {
  InitGflags();
  (void)common::ConfigurationContext::Instance();
  planning_adapter_ = std::make_unique<PlanningAdapter>();
  planning_adapter_->Init();

  // InitLogger();

  // 1.定义cyber node
  ADSNode::Init("planning_node");
  planning_node_ = std::make_shared<ADSNode>("planning_node");

  // 2.定义收发topics
  // -------------- reader topics --------------
  auto fusion_objects_reader_ =
      planning_node_->CreateReader<iflyauto::StructContainer>(
          "/iflytek/fusion/objects",
          [this](const std::shared_ptr<iflyauto::StructContainer>
                     &fusion_objects_info_container) {
            const auto &fusion_objects_info_msg =
                *iflyauto::struct_cast<iflyauto::FusionObjectsInfo>(
                    fusion_objects_info_container);
            planning_adapter_->FeedFusionObjects(fusion_objects_info_msg);
          });

  auto fusion_occupancy_objects_reader_ =
      planning_node_->CreateReader<iflyauto::StructContainer>(
          "/iflytek/fusion/occupancy/objects",
          [this](const std::shared_ptr<iflyauto::StructContainer>
                     &fusion_occupancy_objects_info_container) {
            const auto &fusion_occupancy_objects_info_msg =
                *iflyauto::struct_cast<iflyauto::FusionOccupancyObjectsInfo>(
                    fusion_occupancy_objects_info_container);
            planning_adapter_->FeedFusionOccupancyObjects(
                fusion_occupancy_objects_info_msg);
          });

  auto fusion_road_reader_ =
      planning_node_->CreateReader<iflyauto::StructContainer>(
          "/iflytek/fusion/road_fusion",
          [this](const std::shared_ptr<iflyauto::StructContainer>
                     &road_info_container) {
            const auto &road_info_msg =
                *iflyauto::struct_cast<iflyauto::RoadInfo>(road_info_container);
            planning_adapter_->FeedFusionRoad(road_info_msg);
          });

  auto fusion_groudline_reader_ =
      planning_node_->CreateReader<iflyauto::StructContainer>(
          "/iflytek/fusion/ground_line",
          [this](const std::shared_ptr<iflyauto::StructContainer>
                     &ground_line_perception_container) {
            const auto ground_line_perception_msg =
                *iflyauto::struct_cast<iflyauto::GroundLinePerceptionInfo>(
                    ground_line_perception_container);
            planning_adapter_->FeedGroundLinePerception(
                ground_line_perception_msg);
          });

  auto localization_estimate_reader_ =
      planning_node_->CreateReader<iflyauto::StructContainer>(
          "/iflytek/localization/ego_pose",
          [this](const std::shared_ptr<iflyauto::StructContainer>
                     &localization_estimate_container) {
            const auto localization_estimate_msg =
                *iflyauto::struct_cast<iflyauto::LocalizationEstimate>(
                    localization_estimate_container);
            planning_adapter_->FeedLocalizationEstimateOutput(
                localization_estimate_msg);
          });

  auto localization_reader_ =
      planning_node_->CreateReader<iflyauto::StructContainer>(
          "/iflytek/localization/egomotion",
          [this](const std::shared_ptr<iflyauto::StructContainer>
                     &localization_msg) {
            const auto &localization_struct =
                *iflyauto::struct_cast<iflyauto::IFLYLocalization>(
                    localization_msg);
            planning_adapter_->FeedLocalizationOutput(localization_struct);
          });

  auto prediction_reader_ =
      planning_node_->CreateReader<iflyauto::StructContainer>(
          "/iflytek/prediction/prediction_result",
          [this](const std::shared_ptr<iflyauto::StructContainer>
                     &prediction_result_msg) {
            const auto &prediction_struct =
                *iflyauto::struct_cast<iflyauto::PredictionResult>(
                    prediction_result_msg);
            planning_adapter_->FeedPredictionResult(prediction_struct);
          });

  auto vehicle_service_reader_ =
      planning_node_->CreateReader<iflyauto::StructContainer>(
          "/iflytek/vehicle_service",
          [this](const std::shared_ptr<iflyauto::StructContainer>
                     &vehicle_service_output_info_msg) {
            const auto &vehicle_service_output_struct =
                *iflyauto::struct_cast<iflyauto::VehicleServiceOutputInfo>(
                    vehicle_service_output_info_msg);
            planning_adapter_->FeedVehicleService(
                vehicle_service_output_struct);
          });

  auto control_output_reader_ =
      planning_node_->CreateReader<iflyauto::StructContainer>(
          "/iflytek/control/control_command",
          [this](const std::shared_ptr<iflyauto::StructContainer>
                     &control_output_msg) {
            const auto &control_output_struct =
                *iflyauto::struct_cast<iflyauto::ControlOutput>(
                    control_output_msg);
            planning_adapter_->FeedControlCommand(control_output_struct);
          });

  auto hmi_reader_ = planning_node_->CreateReader<iflyauto::StructContainer>(
      "/iflytek/hmi/mcu_inner",
      [this](const std::shared_ptr<iflyauto::StructContainer>
                 &hmi_inner_info_msg) {
        const auto &hmi_mcu_inner_struct =
            *iflyauto::struct_cast<iflyauto::HmiInner>(hmi_inner_info_msg);
        planning_adapter_->FeedHmiInner(hmi_mcu_inner_struct);
      });

  auto parking_fusion_info_reader_ =
      planning_node_->CreateReader<iflyauto::StructContainer>(
          "/iflytek/fusion/parking_slot",
          [this](const std::shared_ptr<iflyauto::StructContainer>
                     &parking_fusion_info_msg) {
            const auto &parking_fusion_info_struct =
                *iflyauto::struct_cast<iflyauto::ParkingFusionInfo>(
                    parking_fusion_info_msg);
            planning_adapter_->FeedParkingFusion(parking_fusion_info_struct);
          });

  //   auto parking_map_info_reader_ =
  //       planning_node_->CreateReader<iflyauto::StructContainer>(
  //           "/iflytek/ehr/parking_map",
  //           [this](const std::shared_ptr<iflyauto::StructContainer>
  //                      &parking_map_info_msg) {
  //             const auto &parking_map_info_struct =
  //                 *iflyauto::struct_cast<iflyauto::ParkingInfo>(
  //                     parking_map_info_msg);
  //             planning_adapter_->FeedParkingMap(parking_map_info_struct);
  //           });

  auto func_state_machine_reader_ =
      planning_node_->CreateReader<iflyauto::StructContainer>(
          "/iflytek/system_state/soc_state",
          [this](const std::shared_ptr<iflyauto::StructContainer>
                     &func_state_machine_msg) {
            const auto &func_state_machine_struct =
                *iflyauto::struct_cast<iflyauto::FuncStateMachine>(
                    func_state_machine_msg);
            planning_adapter_->FeedFuncStateMachine(func_state_machine_struct);
          });

  auto uss_wave_info_reader_ =
      planning_node_->CreateReader<iflyauto::StructContainer>(
          "/iflytek/uss/usswave_info",
          [this](const std::shared_ptr<iflyauto::StructContainer>
                     &uss_wave_info_msg) {
            const auto &uss_wave_info_struct =
                *iflyauto::struct_cast<iflyauto::UssWaveInfo>(
                    uss_wave_info_msg);
            planning_adapter_->FeedUssWaveInfo(uss_wave_info_struct);
          });

  auto uss_percept_info_reader_ =
      planning_node_->CreateReader<iflyauto::StructContainer>(
          "/iflytek/uss/uss_perception_info",
          [this](const std::shared_ptr<iflyauto::StructContainer>
                     uss_percept_info_struct_msg) {
            const auto &uss_percept_info_struct =
                *iflyauto::struct_cast<iflyauto::UssPerceptInfo>(
                    uss_percept_info_struct_msg);
            planning_adapter_->FeedUssPerceptInfo(uss_percept_info_struct);
          });

  auto map_reader_ = planning_node_->CreateReader<Map::StaticMap>(
      "/iflytek/ehr/static_map",
      [this](const std::shared_ptr<Map::StaticMap> &map_msg) {
        planning_adapter_->FeedMap(map_msg);
      });

  auto sd_map_reader_ = planning_node_->CreateReader<SdMapSwtx::SdMap>(
      "/iflytek/ehr/sdmap",
      [this](const std::shared_ptr<SdMapSwtx::SdMap> &sd_map_msg) {
        planning_adapter_->FeedSdMap(sd_map_msg);
      });

  // -------------- writter topics --------------
  planning_writer_ = planning_node_->CreateWriter<iflyauto::StructContainer>(
      "/iflytek/planning/plan");
  planning_adapter_->RegisterOutputWriter(
      [this](
          const std::shared_ptr<iflyauto::StructContainer> &planning_output) {
        planning_writer_->Write(planning_output);
      });

  planning_debug_writer_ =
      planning_node_->CreateWriter<iflyauto::StructContainer>(
          "/iflytek/planning/debug_info");
  planning_adapter_->RegisterDebugInfoWriter(
      [this](const std::shared_ptr<iflyauto::StructContainer>
                 &planning_debug_info) {
        planning_debug_writer_->Write(planning_debug_info);
      });

  planning_hmi_info_writer_ =
      planning_node_->CreateWriter<iflyauto::StructContainer>(
          "/iflytek/planning/hmi");
  planning_adapter_->RegisterHMIOutputInfoWriter(
      [this](const std::shared_ptr<iflyauto::StructContainer>
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

// void PlanningComponent::InitLogger() {
// //   FLAGS_log_dir = "/asw/planning/log";
// //   FLAGS_alsologtostderr = false;
// //   FLAGS_colorlogtostderr = true;
// //   FLAGS_max_log_size = 500;
// //   FLAGS_minloglevel = 0;
// //   FLAGS_v = 0;

// //   // Init glog
// //   google::InitGoogleLogging("");
// //   google::SetLogDestination(google::ERROR, "");
// //   google::SetLogDestination(google::WARNING, "");
// //   google::SetLogDestination(google::FATAL, "");

// //   // Init async logger
// //   async_logger = new ::apollo::cyber::logger::AsyncLogger(
// //       google::base::GetLogger(FLAGS_minloglevel));
// //   google::base::SetLogger(FLAGS_minloglevel, async_logger);
// //   async_logger->Start();

// //   auto thread = const_cast<std::thread *>(async_logger->LogThread());
// //   scheduler::Instance()->SetInnerThreadAttr("async_log", thread);
// }

void PlanningComponent::InitGflags() const {
  //   const std::string flag_file_path =
  //       "/asw/planning/res/conf/planning_gflags.conf";
  //   google::SetCommandLineOption("flagfile", flag_file_path.c_str());
}

}  // namespace planning

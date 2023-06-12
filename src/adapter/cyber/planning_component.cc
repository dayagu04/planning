#include "planning_component.h"

#include "Platform_Types.h"
#include "debug_info_log.h"
#include "general_planning.h"
#include "common/config_context.h"

namespace planning {

using ParkingFusion::ParkingFusionInfo;

bool PlanningComponent::Init() {
  std::cout << "The planning component init!!!" << std::endl;
  std::string engine_config_path =
      std::string(CONFIG_PATH) + "/planning_engine_config.json";
  common::ConfigurationContext::Instance()->load_engine_config_from_json(
      engine_config_path);

  auto engine_config =
      common::ConfigurationContext::Instance()->engine_config();

  std::string log_file = engine_config.log_conf.log_file_dir + "/planning_log";
  std::cout << "log_file!!!" << log_file << std::endl;
  // Nanolog
  bst::LogLevel log_level;
  if (engine_config.log_conf.log_level == "FETAL") {
    log_level = bst::FETAL;
  } else if (engine_config.log_conf.log_level == "ERROR") {
    log_level = bst::ERROR;
  } else if (engine_config.log_conf.log_level == "WARNING") {
    log_level = bst::WARNING;
  } else if (engine_config.log_conf.log_level == "NOTICE") {
    log_level = bst::NOTICE;
  } else if (engine_config.log_conf.log_level == "DEBUG") {
    log_level = bst::DEBUG;
  } else {
    log_level = bst::ERROR;
  }

  std::cout << "log_level!!!" << engine_config.log_conf.log_level << std::endl;
  bst::Log::getInstance().setConfig("Planning_Log", log_file.c_str(),
                                    log_level);
  LOG_DEBUG("The planning component init!!! \n");

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
            std::cout << "receive fusion fusion_objects_info "
                      << fusion_objects_info_msg->header().timestamp()
                      << std::endl;
            std::lock_guard<std::mutex> lock(msg_mutex_);
            fusion_objects_info_msg_.CopyFrom(*fusion_objects_info_msg);
          });

  auto fusion_road_reader_ = planning_node_->CreateReader<FusionRoad::RoadInfo>(
      "/iflytek/fusion/road_fusion",
      [this](const std::shared_ptr<FusionRoad::RoadInfo> &road_info_msg) {
        std::cout << "receive fusion road_info "
                  << road_info_msg->header().timestamp() << std::endl;
        std::lock_guard<std::mutex> lock(msg_mutex_);
        road_info_msg_.CopyFrom(*road_info_msg);
      });

  auto localization_reader_ =
      planning_node_->CreateReader<LocalizationOutput::LocalizationEstimate>(
          "/iflytek/localization/ego_pose",
          [this](const std::shared_ptr<LocalizationOutput::LocalizationEstimate>
                     &localization_estimate_msg) {
            std::cout << "receive localization_estimate "
                      << localization_estimate_msg->header().timestamp()
                      << std::endl;
            std::lock_guard<std::mutex> lock(msg_mutex_);
            localization_estimate_msg_.CopyFrom(*localization_estimate_msg);
          });

  auto prediction_reader_ =
      planning_node_->CreateReader<Prediction::PredictionResult>(
          "/iflytek/prediction/prediction_result",
          [this](const std::shared_ptr<Prediction::PredictionResult>
                     &prediction_result_msg) {
            std::cout << "receive prediction_result "
                      << prediction_result_msg->header().timestamp()
                      << std::endl;
            std::lock_guard<std::mutex> lock(msg_mutex_);
            prediction_result_msg_.CopyFrom(*prediction_result_msg);
          });

  auto vehicel_service_reader_ =
      planning_node_->CreateReader<VehicleService::VehicleServiceOutputInfo>(
          "/iflytek/vehicle_service",
          [this](const std::shared_ptr<VehicleService::VehicleServiceOutputInfo>
                     &vehicel_service_output_info_msg) {
            std::cout << "receive vehicel_service_output_info "
                      << vehicel_service_output_info_msg->header().timestamp()
                      << std::endl;
            std::lock_guard<std::mutex> lock(msg_mutex_);
            vehicel_service_output_info_msg_.CopyFrom(
                *vehicel_service_output_info_msg);
          });

  auto radar_perception_objects_reader_ =
      planning_node_
          ->CreateReader<RadarPerceptionObjects::RadarPerceptionObjectsInfo>(
              "/iflytek/radar_perception_info",
              [this](const std::shared_ptr<
                     RadarPerceptionObjects::RadarPerceptionObjectsInfo>
                         &radar_perception_objects_info_msg) {
                std::cout
                    << "receive radar_perception_objects_info "
                    << radar_perception_objects_info_msg->header().timestamp()
                    << std::endl;
                std::lock_guard<std::mutex> lock(msg_mutex_);
                radar_perception_objects_info_msg_.CopyFrom(
                    *radar_perception_objects_info_msg);
              });

  auto control_output_reader_ =
      planning_node_->CreateReader<ControlCommand::ControlOutput>(
          "/iflytek/control/control_command",
          [this](const std::shared_ptr<ControlCommand::ControlOutput>
                     &control_output_msg) {
            std::cout << "receive control_output "
                      << control_output_msg->header().timestamp() << std::endl;
            std::lock_guard<std::mutex> lock(msg_mutex_);
            control_output_msg_.CopyFrom(*control_output_msg);
          });

  auto hmi_reader_ = planning_node_->CreateReader<HmiMcuInner::HmiMcuInner>(
      "/iflytek/hmi/mcu_inner",
      [this](const std::shared_ptr<HmiMcuInner::HmiMcuInner>
                 &hmi_mcu_inner_info_msg) {
        std::cout << "receive hmi_mcu_inner_info_output "
                  << hmi_mcu_inner_info_msg->header().timestamp() << std::endl;
        std::lock_guard<std::mutex> lock(msg_mutex_);
        hmi_mcu_inner_info_msg_.CopyFrom(*hmi_mcu_inner_info_msg);
      });

  auto parking_fusion_info_reader_ =
      planning_node_->CreateReader<ParkingFusionInfo>(
          "/iflytek/fusion/parking_slot",
          [this](const std::shared_ptr<ParkingFusionInfo>
                     &parking_fusion_info_msg) {
            std::lock_guard<std::mutex> lock(msg_mutex_);
            parking_fusion_info_msg_.CopyFrom(*parking_fusion_info_msg);
          });

  auto func_state_machine_reader_ =
      planning_node_->CreateReader<FuncStateMachine::FuncStateMachine>(
          "/iflytek/system_state/soc_state",
          [this](const std::shared_ptr<FuncStateMachine::FuncStateMachine>
                     &func_state_machine_msg) {
            std::lock_guard<std::mutex> lock(msg_mutex_);
            func_state_machine_msg_.CopyFrom(*func_state_machine_msg);
          });

  // -------------- writter topics --------------
  planning_writer_ =
      planning_node_->CreateWriter<PlanningOutput::PlanningOutput>(
          "/iflytek/planning/plan");
  planning_debug_writer_ =
      planning_node_->CreateWriter<planning::common::PlanningDebugInfo>(
          "/iflytek/planning/debug_info");
  planning_hmi_Info_writer_ =
      planning_node_->CreateWriter<PlanningHMI::PlanningHMIOutputInfoStr>(
          "/iflytek/planning/hmi");

  // 3.planning初始化，使用general_planning
  planning_base_ = std::make_unique<GeneralPlanning>();

  return true;
}

bool PlanningComponent::Proc() {
  // system("clear");
  std::cout << "==============The planning component running============="
            << std::endl;
  LOG_DEBUG("GeneralPlanning::RunOnce \n");
  double start_time = IflyTime::Now_ms();

  auto &planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();

  // 1.update all inputs to local_view
  {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    auto input_topic_timestamp = planning_debug_data->mutable_input_topic_timestamp();
    auto input_topic_latency = planning_debug_data->mutable_input_topic_latency();
    constexpr double US_PER_MS = 1000.0;

    local_view_.prediction_result = prediction_result_msg_;
    local_view_.prediction_result_recv_time = start_time;
    input_topic_timestamp->set_prediction(prediction_result_msg_.header().timestamp());
    input_topic_latency->set_prediction(start_time - prediction_result_msg_.header().timestamp() / US_PER_MS);
    prediction_result_msg_.Clear();

    local_view_.road_info = road_info_msg_;
    local_view_.road_info_recv_time = start_time;
    input_topic_timestamp->set_fusion_road(road_info_msg_.header().timestamp());
    input_topic_latency->set_fusion_road(start_time - road_info_msg_.header().timestamp() / US_PER_MS);
    road_info_msg_.Clear();

    local_view_.localization_estimate = localization_estimate_msg_;
    local_view_.localization_estimate_recv_time = start_time;
    input_topic_timestamp->set_localization(localization_estimate_msg_.header().timestamp());
    input_topic_latency->set_localization(start_time - localization_estimate_msg_.header().timestamp() / US_PER_MS);
    localization_estimate_msg_.Clear();

    local_view_.fusion_objects_info = fusion_objects_info_msg_;
    local_view_.fusion_objects_info_recv_time = start_time;
    input_topic_timestamp->set_fusion_object(fusion_objects_info_msg_.header().timestamp());
    input_topic_latency->set_fusion_object(start_time - fusion_objects_info_msg_.header().timestamp() / US_PER_MS);
    fusion_objects_info_msg_.Clear();

    local_view_.vehicel_service_output_info = vehicel_service_output_info_msg_;
    local_view_.vehicel_service_output_info_recv_time = start_time;
    input_topic_timestamp->set_vehicle_service(vehicel_service_output_info_msg_.header().timestamp());
    input_topic_latency->set_vehicle_service(start_time - vehicel_service_output_info_msg_.header().timestamp() / US_PER_MS);
    vehicel_service_output_info_msg_.Clear();

    local_view_.radar_perception_objects_info =
        radar_perception_objects_info_msg_;
    local_view_.radar_perception_objects_info_recv_time = start_time;
    input_topic_timestamp->set_radar_perception(radar_perception_objects_info_msg_.header().timestamp());
    input_topic_latency->set_radar_perception(start_time - radar_perception_objects_info_msg_.header().timestamp() / US_PER_MS);
    radar_perception_objects_info_msg_.Clear();

    local_view_.control_output = control_output_msg_;
    local_view_.control_output_recv_time = start_time;
    input_topic_timestamp->set_control_output(control_output_msg_.header().timestamp());
    input_topic_latency->set_control_output(start_time - control_output_msg_.header().timestamp() / US_PER_MS);
    control_output_msg_.Clear();

    local_view_.hmi_mcu_inner_info = hmi_mcu_inner_info_msg_;
    local_view_.hmi_mcu_inner_info_recv_time = start_time;
    input_topic_timestamp->set_hmi(hmi_mcu_inner_info_msg_.header().timestamp());
    input_topic_latency->set_hmi(start_time - hmi_mcu_inner_info_msg_.header().timestamp() / US_PER_MS);
    hmi_mcu_inner_info_msg_.Clear();

    local_view_.parking_fusion_info = parking_fusion_info_msg_;
    local_view_.parking_fusion_info_recv_time = start_time;
    input_topic_timestamp->set_parking_fusion(parking_fusion_info_msg_.header().timestamp());
    input_topic_latency->set_parking_fusion(start_time - parking_fusion_info_msg_.header().timestamp() / US_PER_MS);
    parking_fusion_info_msg_.Clear();

    local_view_.function_state_machine_info = func_state_machine_msg_;
    local_view_.function_state_machine_info_recv_time = start_time;
    func_state_machine_msg_.Clear();
  }

  // 2.planning run
  PlanningOutput::PlanningOutput planning_output;
  DebugOutput debug_output;
  PlanningHMI::PlanningHMIOutputInfoStr planning_hmi_Info;
  std::cout << "==============The planning enters RunOnce============="
            << std::endl;
  bool run_success = planning_base_->RunOnce(local_view_, &planning_output, debug_output,
                          planning_hmi_Info);

  // 3.get output & publish
  // 重新刷新相对时间 - TBD
  // const double dt = start_time - adc_trajectory_pb.header().timestamp_sec();
  // for (auto& p : *adc_trajectory_pb.mutable_trajectory_point()) {
  //   p.set_relative_time(p.relative_time() + dt);
  // }

  planning_debug_data->set_timestamp(IflyTime::Now_us());
  // 获取debug json信息
  auto debug_info_json = *DebugInfoManager::GetInstance().GetDebugJson();
  planning_debug_data->set_data_json(mjson::Json(debug_info_json).dump());
  planning_debug_writer_->Write(*planning_debug_data);

  // set meta time
  if (run_success) {
    auto time_stamp_us = IflyTime::Now_us();
    auto header = planning_output.mutable_meta()->mutable_header();
    header->set_timestamp(time_stamp_us);
    header->set_version("TEST");
    planning_writer_->Write(planning_output);
  }

  planning_hmi_Info_writer_->Write(planning_hmi_Info);
  double planning_cost_time = IflyTime::Now_ms() - start_time;
  if (planning_cost_time > 50.0) {
    LOG_ERROR("The cost time of proc() is too long: [%f] ms!\n",
              planning_cost_time);
  }
  LOG_WARNING("The cost time of proc() is: [%f] ms\n", planning_cost_time);

  return true;
}

DebugOutput PlanningComponent::GetDebugInfo() { return debug_info_; }

} // namespace planning

#pragma once

#include <functional>

#include "control_command.pb.h"
#include "ehr.pb.h"
#include "func_state_machine.pb.h"
#include "fusion_objects.pb.h"
#include "fusion_road.pb.h"
#include "general_planning.h"
#include "groundline_perception.pb.h"
#include "hmi_mcu_inner.pb.h"
#include "ifly_parking_map.pb.h"
#include "ifly_time.h"
#include "local_view.h"
#include "localization.pb.h"
#include "parking_fusion.pb.h"
#include "parking_slot_list.pb.h"
#include "planning_debug_info.pb.h"
#include "planning_hmi.pb.h"
#include "planning_plan.pb.h"
#include "prediction.pb.h"
#include "uss_wave_info.pb.h"
#include "vehicle_service.pb.h"

namespace planning {

class PlanningAdapter {
 public:
  PlanningAdapter() = default;
  ~PlanningAdapter() = default;

  void Init();
  void Proc();

  void FeedFusionObjects(
      const std::shared_ptr<FusionObjects::FusionObjectsInfo>&
          fusion_objects_info_msg) {
    // std::cout << "receive fusion fusion_objects_info "
    //           << fusion_objects_info_msg->header().timestamp() << std::endl;
    std::lock_guard<std::mutex> lock(msg_mutex_);
    fusion_objects_info_msg_.CopyFrom(*fusion_objects_info_msg);
    fusion_objects_info_msg_recv_time_ = IflyTime::Now_ms();
  }

  void FeedFusionRoad(
      const std::shared_ptr<FusionRoad::RoadInfo>& road_info_msg) {
    // std::cout << "receive fusion road_info "
    //           << road_info_msg->header().timestamp() << std::endl;
    std::lock_guard<std::mutex> lock(msg_mutex_);
    road_info_msg_.CopyFrom(*road_info_msg);
    road_info_msg_recv_time_ = IflyTime::Now_ms();
  }

  void FeedGroundLinePerception(
      const std::shared_ptr<GroundLinePerception::GroundLinePerceptionInfo>&
          ground_line_perception_msg) {
    std::cout << "receive ground_line_perception "
              << ground_line_perception_msg->header().timestamp() << std::endl;
    std::lock_guard<std::mutex> lock(msg_mutex_);
    ground_line_perception_msg_.CopyFrom(*ground_line_perception_msg);
    ground_line_perception_msg_recv_time_ = IflyTime::Now_ms();
  }

  void FeedLocalizationOutput(
      const std::shared_ptr<LocalizationOutput::LocalizationEstimate>&
          localization_estimate_msg) {
    // std::cout << "receive localization_estimate "
    //           << localization_estimate_msg->header().timestamp() <<
    //           std::endl;
    std::lock_guard<std::mutex> lock(msg_mutex_);
    localization_estimate_msg_.CopyFrom(*localization_estimate_msg);
    localization_estimate_msg_recv_time_ = IflyTime::Now_ms();
  }

  void FeedPredictionResult(const std::shared_ptr<Prediction::PredictionResult>&
                                prediction_result_msg) {
    // std::cout << "receive prediction_result "
    //           << prediction_result_msg->header().timestamp() << std::endl;
    std::lock_guard<std::mutex> lock(msg_mutex_);
    prediction_result_msg_.CopyFrom(*prediction_result_msg);
    prediction_result_msg_recv_time_ = IflyTime::Now_ms();
  }

  void FeedVehicleService(
      const std::shared_ptr<VehicleService::VehicleServiceOutputInfo>&
          vehicle_service_output_info_msg) {
    // std::cout << "receive vehicle_service_output_info "
    //           << vehicle_service_output_info_msg->header().timestamp()
    //           << std::endl;
    std::lock_guard<std::mutex> lock(msg_mutex_);
    vehicle_service_output_info_msg_.CopyFrom(*vehicle_service_output_info_msg);
    vehicle_service_output_info_msg_recv_time_ = IflyTime::Now_ms();
  }

  void FeedControlCommand(const std::shared_ptr<ControlCommand::ControlOutput>&
                              control_output_msg) {
    // std::cout << "receive control_output "
    //           << control_output_msg->header().timestamp() << std::endl;
    std::lock_guard<std::mutex> lock(msg_mutex_);
    control_output_msg_.CopyFrom(*control_output_msg);
    control_output_msg_recv_time_ = IflyTime::Now_ms();
  }

  void FeedHmiMcuInner(
      const std::shared_ptr<HmiMcuInner::HmiMcuInner>& hmi_mcu_inner_info_msg) {
    // std::cout << "receive hmi_mcu_inner_info_output "
    //           << hmi_mcu_inner_info_msg->header().timestamp() << std::endl;
    std::lock_guard<std::mutex> lock(msg_mutex_);
    hmi_mcu_inner_info_msg_.CopyFrom(*hmi_mcu_inner_info_msg);
    hmi_mcu_inner_info_msg_recv_time_ = IflyTime::Now_ms();
  }

  void FeedParkingFusion(
      const std::shared_ptr<ParkingFusion::ParkingFusionInfo>&
          parking_fusion_info_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    parking_fusion_info_msg_.CopyFrom(*parking_fusion_info_msg);
    parking_fusion_info_msg_recv_time_ = IflyTime::Now_ms();
  }

  void FeedParkingMap(const std::shared_ptr<IFLYParkingMap::ParkingInfo>&
                          parking_map_info_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    parking_map_info_msg_.CopyFrom(*parking_map_info_msg);
    parking_map_info_msg_recv_time_ = IflyTime::Now_ms();
  }

  void FeedFuncStateMachine(
      const std::shared_ptr<FuncStateMachine::FuncStateMachine>&
          func_state_machine_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    func_state_machine_msg_.CopyFrom(*func_state_machine_msg);
  }

  void FeedUssWaveInfo(
      const std::shared_ptr<UssWaveInfo::UssWaveInfo>& uss_wave_info_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    uss_wave_info_msg_.CopyFrom(*uss_wave_info_msg);
    uss_wave_info_msg_recv_time_ = IflyTime::Now_ms();
  }

  void FeedMap(const std::shared_ptr<Map::StaticMap>& map_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    map_info_msg_.CopyFrom(*map_msg);
    map_info_msg_recv_time_ = IflyTime::Now_ms();
  }

  void RegisterOutputWriter(
      const std::function<void(PlanningOutput::PlanningOutput)>&
          planning_writer) {
    planning_writer_ = planning_writer;
  }
  void RegisterDebugInfoWriter(
      const std::function<void(planning::common::PlanningDebugInfo)>&
          planning_debug_writer) {
    planning_debug_writer_ = planning_debug_writer;
  }
  void RegisterHMIOutputInfoWriter(
      const std::function<void(PlanningHMI::PlanningHMIOutputInfoStr)>&
          planning_hmi_info_writer) {
    planning_hmi_info_writer_ = planning_hmi_info_writer;
  }

 private:
  std::mutex msg_mutex_;

  Prediction::PredictionResult prediction_result_msg_;
  int64_t prediction_result_msg_recv_time_;

  FusionRoad::RoadInfo road_info_msg_;
  int64_t road_info_msg_recv_time_;

  GroundLinePerception::GroundLinePerceptionInfo ground_line_perception_msg_;
  int64_t ground_line_perception_msg_recv_time_;

  LocalizationOutput::LocalizationEstimate localization_estimate_msg_;
  int64_t localization_estimate_msg_recv_time_;

  FusionObjects::FusionObjectsInfo fusion_objects_info_msg_;
  int64_t fusion_objects_info_msg_recv_time_;

  VehicleService::VehicleServiceOutputInfo vehicle_service_output_info_msg_;
  int64_t vehicle_service_output_info_msg_recv_time_;

  ControlCommand::ControlOutput control_output_msg_;
  int64_t control_output_msg_recv_time_;

  HmiMcuInner::HmiMcuInner hmi_mcu_inner_info_msg_;
  int64_t hmi_mcu_inner_info_msg_recv_time_;

  ParkingFusion::ParkingFusionInfo parking_fusion_info_msg_;
  int64_t parking_fusion_info_msg_recv_time_;

  IFLYParkingMap::ParkingInfo parking_map_info_msg_;
  int64_t parking_map_info_msg_recv_time_;

  FuncStateMachine::FuncStateMachine func_state_machine_msg_;

  UssWaveInfo::UssWaveInfo uss_wave_info_msg_;
  int64_t uss_wave_info_msg_recv_time_;

  Map::StaticMap map_info_msg_;
  int64_t map_info_msg_recv_time_;

  std::function<void(PlanningOutput::PlanningOutput)> planning_writer_ =
      nullptr;
  std::function<void(planning::common::PlanningDebugInfo)>
      planning_debug_writer_ = nullptr;
  std::function<void(PlanningHMI::PlanningHMIOutputInfoStr)>
      planning_hmi_info_writer_ = nullptr;

  LocalView local_view_;
  std::unique_ptr<GeneralPlanning> planning_base_ = nullptr;
  PlanningOutput::PlanningOutput last_planning_output_;
};

}  // namespace planning
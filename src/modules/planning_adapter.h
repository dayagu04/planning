#pragma once

#include <atomic>
#include <functional>
#include <memory>

#include "control_command.pb.h"
#include "ehr.pb.h"
#include "func_state_machine.pb.h"
#include "fusion_objects.pb.h"
#include "fusion_road.pb.h"
#include "groundline_perception.pb.h"
#include "hmi_mcu_inner.pb.h"
#include "ifly_localization.pb.h"
#include "ifly_parking_map.pb.h"
#include "ifly_time.h"
#include "local_view.h"
#include "localization.pb.h"
#include "parking_fusion.pb.h"
#include "parking_slot_list.pb.h"
#include "planning_debug_info.pb.h"
#include "planning_hmi.pb.h"
#include "planning_plan.pb.h"
#include "planning_scheduler.h"
#include "prediction.pb.h"
#include "uss_percept_info.pb.h"
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
    is_fusion_objects_info_msg_updated_.store(true);
  }

  void FeedFusionRoad(
      const std::shared_ptr<FusionRoad::RoadInfo>& road_info_msg) {
    // std::cout << "receive fusion road_info "
    //           << road_info_msg->header().timestamp() << std::endl;
    std::lock_guard<std::mutex> lock(msg_mutex_);
    road_info_msg_.CopyFrom(*road_info_msg);
    road_info_msg_recv_time_ = IflyTime::Now_ms();
    is_road_info_msg_updated_.store(true);
  }

  void FeedGroundLinePerception(
      const std::shared_ptr<GroundLinePerception::GroundLinePerceptionInfo>&
          ground_line_perception_msg) {
    // std::cout << "receive ground_line_perception "
    //           << ground_line_perception_msg->header().timestamp() << std::endl;
    std::lock_guard<std::mutex> lock(msg_mutex_);
    ground_line_perception_msg_.CopyFrom(*ground_line_perception_msg);
    ground_line_perception_msg_recv_time_ = IflyTime::Now_ms();
    is_ground_line_perception_msg_updated_.store(true);
  }

  void FeedLocalizationEstimateOutput(
      const std::shared_ptr<LocalizationOutput::LocalizationEstimate>&
          localization_estimate_msg) {
    // std::cout << "receive localization_estimate "
    //           << localization_estimate_msg->header().timestamp() <<
    //           std::endl;
    std::lock_guard<std::mutex> lock(msg_mutex_);
    localization_estimate_msg_.CopyFrom(*localization_estimate_msg);
    localization_estimate_msg_recv_time_ = IflyTime::Now_ms();
    is_localization_estimate_msg_updated_.store(true);
  }

  void FeedLocalizationOutput(
      const std::shared_ptr<IFLYLocalization::IFLYLocalization>&
          localization_msg) {
    // std::cout << "receive localization_estimate "
    //           << localization_estimate_msg->header().timestamp() <<
    //           std::endl;
    std::lock_guard<std::mutex> lock(msg_mutex_);
    localization_msg_.CopyFrom(*localization_msg);
    localization_msg_recv_time_ = IflyTime::Now_ms();
    is_localization_msg_updated_.store(true);
  }

  void FeedPredictionResult(const std::shared_ptr<Prediction::PredictionResult>&
                                prediction_result_msg) {
    // std::cout << "receive prediction_result "
    //           << prediction_result_msg->header().timestamp() << std::endl;
    std::lock_guard<std::mutex> lock(msg_mutex_);
    prediction_result_msg_.CopyFrom(*prediction_result_msg);
    prediction_result_msg_recv_time_ = IflyTime::Now_ms();
    is_prediction_result_msg_updated_.store(true);
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
    is_vehicle_service_output_info_msg_updated_.store(true);
  }

  void FeedControlCommand(const std::shared_ptr<ControlCommand::ControlOutput>&
                              control_output_msg) {
    // std::cout << "receive control_output "
    //           << control_output_msg->header().timestamp() << std::endl;
    std::lock_guard<std::mutex> lock(msg_mutex_);
    control_output_msg_.CopyFrom(*control_output_msg);
    control_output_msg_recv_time_ = IflyTime::Now_ms();
    is_control_output_msg_updated_.store(true);
  }

  void FeedHmiMcuInner(
      const std::shared_ptr<HmiMcuInner::HmiMcuInner>& hmi_mcu_inner_info_msg) {
    // std::cout << "receive hmi_mcu_inner_info_output "
    //           << hmi_mcu_inner_info_msg->header().timestamp() << std::endl;
    std::lock_guard<std::mutex> lock(msg_mutex_);
    hmi_mcu_inner_info_msg_.CopyFrom(*hmi_mcu_inner_info_msg);
    hmi_mcu_inner_info_msg_recv_time_ = IflyTime::Now_ms();
    is_hmi_mcu_inner_info_msg_updated_.store(true);
  }

  void FeedParkingFusion(
      const std::shared_ptr<ParkingFusion::ParkingFusionInfo>&
          parking_fusion_info_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    parking_fusion_info_msg_.CopyFrom(*parking_fusion_info_msg);
    parking_fusion_info_msg_recv_time_ = IflyTime::Now_ms();
    is_parking_fusion_info_msg_updated_.store(true);
  }

  void FeedParkingMap(const std::shared_ptr<IFLYParkingMap::ParkingInfo>&
                          parking_map_info_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    parking_map_info_msg_.CopyFrom(*parking_map_info_msg);
    parking_map_info_msg_recv_time_ = IflyTime::Now_ms();
    is_parking_map_info_msg_updated_.store(true);
  }

  void FeedFuncStateMachine(
      const std::shared_ptr<FuncStateMachine::FuncStateMachine>&
          func_state_machine_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    func_state_machine_msg_.CopyFrom(*func_state_machine_msg);
    is_func_state_machine_msg_updated_.store(true);
  }

  void FeedUssWaveInfo(
      const std::shared_ptr<UssWaveInfo::UssWaveInfo>& uss_wave_info_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    uss_wave_info_msg_.CopyFrom(*uss_wave_info_msg);
    uss_wave_info_msg_recv_time_ = IflyTime::Now_ms();
    is_uss_wave_info_msg_updated_.store(true);
  }

  void FeedUssPerceptInfo(const std::shared_ptr<UssPerceptInfo::UssPerceptInfo>&
                              uss_percept_info_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    uss_percept_info_msg_.CopyFrom(*uss_percept_info_msg);
    uss_percept_info_msg_recv_time_ = IflyTime::Now_ms();
    is_uss_percept_info_msg_updated_.store(true);
  }

  void FeedMap(const std::shared_ptr<Map::StaticMap>& map_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    map_info_msg_.CopyFrom(*map_msg);
    map_info_msg_recv_time_ = IflyTime::Now_ms();
    is_map_info_msg_updated_.store(true);
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
  std::atomic<bool> is_prediction_result_msg_updated_{false};

  FusionRoad::RoadInfo road_info_msg_;
  int64_t road_info_msg_recv_time_;
  std::atomic<bool> is_road_info_msg_updated_{false};

  GroundLinePerception::GroundLinePerceptionInfo ground_line_perception_msg_;
  int64_t ground_line_perception_msg_recv_time_;
  std::atomic<bool> is_ground_line_perception_msg_updated_{false};

  LocalizationOutput::LocalizationEstimate localization_estimate_msg_;
  int64_t localization_estimate_msg_recv_time_;
  std::atomic<bool> is_localization_estimate_msg_updated_{false};

  IFLYLocalization::IFLYLocalization localization_msg_;
  int64_t localization_msg_recv_time_;
  std::atomic<bool> is_localization_msg_updated_{false};

  FusionObjects::FusionObjectsInfo fusion_objects_info_msg_;
  int64_t fusion_objects_info_msg_recv_time_;
  std::atomic<bool> is_fusion_objects_info_msg_updated_{false};

  VehicleService::VehicleServiceOutputInfo vehicle_service_output_info_msg_;
  int64_t vehicle_service_output_info_msg_recv_time_;
  std::atomic<bool> is_vehicle_service_output_info_msg_updated_{false};

  ControlCommand::ControlOutput control_output_msg_;
  int64_t control_output_msg_recv_time_;
  std::atomic<bool> is_control_output_msg_updated_{false};

  HmiMcuInner::HmiMcuInner hmi_mcu_inner_info_msg_;
  int64_t hmi_mcu_inner_info_msg_recv_time_;
  std::atomic<bool> is_hmi_mcu_inner_info_msg_updated_{false};

  ParkingFusion::ParkingFusionInfo parking_fusion_info_msg_;
  int64_t parking_fusion_info_msg_recv_time_;
  std::atomic<bool> is_parking_fusion_info_msg_updated_{false};

  IFLYParkingMap::ParkingInfo parking_map_info_msg_;
  int64_t parking_map_info_msg_recv_time_;
  std::atomic<bool> is_parking_map_info_msg_updated_{false};

  FuncStateMachine::FuncStateMachine func_state_machine_msg_;
  std::atomic<bool> is_func_state_machine_msg_updated_{false};

  UssWaveInfo::UssWaveInfo uss_wave_info_msg_;
  int64_t uss_wave_info_msg_recv_time_;
  std::atomic<bool> is_uss_wave_info_msg_updated_{false};

  UssPerceptInfo::UssPerceptInfo uss_percept_info_msg_;
  int64_t uss_percept_info_msg_recv_time_;
  std::atomic<bool> is_uss_percept_info_msg_updated_{false};

  Map::StaticMap map_info_msg_;
  int64_t map_info_msg_recv_time_;
  std::atomic<bool> is_map_info_msg_updated_{false};

  std::function<void(PlanningOutput::PlanningOutput)> planning_writer_ =
      nullptr;
  std::function<void(planning::common::PlanningDebugInfo)>
      planning_debug_writer_ = nullptr;
  std::function<void(PlanningHMI::PlanningHMIOutputInfoStr)>
      planning_hmi_info_writer_ = nullptr;

  std::shared_ptr<LocalView> local_view_ptr_;

  std::unique_ptr<PlanningScheduler> planning_scheduler_ = nullptr;
  PlanningOutput::PlanningOutput last_planning_output_;

  uint64_t frame_num_ = 0;
};

}  // namespace planning
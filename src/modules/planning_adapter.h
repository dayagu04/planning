#pragma once

#include <atomic>
#include <functional>
#include <memory>

#include "fm_info_c.h"
#include "ifly_time.h"
#include "local_view.h"
#include "planning_scheduler.h"

namespace planning {

class PlanningAdapter : public iflyauto::interface::PlanningInterface {
 public:
  PlanningAdapter() = default;
  ~PlanningAdapter() { StopGlog(); };

  bool Init() override;
  bool Proc() override;

  void Feed_IflytekFusionObjects(
      const iflyauto::FusionObjectsInfo& fusion_objects_info_msg) override {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    fusion_objects_info_msg_ = fusion_objects_info_msg;
    fusion_objects_info_msg_recv_time_ = IflyTime::Now_ms();
    is_fusion_objects_info_msg_updated_.store(true);
  }

  void Feed_IflytekFusionOccupancyObjects(
      const iflyauto::FusionOccupancyObjectsInfo& fusion_occupancy_objects_info_msg) override {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    fusion_occupancy_objects_info_msg_ = fusion_occupancy_objects_info_msg;
    fusion_occupancy_objects_info_msg_recv_time_ = IflyTime::Now_ms();
    is_fusion_occupancy_objects_info_msg_updated_.store(true);
  }

  void Feed_IflytekFusionRoadFusion(const iflyauto::RoadInfo& road_info_msg) override {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    road_info_msg_ = road_info_msg;
    road_info_msg_recv_time_ = IflyTime::Now_ms();
    is_road_info_msg_updated_.store(true);
  }

  // void FeedGroundLinePerception(
  //     const iflyauto::FusionGroundLineInfo& ground_line_perception_msg) {
  //   std::lock_guard<std::mutex> lock(msg_mutex_);
  //   ground_line_perception_msg_ = ground_line_perception_msg;
  //   ground_line_perception_msg_recv_time_ = IflyTime::Now_ms();
  //   is_ground_line_perception_msg_updated_.store(true);
  // }

  void FeedGroundLine(
      const iflyauto::FusionGroundLineInfo& ground_line_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    ground_line_perception_msg_ = ground_line_msg;
    ground_line_perception_msg_recv_time_ = IflyTime::Now_ms();
    is_ground_line_perception_msg_updated_.store(true);
  }

  void FeedFusionSpeedBump(
      const iflyauto::FusionDecelerInfo& fusion_speed_bump_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    fusion_speed_bump_msg_ = fusion_speed_bump_msg;
    fusion_speed_bump_msg_recv_time_ = IflyTime::Now_ms();
    is_fusion_speed_bump_msg_updated_.store(true);
  }

  void Feed_IflytekLocalizationEgomotion(
      const iflyauto::IFLYLocalization& localization_msg) override {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    localization_msg_ = localization_msg;
    localization_msg_recv_time_ = IflyTime::Now_ms();
    is_localization_msg_updated_.store(true);
  }

  void FeedLocalizationEstimateOutput(
      const iflyauto::interface_2_4_6::LocalizationEstimate&
          localization_estimate_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    localization_estimate_msg_ = localization_estimate_msg;
    localization_estimate_msg_recv_time_ = IflyTime::Now_ms();
    is_localization_estimate_msg_updated_.store(true);
  }

  void Feed_IflytekPredictionPredictionResult(
      const iflyauto::PredictionResult& prediction_result_msg) override {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    prediction_result_msg_ = prediction_result_msg;
    prediction_result_msg_recv_time_ = IflyTime::Now_ms();
    is_prediction_result_msg_updated_.store(true);
  }

  void Feed_IflytekVehicleService(
      const iflyauto::VehicleServiceOutputInfo& vehicle_service_output_info_msg) override {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    vehicle_service_output_info_msg_ = vehicle_service_output_info_msg;
    vehicle_service_output_info_msg_recv_time_ = IflyTime::Now_ms();
    is_vehicle_service_output_info_msg_updated_.store(true);
  }

  void Feed_IflytekControlControlCommand(
      const iflyauto::ControlOutput& control_output_msg) override {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    control_output_msg_ = control_output_msg;
    control_output_msg_recv_time_ = IflyTime::Now_ms();
    is_control_output_msg_updated_.store(true);
  }

  // void FeedHmiInner(const iflyauto::HmiInner& hmi_inner_info_msg) {
  //   std::lock_guard<std::mutex> lock(msg_mutex_);
  //   hmi_inner_info_msg_ = hmi_inner_info_msg;
  //   hmi_inner_info_msg_recv_time_ = IflyTime::Now_ms();
  //   is_hmi_inner_info_msg_updated_.store(true);
  // }

  void FeedHmiMcuInner(
      const iflyauto::interface_2_4_5::HmiMcuInner& hmi_mcu_inner_info_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    hmi_mcu_inner_info_msg_ = hmi_mcu_inner_info_msg;
    hmi_mcu_inner_info_msg_recv_time_ = IflyTime::Now_ms();
    is_hmi_mcu_inner_info_msg_updated_.store(true);
  }

  void Feed_IflytekFusionParkingSlot(
      const iflyauto::ParkingFusionInfo& parking_fusion_info_msg) override {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    parking_fusion_info_msg_ = parking_fusion_info_msg;
    parking_fusion_info_msg_recv_time_ = IflyTime::Now_ms();
    is_parking_fusion_info_msg_updated_.store(true);
  }

  // void FeedParkingMap(const IFLYParkingMap::ParkingInfo& parking_map_info_msg) {
  //   std::lock_guard<std::mutex> lock(msg_mutex_);
  //   parking_map_info_msg_ = parking_map_info_msg;
  //   parking_map_info_msg_recv_time_ = IflyTime::Now_ms();
  //   is_parking_map_info_msg_updated_.store(true);
  // }

  void Feed_IflytekFsmSocState(
      const iflyauto::FuncStateMachine& func_state_machine_msg) override {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    func_state_machine_msg_ = func_state_machine_msg;
    func_state_machine_msg_recv_time_ = IflyTime::Now_ms();
    is_func_state_machine_msg_updated_.store(true);
  }

  void Feed_IflytekUssUsswaveInfo(const iflyauto::UssWaveInfo& uss_wave_info_msg) override {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    uss_wave_info_msg_ = uss_wave_info_msg;
    uss_wave_info_msg_recv_time_ = IflyTime::Now_ms();
    is_uss_wave_info_msg_updated_.store(true);
  }

  void Feed_IflytekUssUssPerceptionInfo(
      const iflyauto::UssPerceptInfo& uss_percept_info_msg) override {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    uss_percept_info_msg_ = uss_percept_info_msg;
    uss_percept_info_msg_recv_time_ = IflyTime::Now_ms();
    is_uss_percept_info_msg_updated_.store(true);
  }

  void FeedMap(const std::shared_ptr<Map::StaticMap>& map_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    map_info_msg_.CopyFrom(*map_msg);
    map_info_msg_recv_time_ = IflyTime::Now_ms();
    is_map_info_msg_updated_.store(true);
    std::cout << "feed static map_info_msg_ end" << std::endl;
  }

  void Feed_IflytekEhrSdmapInfo(const SdMapSwtx::SdMap& sd_map_info_msg) override {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    sd_map_info_msg_.CopyFrom(sd_map_info_msg);
    std::cout << "feed sd_map_info_msg_ end" << std::endl;
    sd_map_info_msg_recv_time_ = IflyTime::Now_ms();
    is_sd_map_info_msg_updated_.store(true);
  }

  void Feed_IflytekCameraPerceptionTrafficSignRecognition(
      const iflyauto::CameraPerceptionTsrInfo& perception_tsr_msg) override {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    perception_tsr_msg_ = perception_tsr_msg;
    perception_tsr_msg_recv_time_ = IflyTime::Now_ms();
    is_perception_tsr_msg_updated_.store(true);
  }

  void RegWriter_IflytekPlanningPlan(
      const std::function<void(const iflyauto::PlanningOutput&)>& planning_writer)
      override {
    planning_writer_ = planning_writer;
  }
  void RegWriter_IflytekPlanningHmi(
      const std::function<void(const iflyauto::PlanningHMIOutputInfoStr&)>&
          planning_hmi_info_writer) override {
    planning_hmi_info_writer_ = planning_hmi_info_writer;
  }
  void RegWriter_IflytekPlanningDebugInfo(
      const std::function<void(const iflyauto::StructContainer &)>&
          planning_debug_writer) override {
    planning_debug_writer_ = planning_debug_writer;
  }

  void RegFmWriter_IflytekAlarmInfoPlanning(
      const std::function<void(const iflyauto::FmInfo&)>& fm_info_writer) override {
    fm_info_writer_ = fm_info_writer;
  }

 private:
  void ReportFmIfno(uint64 alarmId, uint64 alarmObj, bool fault_exist);

  void UpdateInputListInfo(iflyauto::MsgMeta& msg_meta);

  void UpdateApaResetFlag();

 private:
  std::mutex msg_mutex_;

  iflyauto::PredictionResult prediction_result_msg_;
  int64_t prediction_result_msg_recv_time_;
  std::atomic<bool> is_prediction_result_msg_updated_{false};

  iflyauto::RoadInfo road_info_msg_;
  int64_t road_info_msg_recv_time_;
  std::atomic<bool> is_road_info_msg_updated_{false};

  iflyauto::FusionGroundLineInfo ground_line_perception_msg_;
  int64_t ground_line_perception_msg_recv_time_;
  std::atomic<bool> is_ground_line_perception_msg_updated_{false};

  iflyauto::FusionDecelerInfo fusion_speed_bump_msg_;
  int64_t fusion_speed_bump_msg_recv_time_;
  std::atomic<bool> is_fusion_speed_bump_msg_updated_{false};

  iflyauto::IFLYLocalization localization_msg_;
  int64_t localization_msg_recv_time_;
  std::atomic<bool> is_localization_msg_updated_{false};

  iflyauto::interface_2_4_6::LocalizationEstimate localization_estimate_msg_;
  int64_t localization_estimate_msg_recv_time_;
  std::atomic<bool> is_localization_estimate_msg_updated_{false};

  iflyauto::FusionObjectsInfo fusion_objects_info_msg_;
  int64_t fusion_objects_info_msg_recv_time_;
  std::atomic<bool> is_fusion_objects_info_msg_updated_{false};

  iflyauto::FusionOccupancyObjectsInfo fusion_occupancy_objects_info_msg_;
  int64_t fusion_occupancy_objects_info_msg_recv_time_;
  std::atomic<bool> is_fusion_occupancy_objects_info_msg_updated_{false};

  iflyauto::VehicleServiceOutputInfo vehicle_service_output_info_msg_;
  int64_t vehicle_service_output_info_msg_recv_time_;
  std::atomic<bool> is_vehicle_service_output_info_msg_updated_{false};

  iflyauto::ControlOutput control_output_msg_;
  int64_t control_output_msg_recv_time_;
  std::atomic<bool> is_control_output_msg_updated_{false};

  // iflyauto::HmiInner hmi_inner_info_msg_;
  // int64_t hmi_inner_info_msg_recv_time_;
  // std::atomic<bool> is_hmi_inner_info_msg_updated_{false};

  iflyauto::interface_2_4_5::HmiMcuInner hmi_mcu_inner_info_msg_;
  int64_t hmi_mcu_inner_info_msg_recv_time_;
  std::atomic<bool> is_hmi_mcu_inner_info_msg_updated_{false};

  iflyauto::ParkingFusionInfo parking_fusion_info_msg_;
  int64_t parking_fusion_info_msg_recv_time_;
  std::atomic<bool> is_parking_fusion_info_msg_updated_{false};

  // IFLYParkingMap::ParkingInfo parking_map_info_msg_;
  // int64_t parking_map_info_msg_recv_time_;
  // std::atomic<bool> is_parking_map_info_msg_updated_{false};

  iflyauto::FuncStateMachine func_state_machine_msg_;
  int64_t func_state_machine_msg_recv_time_;
  std::atomic<bool> is_func_state_machine_msg_updated_{false};

  iflyauto::UssWaveInfo uss_wave_info_msg_;
  int64_t uss_wave_info_msg_recv_time_;
  std::atomic<bool> is_uss_wave_info_msg_updated_{false};

  iflyauto::UssPerceptInfo uss_percept_info_msg_;
  int64_t uss_percept_info_msg_recv_time_;
  std::atomic<bool> is_uss_percept_info_msg_updated_{false};

  Map::StaticMap map_info_msg_;
  int64_t map_info_msg_recv_time_;
  std::atomic<bool> is_map_info_msg_updated_{false};

  SdMapSwtx::SdMap sd_map_info_msg_;
  int64_t sd_map_info_msg_recv_time_;
  std::atomic<bool> is_sd_map_info_msg_updated_{false};
  iflyauto::CameraPerceptionTsrInfo perception_tsr_msg_;
  int64_t perception_tsr_msg_recv_time_;
  std::atomic<bool> is_perception_tsr_msg_updated_{false};

  std::function<void(const iflyauto::PlanningOutput &)>
      planning_writer_ = nullptr;
  std::function<void(const iflyauto::PlanningHMIOutputInfoStr &)>
      planning_hmi_info_writer_ = nullptr;
  std::function<void(const iflyauto::StructContainer &)>
      planning_debug_writer_ = nullptr;
  std::function<void(const iflyauto::FmInfo&)> fm_info_writer_ = nullptr;

  std::shared_ptr<LocalView> local_view_ptr_;

  std::unique_ptr<PlanningScheduler> planning_scheduler_ = nullptr;
  iflyauto::PlanningOutput last_planning_output_;

  uint64_t frame_num_ = 0;
};

}  // namespace planning

#ifndef X86
REG_COMPONENT(PlanningInterface, planning::PlanningAdapter);
#endif

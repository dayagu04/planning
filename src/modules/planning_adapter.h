#pragma once

#include <atomic>
#include <functional>
#include <memory>

#include "ehr_sdmap.pb.h"
#include "fm_info_c.h"
#include "ifly_time.h"
#include "local_view.h"
#include "planning_scheduler.h"
#include "struct_container.hpp"
#include "interface/src/legacy/interface2.4.5/hmi_mcu_inner_c.h"

namespace planning {

class PlanningAdapter {
 public:
  PlanningAdapter() = default;
  ~PlanningAdapter() = default;

  void Init();
  void Proc();

  void FeedFusionObjects(
      const iflyauto::FusionObjectsInfo& fusion_objects_info_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    fusion_objects_info_msg_ = fusion_objects_info_msg;
    fusion_objects_info_msg_recv_time_ = IflyTime::Now_ms();
    is_fusion_objects_info_msg_updated_.store(true);
  }

  void FeedFusionOccupancyObjects(const iflyauto::FusionOccupancyObjectsInfo&
                                      fusion_occupancy_objects_info_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    fusion_occupancy_objects_info_msg_ = fusion_occupancy_objects_info_msg;
    fusion_occupancy_objects_info_msg_recv_time_ = IflyTime::Now_ms();
    is_fusion_occupancy_objects_info_msg_updated_.store(true);
  }

  void FeedFusionRoad(const iflyauto::RoadInfo& road_info_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    road_info_msg_ = road_info_msg;
    road_info_msg_recv_time_ = IflyTime::Now_ms();
    is_road_info_msg_updated_.store(true);
  }

  void FeedGroundLinePerception(
      const iflyauto::GroundLinePerceptionInfo& ground_line_perception_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    ground_line_perception_msg_ = ground_line_perception_msg;
    ground_line_perception_msg_recv_time_ = IflyTime::Now_ms();
    is_ground_line_perception_msg_updated_.store(true);
  }

  void FeedLocalizationOutput(
      const iflyauto::IFLYLocalization& localization_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    localization_msg_ = localization_msg;
    localization_msg_recv_time_ = IflyTime::Now_ms();
    is_localization_msg_updated_.store(true);
  }

  void FeedPredictionResult(
      const iflyauto::PredictionResult& prediction_result_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    prediction_result_msg_ = prediction_result_msg;
    prediction_result_msg_recv_time_ = IflyTime::Now_ms();
    is_prediction_result_msg_updated_.store(true);
  }

  void FeedVehicleService(const iflyauto::VehicleServiceOutputInfo&
                              vehicle_service_output_info_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    vehicle_service_output_info_msg_ = vehicle_service_output_info_msg;
    vehicle_service_output_info_msg_recv_time_ = IflyTime::Now_ms();
    is_vehicle_service_output_info_msg_updated_.store(true);
  }

  void FeedControlCommand(const iflyauto::ControlOutput& control_output_msg) {
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

  void FeedHmiMcuInner(const iflyauto::interface_2_4_5::HmiMcuInner& hmi_mcu_inner_info_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    hmi_mcu_inner_info_msg_ = hmi_mcu_inner_info_msg;
    hmi_mcu_inner_info_msg_recv_time_ = IflyTime::Now_ms();
    is_hmi_mcu_inner_info_msg_updated_.store(true);
  }

  void FeedParkingFusion(
      const iflyauto::ParkingFusionInfo& parking_fusion_info_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    parking_fusion_info_msg_ = parking_fusion_info_msg;
    parking_fusion_info_msg_recv_time_ = IflyTime::Now_ms();
    is_parking_fusion_info_msg_updated_.store(true);
  }

  // void FeedParkingMap(const iflyauto::ParkingInfo& parking_map_info_msg) {
  //   std::lock_guard<std::mutex> lock(msg_mutex_);
  //   parking_map_info_msg_ = parking_map_info_msg;
  //   parking_map_info_msg_recv_time_ = IflyTime::Now_ms();
  //   is_parking_map_info_msg_updated_.store(true);
  // }

  void FeedFuncStateMachine(
      const iflyauto::FuncStateMachine& func_state_machine_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    func_state_machine_msg_ = func_state_machine_msg;
    func_state_machine_msg_recv_time_ = IflyTime::Now_ms();
    is_func_state_machine_msg_updated_.store(true);
  }

  void FeedUssWaveInfo(const iflyauto::UssWaveInfo& uss_wave_info_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    uss_wave_info_msg_ = uss_wave_info_msg;
    uss_wave_info_msg_recv_time_ = IflyTime::Now_ms();
    is_uss_wave_info_msg_updated_.store(true);
  }

  void FeedUssPerceptInfo(
      const iflyauto::UssPerceptInfo& uss_percept_info_msg) {
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
  }

  void FeedSdMap(const std::shared_ptr<SdMapSwtx::SdMap>& sd_map_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    sd_map_info_msg_.CopyFrom(*sd_map_msg);
    std::cout << "feed sd_map_info_msg_ end" << std::endl;
    sd_map_info_msg_recv_time_ = IflyTime::Now_ms();
    is_sd_map_info_msg_updated_.store(true);
  }

  void FeedPerceptionTsrInfo(const iflyauto::CameraPerceptionTsrInfo& tsr_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    perception_tsr_msg_ = tsr_msg;
    perception_tsr_msg_recv_time_ = IflyTime::Now_ms();
    is_perception_tsr_msg_updated_.store(true);
  }

  void RegisterOutputWriter(
      const std::function<
          void(const std::shared_ptr<iflyauto::StructContainer>&)>&
          planning_writer) {
    planning_writer_ = planning_writer;
  }
  void RegisterDebugInfoWriter(
      const std::function<
          void(const std::shared_ptr<iflyauto::StructContainer>&)>&
          planning_debug_writer) {
    planning_debug_writer_ = planning_debug_writer;
  }
  void RegisterHMIOutputInfoWriter(
      const std::function<
          void(const std::shared_ptr<iflyauto::StructContainer>&)>&
          planning_hmi_info_writer) {
    planning_hmi_info_writer_ = planning_hmi_info_writer;
  }

  void RegisterFmInfoWriter(
      const std::function<void(const iflyauto::FmInfo&)>& fm_info_writer) {
    fm_info_writer_ = fm_info_writer;
  }

 private:
  void ReportFmIfno(uint64 alarmId, uint64 alarmObj, bool fault_exist);

 private:
  void UpdateInputListInfo(iflyauto::MsgMeta& msg_meta);

  std::mutex msg_mutex_;

  iflyauto::PredictionResult prediction_result_msg_;
  int64_t prediction_result_msg_recv_time_;
  std::atomic<bool> is_prediction_result_msg_updated_{false};

  iflyauto::RoadInfo road_info_msg_;
  int64_t road_info_msg_recv_time_;
  std::atomic<bool> is_road_info_msg_updated_{false};

  iflyauto::GroundLinePerceptionInfo ground_line_perception_msg_;
  int64_t ground_line_perception_msg_recv_time_;
  std::atomic<bool> is_ground_line_perception_msg_updated_{false};

  iflyauto::IFLYLocalization localization_msg_;
  int64_t localization_msg_recv_time_;
  std::atomic<bool> is_localization_msg_updated_{false};

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

  // iflyauto::ParkingInfo parking_map_info_msg_;
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

  std::function<void(const std::shared_ptr<iflyauto::StructContainer>&)>
      planning_writer_ = nullptr;
  std::function<void(const std::shared_ptr<iflyauto::StructContainer>&)>
      planning_debug_writer_ = nullptr;
  std::function<void(const std::shared_ptr<iflyauto::StructContainer>&)>
      planning_hmi_info_writer_ = nullptr;
  std::function<void(const iflyauto::FmInfo&)> fm_info_writer_ = nullptr;

  std::shared_ptr<LocalView> local_view_ptr_;

  std::unique_ptr<PlanningScheduler> planning_scheduler_ = nullptr;
  iflyauto::PlanningOutput last_planning_output_;

  uint64_t frame_num_ = 0;
};

}  // namespace planning
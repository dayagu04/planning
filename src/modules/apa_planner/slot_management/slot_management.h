#ifndef __SLOT_MANAGEMENT_H__
#define __SLOT_MANAGEMENT_H__

#include <array>
#include <cstddef>
#include <memory>
#include <unordered_map>
#include <vector>

#include "Eigen/Core"
#include "func_state_machine.pb.h"
#include "localization.pb.h"
#include "parking_fusion.pb.h"
#include "parking_slot_list.pb.h"
#include "slot_management_info.pb.h"

namespace planning {
class SlotManagement {
 public:
  struct Param {
    double lon_dist_rearview_mirror_to_rear_axle = 1.844;
    double lat_dist_rearview_mirror_to_center = 1.135;

    bool force_apa_on = false;
    bool force_clear = false;
    size_t max_slot_size = 100;
    double max_slot_release_dist = 25.0;
    double max_slots_update_angle_dis_limit_deg = 20.0;
    double max_slot_boundary_line_angle_dif_deg = 10.0;
    double max_slot_update_lon_dif_slot_center_to_mirror = 1.6;
    double min_slot_update_lon_dif_slot_center_to_mirror = 0.35;
  };

  struct Measurement {
    double v_ego = 0.0;
    double heading = 0.0;
    Eigen::Vector2d ego_pos = Eigen::Vector2d::Zero();
    Eigen::Vector2d mirror_pos = Eigen::Vector2d::Zero();
  };

  void Reset();
  void Preprocess();
  bool Update(FuncStateMachine::FuncStateMachine* func_statemachine,
              ParkingFusion::ParkingFusionInfo* parking_slot_info,
              LocalizationOutput::LocalizationEstimate* localization_info);

  void SetParam(const Param& param) { param_ = param; }
  void SetMeasurement(Measurement& measurement) { measurement_ = measurement; }

  const common::SlotManagementInfo GetOutput() const {
    return slot_management_info_;
  }

  const common::SlotManagementInfo* GetOutputPtr() const {
    return &slot_management_info_;
  }

 private:
  bool UpdateSlots();
  bool IsValidParkingSlot(const common::SlotInfo& slot_info);
  bool IsInParkingState() const;
  bool ReleaseSlots();

  bool ReleaseSlots(FuncStateMachine::FuncStateMachine& func_statemachine,
                    ParkingFusion::ParkingFusionInfo& parking_slot_info);

  bool IfUpdateSlot(const common::SlotInfo& new_slot_info);
  bool AngleUpdateCondition(const common::SlotInfo& new_slot_info);
  bool LonDifUpdateCondition(const common::SlotInfo& new_slot_info);

  std::unordered_map<int, size_t> slot_info_map_;
  common::SlotManagementInfo slot_management_info_;

  FuncStateMachine::FuncStateMachine* func_state_ptr_;
  ParkingFusion::ParkingFusionInfo* parking_slot_ptr_;
  LocalizationOutput::LocalizationEstimate* localization_ptr_;

  Measurement measurement_;
  Param param_;
};

}  // namespace planning

#endif
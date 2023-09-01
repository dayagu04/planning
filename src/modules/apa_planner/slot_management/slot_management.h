#ifndef __SLOT_MANAGEMENT_H__
#define __SLOT_MANAGEMENT_H__

#include "define/geometry.h"
#include "utils_math.h"

#include "func_state_machine.pb.h"
#include "localization.pb.h"
#include "parking_fusion.pb.h"
#include "parking_slot_list.pb.h"
#include "slot_management_info.pb.h"

#include <array>
#include <cstddef>
#include <memory>
#include <unordered_map>
#include <vector>
#include "Eigen/Core"

namespace planning {
class SlotManagement {
  struct Param {
    size_t max_slot_size = 100;
    double max_slot_release_dis = 6.8;
    double max_nearby_pt_fluctuating_dis = 0.1;
    double max_remote_pt_fluctuating_dis = 0.3;
    double max_slot_heading_tol = 2 / 180.0 * M_PI;
    double lon_dis_mirror_to_rear_axis_center = 2.4;
  };

 public:
  void Reset();
  bool Update(
      std::shared_ptr<FuncStateMachine::FuncStateMachine> func_statemachine,
      std::shared_ptr<ParkingFusion::ParkingFusionInfo> parking_slot_info,
      std::shared_ptr<LocalizationOutput::LocalizationEstimate>
          localization_info);
  void SetParam(const Param& param) { param_ = param; }
  const common::SlotManagementInfo GetOutput() const {
    return slot_management_info_;
  }
  const common::SlotManagementInfo* GetOutputPtr() const {
    return &slot_management_info_;
  }

 private:
  bool UpdateSlots();
  bool IsInParkingState() const;
  bool GetLocalizationInfo(
      std::shared_ptr<LocalizationOutput::LocalizationEstimate>
          localization_info);
  bool ReleaseSlots();

  bool ReleaseSlots(FuncStateMachine::FuncStateMachine& func_statemachine,
                    ParkingFusion::ParkingFusionInfo& parking_slot_info);

  bool IfUpdateSlot(const common::SlotInfo& new_slot_info) const;
  bool IfNewCornerPointsFluctuatingLittle(
      const common::SlotInfo& new_slot_info) const;
  double CalSlotHeading(const common::SlotInfo& slot_info) const;
  bool parking_enable_flag_ = false;

  std::unordered_map<int, size_t> slot_info_map_;
  common::SlotManagementInfo slot_management_info_;

  std::shared_ptr<FuncStateMachine::FuncStateMachine> func_state_ptr_;
  std::shared_ptr<ParkingFusion::ParkingFusionInfo> parking_slot_ptr_;
  Pose2D local_pos_;
  Pose2D mirror_center_pos_;
  Param param_;
};

}  // namespace planning

#endif
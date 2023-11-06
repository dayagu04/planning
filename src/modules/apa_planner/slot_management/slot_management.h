#ifndef __SLOT_MANAGEMENT_H__
#define __SLOT_MANAGEMENT_H__

#include <array>
#include <cstddef>
#include <memory>
#include <unordered_map>
#include <vector>

#include "Eigen/Core"
#include "basic_types.pb.h"
#include "func_state_machine.pb.h"
#include "localization.pb.h"
#include "parking_fusion.pb.h"
#include "parking_slot_list.pb.h"
#include "slot_management_info.pb.h"
#include "task_basic_types.pb.h"

static const size_t slot_corner_pt_nums = 4;
static const size_t max_slot_window_size = 15;
static const size_t max_limiter_window_size = 15;
namespace planning {
class SlotInfoWindow {
 public:
  SlotInfoWindow() { slot_info_vec_.reserve(max_slot_window_size); }
  void Add(common::SlotInfo& fused_slot_info) {
    if (slot_info_vec_.size() < max_slot_window_size) {
      slot_info_vec_.emplace_back(fused_slot_info);
    } else {
      slot_info_vec_[front_index_] = fused_slot_info;
      front_index_++;
      if (front_index_ >= max_slot_window_size) {
        front_index_ = 0;
      }
    }
    Fuse();
  }

  void Reset() {
    front_index_ = 0;
    slot_info_vec_.clear();
    fused_slot_info_.Clear();
  }

  void Fuse() {
    if (slot_info_vec_.size() == 0) {
      return;
    }

    std::vector<double> sum_corner_pts_x_vec;
    std::vector<double> sum_corner_pts_y_vec;
    sum_corner_pts_x_vec.resize(4);
    sum_corner_pts_y_vec.resize(4);

    // TODO 取平均车位
    for (const auto& slot : slot_info_vec_) {
      for (int j = 0; j < slot.corner_points().corner_point_size(); ++j) {
        sum_corner_pts_x_vec[j] += slot.corner_points().corner_point(j).x();
        sum_corner_pts_y_vec[j] += slot.corner_points().corner_point(j).y();
      }
    }

    std::vector<double> fused_corner_pts_x_vec;
    std::vector<double> fused_corner_pts_y_vec;
    fused_corner_pts_x_vec.resize(4);
    fused_corner_pts_y_vec.resize(4);

    const double multiplier = 1.0 / static_cast<double>(slot_info_vec_.size());
    for (size_t j = 0; j < sum_corner_pts_x_vec.size(); ++j) {
      fused_corner_pts_x_vec[j] = sum_corner_pts_x_vec[j] * multiplier;
      fused_corner_pts_y_vec[j] = sum_corner_pts_y_vec[j] * multiplier;
    }

    common::SlotInfo fused_slot;
    fused_slot.set_id(slot_info_vec_.back().id());
    fused_slot.set_is_release(slot_info_vec_.back().is_release());

    double center_x = 0.0;
    double center_y = 0.0;
    for (size_t j = 0; j < fused_corner_pts_x_vec.size(); ++j) {
      auto corner_point =
          fused_slot.mutable_corner_points()->add_corner_point();
      corner_point->set_x(fused_corner_pts_x_vec[j]);
      corner_point->set_y(fused_corner_pts_y_vec[j]);

      center_x += fused_corner_pts_x_vec[j];
      center_y += fused_corner_pts_y_vec[j];
    }
    center_x = center_x * 0.25;
    center_y = center_y * 0.25;

    fused_slot.mutable_center()->set_x(center_x);
    fused_slot.mutable_center()->set_y(center_y);

    fused_slot_info_ = fused_slot;
  }

  common::SlotInfo GetFusedInfo() { return fused_slot_info_; }

 private:
  // std::vector<double> sum_corner_pts_x_vec_;
  // std::vector<double> sum_corner_pts_y_vec_;

  size_t front_index_ = 0;
  common::SlotInfo fused_slot_info_;
  std::vector<common::SlotInfo> slot_info_vec_;
};

class SlotManagement {
 public:
  struct Param {
    double lon_dist_rearview_mirror_to_rear_axle = 2.092;
    double lat_dist_rearview_mirror_to_center = 1.092;

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

  bool Update(
      const FuncStateMachine::FuncStateMachine* func_statemachine,
      const ParkingFusion::ParkingFusionInfo* parking_slot_info,
      const LocalizationOutput::LocalizationEstimate* localization_info);

  const bool SetRealtime();

  void SetParam(const Param& param) { param_ = param; }

  void Reset();

  const bool GetSelectedSlot(common::SlotInfo& slot_info,
                             const int selected_id) const;

  const size_t GetFusedSlotSize() { return slot_info_window_vec_.size(); }

  const common::SlotManagementInfo GetOutput() const {
    return slot_management_info_;
  }

  const common::SlotManagementInfo* GetOutputPtr() const {
    return &slot_management_info_;
  }

  const double GetOccupiedRatio() const { return slot_occupied_ratio_; }

 private:
  void Preprocess();
  bool UpdateSlotsInSearching();
  bool UpdateSlotsInParking();
  bool IsValidParkingSlot(const common::SlotInfo& slot_info) const;
  bool CorrectSlotPointsOrder(common::SlotInfo& slot_info) const;
  bool IsInAPAState() const;
  bool IsInSearchingState() const;
  bool IsInParkingState() const;
  bool ReleaseSlots();

  double CalOccupiedRatio() const;

  common::SlotInfo SlotInfoTransfer(
      const ParkingFusion::ParkingFusionSlot& parking_fusion_slot);

  bool ReleaseSlots(FuncStateMachine::FuncStateMachine& func_statemachine,
                    ParkingFusion::ParkingFusionInfo& parking_slot_info);

  bool IfUpdateSlot(const common::SlotInfo& new_slot_info);
  bool AngleUpdateCondition(const common::SlotInfo& new_slot_info);
  bool LonDifUpdateCondition(const common::SlotInfo& new_slot_info);

  std::unordered_map<int, size_t> slot_info_map_;
  std::vector<SlotInfoWindow> slot_info_window_vec_;
  common::SlotManagementInfo slot_management_info_;

  const FuncStateMachine::FuncStateMachine* func_state_ptr_;
  const ParkingFusion::ParkingFusionInfo* parking_slot_ptr_;
  const LocalizationOutput::LocalizationEstimate* localization_ptr_;

  Measurement measurement_;
  Param param_;
  double slot_occupied_ratio_ = 0.0;
  size_t fusion_order_error_cnt_ = 0;
  bool is_occupied_ = false;
  bool is_fixed_ = false;
};

}  // namespace planning

#endif
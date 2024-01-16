#ifndef __SLOT_MANAGEMENT_H__
#define __SLOT_MANAGEMENT_H__

#include <google/protobuf/stubs/port.h>

#include <array>
#include <cstddef>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "apa_param_setting.h"
#include "basic_types.pb.h"
#include "func_state_machine.pb.h"
#include "local_view.h"
#include "localization.pb.h"
#include "parking_fusion.pb.h"
#include "parking_slot_list.pb.h"
#include "planning_plan.pb.h"
#include "slot_management_info.pb.h"
#include "task_basic_types.pb.h"
#include "uss_wave_info.pb.h"

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

    // TODO Take the average slot
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

  const common::SlotInfo GetFusedInfo() const { return fused_slot_info_; }

 private:
  size_t front_index_ = 0;
  common::SlotInfo fused_slot_info_;
  std::vector<common::SlotInfo> slot_info_vec_;
};

class LimiterPointWindow {
 public:
  LimiterPointWindow() { limiter_points_vec_.reserve(max_limiter_window_size); }

  ~LimiterPointWindow() = default;

  const bool IsEmpty() const { return (limiter_points_vec_.size() == 0); }

  void Reset() {
    front_index_ = 0;
    limiter_points_vec_.clear();
    fused_limiter_points_.first.Clear();
    fused_limiter_points_.second.Clear();
  }

  void Add(const std::pair<Eigen::Vector2d, Eigen::Vector2d> fusion_slot) {
    std::pair<Eigen::Vector2d, Eigen::Vector2d> fusion_slot_tmp = fusion_slot;
    if (limiter_points_vec_.size() > 0) {
      Eigen::Vector2d p0 = fusion_slot.first;
      Eigen::Vector2d p1 = fusion_slot.second;

      const auto distance1 =
          (p0 - limiter_points_vec_[front_index_].first).norm();

      const auto distance2 =
          (p1 - limiter_points_vec_[front_index_].first).norm();

      if (distance1 >= distance2) {
        fusion_slot_tmp.first = p1;
        fusion_slot_tmp.second = p0;
      }
    }
    if (limiter_points_vec_.size() < max_limiter_window_size) {
      limiter_points_vec_.emplace_back(fusion_slot_tmp);
    } else {
      limiter_points_vec_[front_index_] = fusion_slot_tmp;
      front_index_++;

      if (front_index_ >= max_limiter_window_size) {
        front_index_ = 0;
      }
    }
  }

  void Fuse() {
    if (limiter_points_vec_.size() == 0) {
      return;
    }

    Eigen::Vector2d p0_mean = Eigen::Vector2d::Zero();
    Eigen::Vector2d p1_mean = Eigen::Vector2d::Zero();

    for (const auto& limiter_points : limiter_points_vec_) {
      p0_mean += limiter_points.first;
      p1_mean += limiter_points.second;
    }

    const double alpha = 1.0 / static_cast<double>(limiter_points_vec_.size());
    p0_mean *= alpha;
    p1_mean *= alpha;

    fused_limiter_points_.first.set_x(p0_mean.x());
    fused_limiter_points_.first.set_y(p0_mean.y());
    fused_limiter_points_.second.set_x(p1_mean.x());
    fused_limiter_points_.second.set_y(p1_mean.y());
  }

  const std::pair<common::Point2d, common::Point2d> GetFusedLimiterPoints()
      const {
    return fused_limiter_points_;
  }

 private:
  size_t front_index_ = 0;
  std::pair<common::Point2d, common::Point2d> fused_limiter_points_;
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> limiter_points_vec_;
};

class SlotManagement {
 public:
  struct Param {
    double lon_dist_rearview_mirror_to_rear_axle =
        apa_param.GetParam().lon_dist_mirror_to_rear_axle;
    double lat_dist_rearview_mirror_to_center =
        apa_param.GetParam().lat_dist_mirror_to_center;

    bool force_apa_on = false;
    bool force_clear = false;
    size_t max_slot_size = 100;
    double max_slot_release_dist = 25.0;
    double max_slots_update_angle_dis_limit_deg =
        apa_param.GetParam().max_slots_update_angle_dis_limit_deg;
    double max_slot_boundary_line_angle_dif_deg =
        apa_param.GetParam().max_slot_boundary_line_angle_dif_deg;
    double max_slot_update_lon_dif_slot_center_to_mirror =
        apa_param.GetParam().max_slot_update_lon_dif_slot_center_to_mirror;
    double min_slot_update_lon_dif_slot_center_to_mirror =
        apa_param.GetParam().min_slot_update_lon_dif_slot_center_to_mirror;
    double invade_area_ratio = 0.36;
  };

  struct OccupiedInfo {
    OccupiedInfo() {}
    OccupiedInfo(size_t count_r, bool is_occupied_r) {
      count = count_r;
      is_occupied = is_occupied_r;
    }
    size_t count = 0;
    bool is_occupied = false;
  };

  struct Measurement {
    double v_ego = 0.0;
    double heading = 0.0;
    Eigen::Vector2d ego_pos = Eigen::Vector2d::Zero();
    Eigen::Vector2d mirror_pos = Eigen::Vector2d::Zero();
  };

  struct Frame {
    const FuncStateMachine::FuncStateMachine* func_state_ptr_;
    const ParkingFusion::ParkingFusionInfo* parking_slot_ptr_;
    const LocalizationOutput::LocalizationEstimate* localization_ptr_;
    // slot state check by uss
    const UssWaveInfo::UssWaveInfo* uss_wave_info_ptr_;
    std::vector<double> uss_raw_dist_vec_;
    std::unordered_map<int, OccupiedInfo> occupied_info_map_;
    std::vector<PlanningOutput::SuccessfulSlotsInfo> released_slot_info_vec_;

    std::unordered_map<int, size_t> slot_info_map_;
    std::vector<SlotInfoWindow> slot_info_window_vec_;
    common::SlotManagementInfo slot_management_info_;

    LimiterPointWindow limiter_point_window_;

    Measurement measurement_;
    Param param_;
    double slot_occupied_ratio_ = 0.0;
    size_t fusion_order_error_cnt_ = 0;
    bool is_occupied_ = false;
    bool is_fixed_ = false;

    // auto switcher to parking
    bool using_auto_switcher = false;
    bool is_switch_parking = false;
    google::protobuf::uint32 set_seleted_id_mannually = 0;
  };

  bool Update(const LocalView* local_view_ptr);

  bool Update(const FuncStateMachine::FuncStateMachine* func_statemachine,
              const ParkingFusion::ParkingFusionInfo* parking_slot_info,
              const LocalizationOutput::LocalizationEstimate* localization_info,
              const UssWaveInfo::UssWaveInfo* uss_wave_info);

  const bool SetRealtime();

  void SetParam(const Param& param) { frame_.param_ = param; }

  void Reset();
  void SetSlotFix(const bool is_fixed) { frame_.is_fixed_ = is_fixed; }

  const bool GetSelectedSlot(common::SlotInfo& slot_info,
                             const int selected_id) const;

  const bool GetSelectedSlot(common::SlotInfo& slot_info) const;

  const size_t GetFusedSlotSize() {
    return frame_.slot_info_window_vec_.size();
  }

  const common::SlotManagementInfo& GetOutput() const {
    return frame_.slot_management_info_;
  }

  const common::SlotManagementInfo* GetOutputPtr() const {
    return &frame_.slot_management_info_;
  }

  const double GetOccupiedRatio() const { return frame_.slot_occupied_ratio_; }

  const bool GetSelectedLimiter(
      std::pair<Eigen::Vector2d, Eigen::Vector2d>& fused_limiter) const;

  const std::vector<PlanningOutput::SuccessfulSlotsInfo>&
  GetReleasedSlotInfoVec() const {
    return frame_.released_slot_info_vec_;
  }

 private:
  Frame frame_;
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

  void UpdateReleasedSlotInfo();

  bool IfUpdateSlot(const common::SlotInfo& new_slot_info);
  bool AngleUpdateCondition(const common::SlotInfo& new_slot_info);
  bool LonDifUpdateCondition(const common::SlotInfo& new_slot_info);

  void UpdateOccupiedInfo();
  void UpdateLimiterInfoInParking(const common::SlotInfo& slot_info,
                                  const size_t selected_fusin_slot_index);

  void UpdateSlotInfoInParking(common::SlotInfo& select_slot,
                               const size_t selected_fusin_slot_index,
                               const bool is_occupied);

  const bool ProcessRawSlot(
      const ParkingFusion::ParkingFusionSlot& parking_fusion_slot,
      common::SlotInfo& new_slot_info);
  void ModifySlot2Rectangle(common::SlotInfo& slot_info);

  void UpdateLimiterForAutoSetSlot();
};

}  // namespace planning

#endif
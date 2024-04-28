#ifndef __SLOT_MANAGEMENT_H__
#define __SLOT_MANAGEMENT_H__

#include <google/protobuf/stubs/port.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "apa_param_setting.h"
#include "basic_types.pb.h"
#include "camera_preception_parking_slot_c.h"
#include "func_state_machine_c.h"
#include "fusion_parking_slot_c.h"
#include "geometry_math.h"
#include "local_view.h"
#include "localization_c.h"
#include "perpendicular_path_planner.h"
#include "planning_plan_c.h"
#include "slot_management_info.pb.h"
#include "task_basic_types.pb.h"
#include "uss_perception_info_c.h"
#include "uss_wave_info_c.h"

static const size_t slot_corner_pt_nums = 4;
static const size_t max_slot_window_size = 15;
static const size_t max_limiter_window_size = 15;
namespace planning {
class SlotInfoWindow {
 public:
  SlotInfoWindow() {
    slot_info_vec_.reserve(max_slot_window_size);
    fused_slot_info_.set_slot_type(
        Common::ParkingSlotType::PARKING_SLOT_TYPE_INVALID);
    fused_slot_info_.set_id(0);
    fused_slot_info_.set_is_release(false);
    fused_slot_info_.set_is_occupied(true);
  }
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
  }

  void Reset() {
    front_index_ = 0;
    slot_info_vec_.clear();
    fused_slot_info_.Clear();
    fused_slot_info_.set_slot_type(
        Common::ParkingSlotType::PARKING_SLOT_TYPE_INVALID);
    fused_slot_info_.set_id(0);
    fused_slot_info_.set_is_release(false);
    fused_slot_info_.set_is_occupied(true);
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
    fused_slot.set_slot_type(slot_info_vec_.back().slot_type());

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

    size_t id = fused_slot_info_.id();
    auto slot_type = fused_slot_info_.slot_type();
    auto slot_release = fused_slot_info_.is_release();
    auto slot_occupied = fused_slot_info_.is_occupied();
    fused_slot_info_ = fused_slot;
    if (id != 0) {
      fused_slot_info_.set_id(id);
    }
    if (slot_type != Common::ParkingSlotType::PARKING_SLOT_TYPE_INVALID) {
      fused_slot_info_.set_slot_type(slot_type);
    }
    if (slot_release != false) {
      fused_slot_info_.set_is_release(slot_release);
      fused_slot_info_.set_is_occupied(slot_occupied);
    }
  }

  const common::SlotInfo GetFusedInfo() {
    Fuse();
    return fused_slot_info_;
  }

  const bool IsEmpty() const { return slot_info_vec_.empty(); }

  const bool SetOccupied(bool force_set_occupied) {
    force_set_occupied_ = force_set_occupied;
    return true;
  }

  const bool GetOccupied() const { return force_set_occupied_; }

 private:
  size_t front_index_ = 0;
  common::SlotInfo fused_slot_info_;
  std::vector<common::SlotInfo> slot_info_vec_;
  bool force_set_occupied_ = true;
};

class LimiterPointWindow {
 public:
  LimiterPointWindow() { limiter_points_vec_.reserve(max_limiter_window_size); }

  ~LimiterPointWindow() = default;

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

  void Reset() {
    front_index_ = 0;
    limiter_points_vec_.clear();
    fused_limiter_points_.first.Clear();
    fused_limiter_points_.second.Clear();
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

  const std::pair<common::Point2d, common::Point2d> GetFusedLimiterPoints() {
    Fuse();
    return fused_limiter_points_;
  }

  const bool IsEmpty() const { return limiter_points_vec_.empty(); }

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

    size_t max_slot_size = 100;
    double max_slot_release_dist = 25.0;
    double max_slots_update_angle_dis_limit_deg =
        apa_param.GetParam().max_slots_update_angle_dis_limit_deg;
    double max_slot_boundary_line_angle_dif_deg =
        apa_param.GetParam().max_slot_boundary_line_angle_dif_deg;
    double outside_lon_dist_max_slot2mirror =
        apa_param.GetParam().outside_lon_dist_max_slot2mirror;
    double outside_lon_dist_min_slot2mirror =
        apa_param.GetParam().outside_lon_dist_min_slot2mirror;
    double invade_area_ratio = 0.36;

    // auto switcher to parking
    bool force_apa_on = false;
    bool force_clear = false;
    bool is_switch_parking = false;
    google::protobuf::uint32 set_seleted_id_mannually = 0;

    void Reset() {
      force_apa_on = false;
      force_clear = false;
      is_switch_parking = false;
      set_seleted_id_mannually = 0;
    }
  };

  struct Measurement {
    double v_ego = 0.0;
    double heading = 0.0;
    Eigen::Vector2d ego_heading_vec = Eigen::Vector2d::Zero();
    Eigen::Vector2d ego_pos = Eigen::Vector2d::Zero();
    Eigen::Vector2d right_mirror_pos = Eigen::Vector2d::Zero();
    Eigen::Vector2d left_mirror_pos = Eigen::Vector2d::Zero();
  };

  struct EgoSlotInfo {
    uint8_t select_slot_id = 0;
    uint8_t slot_type = Common::PARKING_SLOT_TYPE_INVALID;
    uint8_t slot_side = pnc::geometry_lib::SLOT_SIDE_INVALID;
    iflyauto::ParkingFusionSlot select_fusion_slot;
    common::SlotInfo select_slot;
    common::SlotInfo select_slot_filter;

    Eigen::Vector2d slot_origin_pos = Eigen::Vector2d::Zero();
    double slot_origin_heading = 0.0;
    Eigen::Vector2d slot_origin_heading_vec = Eigen::Vector2d::Zero();

    std::vector<Eigen::Vector2d> slot_corner;

    double slot_length = apa_param.GetParam().normal_slot_length;
    double slot_width = apa_param.GetParam().normal_slot_width;

    pnc::geometry_lib::GlobalToLocalTf g2l_tf;
    pnc::geometry_lib::LocalToGlobalTf l2g_tf;

    Eigen::Vector2d ego_pos_slot = Eigen::Vector2d::Zero();
    Eigen::Vector2d ego_heading_slot_vec = Eigen::Vector2d::Zero();
    double ego_heading_slot = 0.0;

    Eigen::Vector2d target_ego_pos_slot = Eigen::Vector2d::Zero();
    Eigen::Vector2d target_ego_heading_slot_vec = Eigen::Vector2d::Zero();
    double target_ego_heading_slot = 0.0;

    pnc::geometry_lib::PathPoint terminal_err;

    std::pair<Eigen::Vector2d, Eigen::Vector2d> limiter = std::make_pair(
        Eigen::Vector2d(apa_param.GetParam().terminal_target_x,
                        apa_param.GetParam().normal_slot_width / 2.0),
        Eigen::Vector2d(apa_param.GetParam().terminal_target_x,
                        -apa_param.GetParam().normal_slot_width / 2.0));

    double slot_occupied_ratio = 0.0;

    std::vector<Eigen::Vector2d> obs_pt_vec_slot;

    void Reset() {
      select_slot_id = 0;
      slot_type = Common::PARKING_SLOT_TYPE_INVALID;
      memset(&select_fusion_slot, 0, sizeof(select_fusion_slot));
      select_slot.Clear();
      select_slot_filter.Clear();
      slot_origin_pos.setZero();
      slot_origin_heading = 0.0;
      slot_origin_heading_vec.setZero();
      slot_length = apa_param.GetParam().normal_slot_length;
      slot_width = apa_param.GetParam().normal_slot_width;
      ego_pos_slot.setZero();
      ego_heading_slot_vec.setZero();
      ego_heading_slot = 0.0;

      target_ego_pos_slot.setZero();
      target_ego_heading_slot_vec.setZero();
      target_ego_heading_slot = 0.0;
      terminal_err.Reset();
      limiter = std::make_pair(
          Eigen::Vector2d(apa_param.GetParam().terminal_target_x,
                          apa_param.GetParam().normal_slot_width / 2.0),
          Eigen::Vector2d(apa_param.GetParam().terminal_target_x,
                          -apa_param.GetParam().normal_slot_width / 2.0));
      slot_occupied_ratio = 0.0;

      obs_pt_vec_slot.clear();
    }
  };

  struct Frame {
    const iflyauto::FuncStateMachine* func_state_ptr;
    const iflyauto::ParkingFusionInfo* parking_slot_ptr;
    const iflyauto::LocalizationEstimate* localization_ptr;
    // slot state check by uss
    const iflyauto::UssWaveInfo* uss_wave_info_ptr;
    const iflyauto::UssPerceptInfo* uss_percept_info_ptr;

    std::vector<double> uss_raw_dist_vec;
    std::vector<iflyauto::SuccessfulSlotsInfo> released_slot_info_vec;

    std::unordered_map<int, size_t> slot_info_map;
    std::vector<SlotInfoWindow> slot_info_window_vec;
    common::SlotManagementInfo slot_management_info;

    LimiterPointWindow limiter_point_window;

    Measurement measurement;
    Param param;
    size_t fusion_order_error_cnt = 0;

    EgoSlotInfo ego_slot_info;

    size_t no_update_slot_count = 0;
    bool parallel_slot_reseted_once = false;
    bool is_side_calc_in_parking = false;

    std::vector<Eigen::Vector2d> obstacle_point_vec;

    std::unordered_map<size_t, std::vector<Eigen::Vector2d>> obs_pt_map;

    void Reset() {
      uss_raw_dist_vec.clear();
      released_slot_info_vec.clear();
      slot_info_map.clear();
      slot_info_map.reserve(max_slot_window_size);
      slot_info_window_vec.clear();
      slot_management_info.Clear();
      limiter_point_window.Reset();
      param.Reset();
      fusion_order_error_cnt = 0;
      no_update_slot_count = 0;
      parallel_slot_reseted_once = false;
      ego_slot_info.Reset();
      obstacle_point_vec.clear();
      obs_pt_map.clear();
    }
  };

  bool Update(const LocalView* local_view_ptr);

  bool Update(const iflyauto::FuncStateMachine* func_statemachine,
              const iflyauto::ParkingFusionInfo* parking_slot_info,
              const iflyauto::LocalizationEstimate* localization_info,
              const iflyauto::UssWaveInfo* uss_wave_info,
              const iflyauto::UssPerceptInfo* uss_percept_info);

  void AddUssPerceptObstacles();

  const bool AddUssPerceptObstacles(const common::SlotInfo& slot_info);

  const bool SetRealtime();

  void SetParam(const Param& param) { frame_.param = param; }

  void Reset();

  const bool GetSelectedSlot(common::SlotInfo& slot_info,
                             const int selected_id) const;

  const bool GetSelectedSlot(common::SlotInfo& slot_info) const;

  const size_t GetFusedSlotSize() { return frame_.slot_info_window_vec.size(); }

  const common::SlotManagementInfo& GetOutput() const {
    return frame_.slot_management_info;
  }

  const common::SlotManagementInfo* GetOutputPtr() const {
    return &frame_.slot_management_info;
  }

  const EgoSlotInfo& GetEgoSlotInfo() const { return frame_.ego_slot_info; }

  const bool GetSelectedLimiter(
      std::pair<Eigen::Vector2d, Eigen::Vector2d>& fused_limiter) const;

  const std::vector<iflyauto::SuccessfulSlotsInfo>& GetReleasedSlotInfoVec()
      const {
    return frame_.released_slot_info_vec;
  }

 private:
  Frame frame_;

  bool IsInAPAState() const;
  void Preprocess();

  bool IsInSearchingState() const;
  bool UpdateSlotsInSearching();

  bool IsInParkingState() const;
  bool UpdateSlotsInParking();
  bool UpdateEgoSlotInfo(const google::protobuf::uint32& select_slot_id,
                         const common::SlotInfo& select_slot,
                         const iflyauto::ParkingFusionSlot& select_fusion_slot);
  const bool UpdateEgoParallelSlotInfo(
      const google::protobuf::uint32& select_slot_id,
      const common::SlotInfo& select_slot,
      const iflyauto::ParkingFusionSlot& select_fusion_slot);
  bool UpdateEgoSlotInfo(EgoSlotInfo& ego_slot_info,
                         const common::SlotInfo* slot_info);
  bool GenTLane(const EgoSlotInfo& ego_slot_info,
                apa_planner::PerpendicularPathPlanner::Tlane& t_lane);
  void UpdateSlotInfoInParking();
  void UpdateParallelSlotInfoInParking();
  void UpdateLimiterInfoInParking();

  const bool ProcessRawSlot(
      const iflyauto::ParkingFusionSlot& parking_fusion_slot,
      common::SlotInfo& new_slot_info);
  common::SlotInfo SlotInfoTransfer(
      const iflyauto::ParkingFusionSlot& fusion_slot);
  const bool SlotInfoTransfer(const iflyauto::ParkingFusionSlot& fusion_slot,
                              common::SlotInfo& slot_info);
  bool IsValidParkingSlot(const common::SlotInfo& slot_info) const;
  bool CorrectSlotPointsOrder(common::SlotInfo& slot_info) const;
  bool IfUpdateSlot(const common::SlotInfo& new_slot_info,
                    const size_t parking_fusion_slot_source_type);
  bool AngleUpdateCondition(const common::SlotInfo& new_slot_info);
  bool LonDifUpdateCondition(const common::SlotInfo& new_slot_info,
                             const size_t parking_fusion_slot_source_type);
  void ModifySlot2Rectangle(common::SlotInfo& slot_info);

  const double CalLonDistSlot2Car(const common::SlotInfo& new_slot_info) const;

  const double CalAngleSlot2Car(const common::SlotInfo& new_slot_info) const;

  void UpdateReleasedSlotInfo();
};

}  // namespace planning

#endif
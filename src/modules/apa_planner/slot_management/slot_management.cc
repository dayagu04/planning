#include "slot_management.h"

#include <math.h>

#include <cmath>
#include <cstddef>
#include <utility>

#include "Eigen/src/Core/Matrix.h"
#include "basic_types.pb.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "func_state_machine.pb.h"
#include "math_lib.h"
#include "slot_management_info.pb.h"
#include "transform_lib.h"

#define __DEBUG_PRINT__
namespace planning {

static const double kPie = 3.141592653589793;
static const double kMinSlotUpdateOccupiedRatio = 0.6;

void SlotManagement::Reset() {
  // reset slot in
  slot_management_info_.Clear();
  slot_info_window_vec_.clear();

  // reset slot_info_map
  slot_info_map_.clear();
  slot_info_map_.reserve(param_.max_slot_size);
}

void SlotManagement::Preprocess() {
  measurement_.ego_pos << localization_ptr_->pose().local_position().x(),
      localization_ptr_->pose().local_position().y();
  measurement_.heading = localization_ptr_->pose().heading();

  measurement_.mirror_pos << measurement_.ego_pos.x() +
                                 param_.lon_dist_rearview_mirror_to_rear_axle *
                                     std::cos(measurement_.heading),
      measurement_.ego_pos.y() + param_.lon_dist_rearview_mirror_to_rear_axle *
                                     std::sin(measurement_.heading);
}

bool SlotManagement::Update(
    const FuncStateMachine::FuncStateMachine *func_statemachine,
    const ParkingFusion::ParkingFusionInfo *parking_slot_info,
    const LocalizationOutput::LocalizationEstimate *localization_info) {
  // set ptrs
  func_state_ptr_ = func_statemachine;
  parking_slot_ptr_ = parking_slot_info;
  localization_ptr_ = localization_info;

  if (!IsInAPAState() || param_.force_clear) {
    Reset();
    return false;
  }

  // preprocess
  Preprocess();

  bool update_searching_flag = false;
  bool update_occupied_flag = false;
  // update slots
  if (IsInSearchingState()) {
    update_searching_flag = UpdateSlotsInSearching();
  } else if (IsInParkingState()) {
    update_occupied_flag = UpdateSlotsInParking();
  }

  std::cout << "update_searching_flag" << update_searching_flag << std::endl;
  std::cout << "update_occupied_flag" << update_occupied_flag << std::endl;

  ReleaseSlots();

  // restore slot management info
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_slot_management_info()
      ->CopyFrom(slot_management_info_);

  if (update_searching_flag || update_occupied_flag) {
    return true;
  } else {
    return false;
  }
}

common::SlotInfo SlotManagement::SlotInfoTransfer(
    const ParkingFusion::ParkingFusionSlot &fusion_slot) {
  double accumulated_x = 0.0;
  double accumulated_y = 0.0;

  common::SlotInfo slot_info;
  static const auto fusion_slots_size = 4;
  for (auto j = 0; j < fusion_slots_size; j++) {
    auto add_point = slot_info.mutable_corner_points()->add_corner_point();
    add_point->set_x(fusion_slot.corner_points(j).x());
    add_point->set_y(fusion_slot.corner_points(j).y());

    accumulated_x += fusion_slot.corner_points(j).x();
    accumulated_y += fusion_slot.corner_points(j).y();
  }

  slot_info.mutable_center()->set_x(accumulated_x /
                                    static_cast<double>(fusion_slots_size));

  slot_info.mutable_center()->set_y(accumulated_y /
                                    static_cast<double>(fusion_slots_size));

  slot_info.set_id(fusion_slot.id());
  slot_info.set_is_release(false);

  return slot_info;
}

bool SlotManagement::UpdateSlotsInSearching() {
  // Update slots
  for (auto i = 0; i < parking_slot_ptr_->parking_fusion_slot_lists_size();
       ++i) {
    const auto &fusion_slot = parking_slot_ptr_->parking_fusion_slot_lists(i);
    auto slot_info = SlotInfoTransfer(fusion_slot);

    // if parking slot is not valid, continue
    const bool is_slot_valid = IsValidParkingSlot(slot_info);
    if (!is_slot_valid) {
      continue;
    }

    const auto slot_info_vec_size = slot_info_window_vec_.size();
    if (slot_info_map_.count(slot_info.id()) == 0 &&
        LonDifUpdateCondition(slot_info)) {  // get new id
      SlotInfoWindow slot_info_window;
      slot_info_window.Add(slot_info);
      slot_info_window_vec_.emplace_back(slot_info_window);

      slot_info_map_.insert(std::make_pair(slot_info.id(), slot_info_vec_size));
    } else {  // get old id
      // slot update strategy
      if (IfUpdateSlot(slot_info)) {
        auto slot_idx = slot_info_map_[slot_info.id()];
        slot_info_window_vec_[slot_idx].Add(slot_info);
      }
    }
  }

  // assemble slot_management_info_
  slot_management_info_.mutable_slot_info_vec()->Clear();
  for (size_t j = 0; j < slot_info_window_vec_.size(); ++j) {
    auto slot = slot_management_info_.add_slot_info_vec();
    *slot = slot_info_window_vec_[j].GetFusedInfo();
  }

  return true;
}

bool SlotManagement::UpdateSlotsInParking() {
  const auto select_slot_id = parking_slot_ptr_->select_slot_id();
  common::SlotInfo select_slot;
  for (const auto &fusion_slot :
       parking_slot_ptr_->parking_fusion_slot_lists()) {
    if (select_slot_id != fusion_slot.id()) {
      continue;
    }
    select_slot = SlotInfoTransfer(fusion_slot);
  }
  if (!select_slot.has_corner_points()) {
    std::cout << "find no select slot" << std::endl;
    return false;
  }

  // calculate occupied ratio
  CalOccupiedRatio();
  std::cout << "occupied_ratio in sm: " << slot_occupied_ratio_ << std::endl;

  // if occupied percentage is less than certain value,return
  if (slot_occupied_ratio_ < kMinSlotUpdateOccupiedRatio) {
    std::cout << "occupied_ratio is less than min update value " << std::endl;
    return false;
  }

  auto slot_idx = slot_info_map_[select_slot_id];

  // update selected slot in slot_management_info_

  auto slot = slot_management_info_.mutable_slot_info_vec(slot_idx);

  slot_info_window_vec_[slot_idx].DirectlyOutputFusionlot(select_slot);
  *slot = slot_info_window_vec_[slot_idx].GetFusedInfo();

  return true;
}

double SlotManagement::CalOccupiedRatio() const {
  Eigen::Vector2d target_pt = Eigen::Vector2d::Zero();
  Eigen::Vector2d slot01_middle_pt = Eigen::Vector2d::Zero();
  Eigen::Vector2d slot23_middle_pt = Eigen::Vector2d::Zero();
  Eigen::Vector2d slot_heading_unit_vec = Eigen::Vector2d::Zero();

  const int select_slot_id = parking_slot_ptr_->select_slot_id();

  for (const auto &fusion_slot :
       parking_slot_ptr_->parking_fusion_slot_lists()) {
    if (select_slot_id != fusion_slot.id()) {
      continue;
    }

    slot01_middle_pt << (fusion_slot.corner_points(0).x() +
                         fusion_slot.corner_points(1).x()) *
                            0.5,
        (fusion_slot.corner_points(0).y() + fusion_slot.corner_points(1).y()) *
            0.5;

    slot23_middle_pt << (fusion_slot.corner_points(2).x() +
                         fusion_slot.corner_points(3).x()) *
                            0.5,
        (fusion_slot.corner_points(2).y() + fusion_slot.corner_points(3).y()) *
            0.5;

    slot_heading_unit_vec = slot01_middle_pt - slot23_middle_pt;
    slot_heading_unit_vec.normalize();

    const double dst_rear_edge_to_rear_axle = 0.947;
    const double buffer = 0.1;
    target_pt = slot23_middle_pt +
                (dst_rear_edge_to_rear_axle + buffer) * slot_heading_unit_vec;
  }

  std::cout << "current pos: " << measurement_.ego_pos << std::endl;
  std::cout << "target pos: " << target_pt << std::endl;

  Eigen::Vector2d slot_01_middle_to_ego_pos =
      measurement_.ego_pos - slot01_middle_pt;

  Eigen::Vector2d slot_01_middle_to_targt_pos = target_pt - slot01_middle_pt;

  const double dot = slot_01_middle_to_ego_pos.dot(slot_01_middle_to_targt_pos);
  double slot_occupied_ratio = 0.0;
  if (dot <= 0) {
    slot_occupied_ratio = 0.0;
  } else {
    const double ratio =
        dot /
        (slot_01_middle_to_targt_pos.x() * slot_01_middle_to_targt_pos.x() +
         slot_01_middle_to_targt_pos.y() * slot_01_middle_to_targt_pos.y());
    slot_occupied_ratio = pnc::mathlib::Clamp(ratio, 0.0, 1.0);
  }
  return slot_occupied_ratio;
}

bool SlotManagement::IsValidParkingSlot(const common::SlotInfo &slot_info) {
  const auto &pts = slot_info.corner_points();

  Eigen::Vector2d slot_line02_vec(
      pts.corner_point(2).x() - pts.corner_point(0).x(),
      pts.corner_point(2).y() - pts.corner_point(0).y());

  Eigen::Vector2d slot_line13_vec(
      pts.corner_point(3).x() - pts.corner_point(1).x(),
      pts.corner_point(3).y() - pts.corner_point(1).y());

  Eigen::Vector2d slot_line23_vec(
      pts.corner_point(3).x() - pts.corner_point(2).x(),
      pts.corner_point(3).y() - pts.corner_point(2).y());

  Eigen::Vector2d slot_line01_vec(
      pts.corner_point(1).x() - pts.corner_point(0).x(),
      pts.corner_point(1).y() - pts.corner_point(0).y());

  // 1. Check if the boundary lines 02 and 13 of the parking
  // slot are approximately parallel
  const double slot_line_angle_dif = std::fabs(
      pnc::transform::GetAngleFromTwoVec(slot_line02_vec, slot_line13_vec));
  const double slot_line_angle_dif_deg = slot_line_angle_dif * 57.3;

  const bool slot_line_parallel_condition =
      slot_line_angle_dif_deg <= param_.max_slot_boundary_line_angle_dif_deg;

  if (!slot_line_parallel_condition) {
    return false;
  }

  // 2.  Check if the nearby boundary lines of the parking
  // slot are approximately vertical
  Eigen::Vector2d slot_line20_vec = -slot_line02_vec;
  const double corner2_angle_dif =
      std::fabs(kPie * 0.5 - pnc::transform::GetAngleFromTwoVec(
                                 slot_line20_vec, slot_line23_vec));
  const double corner3_angle_dif =
      std::fabs(kPie * 0.5 - pnc::transform::GetAngleFromTwoVec(
                                 slot_line13_vec, slot_line23_vec));
  const double max_corner_angle_dif =
      std::max(corner2_angle_dif, corner3_angle_dif);
  const bool corner_vertical_condition =
      max_corner_angle_dif <= param_.max_slot_boundary_line_angle_dif_deg;
  if (corner_vertical_condition) {
    return true;
  } else {
    return false;
  }
}

bool SlotManagement::IsInAPAState() const {
  if ((func_state_ptr_->current_state() >= FuncStateMachine::PARK_IN_APA_IN &&
       func_state_ptr_->current_state() <=
           FuncStateMachine::PARK_IN_COMPLETED) ||
      param_.force_apa_on) {
    return true;
  } else {
    return false;
  }
}
bool SlotManagement::IsInSearchingState() const {
#ifdef __DEBUG_PRINT__
  std::cout << "func_state_ptr_->current_state() = "
            << func_state_ptr_->current_state() << std::endl;
  // 应该检查一下每帧slot_management_info_车位的值
  std::cout << "managed slot size:"
            << slot_management_info_.slot_info_vec_size() << std::endl;
  std::cout << "magaged slot id: ";
  for (int i = 0; i < slot_management_info_.slot_info_vec_size(); ++i) {
    std::cout << slot_management_info_.slot_info_vec(i).id() << ", ";
  }
  std::cout << std::endl;
#endif

  if ((func_state_ptr_->current_state() >= FuncStateMachine::PARK_IN_APA_IN &&
       func_state_ptr_->current_state() <=
           FuncStateMachine::PARK_IN_NO_READY) ||
      param_.force_apa_on) {
    std::cout << "apa searching on!" << std::endl;
    return true;
  } else {
    std::cout << "apa searching off!" << std::endl;
    return false;
  }
}

bool SlotManagement::IsInParkingState() const {
  return (func_state_ptr_->current_state() ==
          FuncStateMachine::PARK_IN_ACTIVATE_CONTROL);
}

bool SlotManagement::AngleUpdateCondition(
    const common::SlotInfo &new_slot_info) {
  Eigen::Vector2d ego_heading_axis(std::cos(measurement_.heading),
                                   std::sin(measurement_.heading));

  Eigen::Vector2d slot_heading_axis =
      Eigen::Vector2d(new_slot_info.center().x(), new_slot_info.center().y()) -
      measurement_.mirror_pos;

  const auto angle_mag = std::fabs(
      pnc::transform::GetAngleFromTwoVec(ego_heading_axis, slot_heading_axis));

  const auto angle_dis =
      pnc::mathlib::Deg2Rad(param_.max_slots_update_angle_dis_limit_deg);

  bool angle_update_condition = pnc::mathlib::IsInBound(
      angle_mag, kPie * 0.5 - angle_dis, kPie * 0.5 + angle_dis);

#ifdef __DEBUG_PRINT__
  std::cout << "id = " << new_slot_info.id() << std::endl;
  std::cout << "angle_update_condition = " << angle_update_condition
            << std::endl;
  std::cout << "angle_mag_deg_dif = " << fabs(angle_mag * 57.3 - 90.0)
            << std::endl;
#endif

  return angle_update_condition;
}

bool SlotManagement::LonDifUpdateCondition(
    const common::SlotInfo &new_slot_info) {
  const auto new_pts = new_slot_info.corner_points();

  Eigen::Vector2d car_rear_center_to_pt1_vec(
      new_pts.corner_point(0).x() - measurement_.ego_pos(0),
      new_pts.corner_point(0).y() - measurement_.ego_pos(1));
  const Eigen::Vector2d ego_heading_unit(std::cos(measurement_.heading),
                                         std::sin(measurement_.heading));
  const Eigen::Vector2d ego_turn_right_unit(std::sin(measurement_.heading),
                                            -std::cos(measurement_.heading));
  const Eigen::Vector2d ego_turn_left_unit(-std::sin(measurement_.heading),
                                           std::cos(measurement_.heading));
  Eigen::Vector2d corresponding_mirror_pos;
  // ego car heading * ego car to slot vec
  const double prod = ego_heading_unit(0) * car_rear_center_to_pt1_vec(1) -
                      ego_heading_unit(1) * car_rear_center_to_pt1_vec(0);
  // right side slot
  if (prod < 0) {
    corresponding_mirror_pos =
        measurement_.mirror_pos +
        ego_turn_right_unit * param_.lat_dist_rearview_mirror_to_center;
  } else if (prod > 0) {
    corresponding_mirror_pos =
        measurement_.mirror_pos +
        ego_turn_left_unit * param_.lat_dist_rearview_mirror_to_center;
  } else {
    return false;
  }
  Eigen::Vector2d slot_center_to_side_mirror_vec(
      corresponding_mirror_pos(0) - new_slot_info.center().x(),
      corresponding_mirror_pos(1) - new_slot_info.center().y());

  Eigen::Vector2d new_slot_heading_vec(
      (new_pts.corner_point(0).x() + new_pts.corner_point(1).x() -
       new_pts.corner_point(2).x() - new_pts.corner_point(3).x()) *
          0.5,
      (new_pts.corner_point(0).y() + new_pts.corner_point(1).y() -
       new_pts.corner_point(2).y() - new_pts.corner_point(3).y()) *
          0.5);
  const double new_slot_middle_length =
      std::hypot(new_slot_heading_vec(0), new_slot_heading_vec(1));
  Eigen::Vector2d new_slot_heading_unit =
      new_slot_heading_vec / new_slot_middle_length;

  // new_slot_heading_unit prod
  // slot_center_to_side_mirror_vec
  const double lon_dif =
      new_slot_heading_unit(0) * slot_center_to_side_mirror_vec(1) -
      new_slot_heading_unit(1) * slot_center_to_side_mirror_vec(0);

  const bool lon_dif_update_condition =
      (lon_dif >= param_.min_slot_update_lon_dif_slot_center_to_mirror &&
       lon_dif <= param_.max_slot_update_lon_dif_slot_center_to_mirror);

#ifdef __DEBUG_PRINT__
  std::cout << "dif_update_condition = " << lon_dif_update_condition
            << std::endl;
  std::cout << "lon dif of slot center to mirror = " << lon_dif << std::endl;
#endif

  return lon_dif_update_condition;
}

bool SlotManagement::IfUpdateSlot(const common::SlotInfo &new_slot_info) {
  // update by angle between ego_heading_axis and
  // slot_heading_axis (new slot)
  const bool angle_update_condition = AngleUpdateCondition(new_slot_info);

  // update by lon dif between slot center and mirror middle
  // point
  const bool lon_update_condition = LonDifUpdateCondition(new_slot_info);

  return (angle_update_condition && lon_update_condition);
}

bool SlotManagement::ReleaseSlots() {
  for (auto i = 0; i < slot_management_info_.slot_info_vec_size(); i++) {
    auto slot = slot_management_info_.mutable_slot_info_vec(i);

    const double dis =
        std::hypot(slot->center().x() - measurement_.ego_pos.x(),
                   slot->center().y() - measurement_.ego_pos.y());

    if (dis > param_.max_slot_release_dist) {
      slot->set_is_release(false);
    } else {
      slot->set_is_release(true);
    }
  }
  return true;
}

}  // namespace planning
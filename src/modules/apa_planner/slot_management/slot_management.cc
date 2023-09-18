#include "slot_management.h"

#include <math.h>

#include <cmath>
#include <cstddef>
#include <utility>

#include "basic_types.pb.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "func_state_machine.pb.h"
#include "math_lib.h"
#include "slot_management_info.pb.h"
#include "transform_lib.h"
namespace planning {

static const double kPie = 3.141592653589793;

void SlotManagement::Reset() {
  // reset slot in
  slot_management_info_.Clear();

  // reset slot_info_map
  slot_info_map_.clear();
  slot_info_map_.reserve(param_.max_slot_size);
}

void SlotManagement::Preprocess() {
  measurement_.ego_pos << localization_ptr_->pose().local_position().x(),
      localization_ptr_->pose().local_position().y();
  measurement_.heading = localization_ptr_->pose().heading();

  measurement_.mirror_pos << measurement_.ego_pos(0) +
                                 param_.lon_dist_rearview_mirror_to_rear_axle *
                                     std::cos(measurement_.heading),
      measurement_.ego_pos(1) + param_.lon_dist_rearview_mirror_to_rear_axle *
                                    std::sin(measurement_.heading);
}

bool SlotManagement::Update(
    FuncStateMachine::FuncStateMachine *func_statemachine,
    ParkingFusion::ParkingFusionInfo *parking_slot_info,
    LocalizationOutput::LocalizationEstimate *localization_info) {
  // set ptrs
  func_state_ptr_ = func_statemachine;
  parking_slot_ptr_ = parking_slot_info;
  localization_ptr_ = localization_info;

  // preprocess
  Preprocess();

  // update slots
  if (!UpdateSlots()) {
    return false;
  }

  // release slots
  ReleaseSlots();

  // restore slot management info
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_slot_management_info()
      ->CopyFrom(slot_management_info_);

  return true;
}

bool SlotManagement::UpdateSlots() {
  if ((!IsInParkingState()) || param_.force_clear) {
    Reset();
    return false;
  }

  // Update slots
  common::SlotInfo slot_info;

  for (auto i = 0; i < parking_slot_ptr_->parking_fusion_slot_lists_size();
       ++i) {
    const auto &fusion_slot = parking_slot_ptr_->parking_fusion_slot_lists(i);
    double accumulated_x = 0.0;
    double accumulated_y = 0.0;

    slot_info.Clear();
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

    // if parking slot is not valid, continue
    const bool is_slot_valid = IsValidParkingSlot(slot_info);
    if (!is_slot_valid) {
      continue;
    }

    const auto slot_info_vec_size = slot_management_info_.slot_info_vec_size();
    if (slot_info_map_.count(slot_info.id()) == 0) {  // get new id
      if (AngleUpdateCondition(slot_info)) {
        auto add_slot_info_vec = slot_management_info_.add_slot_info_vec();
        add_slot_info_vec->CopyFrom(slot_info);

        slot_info_map_.insert(
            std::make_pair(slot_info.id(), slot_info_vec_size));
      }
    } else {  // get old id
      // slot update strategy
      auto slot_idx = slot_info_map_[slot_info.id()];

      if (IfUpdateSlot(slot_info)) {
        *slot_management_info_.mutable_slot_info_vec(slot_idx) = slot_info;
      }
    }
  }

  // std::cout << "check!" << std::endl;
  // for (auto itr = slot_info_map_.begin(); itr != slot_info_map_.end(); ++itr)
  // {
  //   std::cout << "Key: " << itr->first << ", Value: " << itr->second
  //             << std::endl;
  // }
  // std::cout << "slot_management_info_cc = "
  //           << slot_management_info_.DebugString() << std::endl;
  return true;
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

  // 1. Check if the boundary lines 02 and 13 of the parking slot are
  // approximately parallel
  const double slot_line_angle_dif = std::fabs(
      pnc::transform::GetAngleFromTwoVec(slot_line02_vec, slot_line13_vec));
  const double slot_line_angle_dif_deg = slot_line_angle_dif * 57.3;

  const bool slot_line_parallel_condition =
      slot_line_angle_dif_deg <= param_.max_slot_boundary_line_angle_dif_deg;

  if (!slot_line_parallel_condition) {
    return false;
  }

  // 2.  Check if the nearby boundary lines of the parking slot are
  // approximately vertical
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

bool SlotManagement::IsInParkingState() const {
  std::cout << "func_state_ptr_->current_state() = "
            << func_state_ptr_->current_state() << std::endl;

  if ((func_state_ptr_->current_state() >= FuncStateMachine::PARK_IN_APA_IN &&
       func_state_ptr_->current_state() <=
           FuncStateMachine::PARK_OUT_COMPLETED) ||
      param_.force_apa_on) {
    std::cout << "apa on!" << std::endl;
    return true;
  } else {
    std::cout << "apa off!" << std::endl;
    return false;
  }
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
  std::cout << "new_slot_info = " << new_slot_info.id() << std::endl;
  std::cout << "angle_update_condition = " << angle_update_condition
            << std::endl;
  std::cout << "angle_mag_deg_dif = " << fabs(angle_mag * 57.3 - 90.0)
            << std::endl;

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

  // new_slot_heading_unit prod slot_center_to_side_mirror_vec
  const double lon_dif =
      new_slot_heading_unit(0) * slot_center_to_side_mirror_vec(1) -
      new_slot_heading_unit(1) * slot_center_to_side_mirror_vec(0);

  const bool lon_dif_update_condition =
      (lon_dif >= param_.min_slot_update_lon_dif_slot_center_to_mirror &&
       lon_dif <= param_.max_slot_update_lon_dif_slot_center_to_mirror);

  std::cout << "dif_update_condition = " << lon_dif_update_condition
            << std::endl;
  std::cout << "lon dif of slot center to mirror = " << lon_dif << std::endl;
  return lon_dif_update_condition;
}

bool SlotManagement::IfUpdateSlot(const common::SlotInfo &new_slot_info) {
  // update by angle between ego_heading_axis and slot_heading_axis (new slot)
  const bool angle_update_condition = AngleUpdateCondition(new_slot_info);

  // update by lon dif between slot center and mirror middle point
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
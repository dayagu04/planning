#include "slot_management.h"

#include <cstddef>
#include <utility>

#include "common/apa_cos_sin.h"
#include "debug_info_log.h"
#include "math/math_utils.h"

#include "func_state_machine.pb.h"
#include "slot_management_info.pb.h"
namespace planning {

using planning::common::Point2d;
using planning::common::SlotInfo;
using planning::common::SlotManagementInfo;

void SlotManagement::Reset() {
  // reset slot in
  slot_management_info_.Clear();

  // reset slot_info_map
  slot_info_map_.clear();
  slot_info_map_.reserve(param_.max_slot_size);
}

bool SlotManagement::Update(
    std::shared_ptr<FuncStateMachine::FuncStateMachine> func_statemachine,
    std::shared_ptr<ParkingFusion::ParkingFusionInfo> parking_slot_info,
    std::shared_ptr<LocalizationOutput::LocalizationEstimate>
        localization_info) {
  func_state_ptr_ = func_statemachine;
  parking_slot_ptr_ = parking_slot_info;
  GetLocalizationInfo(localization_info);

  if (!UpdateSlots()) {
    return false;
  }
  ReleaseSlots();

  // restore slot management info
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_slot_management_info()
      ->CopyFrom(slot_management_info_);
}

bool SlotManagement::UpdateSlots() {
  if (!IsInParkingState()) {
    parking_enable_flag_ = false;
    Reset();
    return false;
  }
  parking_enable_flag_ = true;

  // Update slots
  const auto &fusion_slots_size =
      parking_slot_ptr_->parking_fusion_slot_lists_size();

  for (size_t i = 0; i < fusion_slots_size; ++i) {
    SlotInfo slot_info;
    const auto &fusion_slot = parking_slot_ptr_->parking_fusion_slot_lists(i);
    double accumulated_x = 0.0, accumulated_y = 0.0;

    for (size_t j = 0; j < 4; j++) {
      Point2d *tmp_corner_point = slot_info.corner_points().add_corner_point();
      tmp_corner_point->set_x(fusion_slot.corner_points(j).x());
      tmp_corner_point->set_y(fusion_slot.corner_points(j).y());
      accumulated_x += fusion_slot.corner_points(j).x();
      accumulated_y += fusion_slot.corner_points(j).y();
    }
    slot_info.mutable_center()->set_x(accumulated_x * 0.25);
    slot_info.mutable_center()->set_y(accumulated_y * 0.25);
    slot_info.set_id(fusion_slot.id());
    slot_info.set_is_release(false);

    if (slot_info_map_.count(slot_info.id()) == 0) {  // get new id
      slot_management_info_.mutable_slot_info_vec()->CopyFrom(
          std::move(slot_info));
      slot_info_map_.insert(std::make_pair(
          slot_info.id(), slot_management_info_.slot_info_vec_size() - 1));
    } else {  // get old id
      // slot update strategy
      size_t slot_idx = slot_info_map_(slot_info.id());
      auto old_slot_info =
          slot_management_info_.mutable_slot_info_vec(slot_idx);
      if (IfUpdateSlot(slot_info)) {
        *old_slot_info = slot_info;
      }
    }
  }
  return true;
}

bool SlotManagement::IsInParkingState() const {
  if (func_state_ptr_->current_state() >= FuncStateMachine::PARK_IN_APA_IN &&
      func_state_ptr_P > current_state() <=
          FuncStateMachine::PARK_OUT_COMPLETED) {
    return true;
  } else {
    return false;
  }
}

bool SlotManagement::IfUpdateSlot(const SlotInfo &new_slot_info) const {
  const size_t slot_idx = slot_info_map_(new_slot_info.id());
  const auto old_slot_info =
      slot_management_info_.mutable_slot_info_vec(slot_idx);

  const double old_slot_heading =
      NormalizeAngle(CalSlotHeading(*old_slot_info));
  const double new_slot_heading = NormalizeAngle(CalSlotHeading(new_slot_info));
  const double slots_heading_diff =
      std::fabs(NormalizeAngle(old_slot_heading - new_slot_heading));

  const double dis_of_center_mirror_to_new_slot =
      std::sqrt(std::pow(mirror_center_pos_.x - new_slot_info.center().x(), 2) +
                std::pow(mirror_center_pos_.y - new_slot_info.center().y(), 2));
  const double dis_of_center_mirror_to_old_slot = std::sqrt(
      std::pow(mirror_center_pos_.x - old_slot_info->center().x(), 2) +
      std::pow(mirror_center_pos_.y - old_slot_info->center().y(), 2));

  if (dis_of_center_mirror_to_new_slot <= max_slot_release_dis &&
      dis_of_center_mirror_to_old_slot >= dis_of_center_mirror_to_new_slot) {
    if (fabs(NormalizeAngle(new_slot_heading - local_pos_.theta)) <=
            param_.max_slot_heading_tol ||
        fabs(NormalizeAngle(new_slot_heading - local_pos_.theta -
                            M_PI * 0.5)) <= param_.max_slot_heading_tol) {
      if (!IfNewCornerPointsFluctuatingLittle(new_slot_info) ||
          slots_heading_diff >= param_.max_slot_heading_tol) {
        return true;
      }
    }
  }
  return false;
}  // namespace planning

bool SlotManagement::IfNewCornerPointsFluctuatingLittle(
    const SlotInfo &new_slot_info) const {
  const size_t slot_idx = slot_info_map_(new_slot_info.id());
  const auto old_slot_info =
      slot_management_info_.mutable_slot_info_vec(slot_idx);
  const auto corner_point_nums =
      new_slot_info.cornewr_points.corner_point_size();

  for (size_t i = 0; i < corner_point_nums; i++) {
    const double fluctuating_dis =
        std::sqrt(std::pow((new_slot_info.cornewr_points.corner_point(i).x() -
                            old_slot_info->corner_points.corner_point(i).x()),
                           2) +
                  std::pow((new_slot_info.cornewr_points.corner_point(i).y() -
                            old_slot_info->corner_points.corner_point(i).y()),
                           2));
    if (i < 2) {
      if (fluctuating_dis > param_.max_nearby_pt_fluctuating_dis) {
        return false;
      }
    } else {
      if (fluctuating_dis > param_.max_remote_pt_fluctuating_dis) {
        return false;
      }
    }
  }
  return true;
}
double SlotManagement::CalSlotHeading(const SlotInfo &slot_info) const {
  const double slot_heading_dx =
      (new_slot_info.corner_points(0).x() + new_slot_info.corner_points(1).x() -
       new_slot_info.corner_points(2).x() -
       new_slot_info.corner_points(3).x()) *
      0.5;
  const double slot_heading_dy =
      (new_slot_info.corner_points(0).y() + new_slot_info.corner_points(1).y() -
       new_slot_info.corner_points(2).y() -
       new_slot_info.corner_points(3).y()) *
      0.5;
  return NormalizeAngle(slot_heading_dy, slot_heading_dx);
}

bool SlotManagement::GetLocalizationInfo(
    std::shared_ptr<LocalizationOutput::LocalizationEstimate>
        localization_info) {
  const auto &pose = localization_info->Pose();
  local_pos_.x = pose.local_position().x();
  local_pos_.y = pose.local_position().y();
  local_pos_.theta = pose.heading();
  mirror_center_pos_.x =
      local_pos_.x +
      param_.lon_dis_mirror_to_rear_axis_center * apa_cos(local_pos_.theta);
  mirror_center_pos_.y =
      local_pos_.y +
      param_.lon_dis_mirror_to_rear_axis_center * apa_sin(local_pos_.theta);
  mirror_center_pos_.theta = local_pos_.theta;
  return true;
}
bool SlotManagement::ReleaseSlots() {
  size_t slot_nums = slot_management_info_.slot_info_vec_size();
  for (int i = 0; i < slot_nums; i++) {
    auto &slot = slot_management_info_.slot_info_vec(i);
    const double dis = sqrt(std::pow((slot.center().x() - local_pos_.x), 2) +
                            std::pow((slot.center().y() - local_pos_.y), 2));
    if (dis > param_.max_slot_release_dis) {
      slot.set_is_release(false);
    } else {
      slot.set_is_release(true);
    }
  }
  return true;
}

}  // namespace planning
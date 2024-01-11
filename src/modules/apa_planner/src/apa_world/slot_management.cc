#include "slot_management.h"

#include <math.h>

#include <cmath>
#include <cstddef>
#include <utility>

#include "Eigen/Core"
#include "basic_types.pb.h"
#include "common.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "dubins_lib.h"
#include "func_state_machine.pb.h"
#include "geometry_math.h"
#include "math_lib.h"
#include "parking_fusion.pb.h"
#include "slot_management_info.pb.h"
#include "transform_lib.h"

namespace planning {

static const double kPie = 3.141592653589793;
static const double kMinSlotUpdateOccupiedRatio = 0.6;

static const double kMaxSlotUpdateFixedOrientationCos = 0.866;  // cos(30°)
static const double kFixSlotUpdateOccupiedRatio = 0.85;

// for occupied ratio
static const double kTerminalTargetX = 1.35;
static const double kTerminalTargetY = 0.0;
static const double kTerminalTargetYBias = 0.0;
static const double kHeadingBias = std::cos(75.0 / 57.3);
static const double kPosYBias = 0.9;
static const double kNormalkSlotLength = 5.2;

void SlotManagement::Reset() {
  // reset slot info
  frame_.slot_management_info_.Clear();
  frame_.slot_info_window_vec_.clear();

  // reset LimiterPointWindow
  frame_.limiter_point_window_.Reset();

  // reset slot_info_map
  frame_.slot_info_map_.clear();
  frame_.slot_info_map_.reserve(frame_.param_.max_slot_size);
  frame_.set_seleted_id_mannually = 0;

  frame_.slot_occupied_ratio_ = 0.0;
  frame_.fusion_order_error_cnt_ = 0;
  frame_.is_occupied_ = false;
  frame_.is_switch_parking = false;

  // reset slot_management_info_
  frame_.occupied_info_map_.clear();
  frame_.occupied_info_map_.reserve(frame_.param_.max_slot_size);

  frame_.is_fixed_ = false;
}

bool SlotManagement::Update(const LocalView *local_view_ptr) {
  return Update(&local_view_ptr->function_state_machine_info,
                &local_view_ptr->parking_fusion_info,
                &local_view_ptr->localization_estimate,
                &local_view_ptr->uss_wave_info);
}

bool SlotManagement::Update(
    const FuncStateMachine::FuncStateMachine *func_statemachine,
    const ParkingFusion::ParkingFusionInfo *parking_slot_info,
    const LocalizationOutput::LocalizationEstimate *localization_info,
    const UssWaveInfo::UssWaveInfo *uss_wave_info) {
  // set ptrs
  frame_.func_state_ptr_ = func_statemachine;
  frame_.parking_slot_ptr_ = parking_slot_info;
  frame_.localization_ptr_ = localization_info;
  frame_.uss_wave_info_ptr_ = uss_wave_info;

  if (!IsInAPAState() || frame_.param_.force_clear) {
    std::cout << "reset\n";
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

  // must run after updating slots
  UpdateOccupiedInfo();

  // ReleaseSlots();

  // for hmi
  UpdateReleasedSlotInfo();

  // restore slot management info
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_slot_management_info()
      ->CopyFrom(frame_.slot_management_info_);

  if (update_searching_flag || update_occupied_flag) {
    return true;
  } else {
    return false;
  }
}

bool SlotManagement::IsInAPAState() const {
  if ((frame_.func_state_ptr_->current_state() >=
           FuncStateMachine::PARK_IN_APA_IN &&
       frame_.func_state_ptr_->current_state() <=
           FuncStateMachine::PARK_IN_COMPLETED) ||
      frame_.param_.force_apa_on) {
    return true;
  } else {
    return false;
  }
}

void SlotManagement::Preprocess() {
  frame_.measurement_.ego_pos
      << frame_.localization_ptr_->pose().local_position().x(),
      frame_.localization_ptr_->pose().local_position().y();
  frame_.measurement_.heading = frame_.localization_ptr_->pose().heading();

  frame_.measurement_.mirror_pos
      << frame_.measurement_.ego_pos.x() +
             frame_.param_.lon_dist_rearview_mirror_to_rear_axle *
                 std::cos(frame_.measurement_.heading),
      frame_.measurement_.ego_pos.y() +
          frame_.param_.lon_dist_rearview_mirror_to_rear_axle *
              std::sin(frame_.measurement_.heading);
}

bool SlotManagement::IsInSearchingState() const {
#ifdef __DEBUG_PRINT__
  std::cout << "func_state_ptr_->current_state() = "
            << func_state_ptr_->current_state() << std::endl;
  // should check each frame slot_management_info value
  std::cout << "managed slot size:"
            << slot_management_info_.slot_info_vec_size() << std::endl;
  std::cout << "magaged slot id: ";
  for (int i = 0; i < slot_management_info_.slot_info_vec_size(); ++i) {
    std::cout << slot_management_info_.slot_info_vec(i).id() << ", ";
  }
  std::cout << std::endl;
#endif

  if ((frame_.func_state_ptr_->current_state() >=
           FuncStateMachine::PARK_IN_APA_IN &&
       frame_.func_state_ptr_->current_state() <=
           FuncStateMachine::PARK_IN_NO_READY) ||
      (frame_.param_.force_apa_on && (!frame_.is_switch_parking))) {
    std::cout << "apa searching on!" << std::endl;
    return true;
  } else {
    std::cout << "apa searching off!" << std::endl;
    return false;
  }
}

bool SlotManagement::UpdateSlotsInSearching() {
  // Update slots
  for (auto i = 0;
       i < frame_.parking_slot_ptr_->parking_fusion_slot_lists_size(); ++i) {
    const auto &fusion_slot =
        frame_.parking_slot_ptr_->parking_fusion_slot_lists(i);

    common::SlotInfo slot_info;
    auto is_valid_slot = ProcessRawSlot(fusion_slot, slot_info);

    if (!is_valid_slot) {
      continue;
    }

    const auto slot_info_vec_size = frame_.slot_info_window_vec_.size();
    if (frame_.slot_info_map_.count(slot_info.id()) == 0 &&
        LonDifUpdateCondition(slot_info)) {  // get new id

      SlotInfoWindow slot_info_window;
      slot_info_window.Add(slot_info);

      frame_.slot_info_window_vec_.emplace_back(slot_info_window);

      frame_.slot_info_map_.insert(
          std::make_pair(slot_info.id(), slot_info_vec_size));
    } else {  // get old id
      // slot update strategy
      if (IfUpdateSlot(slot_info)) {
        auto slot_idx = frame_.slot_info_map_[slot_info.id()];
        frame_.slot_info_window_vec_[slot_idx].Add(slot_info);
      }
    }
  }

  // assemble slot_management_info_
  frame_.slot_management_info_.mutable_slot_info_vec()->Clear();
  for (size_t j = 0; j < frame_.slot_info_window_vec_.size(); ++j) {
    auto slot = frame_.slot_management_info_.add_slot_info_vec();
    // auto slot_info = frame_.slot_info_window_vec_[j].GetFusedInfo();
    // ModifySlot2Rectangle(slot_info);
    // *slot = slot_info;
    *slot = frame_.slot_info_window_vec_[j].GetFusedInfo();
  }
  // set seleted_slot manually and update limiter
  UpdateLimiterForAutoSetSlot();
  return true;
}

bool SlotManagement::IsInParkingState() const {
  return (frame_.func_state_ptr_->current_state() ==
          FuncStateMachine::PARK_IN_ACTIVATE_CONTROL) ||
         frame_.is_switch_parking;
}

bool SlotManagement::UpdateSlotsInParking() {
  google::protobuf::uint32 select_slot_id = 0;
  if (frame_.parking_slot_ptr_->select_slot_id() != 0) {
    select_slot_id = frame_.parking_slot_ptr_->select_slot_id();
  } else {
    select_slot_id = frame_.set_seleted_id_mannually;
  }

  common::SlotInfo select_slot;
  ParkingFusion::ParkingFusionSlot selecte_fusion_slot;
  size_t selected_fusion_slot_index = 0;
  for (int i = 0;
       i < frame_.parking_slot_ptr_->parking_fusion_slot_lists_size(); ++i) {
    const auto &fusion_slot =
        frame_.parking_slot_ptr_->parking_fusion_slot_lists(i);
    if (select_slot_id == fusion_slot.id()) {
      selected_fusion_slot_index = i;
      selecte_fusion_slot = fusion_slot;
      break;
    }
  }

  auto is_valid_slot = ProcessRawSlot(selecte_fusion_slot, select_slot);

  if (!is_valid_slot) {
    return false;
  }

  // calculate occupied ratio
  frame_.slot_occupied_ratio_ = CalOccupiedRatio();

  std::cout << "occupied_ratio in slm: " << frame_.slot_occupied_ratio_
            << std::endl;

  // update slot management info :: limiter info
  UpdateLimiterInfoInParking(select_slot, selected_fusion_slot_index);

  // if occupied percentage is less than certain value,return
  bool is_occupied = false;
  if (frame_.slot_occupied_ratio_ < kMinSlotUpdateOccupiedRatio) {
    is_occupied = false;
    frame_.is_occupied_ = is_occupied;
    // std::cout << "occupied_ratio is less than min update value " <<
    // std::endl;
    return false;
  } else {
    is_occupied = true;
  }

  // update slot_management_info :: slot info
  // UpdateSlotInfoInParking(select_slot, select_slot_id, is_occupied);

  frame_.is_occupied_ = is_occupied;

  return true;
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
  if (IsInSearchingState()) {
    slot_info.set_is_release((fusion_slot.allow_parking() == 1));
    slot_info.set_is_occupied((fusion_slot.allow_parking() == 0));
  }

  if (IsInParkingState()) {
    slot_info.set_is_release(true);
    slot_info.set_is_occupied(false);
  }

  return slot_info;
}

bool SlotManagement::IsValidParkingSlot(
    const common::SlotInfo &slot_info) const {
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
      slot_line_angle_dif_deg <=
      frame_.param_.max_slot_boundary_line_angle_dif_deg;

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
      max_corner_angle_dif <=
      frame_.param_.max_slot_boundary_line_angle_dif_deg;
  if (corner_vertical_condition) {
    return true;
  } else {
    return false;
  }
}

bool SlotManagement::CorrectSlotPointsOrder(common::SlotInfo &slot_info) const {
  Eigen::Vector2d slot_pt_0(slot_info.corner_points().corner_point(0).x(),
                            slot_info.corner_points().corner_point(0).y());

  Eigen::Vector2d slot_pt_1(slot_info.corner_points().corner_point(1).x(),
                            slot_info.corner_points().corner_point(1).y());

  Eigen::Vector2d slot_pt_2(slot_info.corner_points().corner_point(2).x(),
                            slot_info.corner_points().corner_point(2).y());

  Eigen::Vector2d slot_pt_3(slot_info.corner_points().corner_point(3).x(),
                            slot_info.corner_points().corner_point(3).y());

  Eigen::Vector2d middle_pt_01 = (slot_pt_0 + slot_pt_1) * 0.5;
  Eigen::Vector2d middle_pt_23 = (slot_pt_2 + slot_pt_3) * 0.5;

  Eigen::Vector2d slot_middle_vec = middle_pt_01 - middle_pt_23;

  Eigen::Vector2d middle_pt23_to_pt0_vec = slot_pt_0 - middle_pt_23;

  const double cross = slot_middle_vec(0) * middle_pt23_to_pt0_vec(1) -
                       slot_middle_vec(1) * middle_pt23_to_pt0_vec(0);
  // slot pt 0 need to change with pt 1   , 2 <->3
  if (cross > 0) {
    slot_info.mutable_corner_points()->mutable_corner_point(0)->set_x(
        slot_pt_1(0));
    slot_info.mutable_corner_points()->mutable_corner_point(0)->set_y(
        slot_pt_1(1));

    slot_info.mutable_corner_points()->mutable_corner_point(1)->set_x(
        slot_pt_0(0));
    slot_info.mutable_corner_points()->mutable_corner_point(1)->set_y(
        slot_pt_0(1));

    slot_info.mutable_corner_points()->mutable_corner_point(2)->set_x(
        slot_pt_3(0));
    slot_info.mutable_corner_points()->mutable_corner_point(2)->set_y(
        slot_pt_3(1));

    slot_info.mutable_corner_points()->mutable_corner_point(3)->set_x(
        slot_pt_2(0));
    slot_info.mutable_corner_points()->mutable_corner_point(3)->set_y(
        slot_pt_2(1));
    return true;
  }
  return false;
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

bool SlotManagement::LonDifUpdateCondition(
    const common::SlotInfo &new_slot_info) {
  const auto new_pts = new_slot_info.corner_points();

  Eigen::Vector2d car_rear_center_to_pt1_vec(
      new_pts.corner_point(0).x() - frame_.measurement_.ego_pos(0),
      new_pts.corner_point(0).y() - frame_.measurement_.ego_pos(1));
  const Eigen::Vector2d ego_heading_unit(std::cos(frame_.measurement_.heading),
                                         std::sin(frame_.measurement_.heading));
  const Eigen::Vector2d ego_turn_right_unit(
      std::sin(frame_.measurement_.heading),
      -std::cos(frame_.measurement_.heading));
  const Eigen::Vector2d ego_turn_left_unit(
      -std::sin(frame_.measurement_.heading),
      std::cos(frame_.measurement_.heading));
  Eigen::Vector2d corresponding_mirror_pos;
  // ego car heading * ego car to slot vec
  const double prod = ego_heading_unit(0) * car_rear_center_to_pt1_vec(1) -
                      ego_heading_unit(1) * car_rear_center_to_pt1_vec(0);
  // right side slot
  if (prod < 0) {
    corresponding_mirror_pos =
        frame_.measurement_.mirror_pos +
        ego_turn_right_unit * frame_.param_.lat_dist_rearview_mirror_to_center;
  } else if (prod > 0) {
    corresponding_mirror_pos =
        frame_.measurement_.mirror_pos +
        ego_turn_left_unit * frame_.param_.lat_dist_rearview_mirror_to_center;
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
      std::fabs(new_slot_heading_unit(0) * slot_center_to_side_mirror_vec(1) -
                new_slot_heading_unit(1) * slot_center_to_side_mirror_vec(0));

  const bool lon_dif_update_condition =
      (lon_dif >= frame_.param_.min_slot_update_lon_dif_slot_center_to_mirror &&
       lon_dif <= frame_.param_.max_slot_update_lon_dif_slot_center_to_mirror);

#ifdef __DEBUG_PRINT__
  std::cout << "dif_update_condition = " << lon_dif_update_condition
            << std::endl;
  std::cout << "lon dif of slot center to mirror = " << lon_dif << std::endl;
#endif

  return lon_dif_update_condition;
}

bool SlotManagement::AngleUpdateCondition(
    const common::SlotInfo &new_slot_info) {
  Eigen::Vector2d ego_heading_axis(std::cos(frame_.measurement_.heading),
                                   std::sin(frame_.measurement_.heading));

  Eigen::Vector2d slot_heading_axis =
      Eigen::Vector2d(new_slot_info.center().x(), new_slot_info.center().y()) -
      frame_.measurement_.mirror_pos;

  const auto angle_mag = std::fabs(
      pnc::transform::GetAngleFromTwoVec(ego_heading_axis, slot_heading_axis));

  const auto angle_dis =
      pnc::mathlib::Deg2Rad(frame_.param_.max_slots_update_angle_dis_limit_deg);

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

double SlotManagement::CalOccupiedRatio() const {
  Eigen::Vector2d middle_pt_01 = Eigen::Vector2d::Zero();
  Eigen::Vector2d middle_pt_23 = Eigen::Vector2d::Zero();
  Eigen::Vector2d slot_heading_unit_vec = Eigen::Vector2d::Zero();

  google::protobuf::uint32 select_slot_id = 0;
  if (frame_.parking_slot_ptr_->select_slot_id() != 0) {
    select_slot_id = frame_.parking_slot_ptr_->select_slot_id();
  } else {
    select_slot_id = frame_.set_seleted_id_mannually;
  }

  common::SlotInfo fusion_slot_target;
  for (const auto &fusion_slot : frame_.slot_management_info_.slot_info_vec()) {
    if (select_slot_id == fusion_slot.id()) {
      fusion_slot_target = fusion_slot;
      break;
    }
  }

  // to classify perpendicular or parallel
  Eigen::Vector2d slot_p0(
      fusion_slot_target.corner_points().corner_point(0).x(),
      fusion_slot_target.corner_points().corner_point(0).y());

  Eigen::Vector2d slot_p1(
      fusion_slot_target.corner_points().corner_point(1).x(),
      fusion_slot_target.corner_points().corner_point(1).y());

  Eigen::Vector2d slot_p2(
      fusion_slot_target.corner_points().corner_point(2).x(),
      fusion_slot_target.corner_points().corner_point(2).y());

  const auto slot_p0_p1 = (slot_p0 - slot_p1).norm();
  const auto slot_p0_p2 = (slot_p0 - slot_p2).norm();
  bool is_perpendicular_flag = false;
  if (slot_p0_p1 < slot_p0_p2) {
    is_perpendicular_flag = true;
  }

  if (is_perpendicular_flag) {
    middle_pt_01
        << 0.5 * (fusion_slot_target.corner_points().corner_point(0).x() +
                  fusion_slot_target.corner_points().corner_point(1).x()),
        0.5 * (fusion_slot_target.corner_points().corner_point(0).y() +
               fusion_slot_target.corner_points().corner_point(1).y());

    middle_pt_23
        << 0.5 * (fusion_slot_target.corner_points().corner_point(2).x() +
                  fusion_slot_target.corner_points().corner_point(3).x()),
        0.5 * (fusion_slot_target.corner_points().corner_point(2).y() +
               fusion_slot_target.corner_points().corner_point(3).y());
  } else {
    middle_pt_01
        << 0.5 * (fusion_slot_target.corner_points().corner_point(1).x() +
                  fusion_slot_target.corner_points().corner_point(3).x()),
        0.5 * (fusion_slot_target.corner_points().corner_point(1).y() +
               fusion_slot_target.corner_points().corner_point(3).y());

    middle_pt_23
        << 0.5 * (fusion_slot_target.corner_points().corner_point(0).x() +
                  fusion_slot_target.corner_points().corner_point(2).x()),
        0.5 * (fusion_slot_target.corner_points().corner_point(0).y() +
               fusion_slot_target.corner_points().corner_point(2).y());
  }

  const Eigen::Vector2d target_pt(kTerminalTargetX,
                                  kTerminalTargetY + kTerminalTargetYBias);

  slot_heading_unit_vec = (middle_pt_01 - middle_pt_23).normalized();

  const auto slot_heading =
      std::atan2(slot_heading_unit_vec.y(), slot_heading_unit_vec.x());

  const auto slot_local =
      pnc::geometry_lib::GlobalToLocalTf(middle_pt_23, slot_heading);

  const auto car_local = slot_local.GetPos(frame_.measurement_.ego_pos);

  // std::cout << "car_local:" << car_local.x() << " "
  //           << car_local.y() << "\n";

  const auto terminal_pos_err = car_local - target_pt;
  const Eigen::Vector2d car_heading_norm(std::cos(frame_.measurement_.heading),
                                         std::sin(frame_.measurement_.heading));

  const double terminal_heading_err =
      slot_heading_unit_vec.dot(car_heading_norm);

  double slot_occupied_ratio = 0.0;
  if (std::fabs(terminal_pos_err.y()) < kPosYBias &&
      std::fabs(terminal_heading_err) > kHeadingBias) {
    slot_occupied_ratio = pnc::mathlib::Clamp(
        1.0 - (terminal_pos_err.x() / kNormalkSlotLength), 0.0, 1.0);
  }
  return slot_occupied_ratio;
}

void SlotManagement::UpdateLimiterInfoInParking(
    const common::SlotInfo &select_slot,
    const size_t selected_fusin_slot_index) {
  std::pair<Eigen::Vector2d, Eigen::Vector2d> limiter_points;

  // to classify perpendicular or parallel
  Eigen::Vector2d slot_p0(select_slot.corner_points().corner_point(0).x(),
                          select_slot.corner_points().corner_point(0).y());

  Eigen::Vector2d slot_p1(select_slot.corner_points().corner_point(1).x(),
                          select_slot.corner_points().corner_point(1).y());

  Eigen::Vector2d slot_p2(select_slot.corner_points().corner_point(2).x(),
                          select_slot.corner_points().corner_point(2).y());

  const auto slot_p0_p1 = (slot_p0 - slot_p1).norm();
  const auto slot_p0_p2 = (slot_p0 - slot_p2).norm();
  bool is_perpendicular_flag = false;
  if (slot_p0_p1 < slot_p0_p2) {
    is_perpendicular_flag = true;
  }
  // smoothing limiter position
  if (frame_.limiter_point_window_.IsEmpty()) {
    // add default value
    const double K = apa_param.GetParam().normal_slot_length -
                     apa_param.GetParam().terminal_length;
    Eigen::Vector2d direction_vec;
    if (is_perpendicular_flag) {
      direction_vec << select_slot.corner_points().corner_point(3).x() -
                           select_slot.corner_points().corner_point(1).x(),
          select_slot.corner_points().corner_point(3).y() -
              select_slot.corner_points().corner_point(1).y();

      const auto direction_norm = direction_vec.normalized();

      limiter_points.first << select_slot.corner_points().corner_point(0).x() +
                                  K * direction_norm.x(),
          select_slot.corner_points().corner_point(0).y() +
              K * direction_norm.y();

      limiter_points.second << select_slot.corner_points().corner_point(1).x() +
                                   K * direction_norm.x(),
          select_slot.corner_points().corner_point(1).y() +
              K * direction_norm.y();

    } else {
      direction_vec << select_slot.corner_points().corner_point(2).x() -
                           select_slot.corner_points().corner_point(3).x(),
          select_slot.corner_points().corner_point(2).y() -
              select_slot.corner_points().corner_point(3).y();

      const auto direction_norm = direction_vec.normalized();

      limiter_points.first << select_slot.corner_points().corner_point(1).x() +
                                  K * direction_norm.x(),
          select_slot.corner_points().corner_point(1).y() +
              K * direction_norm.y();

      limiter_points.second << select_slot.corner_points().corner_point(3).x() +
                                   K * direction_norm.x(),
          select_slot.corner_points().corner_point(3).y() +
              K * direction_norm.y();
    }

    frame_.limiter_point_window_.Add(limiter_points);
  }

  const bool update_limiter_flag_low =
      frame_.slot_occupied_ratio_ <=
          apa_param.GetParam().max_limiter_update_occupied_ratio &&
      frame_.slot_occupied_ratio_ >=
          apa_param.GetParam().min_limiter_update_occupied_ratio;

  frame_.limiter_point_window_.Fuse();
  const auto limiter_points_line =
      frame_.limiter_point_window_.GetFusedLimiterPoints();

  Eigen::Vector2d p0;
  Eigen::Vector2d p1;
  p0 << limiter_points_line.first.x(), limiter_points_line.first.y();
  p1 << limiter_points_line.second.x(), limiter_points_line.second.y();
  const pnc::geometry_lib::LineSegment limiter_line(p0, p1);
  const auto res_distance = pnc::geometry_lib::CalPoint2LineDist(
      frame_.measurement_.ego_pos, limiter_line);

  const bool update_limiter_flag_upper =
      res_distance >= apa_param.GetParam().res_distance &&
      frame_.slot_occupied_ratio_ >=
          apa_param.GetParam().limiter_update_occupied_ratio;

  if (update_limiter_flag_low || update_limiter_flag_upper) {
    // add realtime value
    const auto selected_fusion_slot =
        frame_.parking_slot_ptr_->parking_fusion_slot_lists(
            selected_fusin_slot_index);

    if (selected_fusion_slot.limiter_position_size() > 0) {
      Eigen::Vector2d p0(selected_fusion_slot.limiter_position(0).x(),
                         selected_fusion_slot.limiter_position(0).y());

      Eigen::Vector2d p1(selected_fusion_slot.limiter_position(1).x(),
                         selected_fusion_slot.limiter_position(1).y());

      const auto heading_norm = (slot_p0 - slot_p2).normalized();

      const Eigen::Vector2d delta =
          apa_param.GetParam().res_limiter * heading_norm;
      limiter_points.first = p0 + delta;
      limiter_points.second = p1 + delta;
      frame_.limiter_point_window_.Add(limiter_points);
    } else {
      // if slot does not contain limiter, to set a virtual limiter
      const double K = apa_param.GetParam().normal_slot_length -
                       apa_param.GetParam().limiter_length;
      Eigen::Vector2d direction_vec;
      if (is_perpendicular_flag) {
        direction_vec << select_slot.corner_points().corner_point(3).x() -
                             select_slot.corner_points().corner_point(1).x(),
            select_slot.corner_points().corner_point(3).y() -
                select_slot.corner_points().corner_point(1).y();

        const auto direction_norm = direction_vec.normalized();

        limiter_points.first
            << select_slot.corner_points().corner_point(0).x() +
                   K * direction_norm.x(),
            select_slot.corner_points().corner_point(0).y() +
                K * direction_norm.y();

        limiter_points.second
            << select_slot.corner_points().corner_point(1).x() +
                   K * direction_norm.x(),
            select_slot.corner_points().corner_point(1).y() +
                K * direction_norm.y();

      } else {
        direction_vec << select_slot.corner_points().corner_point(2).x() -
                             select_slot.corner_points().corner_point(3).x(),
            select_slot.corner_points().corner_point(2).y() -
                select_slot.corner_points().corner_point(3).y();

        const auto direction_norm = direction_vec.normalized();

        limiter_points.first
            << select_slot.corner_points().corner_point(1).x() +
                   K * direction_norm.x(),
            select_slot.corner_points().corner_point(1).y() +
                K * direction_norm.y();

        limiter_points.second
            << select_slot.corner_points().corner_point(3).x() +
                   K * direction_norm.x(),
            select_slot.corner_points().corner_point(3).y() +
                K * direction_norm.y();
      }

      frame_.limiter_point_window_.Add(limiter_points);
    }
  }

  frame_.limiter_point_window_.Fuse();
  const auto fused_limiter_points =
      frame_.limiter_point_window_.GetFusedLimiterPoints();

  if (frame_.slot_management_info_.limiter_points_size() == 0) {
    auto limiter = frame_.slot_management_info_.add_limiter_points();
    *limiter = fused_limiter_points.first;
    limiter = frame_.slot_management_info_.add_limiter_points();
    *limiter = fused_limiter_points.second;
  } else {
    auto limiter = frame_.slot_management_info_.mutable_limiter_points(0);
    limiter->set_x(fused_limiter_points.first.x());
    limiter->set_y(fused_limiter_points.first.y());
    limiter = frame_.slot_management_info_.mutable_limiter_points(1);
    limiter->set_x(fused_limiter_points.second.x());
    limiter->set_y(fused_limiter_points.second.y());
  }
}

void SlotManagement::UpdateSlotInfoInParking(common::SlotInfo &select_slot,
                                             const size_t select_slot_id,
                                             const bool is_occupied) {
  const Eigen::Vector2d car_heading_norm(std::cos(frame_.measurement_.heading),
                                         std::sin(frame_.measurement_.heading));

  const Eigen::Vector2d slot_middle1(
      0.5 * (select_slot.corner_points().corner_point(0).x() +
             select_slot.corner_points().corner_point(1).x()),
      0.5 * (select_slot.corner_points().corner_point(0).y() +
             select_slot.corner_points().corner_point(1).y()));

  const Eigen::Vector2d slot_middle2(
      0.5 * (select_slot.corner_points().corner_point(2).x() +
             select_slot.corner_points().corner_point(3).x()),
      0.5 * (select_slot.corner_points().corner_point(2).y() +
             select_slot.corner_points().corner_point(3).y()));

  const auto slot_heading_norm = (slot_middle1 - slot_middle2).normalized();

  const auto slot_car_heading_cos =
      std::fabs(slot_heading_norm.dot(car_heading_norm));

  // std::cout<<"cos heading:"<< slot_car_heading_cos<<std::endl;

  const bool is_update_slot =
      (frame_.slot_occupied_ratio_ >= kMinSlotUpdateOccupiedRatio) &&
      (slot_car_heading_cos >= kMaxSlotUpdateFixedOrientationCos);

  frame_.is_fixed_ = frame_.is_fixed_ || (frame_.slot_occupied_ratio_ >
                                          kFixSlotUpdateOccupiedRatio);

  // std::cout << "car_limiter_distance:" << car_limiter_distance << "\n";
  // std::cout << "slot_car_heading_cos:" << slot_car_heading_cos << "\n";
  // std::cout << "is_update_slot:" << is_update_slot << "\n";

  if (is_update_slot && !frame_.is_fixed_) {
    std::cout << "update slot last time\n";
    // update slot
    auto slot_idx = frame_.slot_info_map_[select_slot_id];
    auto slot = frame_.slot_management_info_.mutable_slot_info_vec(slot_idx);

    // reset when latest occupied
    if ((frame_.is_occupied_ == false) && (is_occupied == true)) {
      frame_.slot_info_window_vec_[slot_idx].Reset();
    }

    frame_.slot_info_window_vec_[slot_idx].Add(select_slot);
    *slot = frame_.slot_info_window_vec_[slot_idx].GetFusedInfo();
  }
}

void SlotManagement::UpdateOccupiedInfo() {
  // set occupied info
  for (auto &slot_info :
       *frame_.slot_management_info_.mutable_slot_info_vec()) {
    slot_info.set_is_occupied(
        frame_.occupied_info_map_[slot_info.id()].is_occupied);

    common::SlotInfo slot;
    if (GetSelectedSlot(slot)) {
      if (slot_info.id() == slot.id()) {
        slot_info.set_is_occupied(false);
        slot_info.set_is_release(true);
      }
    }
  }
}

bool SlotManagement::ReleaseSlots() {
  for (auto i = 0; i < frame_.slot_management_info_.slot_info_vec_size(); i++) {
    auto slot = frame_.slot_management_info_.mutable_slot_info_vec(i);

    const double dis =
        std::hypot(slot->center().x() - frame_.measurement_.ego_pos.x(),
                   slot->center().y() - frame_.measurement_.ego_pos.y());

    if (dis > frame_.param_.max_slot_release_dist) {
      slot->set_is_release(false);
    } else {
      slot->set_is_release(true);
    }
  }
  return true;
}

void SlotManagement::UpdateReleasedSlotInfo() {
  frame_.released_slot_info_vec_.clear();
  PlanningOutput::SuccessfulSlotsInfo released_slot_info;
  for (const auto &slot_info : frame_.slot_management_info_.slot_info_vec()) {
    if (slot_info.is_release()) {
      released_slot_info.Clear();
      released_slot_info.set_id(slot_info.id());
      frame_.released_slot_info_vec_.emplace_back(released_slot_info);
    }
  }
}
const bool SlotManagement::GetSelectedSlot(common::SlotInfo &slot_info,
                                           const int selected_id) const {
  if (frame_.slot_info_map_.count(selected_id) == 0) {
    return false;
  } else {
    slot_info = frame_.slot_management_info_.slot_info_vec(
        frame_.slot_info_map_.at(selected_id));
    return true;
  }
}

const bool SlotManagement::GetSelectedSlot(common::SlotInfo &slot_info) const {
  if (frame_.parking_slot_ptr_->has_select_slot_id()) {
    const auto &selected_id = frame_.parking_slot_ptr_->select_slot_id();
    if (frame_.slot_info_map_.count(selected_id) == 0) {
      return false;
    } else {
      slot_info = frame_.slot_management_info_.slot_info_vec(
          frame_.slot_info_map_.at(selected_id));
      return true;
    }
  } else {
    return false;
  }
}

const bool SlotManagement::GetSelectedLimiter(
    std::pair<Eigen::Vector2d, Eigen::Vector2d> &fused_limiter) const {
  if (frame_.slot_management_info_.limiter_points_size() > 0) {
    fused_limiter.first << frame_.slot_management_info_.limiter_points(0).x(),
        frame_.slot_management_info_.limiter_points(0).y();

    fused_limiter.second << frame_.slot_management_info_.limiter_points(1).x(),
        frame_.slot_management_info_.limiter_points(1).y();
    return true;
  }
  return false;
}

const bool SlotManagement::SetRealtime() {
  std::cout << "use real time slot\n";
  google::protobuf::uint32 select_slot_id = 0;
  if (frame_.parking_slot_ptr_->select_slot_id() != 0) {
    select_slot_id = frame_.parking_slot_ptr_->select_slot_id();
  } else {
    select_slot_id = frame_.set_seleted_id_mannually;
  }
  common::SlotInfo select_slot;
  ParkingFusion::ParkingFusionSlot select_fusion_slot;
  for (const auto &fusion_slot :
       frame_.parking_slot_ptr_->parking_fusion_slot_lists()) {
    if (select_slot_id == fusion_slot.id()) {
      select_fusion_slot = fusion_slot;
      break;
    }
  }
  const bool is_valid_slot = ProcessRawSlot(select_fusion_slot, select_slot);
  if (!is_valid_slot) {
    return false;
  }
  // update slot
  auto slot_idx = frame_.slot_info_map_[select_slot_id];
  auto slot = frame_.slot_management_info_.mutable_slot_info_vec(slot_idx);
  frame_.slot_info_window_vec_[slot_idx].Reset();
  frame_.slot_info_window_vec_[slot_idx].Add(select_slot);
  *slot = frame_.slot_info_window_vec_[slot_idx].GetFusedInfo();

  return true;
}

const bool SlotManagement::ProcessRawSlot(
    const ParkingFusion::ParkingFusionSlot &parking_fusion_slot,
    common::SlotInfo &slot_info) {
  slot_info.Clear();
  slot_info = SlotInfoTransfer(parking_fusion_slot);

  if (!slot_info.has_corner_points()) {
    std::cout << "find no select slot" << std::endl;
    frame_.is_occupied_ = false;
    return false;
  }
  // if parking slot is not valid, continue
  const bool is_slot_valid = IsValidParkingSlot(slot_info);
  if (!is_slot_valid) {
    std::cout << "fusion slot is not valid" << std::endl;
    frame_.is_occupied_ = false;
    return false;
  }

  bool correct_slot_order = CorrectSlotPointsOrder(slot_info);
  if (correct_slot_order) {
    frame_.fusion_order_error_cnt_++;
  }

  // to get a similar rectangle
  ModifySlot2Rectangle(slot_info);

  return true;
}

void SlotManagement::ModifySlot2Rectangle(common::SlotInfo &slot_info) {
  std::vector<Eigen::Vector2d> original_vertices;
  std::vector<Eigen::Vector2d> target_boundingbox;
  original_vertices.reserve(4);
  original_vertices.clear();
  target_boundingbox.reserve(4);
  target_boundingbox.clear();
  for (google::protobuf::int32 i = 0;
       i < slot_info.corner_points().corner_point_size(); i++) {
    original_vertices.emplace_back(
        Eigen::Vector2d(slot_info.corner_points().corner_point(i).x(),
                        slot_info.corner_points().corner_point(i).y()));
  }

  const auto is_need_correct = pnc::geometry_lib::MinimumBoundingBox(
      original_vertices, target_boundingbox);

  if (is_need_correct) {
    for (size_t i = 0; i < target_boundingbox.size(); i++) {
      std::cout << i << " : " << target_boundingbox[i].x() << " "
                << target_boundingbox[i].y() << " \n";
    }
    slot_info.mutable_corner_points()->mutable_corner_point(0)->set_x(
        target_boundingbox[0].x());
    slot_info.mutable_corner_points()->mutable_corner_point(0)->set_y(
        target_boundingbox[0].y());

    slot_info.mutable_corner_points()->mutable_corner_point(1)->set_x(
        target_boundingbox[1].x());
    slot_info.mutable_corner_points()->mutable_corner_point(1)->set_y(
        target_boundingbox[1].y());

    slot_info.mutable_corner_points()->mutable_corner_point(2)->set_x(
        target_boundingbox[2].x());
    slot_info.mutable_corner_points()->mutable_corner_point(2)->set_y(
        target_boundingbox[2].y());

    slot_info.mutable_corner_points()->mutable_corner_point(3)->set_x(
        target_boundingbox[3].x());
    slot_info.mutable_corner_points()->mutable_corner_point(3)->set_y(
        target_boundingbox[3].y());
  }
}

void SlotManagement::UpdateLimiterForAutoSetSlot() {
  // set seleted_slot manually
  double car_slot_distance = 1e4;
  if (frame_.using_auto_switcher) {
    for (auto slot : frame_.slot_management_info_.slot_info_vec()) {
      Eigen::Vector2d slot_center;
      slot_center.x() = slot.center().x();
      slot_center.y() = slot.center().y();
      const auto distance = (frame_.measurement_.ego_pos - slot_center).norm();
      if (distance < car_slot_distance) {
        car_slot_distance = distance;
        frame_.set_seleted_id_mannually = slot.id();
      }
    }
    std::cout << "set_seleted_id_mannually:" << frame_.set_seleted_id_mannually
              << std::endl;
    if (static_cast<int>(frame_.set_seleted_id_mannually) != 0 &&
        frame_.parking_slot_ptr_->select_slot_id() == 0) {
      frame_.is_switch_parking = true;
      auto slot_idx = frame_.slot_info_map_[static_cast<int>(
          frame_.set_seleted_id_mannually)];

      const auto slot_info =
          frame_.slot_info_window_vec_[slot_idx].GetFusedInfo();

      // init limiter position
      if (frame_.limiter_point_window_.IsEmpty()) {
        // add default value
        const double K = apa_param.GetParam().normal_slot_length -
                         apa_param.GetParam().terminal_length;
        Eigen::Vector2d direction_vec;
        direction_vec << slot_info.corner_points().corner_point(3).x() -
                             slot_info.corner_points().corner_point(1).x(),
            slot_info.corner_points().corner_point(3).y() -
                slot_info.corner_points().corner_point(1).y();

        const auto direction_norm = direction_vec.normalized();
        std::pair<Eigen::Vector2d, Eigen::Vector2d> limiter_points;
        limiter_points.first << slot_info.corner_points().corner_point(0).x() +
                                    K * direction_norm.x(),
            slot_info.corner_points().corner_point(0).y() +
                K * direction_norm.y();

        limiter_points.second << slot_info.corner_points().corner_point(1).x() +
                                     K * direction_norm.x(),
            slot_info.corner_points().corner_point(1).y() +
                K * direction_norm.y();

        frame_.limiter_point_window_.Add(limiter_points);
        frame_.limiter_point_window_.Fuse();
        const auto fused_limiter_points =
            frame_.limiter_point_window_.GetFusedLimiterPoints();

        if (frame_.slot_management_info_.limiter_points_size() == 0) {
          auto limiter = frame_.slot_management_info_.add_limiter_points();
          *limiter = fused_limiter_points.first;
          limiter = frame_.slot_management_info_.add_limiter_points();
          *limiter = fused_limiter_points.second;
        } else {
          frame_.slot_management_info_.mutable_limiter_points()->Clear();
          auto limiter = frame_.slot_management_info_.mutable_limiter_points(0);
          limiter->set_x(fused_limiter_points.first.x());
          limiter->set_y(fused_limiter_points.first.y());
          limiter = frame_.slot_management_info_.mutable_limiter_points(1);
          limiter->set_x(fused_limiter_points.second.x());
          limiter->set_y(fused_limiter_points.second.y());
        }
      }
    }
  }
}

}  // namespace planning
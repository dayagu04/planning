#include "slot_management.h"

#include <math.h>

#include <cmath>
#include <cstddef>
#include <utility>

#include "Eigen/Core"
#include "apa_param_setting.h"
#include "basic_types.pb.h"
#include "common.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "dubins_lib.h"
#include "func_state_machine.pb.h"
#include "math_lib.h"
#include "parking_fusion.pb.h"
#include "slot_management_info.pb.h"
#include "transform_lib.h"

namespace planning {

static const double kPie = 3.141592653589793;

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

  bool update_slot_in_searching_flag = false;
  bool update_slot_in_parking_flag = false;
  // update_slot_in_searching_flag is always false, only update slot
  if (IsInSearchingState()) {
    update_slot_in_searching_flag = UpdateSlotsInSearching();
  } else if (IsInParkingState()) {
    update_slot_in_parking_flag = UpdateSlotsInParking();
  }

  // for hmi
  UpdateReleasedSlotInfo();

  // restore slot management info
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_slot_management_info()
      ->CopyFrom(frame_.slot_management_info_);

  return update_slot_in_searching_flag || update_slot_in_parking_flag;
}

bool SlotManagement::IsInAPAState() const {
  if ((frame_.func_state_ptr_->current_state() >=
           FuncStateMachine::PARK_IN_APA_IN &&
       frame_.func_state_ptr_->current_state() <=
           FuncStateMachine::PARK_IN_COMPLETED) ||
      frame_.param_.force_apa_on) {
    std::cout << "apa in at present\n";
    return true;
  }
  return false;
}

void SlotManagement::Reset() { frame_.Reset(); }

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
  if ((frame_.func_state_ptr_->current_state() >=
           FuncStateMachine::PARK_IN_APA_IN &&
       frame_.func_state_ptr_->current_state() <=
           FuncStateMachine::PARK_IN_NO_READY) ||
      (frame_.param_.force_apa_on && (!frame_.param_.is_switch_parking))) {
    return true;
  }
  return false;
}

bool SlotManagement::UpdateSlotsInSearching() {
  std::cout << "apa state is in searching!\n";
  // Update slots
  for (size_t i = 0;
       i < frame_.parking_slot_ptr_->parking_fusion_slot_lists_size(); ++i) {
    const auto &fusion_slot =
        frame_.parking_slot_ptr_->parking_fusion_slot_lists(i);

    common::SlotInfo slot_info;
    if (!ProcessRawSlot(fusion_slot, slot_info)) {
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

  return false;
}

const bool SlotManagement::ProcessRawSlot(
    const ParkingFusion::ParkingFusionSlot &parking_fusion_slot,
    common::SlotInfo &slot_info) {
  slot_info.Clear();
  slot_info = SlotInfoTransfer(parking_fusion_slot);

  if (!slot_info.has_corner_points()) {
    std::cout << "slot doesnot have corner points\n" << std::endl;
    return false;
  }

  // todo: should adapt slanting slot

  // check slot is valid
  if (!IsValidParkingSlot(slot_info)) {
    std::cout << "slot line is not parallel or vertical\n";
    return false;
  }

  // correct slot corner point order
  if (CorrectSlotPointsOrder(slot_info)) {
    frame_.fusion_order_error_cnt_++;
  }

  // make slot more rectangular
  ModifySlot2Rectangle(slot_info);

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
    // the selected slot in parking state is forced to release
    slot_info.set_is_release(true);
    slot_info.set_is_occupied(false);
  }

  return slot_info;
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
  // update by angle between ego_heading_axis and slot_heading_axis (new slot)
  const bool angle_update_condition = AngleUpdateCondition(new_slot_info);

  // update by lon dif between slot center and mirror middle point
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

  return angle_update_condition;
}

bool SlotManagement::IsInParkingState() const {
  if ((frame_.func_state_ptr_->current_state() ==
           FuncStateMachine::PARK_IN_ACTIVATE_WAIT ||
       frame_.func_state_ptr_->current_state() ==
           FuncStateMachine::PARK_IN_ACTIVATE_CONTROL) ||
      (frame_.param_.force_apa_on && frame_.param_.is_switch_parking)) {
    return true;
  }
  return false;
}

bool SlotManagement::UpdateSlotsInParking() {
  std::cout << "apa state is in parking\n";
  if (!frame_.parking_slot_ptr_->has_select_slot_id()) {
    std::cout << "Error: no selected id" << std::endl;
    return false;
  }

  google::protobuf::uint32 select_slot_id =
      frame_.parking_slot_ptr_->select_slot_id();
  if (select_slot_id == 0) {
    std::cout << "select_slot_id = 0, is not valid\n";
    return false;
  }
  std::cout << "select_slot_id:" << select_slot_id << std::endl;

  if (frame_.slot_info_window_vec_.empty() ||
      frame_.slot_info_window_vec_[select_slot_id].IsEmpty()) {
    std::cout << "slot_info_window_vec_ is empty!\n";
    return false;
  }

  ParkingFusion::ParkingFusionSlot select_fusion_slot;
  bool valid_select_slot = false;
  for (size_t i = 0;
       i < frame_.parking_slot_ptr_->parking_fusion_slot_lists_size(); ++i) {
    const auto &fusion_slot =
        frame_.parking_slot_ptr_->parking_fusion_slot_lists(i);
    if (select_slot_id == fusion_slot.id()) {
      select_fusion_slot = fusion_slot;
      if (fusion_slot.type() ==
          Common::ParkingSlotType::PARKING_SLOT_TYPE_VERTICAL) {
        std::cout << "perpendicular slot selected in fusion\n";
      } else if (fusion_slot.type() ==
                 Common::ParkingSlotType::PARKING_SLOT_TYPE_HORIZONTAL) {
        std::cout << "parallel slot selected in fusion\n";
      } else {
        std::cout << "current slot selected is no supported\n";
        break;
      }
      valid_select_slot = true;
      break;
    }
  }

  if (!valid_select_slot) {
    std::cout << "selected slot is invalid!" << std::endl;
    return false;
  }

  common::SlotInfo select_slot;
  if (!ProcessRawSlot(select_fusion_slot, select_slot)) {
    return false;
  }

  if (select_slot.is_release() == false) {
    std::cout << "selected slot is not released!\n";
    return false;
  }

  if (!UpdateEgoSlotInfo(select_slot_id, select_slot, select_fusion_slot)) {
    return false;
  }

  UpdateSlotInfoInParking();

  UpdateLimiterInfoInParking();

  return true;
}

bool SlotManagement::UpdateEgoSlotInfo(
    const google::protobuf::uint32 &select_slot_id,
    const common::SlotInfo &select_slot,
    const ParkingFusion::ParkingFusionSlot &selecte_fusion_slot) {
  auto &ego_slot_info = frame_.ego_slot_info;
  auto &ego_pose_info = frame_.measurement_;

  if (frame_.slot_info_map_.count(select_slot_id) == 0) {
    // selected slot is not found when seaching, should quit
    std::cout << "slot_info_map_ doesnot have the select_slot_id\n";
    return false;
  }

  ego_slot_info.slot_type = selecte_fusion_slot.type();
  ego_slot_info.select_slot_id = select_slot_id;
  ego_slot_info.select_fusion_slot = selecte_fusion_slot;
  ego_slot_info.select_slot = select_slot;

  const size_t slot_idx = frame_.slot_info_map_[select_slot_id];
  ego_slot_info.select_slot_filter =
      frame_.slot_info_window_vec_[slot_idx].GetFusedInfo();

  const auto &slot_points =
      ego_slot_info.select_slot_filter.corner_points().corner_point();

  std::vector<Eigen::Vector2d> pt;
  pt.resize(4);
  for (size_t i = 0; i < 4; ++i) {
    pt[i] << slot_points[i].x(), slot_points[i].y();
  }

  const auto pM01 = 0.5 * (pt[0] + pt[1]);
  const auto pM23 = 0.5 * (pt[2] + pt[3]);
  const auto n = (pM01 - pM23).normalized();
  ego_slot_info.slot_origin_pos =
      pM01 - apa_param.GetParam().normal_slot_length * n;

  ego_slot_info.slot_origin_heading = std::atan2(n.y(), n.x());
  ego_slot_info.slot_origin_heading_vec = n;
  ego_slot_info.slot_length = apa_param.GetParam().normal_slot_length;
  ego_slot_info.slot_width = (pt[0] - pt[1]).norm();

  ego_slot_info.g2l_tf.Init(ego_slot_info.slot_origin_pos,
                            ego_slot_info.slot_origin_heading);

  ego_slot_info.l2g_tf.Init(ego_slot_info.slot_origin_pos,
                            ego_slot_info.slot_origin_heading);

  ego_slot_info.ego_pos_slot =
      ego_slot_info.g2l_tf.GetPos(ego_pose_info.ego_pos);

  ego_slot_info.ego_heading_slot =
      ego_slot_info.g2l_tf.GetHeading(ego_pose_info.heading);

  ego_slot_info.ego_heading_slot_vec
      << std::cos(ego_slot_info.ego_heading_slot),
      std::sin(ego_slot_info.ego_heading_slot);

  // update limiter
  if (!frame_.limiter_point_window_.IsEmpty()) {
    const auto limiter = frame_.limiter_point_window_.GetFusedLimiterPoints();
    ego_slot_info.limiter.first << limiter.first.x(), limiter.first.y();
    ego_slot_info.limiter.second << limiter.second.x(), limiter.second.y();
  }
  // cal target pos
  ego_slot_info.target_ego_pos_slot
      << (ego_slot_info.limiter.first.x() + ego_slot_info.limiter.second.x()) /
             2.0,
      apa_param.GetParam().terminal_target_y;

  ego_slot_info.target_ego_heading_slot =
      apa_param.GetParam().terminal_target_heading;

  // cal terminal err
  ego_slot_info.terminal_err.Set(
      ego_slot_info.ego_pos_slot - ego_slot_info.target_ego_pos_slot,
      ego_slot_info.ego_heading_slot - ego_slot_info.target_ego_heading_slot);

  // cal occupied ratio
  if (std::fabs(ego_slot_info.terminal_err.pos.y()) <
          apa_param.GetParam().slot_occupied_ratio_max_lat_err &&
      std::fabs(ego_slot_info.ego_heading_slot) <
          apa_param.GetParam().slot_occupied_ratio_max_heading_err / 57.3) {
    ego_slot_info.slot_occupied_ratio =
        pnc::mathlib::Clamp(1.0 - (ego_slot_info.terminal_err.pos.x() /
                                   apa_param.GetParam().normal_slot_length),
                            0.0, 1.0);
  } else {
    ego_slot_info.slot_occupied_ratio = 0.0;
  }

  return true;
}

void SlotManagement::UpdateSlotInfoInParking() {
  auto &ego_slot_info = frame_.ego_slot_info;

  bool reset_slot_flag = false;
  bool update_slot_flag = false;

  bool update_slot_condition_1 = IfUpdateSlot(ego_slot_info.select_slot);

  bool update_slot_condition_2 =
      (ego_slot_info.slot_occupied_ratio <
           apa_param.GetParam().slot_update_in_or_out_occupied_ratio &&
       std::fabs(ego_slot_info.ego_heading_slot) <
           apa_param.GetParam().slot_update_out_heading / 57.3 &&
       std::fabs(ego_slot_info.ego_pos_slot.y()) <
           apa_param.GetParam().slot_update_out_lat);

  bool update_slot_condition_3 =
      (ego_slot_info.slot_occupied_ratio >=
           apa_param.GetParam().slot_update_in_or_out_occupied_ratio &&
       std::fabs(ego_slot_info.ego_heading_slot) <
           apa_param.GetParam().slot_update_in_heading / 57.3 &&
       std::fabs(ego_slot_info.ego_pos_slot.y()) <
           apa_param.GetParam().slot_update_in_lat);

  update_slot_flag = update_slot_condition_1 || update_slot_condition_2 ||
                     update_slot_condition_3;

  if (!update_slot_flag) {
    frame_.no_update_slot_count++;
  }

  if (update_slot_flag) {
    if (frame_.no_update_slot_count >
        apa_param.GetParam().slot_reset_threshold) {
      reset_slot_flag = true;
      frame_.no_update_slot_count = 0;
    }
    auto slot_idx = frame_.slot_info_map_[ego_slot_info.select_slot_id];
    if (reset_slot_flag) {
      frame_.slot_info_window_vec_[slot_idx].Reset();
    }
    frame_.slot_info_window_vec_[slot_idx].Add(ego_slot_info.select_slot);
    auto slot = frame_.slot_management_info_.mutable_slot_info_vec(slot_idx);
    *slot = frame_.slot_info_window_vec_[slot_idx].GetFusedInfo();
    ego_slot_info.select_slot_filter =
        frame_.slot_info_window_vec_[slot_idx].GetFusedInfo();
  }
}

void SlotManagement::UpdateLimiterInfoInParking() {
  const auto &ego_slot_info = frame_.ego_slot_info;
  if (frame_.limiter_point_window_.IsEmpty()) {
    std::pair<Eigen::Vector2d, Eigen::Vector2d> limiter_slot =
        std::make_pair(Eigen::Vector2d(apa_param.GetParam().terminal_target_x,
                                       ego_slot_info.slot_width / 2.0),
                       Eigen::Vector2d(apa_param.GetParam().terminal_target_x,
                                       -ego_slot_info.slot_width / 2.0));
    frame_.limiter_point_window_.Add(limiter_slot);
  }

  const bool update_limiter_flag_1 =
      (ego_slot_info.slot_occupied_ratio >=
           apa_param.GetParam().limiter_update_min_occupied_ratio &&
       ego_slot_info.slot_occupied_ratio <=
           apa_param.GetParam().limiter_update_max_occupied_ratio);

  auto current_limiter_slot =
      frame_.limiter_point_window_.GetFusedLimiterPoints();

  Eigen::Vector2d p0(current_limiter_slot.first.x(),
                     current_limiter_slot.first.y());

  Eigen::Vector2d p1(current_limiter_slot.second.x(),
                     current_limiter_slot.second.y());

  const pnc::geometry_lib::LineSegment limiter_line(p0, p1);
  const auto limiter_update_distance_to_car =
      pnc::geometry_lib::CalPoint2LineDist(ego_slot_info.ego_pos_slot,
                                           limiter_line);

  const bool update_limiter_flag_2 =
      (limiter_update_distance_to_car >=
           apa_param.GetParam().limiter_update_distance_to_car &&
       ego_slot_info.slot_occupied_ratio >=
           apa_param.GetParam().limiter_update_occupied_ratio);

  const bool update_limiter_flag =
      (update_limiter_flag_1 || update_limiter_flag_2);

  if (update_limiter_flag) {
    const auto &select_fusion_slot = ego_slot_info.select_fusion_slot;
    const auto &select_slot_filter = ego_slot_info.select_slot_filter;
    std::pair<Eigen::Vector2d, Eigen::Vector2d> limiter_global;
    std::pair<Eigen::Vector2d, Eigen::Vector2d> limiter_slot;
    double move_dist;
    if (select_fusion_slot.limiter_position_size() > 0) {
      // there is limiter in slot
      limiter_global.first << select_fusion_slot.limiter_position(0).x(),
          select_fusion_slot.limiter_position(0).y();

      limiter_global.second << select_fusion_slot.limiter_position(1).x(),
          select_fusion_slot.limiter_position(1).y();

      move_dist = apa_param.GetParam().limiter_move_dist;

    } else {
      // there is no limiter in slot
      limiter_global.first
          << select_slot_filter.corner_points().corner_point(2).x(),
          select_slot_filter.corner_points().corner_point(2).y();

      limiter_global.second
          << select_slot_filter.corner_points().corner_point(3).x(),
          select_slot_filter.corner_points().corner_point(3).y();

      move_dist = apa_param.GetParam().terminal_target_x;
    }

    limiter_slot.first = ego_slot_info.g2l_tf.GetPos(limiter_global.first);
    limiter_slot.second = ego_slot_info.g2l_tf.GetPos(limiter_global.second);
    limiter_slot.first.y() = ego_slot_info.slot_width / 2.0;
    limiter_slot.second.y() = -ego_slot_info.slot_width / 2.0;
    limiter_slot.first.x() += move_dist;
    limiter_slot.second.x() += move_dist;
    frame_.limiter_point_window_.Add(limiter_slot);
  }

  current_limiter_slot = frame_.limiter_point_window_.GetFusedLimiterPoints();
  Eigen::Vector2d current_limiter_slot_left(current_limiter_slot.first.x(),
                                            current_limiter_slot.first.y());

  Eigen::Vector2d current_limiter_slot_right(current_limiter_slot.second.x(),
                                             current_limiter_slot.second.y());

  Eigen::Vector2d current_limiter_global_left =
      ego_slot_info.l2g_tf.GetPos(current_limiter_slot_left);

  Eigen::Vector2d current_limiter_global_right =
      ego_slot_info.l2g_tf.GetPos(current_limiter_slot_right);

  common::Point2d current_limiter_global_left_p;
  current_limiter_global_left_p.set_x(current_limiter_global_left.x());
  current_limiter_global_left_p.set_y(current_limiter_global_left.y());
  common::Point2d current_limiter_global_right_p;
  current_limiter_global_right_p.set_x(current_limiter_global_right.x());
  current_limiter_global_right_p.set_y(current_limiter_global_right.y());

  if (frame_.slot_management_info_.limiter_points_size() == 0) {
    auto limiter = frame_.slot_management_info_.add_limiter_points();
    *limiter = current_limiter_global_left_p;
    limiter = frame_.slot_management_info_.add_limiter_points();
    *limiter = current_limiter_global_right_p;
  } else {
    auto limiter = frame_.slot_management_info_.mutable_limiter_points(0);
    limiter->set_x(current_limiter_global_left_p.x());
    limiter->set_y(current_limiter_global_left_p.y());
    limiter = frame_.slot_management_info_.mutable_limiter_points(1);
    limiter->set_x(current_limiter_global_right_p.x());
    limiter->set_y(current_limiter_global_right_p.y());
  }
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
  select_slot_id = frame_.parking_slot_ptr_->select_slot_id();
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

}  // namespace planning
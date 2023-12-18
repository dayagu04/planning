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
#include "slot_management_info.pb.h"
#include "transform_lib.h"

// #define __DEBUG_PRINT__
namespace planning {

static const double kPie = 3.141592653589793;
static const double kMinSlotUpdateOccupiedRatio = 0.6;

static const double kMaxSlotUpdateFixedOrientationCos = 0.866;  // cos(30°)
static const double kFixSlotUpdateOccupiedRatio = 0.85;

static const double KMinLimiterUpdateOccupiedRatio = 0.4;
static const double KMaxLimiterUpdateOccupiedRatio = 0.6;
static const double KLimiterUpdateOccupiedRatio = 0.6;

static const double SlotLength = 4.8;
static const double TerminalLength = 1.2;
static const double ResDistance = 1.7;
static const double ResLimiter = 0.55;

// uss data
static const std::vector<double> uss_vertex_x_vec = {
    3.187342,  3.424531,  3.593071,  3.593071,  3.424531,  3.187342,
    -0.476357, -0.798324, -0.879389, -0.879389, -0.798324, -0.476357};

static const std::vector<double> uss_vertex_y_vec = {
    0.887956,  0.681712,  0.334651,  -0.334651, -0.681712, -0.887956,
    -0.887956, -0.706505, -0.334845, 0.334845,  0.706505,  0.887956};

static const std::vector<double> uss_normal_angle_deg_vec = {
    170.0, 130.0, 92.0,  88.0,  50.0,  8.0,
    352.0, 298.0, 275.0, 264.0, 242.0, 187.0};

static const std::vector<size_t> wdis_index_front = {0, 9, 6, 3, 1, 11};
static const std::vector<size_t> wdis_index_back = {0, 1, 3, 6, 9, 11};

static const Eigen::Vector2d uss0_vertex(uss_vertex_x_vec[0],
                                         uss_vertex_y_vec[0]);

static const Eigen::Vector2d uss5_vertex(uss_vertex_x_vec[5],
                                         uss_vertex_y_vec[5]);

static const Eigen::Vector2d uss6_vertex(uss_vertex_x_vec[6],
                                         uss_vertex_y_vec[6]);

static const Eigen::Vector2d uss11_vertex(uss_vertex_x_vec[11],
                                          uss_vertex_y_vec[11]);

static const size_t MAX_OCCUPIED_COUNT = 4;

void SlotManagement::Reset() {
  // reset slot in
  slot_management_info_.Clear();
  slot_info_window_vec_.clear();

  // reset LimiterPointWindow
  limiter_point_window_.Reset();

  // reset slot_info_map
  slot_info_map_.clear();
  slot_info_map_.reserve(param_.max_slot_size);
  set_seleted_id_mannually = 0;

  slot_occupied_ratio_ = 0.0;
  fusion_order_error_cnt_ = 0;
  is_occupied_ = false;
  is_switch_parking = false;

  // reset slot_management_info_
  occupied_info_map_.clear();
  occupied_info_map_.reserve(param_.max_slot_size);

  is_fixed_ = false;
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
  func_state_ptr_ = func_statemachine;
  parking_slot_ptr_ = parking_slot_info;
  localization_ptr_ = localization_info;
  uss_wave_info_ptr_ = uss_wave_info;

  if (!IsInAPAState() || param_.force_clear) {
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

  // std::cout << "update_searching_flag: " << update_searching_flag <<
  // std::endl; std::cout << "update_occupied_flag: " << update_occupied_flag <<
  // std::endl;

  // ReleaseSlots();

  // for hmi
  UpdateReleasedSlotInfo();

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

const bool SlotManagement::GetSelectedSlot(common::SlotInfo &slot_info,
                                           const int selected_id) const {
  if (slot_info_map_.count(selected_id) == 0) {
    return false;
  } else {
    slot_info =
        slot_management_info_.slot_info_vec(slot_info_map_.at(selected_id));
    return true;
  }
}

const bool SlotManagement::GetSelectedSlot(common::SlotInfo &slot_info) const {
  if (parking_slot_ptr_->has_select_slot_id()) {
    const auto &selected_id = parking_slot_ptr_->select_slot_id();
    if (slot_info_map_.count(selected_id) == 0) {
      return false;
    } else {
      slot_info =
          slot_management_info_.slot_info_vec(slot_info_map_.at(selected_id));
      return true;
    }
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

  if (IsInSearchingState()) {
    slot_info.set_is_release((fusion_slot.allow_parking() == 1));
    slot_info.set_is_occupied((fusion_slot.allow_parking() == 0));
  }

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

    bool correct_slot_order = CorrectSlotPointsOrder(slot_info);
    if (correct_slot_order) {
      fusion_order_error_cnt_++;
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

  // set seleted_slot manually
  // double car_slot_distance = 1e4;
  if (using_auto_switcher) {
    double car_slot_distance = 0.0;
    for (auto slot : slot_management_info_.slot_info_vec()) {
      Eigen::Vector2d slot_center;
      slot_center.x() = slot.center().x();
      slot_center.y() = slot.center().y();
      const auto distance = (measurement_.ego_pos - slot_center).norm();
      if (distance < car_slot_distance) {
        car_slot_distance = distance;
        set_seleted_id_mannually = slot.id();
      }
    }
    std::cout << "set_seleted_id_mannually:" << set_seleted_id_mannually
              << std::endl;
    if (static_cast<int>(set_seleted_id_mannually) != 0 &&
        parking_slot_ptr_->select_slot_id() == 0) {
      is_switch_parking = true;
      auto slot_idx =
          slot_info_map_[static_cast<int>(set_seleted_id_mannually)];

      const auto slot_info = slot_info_window_vec_[slot_idx].GetFusedInfo();

      // init limiter position
      if (limiter_point_window_.IsEmpty()) {
        // add default value
        const double K = SlotLength - TerminalLength;
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

        limiter_point_window_.Add(limiter_points);
        limiter_point_window_.Fuse();
        const auto fused_limiter_points =
            limiter_point_window_.GetFusedLimiterPoints();

        if (slot_management_info_.limiter_points_size() == 0) {
          auto limiter = slot_management_info_.add_limiter_points();
          *limiter = fused_limiter_points.first;
          limiter = slot_management_info_.add_limiter_points();
          *limiter = fused_limiter_points.second;
        } else {
          slot_management_info_.mutable_limiter_points()->Clear();
          auto limiter = slot_management_info_.mutable_limiter_points(0);
          limiter->set_x(fused_limiter_points.first.x());
          limiter->set_y(fused_limiter_points.first.y());
          limiter = slot_management_info_.mutable_limiter_points(1);
          limiter->set_x(fused_limiter_points.second.x());
          limiter->set_y(fused_limiter_points.second.y());
        }
      }
    }
  }
  return true;
}

const bool SlotManagement::SetRealtime() {
  google::protobuf::uint32 select_slot_id = 0;
  if (parking_slot_ptr_->select_slot_id() != 0) {
    select_slot_id = parking_slot_ptr_->select_slot_id();
  } else {
    select_slot_id = set_seleted_id_mannually;
  }
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
    is_occupied_ = false;
    return false;
  }

  const bool is_slot_valid = IsValidParkingSlot(select_slot);
  if (!is_slot_valid) {
    std::cout << "fusion slot is not valid" << std::endl;
    is_occupied_ = false;
    return false;
  }

  CorrectSlotPointsOrder(select_slot);

  // update slot
  auto slot_idx = slot_info_map_[select_slot_id];
  auto slot = slot_management_info_.mutable_slot_info_vec(slot_idx);
  slot_info_window_vec_[slot_idx].Reset();
  slot_info_window_vec_[slot_idx].Add(select_slot);
  *slot = slot_info_window_vec_[slot_idx].GetFusedInfo();

  return true;
}

bool SlotManagement::UpdateSlotsInParking() {
  google::protobuf::uint32 select_slot_id = 0;
  if (parking_slot_ptr_->select_slot_id() != 0) {
    select_slot_id = parking_slot_ptr_->select_slot_id();
  } else {
    select_slot_id = set_seleted_id_mannually;
  }

  common::SlotInfo select_slot;

  size_t selected_fusin_slot_index = 0;
  for (int i = 0; i < parking_slot_ptr_->parking_fusion_slot_lists_size();
       ++i) {
    const auto &fusion_slot = parking_slot_ptr_->parking_fusion_slot_lists(i);
    if (select_slot_id == fusion_slot.id()) {
      selected_fusin_slot_index = i;
      select_slot = SlotInfoTransfer(fusion_slot);
      break;
    }
  }

  if (!select_slot.has_corner_points()) {
    std::cout << "find no select slot" << std::endl;
    is_occupied_ = false;
    return false;
  }

  const bool is_slot_valid = IsValidParkingSlot(select_slot);
  if (!is_slot_valid) {
    std::cout << "fusion slot is not valid" << std::endl;
    is_occupied_ = false;
    return false;
  }

  bool correct_slot_order = CorrectSlotPointsOrder(select_slot);
  if (correct_slot_order) {
    fusion_order_error_cnt_++;
  }

  // calculate occupied ratio
  slot_occupied_ratio_ = CalOccupiedRatio();

  std::cout << "occupied_ratio in slm: " << slot_occupied_ratio_ << std::endl;

  std::pair<Eigen::Vector2d, Eigen::Vector2d> limiter_points;
  // smoothing limiter position
  if (limiter_point_window_.IsEmpty()) {
    // add default value
    const double K = SlotLength - TerminalLength;
    Eigen::Vector2d direction_vec;
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

    limiter_point_window_.Add(limiter_points);
  }

  const bool update_limiter_flag_low =
      slot_occupied_ratio_ <= KMaxLimiterUpdateOccupiedRatio &&
      slot_occupied_ratio_ >= KMinLimiterUpdateOccupiedRatio;

  limiter_point_window_.Fuse();
  const auto limiter_points_line =
      limiter_point_window_.GetFusedLimiterPoints();

  Eigen::Vector2d p0;
  Eigen::Vector2d p1;
  p0 << limiter_points_line.first.x(), limiter_points_line.first.y();
  p1 << limiter_points_line.second.x(), limiter_points_line.second.y();
  const pnc::geometry_lib::LineSegment limiter_line(p0, p1);
  const auto res_distance =
      pnc::geometry_lib::CalPoint2LineDist(measurement_.ego_pos, limiter_line);
  const bool update_limiter_flag_upper =
      res_distance >= ResDistance &&
      slot_occupied_ratio_ >= KLimiterUpdateOccupiedRatio;

  if (update_limiter_flag_low || update_limiter_flag_upper) {
    // add realtime value
    const auto selected_fusion_slot =
        parking_slot_ptr_->parking_fusion_slot_lists(selected_fusin_slot_index);

    if (selected_fusion_slot.limiter_position_size() > 0) {
      Eigen::Vector2d p0(selected_fusion_slot.limiter_position(0).x(),
                         selected_fusion_slot.limiter_position(0).y());

      Eigen::Vector2d p1(selected_fusion_slot.limiter_position(1).x(),
                         selected_fusion_slot.limiter_position(1).y());

      const Eigen::Vector2d heading_norm(std::cos(measurement_.heading),
                                         std::sin(measurement_.heading));

      const Eigen::Vector2d delta = ResLimiter * heading_norm;
      limiter_points.first = p0 + delta;
      limiter_points.second = p1 + delta;
      limiter_point_window_.Add(limiter_points);
    }
  }

  limiter_point_window_.Fuse();
  const auto fused_limiter_points =
      limiter_point_window_.GetFusedLimiterPoints();

  if (slot_management_info_.limiter_points_size() == 0) {
    auto limiter = slot_management_info_.add_limiter_points();
    *limiter = fused_limiter_points.first;
    limiter = slot_management_info_.add_limiter_points();
    *limiter = fused_limiter_points.second;
  } else {
    auto limiter = slot_management_info_.mutable_limiter_points(0);
    limiter->set_x(fused_limiter_points.first.x());
    limiter->set_y(fused_limiter_points.first.y());
    limiter = slot_management_info_.mutable_limiter_points(1);
    limiter->set_x(fused_limiter_points.second.x());
    limiter->set_y(fused_limiter_points.second.y());
  }

  // if occupied percentage is less than certain value,return
  bool is_occupied = false;
  if (slot_occupied_ratio_ < kMinSlotUpdateOccupiedRatio) {
    is_occupied = false;
    is_occupied_ = is_occupied;
    // std::cout << "occupied_ratio is less than min update value " <<
    // std::endl;
    return false;
  } else {
    is_occupied = true;
  }

  // update slot_management_info
  const Eigen::Vector2d mid_limiters_point(
      fused_limiter_points.first.x() + 0.5 * (fused_limiter_points.second.x() -
                                              fused_limiter_points.first.x()),
      fused_limiter_points.first.y() + 0.5 * (fused_limiter_points.second.y() -
                                              fused_limiter_points.first.y()));

  const auto car_limiter_distance =
      (measurement_.ego_pos - mid_limiters_point).norm();

  const Eigen::Vector2d car_heading_norm(std::cos(measurement_.heading),
                                         std::sin(measurement_.heading));

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
      (slot_occupied_ratio_ >= kMinSlotUpdateOccupiedRatio) &&
      (slot_car_heading_cos >= kMaxSlotUpdateFixedOrientationCos);

  is_fixed_ = is_fixed_ || (slot_occupied_ratio_ > kFixSlotUpdateOccupiedRatio);

  // std::cout << "car_limiter_distance:" << car_limiter_distance << "\n";
  // std::cout << "slot_car_heading_cos:" << slot_car_heading_cos << "\n";
  // std::cout << "is_update_slot:" << is_update_slot << "\n";

  if (is_update_slot && !is_fixed_) {
    std::cout << "update slot last time\n";
    // update slot
    auto slot_idx = slot_info_map_[select_slot_id];
    auto slot = slot_management_info_.mutable_slot_info_vec(slot_idx);

    // reset when latest occupied
    if ((is_occupied_ == false) && (is_occupied == true)) {
      slot_info_window_vec_[slot_idx].Reset();
    }

    slot_info_window_vec_[slot_idx].Add(select_slot);
    *slot = slot_info_window_vec_[slot_idx].GetFusedInfo();
  }

  is_occupied_ = is_occupied;

  return true;
}

double SlotManagement::CalOccupiedRatio() const {
  Eigen::Vector2d target_pt = Eigen::Vector2d::Zero();
  Eigen::Vector2d middle_pt_01 = Eigen::Vector2d::Zero();
  Eigen::Vector2d middle_pt_23 = Eigen::Vector2d::Zero();
  Eigen::Vector2d slot_heading_unit_vec = Eigen::Vector2d::Zero();

  google::protobuf::uint32 select_slot_id = 0;
  if (parking_slot_ptr_->select_slot_id() != 0) {
    select_slot_id = parking_slot_ptr_->select_slot_id();
  } else {
    select_slot_id = set_seleted_id_mannually;
  }
  for (const auto &fusion_slot :
       parking_slot_ptr_->parking_fusion_slot_lists()) {
    if (select_slot_id != fusion_slot.id()) {
      continue;
    }

    middle_pt_01 << (fusion_slot.corner_points(0).x() +
                     fusion_slot.corner_points(1).x()) *
                        0.5,
        (fusion_slot.corner_points(0).y() + fusion_slot.corner_points(1).y()) *
            0.5;

    middle_pt_23 << (fusion_slot.corner_points(2).x() +
                     fusion_slot.corner_points(3).x()) *
                        0.5,
        (fusion_slot.corner_points(2).y() + fusion_slot.corner_points(3).y()) *
            0.5;

    slot_heading_unit_vec = middle_pt_01 - middle_pt_23;
    slot_heading_unit_vec.normalize();

    const double dst_rear_edge_to_rear_axle = 0.947;
    const double buffer = 0.1;
    target_pt = middle_pt_23 +
                (dst_rear_edge_to_rear_axle + buffer) * slot_heading_unit_vec;
  }

  // std::cout << "current pos: " << measurement_.ego_pos << std::endl;
  // std::cout << "target pos: " << target_pt << std::endl;

  Eigen::Vector2d slot_01_middle_to_ego_pos =
      measurement_.ego_pos - middle_pt_01;

  Eigen::Vector2d slot_01_middle_to_targt_pos = target_pt - middle_pt_01;

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
      (param_.force_apa_on && (!is_switch_parking))) {
    std::cout << "apa searching on!" << std::endl;
    return true;
  } else {
    std::cout << "apa searching off!" << std::endl;
    return false;
  }
}

bool SlotManagement::IsInParkingState() const {
  return (func_state_ptr_->current_state() ==
          FuncStateMachine::PARK_IN_ACTIVATE_CONTROL) ||
         is_switch_parking;
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
      std::fabs(new_slot_heading_unit(0) * slot_center_to_side_mirror_vec(1) -
                new_slot_heading_unit(1) * slot_center_to_side_mirror_vec(0));

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

void SlotManagement::UpdateReleasedSlotInfo() {
  released_slot_info_vec_.clear();
  PlanningOutput::SuccessfulSlotsInfo released_slot_info;
  for (const auto &slot_info : slot_management_info_.slot_info_vec()) {
    if (slot_info.is_release()) {
      released_slot_info.Clear();
      released_slot_info.set_id(slot_info.id());
      released_slot_info_vec_.emplace_back(released_slot_info);
    }
  }
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

void SlotManagement::UpdateOccupiedInfo() {
  // process uss raw data
  const auto &upa_dis_info_buf = uss_wave_info_ptr_->upa_dis_info_buf();

  uss_raw_dist_vec_.clear();
  uss_raw_dist_vec_.reserve(uss_vertex_x_vec.size());

  if (upa_dis_info_buf.size() < 2) {
    return;
  }

  // front uss
  for (size_t i = 0; i < wdis_index_front.size(); ++i) {
    uss_raw_dist_vec_.emplace_back(
        upa_dis_info_buf[0].wdis(wdis_index_front[i]).wdis_value(0));
  }

  // back uss
  for (size_t i = 0; i < wdis_index_back.size(); ++i) {
    uss_raw_dist_vec_.emplace_back(
        upa_dis_info_buf[1].wdis(wdis_index_back[i]).wdis_value(0));
  }

  // get uss 0 and 5 radius
  const double radius0 = uss_raw_dist_vec_[0];
  const double radius5 = uss_raw_dist_vec_[5];
  const double radius6 = uss_raw_dist_vec_[6];
  const double radius11 = uss_raw_dist_vec_[11];
  // transform to global coordinate
  const auto oreintation0 =
      pnc::mathlib::Deg2Rad(uss_normal_angle_deg_vec[0] - 90.0);

  const auto oreintation5 =
      pnc::mathlib::Deg2Rad(uss_normal_angle_deg_vec[5] - 90.0);

  const auto oreintation6 =
      pnc::mathlib::Deg2Rad(uss_normal_angle_deg_vec[6] - 90.0);

  const auto oreintation11 =
      pnc::mathlib::Deg2Rad(uss_normal_angle_deg_vec[11] - 90.0);

  const Eigen::Vector2d uss0_delta(radius0 * std::cos(oreintation0),
                                   radius0 * std::sin(oreintation0));

  const Eigen::Vector2d uss5_delta(radius5 * std::cos(oreintation5),
                                   radius5 * std::sin(oreintation5));

  const Eigen::Vector2d uss6_delta(radius6 * std::cos(oreintation6),
                                   radius6 * std::sin(oreintation6));

  const Eigen::Vector2d uss11_delta(radius11 * std::cos(oreintation11),
                                    radius11 * std::sin(oreintation11));

  const auto uss0_local_pos = uss0_vertex + uss0_delta;
  const auto uss5_local_pos = uss5_vertex + uss5_delta;
  const auto uss6_local_pos = uss6_vertex + uss6_delta;
  const auto uss11_local_pos = uss11_vertex + uss11_delta;

  const auto car_global = pnc::geometry_lib::LocalToGlobalTf(
      measurement_.ego_pos, measurement_.heading);

  const auto uss0_global = car_global.GetPos(uss0_local_pos);
  const auto uss5_global = car_global.GetPos(uss5_local_pos);
  const auto uss6_global = car_global.GetPos(uss6_local_pos);
  const auto uss11_global = car_global.GetPos(uss11_local_pos);

  // for each slot judge if be occupied
  // count satisfied max_count  using flag to make sure start count
  for (auto &slot_info : *slot_management_info_.mutable_slot_info_vec()) {
    std::cout << slot_info.id() << "  is_occupied:" << slot_info.is_occupied()
              << "\n";

    if (!IsInSearchingState()) {
      break;
    }

    if (occupied_info_map_.count(slot_info.id()) > 0 &&
        occupied_info_map_[slot_info.id()].is_occupied) {
      continue;
    }

    Eigen::Vector2d slot_coner0;
    Eigen::Vector2d slot_coner1;
    Eigen::Vector2d slot_coner2;
    Eigen::Vector2d slot_coner3;

    slot_coner0 << slot_info.corner_points().corner_point(0).x(),
        slot_info.corner_points().corner_point(0).y();

    slot_coner1 << slot_info.corner_points().corner_point(1).x(),
        slot_info.corner_points().corner_point(1).y();

    slot_coner2 << slot_info.corner_points().corner_point(2).x(),
        slot_info.corner_points().corner_point(2).y();

    slot_coner3 << slot_info.corner_points().corner_point(3).x(),
        slot_info.corner_points().corner_point(3).y();

    const auto slot_invade_coner2 =
        slot_coner0 + param_.invade_area_ratio * (slot_coner2 - slot_coner0);

    const auto slot_invade_coner3 =
        slot_coner1 + param_.invade_area_ratio * (slot_coner3 - slot_coner1);

    std::vector<Eigen::Vector2d> polygon;
    polygon.reserve(4);
    polygon.emplace_back(slot_coner0);
    polygon.emplace_back(slot_coner1);
    polygon.emplace_back(slot_invade_coner3);
    polygon.emplace_back(slot_invade_coner2);

    const bool is_invade =
        (IsPointInPolygon(polygon, uss0_global) && radius0 < 3.5) ||
        (IsPointInPolygon(polygon, uss5_global) && radius5 < 3.5) ||
        (IsPointInPolygon(polygon, uss6_global) && radius6 < 3.5) ||
        (IsPointInPolygon(polygon, uss11_global) && radius11 < 3.5);

    if (occupied_info_map_.count(slot_info.id()) == 0) {
      occupied_info_map_.insert(
          std::make_pair(slot_info.id(), OccupiedInfo(0, false)));
    }

    if (is_invade) {
      // std::cout << slot_info.id() << " is_invade\n";
      occupied_info_map_[slot_info.id()].count++;
      if (occupied_info_map_[slot_info.id()].count >= MAX_OCCUPIED_COUNT) {
        // modify  SlotInfo : is_occupied
        occupied_info_map_[slot_info.id()].is_occupied = true;
      }
    }
  }

  // set occupied info
  for (auto &slot_info : *slot_management_info_.mutable_slot_info_vec()) {
    slot_info.set_is_occupied(occupied_info_map_[slot_info.id()].is_occupied);

    common::SlotInfo slot;
    if (GetSelectedSlot(slot)) {
      if (slot_info.id() == slot.id()) {
        slot_info.set_is_occupied(false);
      }
    }
  }
}

const bool SlotManagement::IsPointInPolygon(
    const std::vector<Eigen::Vector2d> &polygon,
    const Eigen::Vector2d &point) const {
  int crossProductSign = 0;

  for (size_t i = 0; i < polygon.size(); i++) {
    Eigen::Vector2d vertex1 = polygon[i];
    Eigen::Vector2d vertex2 = polygon[(i + 1) % polygon.size()];

    const double crossProduct =
        (vertex2.x() - vertex1.x()) * (point.y() - vertex1.y()) -
        (point.x() - vertex1.x()) * (vertex2.y() - vertex1.y());

    if (std::fabs(crossProduct) < 1.0e-4) {
      return true;
    } else if (crossProductSign == 0) {
      crossProductSign = (crossProduct > 0) ? 1 : -1;
    } else if (crossProductSign != ((crossProduct > 0.0) ? 1 : -1)) {
      return false;
    }
  }

  return true;
}

}  // namespace planning
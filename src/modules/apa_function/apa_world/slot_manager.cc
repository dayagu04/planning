#include "slot_manager.h"

#include <bits/stdint-uintn.h>
#include <math.h>
#include <sys/types.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Eigen/Core"

#include "apa_param_config.h"
#include "apa_state_machine_manager.h"
#include "basic_types.pb.h"
#include "camera_preception_groundline_c.h"
#include "common.h"
#include "common.pb.h"
#include "common_c.h"
#include "common_platform_type_soc.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "dubins_lib.h"
#include "func_state_machine_c.h"
#include "fusion_objects_c.h"
#include "fusion_parking_slot_c.h"
#include "geometry_math.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "math_lib.h"
#include "slot_management_info.pb.h"
#include "transform_lib.h"

namespace planning {

namespace apa_planner {
namespace {
constexpr double kPie = 3.141592653589793;
}  // namespace

bool SlotManager::Update(
    const LocalView *local_view,
    const std::shared_ptr<ApaStateMachineManager> state_machine_ptr,
    const std::shared_ptr<ApaMeasureDataManager> measure_data_ptr,
    const std::shared_ptr<ApaObstacleManager> obstacle_manager_ptr) {
  ILOG_INFO << "Update old slot management, should use new slot manager, old "
               "should delete as soon as possible";
  // set input
  frame_.parking_slot_ptr = &local_view->parking_fusion_info;

  state_machine_ptr_ = state_machine_ptr;
  measure_data_ptr_ = measure_data_ptr;
  obstacle_manager_ptr_ = obstacle_manager_ptr;

  bool update_slot_in_searching_flag = false;
  bool update_slot_in_parking_flag = false;
  // update_slot_in_searching_flag is always false, only update slot
  if (state_machine_ptr_->IsSeachingStatus()) {
    update_slot_in_searching_flag = UpdateSlotsInSearching();
  } else if (state_machine_ptr_->IsParkingStatus()) {
    update_slot_in_parking_flag = UpdateSlotsInParking();
  } else if (state_machine_ptr_->GetStateMachine() ==
             ApaStateMachine::SUSPEND) {
  } else {
    Reset();
  }

  CopySlotReleaseInfo();

  return update_slot_in_searching_flag || update_slot_in_parking_flag;
}

void SlotManager::Reset() { frame_.Reset(); }

bool SlotManager::UpdateSlotsInSearching() {
  ILOG_INFO << "apa state is in searching!";
  // Update slots
  std::unordered_map<size_t, iflyauto::ParkingFusionSlot> fusion_slot_map;
  for (uint32 i = 0;
       i < frame_.parking_slot_ptr->parking_fusion_slot_lists_size; ++i) {
    const auto &fusion_slot =
        frame_.parking_slot_ptr->parking_fusion_slot_lists[i];
    fusion_slot_map[fusion_slot.id] = fusion_slot;
    common::SlotInfo slot_info;
    if (!ProcessRawSlot(fusion_slot, slot_info)) {
      continue;
    }
    const auto fusion_slot_source_type = fusion_slot.fusion_source;
    if (frame_.slot_info_window_map.count(slot_info.id()) == 0) {  // get new id
      if (LonDifUpdateCondition(slot_info, fusion_slot_source_type)) {
        SlotInfoWindow slot_info_window;
        slot_info_window.Add(slot_info);
        frame_.slot_info_window_map.insert(
            std::make_pair((slot_info.id()), slot_info_window));
      }
    } else {  // get old id
      // slot update strategy
      if (IfUpdateSlot(slot_info, fusion_slot_source_type)) {
        frame_.slot_info_window_map[slot_info.id()].Add(slot_info);
      }
    }
  }

  // delete slot in window map when the slot is not exist in fusion slot
  std::vector<size_t> del_id_vec;
  for (const auto &pair : frame_.slot_info_window_map) {
    if (fusion_slot_map.count(pair.first) == 0) {
      del_id_vec.emplace_back(pair.first);
    }
  }
  for (const size_t &id : del_id_vec) {
    frame_.slot_info_angle.erase(id);
    frame_.slot_info_direction.erase(id);
    frame_.slot_info_corner_01.erase(id);
    frame_.slot_info_window_map.erase(id);
  }

  // 点击了车位,更新车位基本信息
  if (!frame_.slot_info_window_map.empty() &&
      !frame_.slot_info_window_map[frame_.parking_slot_ptr->select_slot_id]
           .IsEmpty()) {
    UpdateSlotsInParking();
    ILOG_INFO << "try park planning";
  }

  return false;
}

const bool SlotManager::ProcessRawSlot(
    const iflyauto::ParkingFusionSlot &parking_fusion_slot,
    common::SlotInfo &slot_info) {
  slot_info.Clear();
  // transform perception slot to planning slot.
  if (!SlotInfoTransfer(parking_fusion_slot, slot_info)) {
    ILOG_INFO << "fusion slot is err";
    return false;
  }

  if (!slot_info.has_corner_points()) {
    ILOG_INFO << "slot doesnot have corner points";
    return false;
  }

  // check slot line position.
  if (!IsValidParkingSlot(slot_info)) {
    ILOG_INFO << "slot line is not parallel or vertical";
    return false;
  }

  // correct slot corner point order
  if (CorrectSlotPointsOrder(slot_info)) {
    frame_.fusion_order_error_cnt++;
  }

  // if slot type is slant, should postprocess to vertical slot
  ProcessSlantSlot(slot_info, parking_fusion_slot);

  // make slot more rectangular
  ModifySlot2Rectangle(slot_info);

  return true;
}

const bool SlotManager::ProcessSlantSlot(
    common::SlotInfo &slot_info,
    const iflyauto::ParkingFusionSlot &parking_fusion_slot) {
  if (slot_info.slot_type() != Common::PARKING_SLOT_TYPE_SLANTING) {
    return false;
  }
  ILOG_INFO << "slant slot, should postprocess corner to vertical";
  Eigen::Vector2d slot_pt_0(slot_info.corner_points().corner_point(0).x(),
                            slot_info.corner_points().corner_point(0).y());

  Eigen::Vector2d slot_pt_1(slot_info.corner_points().corner_point(1).x(),
                            slot_info.corner_points().corner_point(1).y());

  Eigen::Vector2d slot_pt_2(slot_info.corner_points().corner_point(2).x(),
                            slot_info.corner_points().corner_point(2).y());

  Eigen::Vector2d slot_pt_3(slot_info.corner_points().corner_point(3).x(),
                            slot_info.corner_points().corner_point(3).y());

  frame_.slot_info_corner_01[slot_info.id()] =
      std::make_pair(slot_pt_0, slot_pt_1);

  const Eigen::Vector2d pt_01_vec = slot_pt_1 - slot_pt_0;
  const Eigen::Vector2d pt_01_unit_vec = pt_01_vec.normalized();
  const Eigen::Vector2d pt_02_vec = slot_pt_2 - slot_pt_0;
  const Eigen::Vector2d pt_02_unit_vec = pt_02_vec.normalized();
  const Eigen::Vector2d pt_13_vec = slot_pt_3 - slot_pt_1;
  const Eigen::Vector2d pt_13_unit_vec = pt_13_vec.normalized();

  const double cos_theta = pt_01_unit_vec.dot(pt_02_unit_vec);

  if (cos_theta > 0.0) {
    // toward right
    const double dis_0_0dot = pt_01_vec.dot(pt_02_unit_vec);
    const Eigen::Vector2d pt_0dot = slot_pt_0 + dis_0_0dot * pt_02_unit_vec;
    const double dist_0dot_2 = pt_02_vec.norm() - dis_0_0dot;
    const Eigen::Vector2d pt_3dot = slot_pt_1 + dist_0dot_2 * pt_02_unit_vec;
    slot_info.mutable_corner_points()->mutable_corner_point(0)->set_x(
        pt_0dot.x());
    slot_info.mutable_corner_points()->mutable_corner_point(0)->set_y(
        pt_0dot.y());
    slot_info.mutable_corner_points()->mutable_corner_point(3)->set_x(
        pt_3dot.x());
    slot_info.mutable_corner_points()->mutable_corner_point(3)->set_y(
        pt_3dot.y());
  } else {
    // toward left
    const Eigen::Vector2d pt_10_vec = -pt_01_vec;
    const double dist_1_1dot = pt_10_vec.dot(pt_13_unit_vec);
    const Eigen::Vector2d pt_1dot = slot_pt_1 + dist_1_1dot * pt_13_unit_vec;
    const double dist_1dot_3 = pt_13_vec.norm() - dist_1_1dot;
    const Eigen::Vector2d pt_2dot = slot_pt_0 + dist_1dot_3 * pt_13_unit_vec;
    slot_info.mutable_corner_points()->mutable_corner_point(1)->set_x(
        pt_1dot.x());
    slot_info.mutable_corner_points()->mutable_corner_point(1)->set_y(
        pt_1dot.y());
    slot_info.mutable_corner_points()->mutable_corner_point(2)->set_x(
        pt_2dot.x());
    slot_info.mutable_corner_points()->mutable_corner_point(2)->set_y(
        pt_2dot.y());
  }

  double accumulated_x = 0.0;
  double accumulated_y = 0.0;
  for (size_t i = 0; i < 4; ++i) {
    accumulated_x += slot_info.corner_points().corner_point(i).x();
    accumulated_y += slot_info.corner_points().corner_point(i).y();
  }

  slot_info.mutable_center()->set_x(accumulated_x / 4.0);
  slot_info.mutable_center()->set_y(accumulated_y / 4.0);

  // cal slot angle
  // get origin slant slot info
  const Eigen::Vector2d origin_pt_01_vec = pt_01_vec;

  slot_pt_0 << slot_info.corner_points().corner_point(0).x(),
      slot_info.corner_points().corner_point(0).y();
  slot_pt_1 << slot_info.corner_points().corner_point(1).x(),
      slot_info.corner_points().corner_point(1).y();
  slot_pt_2 << slot_info.corner_points().corner_point(2).x(),
      slot_info.corner_points().corner_point(2).y();
  slot_pt_3 << slot_info.corner_points().corner_point(3).x(),
      slot_info.corner_points().corner_point(3).y();

  const Eigen::Vector2d pt_23mid_01_mid =
      (slot_pt_0 + slot_pt_1 - slot_pt_2 - slot_pt_3) * 0.5;

  double angle = std::fabs(pnc::geometry_lib::GetAngleFromTwoVec(
                     pt_23mid_01_mid, origin_pt_01_vec)) *
                 57.3;

  if (angle > 90.0) {
    angle = 180.0 - angle;
  }
  angle = pnc::mathlib::DoubleConstrain(angle, 10.0, 80.0);
  const double sin_angle = std::sin(angle * kDeg2Rad);
  frame_.slot_info_angle[slot_info.id()] = std::make_pair(angle, sin_angle);

  const Eigen::Vector2d slot_heading_vec = pt_23mid_01_mid;
  const Eigen::Vector2d ego_heading_vec = measure_data_ptr_->GetHeadingVec();
  const Eigen::Vector2d slot_center =
      Eigen::Vector2d(slot_info.center().x(), slot_info.center().y());

  const Eigen::Vector2d ego_slot_vec =
      slot_center - Eigen::Vector2d(measure_data_ptr_->GetPos().x(),
                                    measure_data_ptr_->GetPos().y());

  const double cross_ego_slot_heading = pnc::geometry_lib::GetCrossFromTwoVec2d(
      ego_heading_vec, slot_heading_vec);
  const double cross_ego_slot_center =
      pnc::geometry_lib::GetCrossFromTwoVec2d(ego_heading_vec, ego_slot_vec);

  size_t slot_side = pnc::geometry_lib::SLOT_SIDE_RIGHT;
  if (cross_ego_slot_heading > 0.0 && cross_ego_slot_center < 0.0) {
    slot_side = pnc::geometry_lib::SLOT_SIDE_RIGHT;
  } else if (cross_ego_slot_heading < 0.0 && cross_ego_slot_center > 0.0) {
    slot_side = pnc::geometry_lib::SLOT_SIDE_LEFT;
  } else {
    return false;
  }

  bool is_same_direction = false;
  if (slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
    if (cos_theta > 0.0) {
      is_same_direction = true;
    }
  } else {
    if (cos_theta < 0.0) {
      is_same_direction = true;
    }
  }
  frame_.slot_info_direction[slot_info.id()] = is_same_direction;

  return true;
}

common::SlotInfo SlotManager::SlotInfoTransfer(
    const iflyauto::ParkingFusionSlot &fusion_slot) {
  double accumulated_x = 0.0;
  double accumulated_y = 0.0;
  common::SlotInfo slot_info;
  static const auto fusion_slots_size = 4;
  for (auto j = 0; j < fusion_slots_size; j++) {
    auto add_point = slot_info.mutable_corner_points()->add_corner_point();
    add_point->set_x(fusion_slot.corner_points[j].x);
    add_point->set_y(fusion_slot.corner_points[j].y);

    accumulated_x += fusion_slot.corner_points[j].x;
    accumulated_y += fusion_slot.corner_points[j].y;
  }

  slot_info.mutable_center()->set_x(accumulated_x /
                                    static_cast<double>(fusion_slots_size));

  slot_info.mutable_center()->set_y(accumulated_y /
                                    static_cast<double>(fusion_slots_size));

  slot_info.set_id(fusion_slot.id);

  if (state_machine_ptr_->IsSeachingStatus()) {
    slot_info.set_is_release((fusion_slot.allow_parking == 1));
    slot_info.set_is_occupied((fusion_slot.allow_parking == 0));
  }

  if (state_machine_ptr_->IsParkingStatus()) {
    // the selected slot in parking state is forced to release
    slot_info.set_is_release(true);
    slot_info.set_is_occupied(false);
  }

  slot_info.set_slot_type(fusion_slot.type);

  return slot_info;
}

const bool SlotManager::SlotInfoTransfer(
    const iflyauto::ParkingFusionSlot &fusion_slot,
    common::SlotInfo &slot_info) {
  double accumulated_x = 0.0;
  double accumulated_y = 0.0;
  static const int fusion_slots_size = 4;
  if (NUM_OF_CORNER_POINT_NUM != fusion_slots_size) {
    return false;
  }
  for (int j = 0; j < fusion_slots_size; j++) {
    auto add_point = slot_info.mutable_corner_points()->add_corner_point();
    add_point->set_x(fusion_slot.corner_points[j].x);
    add_point->set_y(fusion_slot.corner_points[j].y);

    accumulated_x += fusion_slot.corner_points[j].x;
    accumulated_y += fusion_slot.corner_points[j].y;
  }

  slot_info.mutable_center()->set_x(accumulated_x /
                                    static_cast<double>(fusion_slots_size));

  slot_info.mutable_center()->set_y(accumulated_y /
                                    static_cast<double>(fusion_slots_size));

  slot_info.set_id(fusion_slot.id);

  if (state_machine_ptr_->IsSeachingStatus()) {
    slot_info.set_is_release((fusion_slot.allow_parking == 1));
    slot_info.set_is_occupied((fusion_slot.allow_parking == 0));
  }

  if (state_machine_ptr_->IsParkingStatus()) {
    // the selected slot in parking state is forced to release
    slot_info.set_is_release(true);
    slot_info.set_is_occupied(false);
  }

  slot_info.set_slot_type(fusion_slot.type);

  return true;
}

void SlotManager::ModifySlot2Rectangle(common::SlotInfo &slot_info) {
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
    ILOG_INFO << "slot should modify to rectangle";
    for (size_t i = 0; i < target_boundingbox.size(); i++) {
      ILOG_INFO << i << " : " << target_boundingbox[i].x() << " "
                << target_boundingbox[i].y() << " ";
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

bool SlotManager::IsValidParkingSlot(const common::SlotInfo &slot_info) const {
  if (slot_info.slot_type() ==
      Common::ParkingSlotType::PARKING_SLOT_TYPE_HORIZONTAL) {
    return true;
  }
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
  const double slot_line_angle_dif_deg = slot_line_angle_dif * kRad2Deg;

  const bool slot_line_parallel_condition =
      slot_line_angle_dif_deg <=
      apa_param.GetParam().max_slot_boundary_line_angle_dif_deg;

  if (!slot_line_parallel_condition) {
    return false;
  }

  if (slot_info.slot_type() ==
      Common::ParkingSlotType::PARKING_SLOT_TYPE_SLANTING) {
    return true;
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
      apa_param.GetParam().max_slot_boundary_line_angle_dif_deg;
  if (corner_vertical_condition) {
    return true;
  } else {
    return false;
  }
}

bool SlotManager::CorrectSlotPointsOrder(common::SlotInfo &slot_info) const {
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

bool SlotManager::IfUpdateSlot(const common::SlotInfo &new_slot_info,
                               const size_t fusion_slot_source_type) {
  if ((fusion_slot_source_type ==
       iflyauto::SlotSourceType::SLOT_SOURCE_TYPE_ONLY_USS) ||
      (fusion_slot_source_type ==
       iflyauto::SlotSourceType::SLOT_SOURCE_TYPE_CAMERA_USS)) {
    // ILOG_INFO <<"it is uss slot";
    return true;
  }
  // ILOG_INFO <<"it is vision slot";
  // update by angle between ego_heading_axis and slot_heading_axis (new
  // slot)
  const bool angle_update_condition = AngleUpdateCondition(new_slot_info);

  // update by lon dif between slot center and mirror middle point
  const bool lon_update_condition =
      LonDifUpdateCondition(new_slot_info, fusion_slot_source_type);

  return true || (angle_update_condition && lon_update_condition);
}

bool SlotManager::LonDifUpdateCondition(
    const common::SlotInfo &new_slot_info,
    const size_t parking_fusion_slot_source_type) {
  if ((parking_fusion_slot_source_type ==
       iflyauto::SLOT_SOURCE_TYPE_ONLY_USS) ||
      (parking_fusion_slot_source_type ==
       iflyauto::SLOT_SOURCE_TYPE_CAMERA_USS)) {
    return true;
  }

  const auto slot_pts = new_slot_info.corner_points().corner_point();
  const Eigen::Vector2d ego_pos_to_pt0_vec(
      slot_pts[0].x() - measure_data_ptr_->GetPos().x(),
      slot_pts[0].y() - measure_data_ptr_->GetPos().y());

  const Eigen::Vector2d ego_pos_to_pt2_vec(
      slot_pts[2].x() - measure_data_ptr_->GetPos().x(),
      slot_pts[2].y() - measure_data_ptr_->GetPos().y());

  const auto ego_unit_heading = measure_data_ptr_->GetHeadingVec();

  const Eigen::Vector2d slot_pt01_mid(
      (slot_pts[0].x() + slot_pts[1].x()) * 0.5,
      (slot_pts[0].y() + slot_pts[1].y()) * 0.5);

  const Eigen::Vector2d slot_pt23_mid(
      (slot_pts[2].x() + slot_pts[3].x()) * 0.5,
      (slot_pts[2].y() + slot_pts[3].y()) * 0.5);

  const auto slot_heading_vec_unit =
      (slot_pt01_mid - slot_pt23_mid).normalized();

  bool lon_dif_update_condition = false;

  if (new_slot_info.slot_type() ==
      Common::ParkingSlotType::PARKING_SLOT_TYPE_VERTICAL) {
    const auto cross_product = pnc::geometry_lib::GetCrossFromTwoVec2d(
        ego_unit_heading, ego_pos_to_pt0_vec);

    Eigen::Vector2d mirror_pos;
    if (cross_product < -1e-5) {
      // right side slot
      mirror_pos = measure_data_ptr_->GetRightMirrorPos();
    } else if (cross_product > 1e-5) {
      // left side slot
      mirror_pos = measure_data_ptr_->GetLeftMirrorPos();
    } else {
      return false;
    }

    const Eigen::Vector2d slot_center_to_mirror_vec(
        mirror_pos.x() - new_slot_info.center().x(),
        mirror_pos.y() - new_slot_info.center().y());

    // longitudinal distance from slot center to car mirror
    double lon_dist = pnc::geometry_lib::GetCrossFromTwoVec2d(
        slot_heading_vec_unit, slot_center_to_mirror_vec);

    bool outside_case = false;
    bool inside_case = false;
    if (cross_product < -1e-5) {
      // right side
      outside_case =
          lon_dist >= apa_param.GetParam().outside_lon_dist_min_slot2mirror &&
          lon_dist <= apa_param.GetParam().outside_lon_dist_max_slot2mirror;
      inside_case =
          lon_dist >= -apa_param.GetParam().inside_lon_dist_max_slot2mirror &&
          lon_dist <= -apa_param.GetParam().inside_lon_dist_min_slot2mirror;
    } else {
      // left side
      outside_case =
          lon_dist >= -apa_param.GetParam().outside_lon_dist_max_slot2mirror &&
          lon_dist <= -apa_param.GetParam().outside_lon_dist_min_slot2mirror;
      inside_case =
          lon_dist >= apa_param.GetParam().inside_lon_dist_min_slot2mirror &&
          lon_dist <= apa_param.GetParam().inside_lon_dist_max_slot2mirror;
    }
    lon_dif_update_condition = outside_case || inside_case;
  }

  else if (new_slot_info.slot_type() ==
           Common::ParkingSlotType::PARKING_SLOT_TYPE_HORIZONTAL) {
    // calc slot side first
    const bool is_left_side =
        (pnc::geometry_lib::GetCrossFromTwoVec2d(
             measure_data_ptr_->GetHeadingVec(), ego_pos_to_pt2_vec) > 0.0);

    Eigen::Vector2d mirror_pos;
    if (!is_left_side) {
      // right side slot
      mirror_pos = measure_data_ptr_->GetRightMirrorPos();
    } else if (is_left_side) {
      // left side slot
      mirror_pos = measure_data_ptr_->GetLeftMirrorPos();
    } else {
      return false;
    }

    const Eigen::Vector2d slot_center_to_mirror_vec(
        mirror_pos.x() - new_slot_info.center().x(),
        mirror_pos.y() - new_slot_info.center().y());

    // make lon_dif has the meaning of positive value crossing the slot
    // center line, negative value before the slot center line
    double lon_dif = pnc::geometry_lib::GetCrossFromTwoVec2d(
        slot_heading_vec_unit, slot_center_to_mirror_vec);
    if (!is_left_side) {
      lon_dif = -lon_dif;
    }

    // ILOG_INFO <<"---parallel slot id =" << new_slot_info.id()
    //           << " type =" << new_slot_info.slot_type());
    // ILOG_INFO <<"parallel is left side =" << is_left_side);
    // ILOG_INFO <<"lon dif =" << lon_dif);

    lon_dif_update_condition = pnc::mathlib::IsInBound(lon_dif, -5.0, -1.7) ||
                               pnc::mathlib::IsInBound(lon_dif, 0.3, 1.0);
  }

  else if (new_slot_info.slot_type() ==
           Common::ParkingSlotType::PARKING_SLOT_TYPE_SLANTING) {
    const auto cross_product = pnc::geometry_lib::GetCrossFromTwoVec2d(
        ego_unit_heading, ego_pos_to_pt0_vec);

    Eigen::Vector2d mirror_pos;
    if (cross_product < -1e-5) {
      // right side slot
      mirror_pos = measure_data_ptr_->GetRightMirrorPos();
    } else if (cross_product > 1e-5) {
      // left side slot
      mirror_pos = measure_data_ptr_->GetLeftMirrorPos();
    } else {
      return false;
    }

    Eigen::Vector2d origin_pt_0 =
        Eigen::Vector2d(slot_pts[0].x(), slot_pts[0].y());
    Eigen::Vector2d origin_pt_1 =
        Eigen::Vector2d(slot_pts[1].x(), slot_pts[1].y());

    if (frame_.slot_info_corner_01.count(new_slot_info.id()) != 0) {
      origin_pt_0 = frame_.slot_info_corner_01[new_slot_info.id()].first;
      origin_pt_1 = frame_.slot_info_corner_01[new_slot_info.id()].second;
    }

    const Eigen::Vector2d origin_pt_01_vec = origin_pt_1 - origin_pt_0;
    const Eigen::Vector2d origin_pt_01_vec_n(origin_pt_01_vec.y(),
                                             -origin_pt_01_vec.x());
    const Eigen::Vector2d origin_pt_01_mid = (origin_pt_0 + origin_pt_1) * 0.5;
    pnc::geometry_lib::LineSegment line(
        origin_pt_01_mid, origin_pt_01_mid + origin_pt_01_vec_n.normalized());

    double lon_dist = pnc::geometry_lib::CalPoint2LineDist(mirror_pos, line);

    const Eigen::Vector2d pt_01_mid_mirr_vec = mirror_pos - origin_pt_01_mid;
    const double cros = pnc::geometry_lib::GetCrossFromTwoVec2d(
        origin_pt_01_vec_n, pt_01_mid_mirr_vec);

    if (cros < 0.0) {
      // car is at inside
      lon_dist *= -1.0;
    }

    bool outside_case = false;
    bool inside_case = false;
    if (cross_product < -1e-5) {
      // right side
      outside_case =
          lon_dist >= apa_param.GetParam().outside_lon_dist_min_slot2mirror &&
          lon_dist <= apa_param.GetParam().outside_lon_dist_max_slot2mirror;
      inside_case =
          lon_dist >= -apa_param.GetParam().inside_lon_dist_max_slot2mirror &&
          lon_dist <= -apa_param.GetParam().inside_lon_dist_min_slot2mirror;
    } else {
      // left side
      outside_case =
          lon_dist >= -apa_param.GetParam().outside_lon_dist_max_slot2mirror &&
          lon_dist <= -apa_param.GetParam().outside_lon_dist_min_slot2mirror;
      inside_case =
          lon_dist >= apa_param.GetParam().inside_lon_dist_min_slot2mirror &&
          lon_dist <= apa_param.GetParam().inside_lon_dist_max_slot2mirror;
    }
    lon_dif_update_condition = outside_case || inside_case;
  }

  return true || lon_dif_update_condition;
}

bool SlotManager::AngleUpdateCondition(const common::SlotInfo &new_slot_info) {
  const Eigen::Vector2d mirror_center_pos =
      (measure_data_ptr_->GetLeftMirrorPos() +
       measure_data_ptr_->GetRightMirrorPos()) *
      0.5;

  const Eigen::Vector2d mirror_center_to_slot_center_vec(
      new_slot_info.center().x() - mirror_center_pos.x(),
      new_slot_info.center().y() - mirror_center_pos.y());

  double angle = std::fabs(pnc::transform::GetAngleFromTwoVec(
      measure_data_ptr_->GetHeadingVec(), mirror_center_to_slot_center_vec));

  const double angle_offset = pnc::mathlib::Deg2Rad(
      apa_param.GetParam().max_slots_update_angle_dis_limit_deg);

  if (new_slot_info.slot_type() == Common::PARKING_SLOT_TYPE_SLANTING &&
      frame_.slot_info_corner_01.count(new_slot_info.id()) != 0) {
    const Eigen::Vector2d origin_pt_0 =
        frame_.slot_info_corner_01[new_slot_info.id()].first;
    const Eigen::Vector2d origin_pt_1 =
        frame_.slot_info_corner_01[new_slot_info.id()].second;

    const Eigen::Vector2d origin_pt_01_vec = origin_pt_1 - origin_pt_0;
    const Eigen::Vector2d origin_pt_01_vec_n_down(-origin_pt_01_vec.y(),
                                                  origin_pt_01_vec.x());
    const Eigen::Vector2d origin_pt_01_mid = (origin_pt_0 + origin_pt_1) * 0.5;
    const Eigen::Vector2d slot_center =
        origin_pt_01_mid + 2.5 * origin_pt_01_vec_n_down;
    const Eigen::Vector2d mirror_center_vec = slot_center - mirror_center_pos;
    angle = std::fabs(pnc::transform::GetAngleFromTwoVec(
        measure_data_ptr_->GetHeadingVec(), mirror_center_vec));
  }

  const double mid_angle = 0.5 * kPie;

  return pnc::mathlib::IsInBound(angle, mid_angle - angle_offset,
                                 mid_angle + angle_offset);
}

const double SlotManager::CalAngleSlot2Car(
    const common::SlotInfo &new_slot_info) const {
  const Eigen::Vector2d mirror_center_pos =
      (measure_data_ptr_->GetLeftMirrorPos() +
       measure_data_ptr_->GetRightMirrorPos()) *
      0.5;

  const Eigen::Vector2d mirror_center_to_slot_center_vec(
      new_slot_info.center().x() - mirror_center_pos.x(),
      new_slot_info.center().y() - mirror_center_pos.y());

  double angle = std::fabs(pnc::transform::GetAngleFromTwoVec(
      measure_data_ptr_->GetHeadingVec(), mirror_center_to_slot_center_vec));

  if (new_slot_info.slot_type() == Common::PARKING_SLOT_TYPE_SLANTING &&
      frame_.slot_info_corner_01.count(new_slot_info.id()) != 0) {
    const Eigen::Vector2d origin_pt_0 =
        frame_.slot_info_corner_01.at(new_slot_info.id()).first;
    const Eigen::Vector2d origin_pt_1 =
        frame_.slot_info_corner_01.at(new_slot_info.id()).second;

    const Eigen::Vector2d origin_pt_01_vec = origin_pt_1 - origin_pt_0;
    const Eigen::Vector2d origin_pt_01_vec_n_down(-origin_pt_01_vec.y(),
                                                  origin_pt_01_vec.x());
    const Eigen::Vector2d origin_pt_01_mid = (origin_pt_0 + origin_pt_1) * 0.5;
    const Eigen::Vector2d slot_center =
        origin_pt_01_mid + 2.5 * origin_pt_01_vec_n_down;
    const Eigen::Vector2d mirror_center_vec = slot_center - mirror_center_pos;
    angle = std::fabs(pnc::transform::GetAngleFromTwoVec(
        measure_data_ptr_->GetHeadingVec(), mirror_center_vec));
  }

  return angle;
}

bool SlotManager::UpdateSlotsInParking() {
  ILOG_INFO << "apa state is in parking";

  size_t select_slot_id = frame_.parking_slot_ptr->select_slot_id;

  if (state_machine_ptr_->GetStateMachine() ==
          ApaStateMachine::ACTIVE_OUT_CAR_FRONT ||
      state_machine_ptr_->GetStateMachine() ==
          ApaStateMachine::ACTIVE_OUT_CAR_REAR) {
    if (frame_.park_out_select_id == 0) {
      double dist = std::numeric_limits<double>::infinity();
      for (auto &pair : frame_.slot_info_window_map) {
        if (pair.first != 0) {
          const auto &slot_center_pt = pair.second.GetFusedInfo().center();
          const Eigen::Vector2d slot_center(slot_center_pt.x(),
                                            slot_center_pt.y());
          const double temp_dist =
              (measure_data_ptr_->GetPos() - slot_center).norm();
          if (temp_dist < dist) {
            dist = temp_dist;
            frame_.park_out_select_id = pair.first;
          }
        }
      }
      ILOG_INFO << "park_out_select_id " << frame_.park_out_select_id;
    }
    select_slot_id = frame_.park_out_select_id;
  }

  if (select_slot_id == 0) {
    ILOG_INFO << "select_slot_id = 0, is not valid";
    return false;
  }
  ILOG_INFO << "select_slot_id:" << select_slot_id;

  if (frame_.slot_info_window_map.count(select_slot_id) == 0) {
    ILOG_INFO << "select slot is not in slot_info_window_vec";
    return false;
  }

  ILOG_INFO << "select slot is in slot_info_window_map";
  if (frame_.slot_info_window_map.empty() ||
      frame_.slot_info_window_map[select_slot_id].IsEmpty()) {
    ILOG_INFO << "slot_info_window_map is empty!";
    return false;
  }

  iflyauto::ParkingFusionSlot select_fusion_slot;
  bool valid_select_slot = false;
  for (uint32 i = 0;
       i < frame_.parking_slot_ptr->parking_fusion_slot_lists_size; ++i) {
    const auto &fusion_slot =
        frame_.parking_slot_ptr->parking_fusion_slot_lists[i];
    if (select_slot_id == fusion_slot.id) {
      select_fusion_slot = fusion_slot;
      if (fusion_slot.type == iflyauto::PARKING_SLOT_TYPE_VERTICAL) {
        ILOG_INFO << "perpendicular slot selected in fusion";
      } else if (fusion_slot.type == iflyauto::PARKING_SLOT_TYPE_HORIZONTAL) {
        ILOG_INFO << "parallel slot selected in fusion";
      } else if (fusion_slot.type == iflyauto::PARKING_SLOT_TYPE_SLANTING) {
        ILOG_INFO << "slant slot selected in fusion";
      } else {
        ILOG_INFO << "current slot selected is no supported";
        break;
      }
      valid_select_slot = true;
      break;
    }
  }

  if (!valid_select_slot) {
    ILOG_INFO << "selected slot is invalid!";
    return false;
  }

  common::SlotInfo select_slot;
  if (!ProcessRawSlot(select_fusion_slot, select_slot)) {
    select_slot = frame_.slot_info_window_map[select_slot_id].GetFusedInfo();
    select_slot.set_is_release(true);
    select_slot.set_is_occupied(false);
  }

  // make sure apa is always running when begin, todo, should change
  // according to fusion current is only put a patch
  if (select_slot.is_release() == false) {
    ILOG_INFO << "selected slot is not released!";
    return false;
  }

  if (select_slot.slot_type() ==
          Common::ParkingSlotType::PARKING_SLOT_TYPE_VERTICAL ||
      select_slot.slot_type() ==
          Common::ParkingSlotType::PARKING_SLOT_TYPE_SLANTING) {
    if (!UpdateEgoSlotInfo(select_slot_id, select_slot, select_fusion_slot)) {
      return false;
    }
    UpdateSlotInfoInParking();

    UpdateLimiterInfoInParking();
  }

  else if (select_slot.slot_type() ==
           Common::ParkingSlotType::PARKING_SLOT_TYPE_HORIZONTAL) {
    if (!UpdateEgoParallelSlotInfo(select_slot_id, select_slot,
                                   select_fusion_slot)) {
      return false;
    }
    UpdateParallelSlotInfoInParking();
  }

  else if (select_slot.slot_type() ==
           Common::ParkingSlotType::PARKING_SLOT_TYPE_SLANTING) {
  }

  return true;
}

bool SlotManager::UpdateEgoSlotInfo(
    const google::protobuf::uint32 &select_slot_id,
    const common::SlotInfo &select_slot,
    const iflyauto::ParkingFusionSlot &select_fusion_slot) {
  auto &ego_slot_info = frame_.ego_slot_info;

  if (select_fusion_slot.type ==
      iflyauto::ParkingSlotType::PARKING_SLOT_TYPE_INVALID) {
    return false;
  }
  if (ego_slot_info.slot_type ==
      iflyauto::ParkingSlotType::PARKING_SLOT_TYPE_INVALID) {
    ego_slot_info.slot_type = select_fusion_slot.type;
  } else {
    if (ego_slot_info.slot_type != select_fusion_slot.type) {
      ILOG_INFO << "selecte_fusion_slot type is changed, error";
      return false;
    }
  }

  if (!frame_.is_fix_slot) {
    ego_slot_info.slot_type = select_fusion_slot.type;
    ego_slot_info.select_slot_id = select_slot_id;
    ego_slot_info.select_fusion_slot = select_fusion_slot;
    ego_slot_info.select_slot = select_slot;
    ego_slot_info.select_slot_filter =
        frame_.slot_info_window_map[select_slot_id].GetFusedInfo();

    const auto &slot_points =
        ego_slot_info.select_slot_filter.corner_points().corner_point();

    std::vector<Eigen::Vector2d> pt;
    pt.resize(4);
    for (size_t i = 0; i < 4; ++i) {
      pt[i] << slot_points[i].x(), slot_points[i].y();
    }

    const Eigen::Vector2d pM01 = 0.5 * (pt[0] + pt[1]);
    const Eigen::Vector2d pM23 = 0.5 * (pt[2] + pt[3]);
    const double real_slot_length = (pM01 - pM23).norm();
    const Eigen::Vector2d t = (pt[1] - pt[0]).normalized();
    // n is vec that slot opening orientation
    Eigen::Vector2d n = Eigen::Vector2d(t.y(), -t.x());
    n = (pM01 - pM23).normalized();
    pt[2] = pt[0] - real_slot_length * n;
    pt[3] = pt[1] - real_slot_length * n;

    // const double virtual_slot_length =
    //     apa_param.GetParam().car_length +
    //     apa_param.GetParam().slot_compare_to_car_length;

    // const double use_slot_length =
    //     std::min(real_slot_length, virtual_slot_length);

    const double use_slot_length = real_slot_length;

    ego_slot_info.slot_origin_pos = pM01 - use_slot_length * n;
    ego_slot_info.slot_origin_heading = std::atan2(n.y(), n.x());
    ego_slot_info.slot_origin_heading_vec = n;
    ego_slot_info.slot_length = use_slot_length;
    ego_slot_info.slot_width = (pt[0] - pt[1]).norm();

    ego_slot_info.g2l_tf.Init(ego_slot_info.slot_origin_pos,
                              ego_slot_info.slot_origin_heading);

    ego_slot_info.l2g_tf.Init(ego_slot_info.slot_origin_pos,
                              ego_slot_info.slot_origin_heading);

    ego_slot_info.sin_angle = 1.0;
    ego_slot_info.origin_pt_0_heading = 0.0;
    ego_slot_info.pt_0 = ego_slot_info.g2l_tf.GetPos(pt[0]);
    ego_slot_info.pt_1 = ego_slot_info.g2l_tf.GetPos(pt[1]);
    if (select_slot.slot_type() == Common::PARKING_SLOT_TYPE_SLANTING) {
      if (frame_.slot_info_angle.count(select_slot.id()) != 0) {
        ego_slot_info.sin_angle =
            frame_.slot_info_angle[select_slot.id()].second;
        ego_slot_info.origin_pt_0_heading =
            90.0 - frame_.slot_info_angle[select_slot.id()].first;
        ego_slot_info.pt_0 = ego_slot_info.g2l_tf.GetPos(
            frame_.slot_info_corner_01[select_slot.id()].first);
        ego_slot_info.pt_1 = ego_slot_info.g2l_tf.GetPos(
            frame_.slot_info_corner_01[select_slot.id()].second);
      }
    }
    if (ego_slot_info.pt_0.y() > ego_slot_info.pt_1.y()) {
      std::swap(ego_slot_info.pt_0, ego_slot_info.pt_1);
    }
  }

  ego_slot_info.ego_pos_slot =
      ego_slot_info.g2l_tf.GetPos(measure_data_ptr_->GetPos());

  ego_slot_info.ego_heading_slot =
      ego_slot_info.g2l_tf.GetHeading(measure_data_ptr_->GetHeading());

  ego_slot_info.ego_heading_slot_vec
      << std::cos(ego_slot_info.ego_heading_slot),
      std::sin(ego_slot_info.ego_heading_slot);

  // update limiter
  if (!frame_.limiter_point_window.IsEmpty()) {
    const auto limiter = frame_.limiter_point_window.GetFusedLimiterPoints();
    ego_slot_info.limiter.first << limiter.first.x(), limiter.first.y();
    ego_slot_info.limiter.second << limiter.second.x(), limiter.second.y();
  }

  // cal target pos
  ego_slot_info.target_ego_pos_slot
      << (ego_slot_info.limiter.first.x() + ego_slot_info.limiter.second.x()) *
             0.5,
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
          apa_param.GetParam().slot_occupied_ratio_max_heading_err * kDeg2Rad) {
    const std::vector<double> x_tab = {
        ego_slot_info.target_ego_pos_slot.x(),
        ego_slot_info.slot_length + apa_param.GetParam().rear_overhanging};

    const std::vector<double> occupied_ratio_tab = {1.0, 0.0};
    ego_slot_info.slot_occupied_ratio = pnc::mathlib::Interp1(
        x_tab, occupied_ratio_tab, ego_slot_info.ego_pos_slot.x());
  } else {
    ego_slot_info.slot_occupied_ratio = 0.0;
  }

  // fix slot
  if (ego_slot_info.slot_occupied_ratio >
          apa_param.GetParam().fix_slot_occupied_ratio &&
      !frame_.is_fix_slot && measure_data_ptr_->GetStaticFlag()) {
    frame_.is_fix_slot = true;
  }

  ego_slot_info.fus_obj_valid_flag = frame_.fus_obj_valid_flag;
  ego_slot_info.obs_pt_vec_slot.clear();
  if (!frame_.fus_obj_valid_flag) {
    // use uss obs
    if (frame_.obs_pt_map.count(select_slot.id()) == 0) {
      return true;
    }
    const auto &obs_pt_vec = frame_.obs_pt_map[select_slot.id()];
    ego_slot_info.obs_pt_vec_slot.reserve(obs_pt_vec.size());
    ego_slot_info.obs_pt_vec_slot = obs_pt_vec;
    // for (const auto &obs_pt : obs_pt_vec) {
    //   const auto obs_pt_slot = ego_slot_info.g2l_tf.GetPos(obs_pt);
    //   ego_slot_info.obs_pt_vec_slot.emplace_back(std::move(obs_pt_slot));
    // }
  }

  else {
    // use fus obj and ground line
    ego_slot_info.obs_pt_vec_slot.reserve(frame_.obs_pt_vec.size());
    // obs global coord transform to local coord
    ego_slot_info.obs_pt_vec_slot = frame_.obs_pt_vec;
    // for (const auto &obs_pt : frame_.obs_pt_vec) {
    //   const auto obs_pt_slot = ego_slot_info.g2l_tf.GetPos(obs_pt);
    //   ego_slot_info.obs_pt_vec_slot.emplace_back(std::move(obs_pt_slot));
    // }
  }

  return true;
}

const bool SlotManager::UpdateEgoParallelSlotInfo(
    const google::protobuf::uint32 &select_slot_id,
    const common::SlotInfo &select_slot,
    const iflyauto::ParkingFusionSlot &select_fusion_slot) {
  auto &ego_slot_info = frame_.ego_slot_info;

  if (select_fusion_slot.type ==
      iflyauto::ParkingSlotType::PARKING_SLOT_TYPE_INVALID) {
    return false;
  }

  if (ego_slot_info.slot_type ==
      iflyauto::ParkingSlotType::PARKING_SLOT_TYPE_INVALID) {
    ego_slot_info.slot_type = select_fusion_slot.type;
  } else {
    if (ego_slot_info.slot_type != select_fusion_slot.type) {
      ILOG_INFO << "selecte_fusion_slot type is changed, error";
      return false;
    }
  }

  ego_slot_info.select_slot_id = select_slot_id;
  ego_slot_info.select_fusion_slot = select_fusion_slot;
  ego_slot_info.select_slot = select_slot;
  ego_slot_info.select_slot_filter =
      frame_.slot_info_window_map[select_slot_id].GetFusedInfo();

  const auto &slot_points =
      ego_slot_info.select_slot_filter.corner_points().corner_point();

  std::vector<Eigen::Vector2d> pt;
  pt.resize(4);
  // ILOG_INFO <<"pt in select_slot_filter";
  for (size_t i = 0; i < 4; ++i) {
    pt[i] << slot_points[i].x(), slot_points[i].y();
    // ILOG_INFO <<"no. " << i << " pt : " << pt[i].transpose());
  }

  if (!frame_.is_side_calc_in_parking) {
    const Eigen::Vector2d v_10_unit = (pt[0] - pt[1]).normalized();
    // ILOG_INFO <<"v10_unit = " << v_10_unit.transpose());
    // ILOG_INFO <<
    //     "ego heading vec = " <<
    //     frame_.measurement_data_ptr->ego_heading_vec.transpose());

    const double dot_ego_to_v10 =
        measure_data_ptr_->GetHeadingVec().dot(v_10_unit);
    // ILOG_INFO <<"dot ego to v10 = " << dot_ego_to_v10);

    // judge slot side via slot pt3
    if (dot_ego_to_v10 < -1e-8) {
      frame_.ego_slot_info.slot_side = pnc::geometry_lib::SLOT_SIDE_LEFT;
      ILOG_INFO << "left!";
    } else if (dot_ego_to_v10 > 1e-8) {
      frame_.ego_slot_info.slot_side = pnc::geometry_lib::SLOT_SIDE_RIGHT;
      ILOG_INFO << "right!";
    } else {
      frame_.ego_slot_info.slot_side = pnc::geometry_lib::SLOT_SIDE_INVALID;
      ILOG_INFO << "calculate parallel slot side error ";
      return false;
    }
    frame_.is_side_calc_in_parking = true;
  }

  Eigen::Vector2d n = Eigen::Vector2d::Zero();
  Eigen::Vector2d t = Eigen::Vector2d::Zero();

  ego_slot_info.slot_length = (pt[0] - pt[1]).norm();
  pnc::geometry_lib::LineSegment line_01(pt[0], pt[1]);

  ILOG_INFO << "slot side in slm = "
            << static_cast<int>(frame_.ego_slot_info.slot_side);

  // note: slot points' order is corrected in slot management
  if (frame_.ego_slot_info.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
    ego_slot_info.slot_width =
        pnc::geometry_lib::CalPoint2LineDist(pt[2], line_01);

    n = (pt[0] - pt[1]).normalized();
    t << -n.y(), n.x();
    ego_slot_info.slot_origin_pos = pt[0] - ego_slot_info.slot_length * n -
                                    0.5 * ego_slot_info.slot_width * t;
  } else {
    ego_slot_info.slot_width =
        pnc::geometry_lib::CalPoint2LineDist(pt[3], line_01);

    n = -(pt[0] - pt[1]).normalized();
    t << -n.y(), n.x();
    ego_slot_info.slot_origin_pos = pt[1] - ego_slot_info.slot_length * n +
                                    0.5 * ego_slot_info.slot_width * t;
  }

  ILOG_INFO << "slot width =" << ego_slot_info.slot_width;

  ego_slot_info.slot_origin_heading = std::atan2(n.y(), n.x());
  ego_slot_info.slot_origin_heading_vec = n;

  ILOG_INFO << "origin heading ="
            << ego_slot_info.slot_origin_heading * kRad2Deg;

  ego_slot_info.g2l_tf.Init(ego_slot_info.slot_origin_pos,
                            ego_slot_info.slot_origin_heading);

  ego_slot_info.l2g_tf.Init(ego_slot_info.slot_origin_pos,
                            ego_slot_info.slot_origin_heading);

  ego_slot_info.ego_pos_slot =
      ego_slot_info.g2l_tf.GetPos(measure_data_ptr_->GetPos());

  ego_slot_info.ego_heading_slot =
      ego_slot_info.g2l_tf.GetHeading(measure_data_ptr_->GetHeading());

  ego_slot_info.ego_heading_slot_vec
      << std::cos(ego_slot_info.ego_heading_slot),
      std::sin(ego_slot_info.ego_heading_slot);

  static const double kRearStopBuffer = 0.55;
  ego_slot_info.target_ego_pos_slot
      << apa_param.GetParam().rear_overhanging + kRearStopBuffer,
      0.0;

  ego_slot_info.target_ego_heading_slot = 0.0;

  ILOG_INFO << "target ego pos in slot ="
            << ego_slot_info.target_ego_pos_slot.transpose()
            << " heading =" << ego_slot_info.target_ego_heading_slot * kRad2Deg;

  // calc terminal error once
  ego_slot_info.terminal_err.Set(
      ego_slot_info.ego_pos_slot - ego_slot_info.target_ego_pos_slot,
      pnc::geometry_lib::NormalizeAngle(ego_slot_info.ego_heading_slot -
                                        ego_slot_info.target_ego_heading_slot));

  // calc slot occupied ratio

  double slot_occupied_ratio = 0.0;
  if (pnc::mathlib::IsInBound(ego_slot_info.terminal_err.pos.x(), -3.0, 4.0)) {
    const double y_err_ratio =
        ego_slot_info.terminal_err.pos.y() / (0.5 * ego_slot_info.slot_width);

    if (ego_slot_info.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
      slot_occupied_ratio = pnc::mathlib::Clamp(1 - y_err_ratio, 0.0, 1.0);
    } else if (ego_slot_info.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
      slot_occupied_ratio = pnc::mathlib::Clamp(1.0 + y_err_ratio, 0.0, 1.0);
    }
  }
  ego_slot_info.slot_occupied_ratio = slot_occupied_ratio;

  ILOG_INFO << "ego_slot_info.slot_occupied_ratio = "
            << ego_slot_info.slot_occupied_ratio;

  // set obs
  ego_slot_info.obs_pt_vec_slot.clear();

  // const auto &obs_pt_vec = frame_.obs_pt_map[select_slot.id()];
  // ego_slot_info.obs_pt_vec_slot.reserve(obs_pt_vec.size());

  // for (const auto &obs_pt : obs_pt_vec) {
  //   const auto obs_pt_slot = ego_slot_info.g2l_tf.GetPos(obs_pt);
  //   ego_slot_info.obs_pt_vec_slot.emplace_back(std::move(obs_pt_slot));
  // }

  return true;
}

void SlotManager::UpdateSlotInfoInParking() {
  auto &ego_slot_info = frame_.ego_slot_info;

  bool reset_slot_flag = false;
  bool update_slot_flag = true;

  if (!update_slot_flag) {
    bool update_slot_condition_1 =
        IfUpdateSlot(ego_slot_info.select_slot,
                     ego_slot_info.select_fusion_slot.fusion_source);

    double slot_update_out_heading_max =
        apa_param.GetParam().slot_update_out_heading_max;
    double slot_update_out_heading_min =
        apa_param.GetParam().slot_update_out_heading_min;
    double slot_update_out_lat_max =
        apa_param.GetParam().slot_update_out_lat_max;
    double slot_update_out_lat_min =
        apa_param.GetParam().slot_update_out_lat_min;

    if (ego_slot_info.slot_type == Common::PARKING_SLOT_TYPE_SLANTING) {
      slot_update_out_heading_max -= ego_slot_info.origin_pt_0_heading;
      slot_update_out_heading_min -= ego_slot_info.origin_pt_0_heading;
      slot_update_out_lat_min = 0.01;
    }

    bool update_slot_condition_2 =
        (ego_slot_info.slot_occupied_ratio <
             apa_param.GetParam().slot_update_in_or_out_occupied_ratio ||
         std::fabs(ego_slot_info.ego_heading_slot * kRad2Deg > 22.8)) &&
        (std::fabs(ego_slot_info.ego_heading_slot) <
             slot_update_out_heading_max * kDeg2Rad &&
         std::fabs(ego_slot_info.ego_heading_slot) >
             slot_update_out_heading_min * kDeg2Rad) &&
        (std::fabs(ego_slot_info.ego_pos_slot.y()) < slot_update_out_lat_max &&
         std::fabs(ego_slot_info.ego_pos_slot.y()) > slot_update_out_lat_min) &&
        (std::fabs(ego_slot_info.ego_pos_slot.x() < 7.86));

    bool update_slot_condition_3 =
        (ego_slot_info.slot_occupied_ratio >=
             apa_param.GetParam().slot_update_in_or_out_occupied_ratio &&
         std::fabs(ego_slot_info.ego_heading_slot) <
             apa_param.GetParam().slot_update_in_heading * kDeg2Rad &&
         std::fabs(ego_slot_info.ego_pos_slot.y()) <
             apa_param.GetParam().slot_update_in_lat);

    // ILOG_INFO <<"update_slot_condition_1 = "
    //             << update_slot_condition_1
    //             << " update_slot_condition_2 = " << update_slot_condition_2
    //             << " update_slot_condition_3 = " << update_slot_condition_3);
    update_slot_flag = update_slot_condition_1 || update_slot_condition_2 ||
                       update_slot_condition_3;
  }

  if (!update_slot_flag) {
    frame_.no_update_slot_count++;
  }

  if (update_slot_flag) {
    if (frame_.no_update_slot_count >
        apa_param.GetParam().slot_reset_threshold) {
      reset_slot_flag = true;
      frame_.no_update_slot_count = 0;
    }

    if (reset_slot_flag) {
      frame_.slot_info_window_map[ego_slot_info.select_slot_id].Reset();
    }
    frame_.slot_info_window_map[ego_slot_info.select_slot_id].Add(
        ego_slot_info.select_slot);

    // auto slot =
    // frame_.slot_management_info.mutable_slot_info_vec(slot_idx); *slot =
    // frame_.slot_info_window_vec[slot_idx].GetFusedInfo();

    ego_slot_info.select_slot_filter =
        frame_.slot_info_window_map[ego_slot_info.select_slot_id]
            .GetFusedInfo();
  }
}

void SlotManager::UpdateParallelSlotInfoInParking() {
  // update real time outside slot
  auto &ego_slot_info = frame_.ego_slot_info;
  auto &select_slot_window =
      frame_.slot_info_window_map[ego_slot_info.select_slot_id];

  select_slot_window.Reset();
  select_slot_window.Add(ego_slot_info.select_slot);
  ego_slot_info.select_slot_filter = select_slot_window.GetFusedInfo();
}

void SlotManager::UpdateLimiterInfoInParking() {
  const auto &ego_slot_info = frame_.ego_slot_info;
  if (frame_.limiter_point_window.IsEmpty()) {
    std::pair<Eigen::Vector2d, Eigen::Vector2d> limiter_slot =
        std::make_pair(Eigen::Vector2d(apa_param.GetParam().terminal_target_x,
                                       ego_slot_info.slot_width * 0.5),
                       Eigen::Vector2d(apa_param.GetParam().terminal_target_x,
                                       -ego_slot_info.slot_width * 0.5));
    frame_.limiter_point_window.Add(limiter_slot);
  }

  const bool update_limiter_flag_1 =
      (ego_slot_info.slot_occupied_ratio >=
           apa_param.GetParam().limiter_update_min_occupied_ratio &&
       ego_slot_info.slot_occupied_ratio <=
           apa_param.GetParam().limiter_update_max_occupied_ratio);

  auto current_limiter_slot =
      frame_.limiter_point_window.GetFusedLimiterPoints();

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
    double move_dist = 0.0;
    if (select_fusion_slot.limiters_size == 0) {
      // there is no limiter in slot
      ILOG_INFO << "fus has not limiter";
      limiter_global.first
          << select_slot_filter.corner_points().corner_point(2).x(),
          select_slot_filter.corner_points().corner_point(2).y();

      limiter_global.second
          << select_slot_filter.corner_points().corner_point(3).x(),
          select_slot_filter.corner_points().corner_point(3).y();

      move_dist = apa_param.GetParam().terminal_target_x;
    } else if (select_fusion_slot.limiters_size == 1) {
      // there is one limiter in slot
      ILOG_INFO << "fus has one limiter";
      limiter_global.first << select_fusion_slot.limiters[0].end_points[0].x,
          select_fusion_slot.limiters[0].end_points[0].y;

      limiter_global.second << select_fusion_slot.limiters[0].end_points[1].x,
          select_fusion_slot.limiters[0].end_points[1].y;

      move_dist = apa_param.GetParam().limiter_move_dist;
    } else {
      // there are two limiter in slot
      ILOG_INFO << "fus has two limiter";
      limiter_global.first << select_fusion_slot.limiters[0].end_points[0].x,
          select_fusion_slot.limiters[0].end_points[0].y;

      limiter_global.second << select_fusion_slot.limiters[1].end_points[1].x,
          select_fusion_slot.limiters[1].end_points[1].y;

      move_dist = apa_param.GetParam().limiter_move_dist;
    }

    limiter_slot.first = ego_slot_info.g2l_tf.GetPos(limiter_global.first);
    limiter_slot.second = ego_slot_info.g2l_tf.GetPos(limiter_global.second);
    limiter_slot.first.y() = ego_slot_info.slot_width * 0.5;
    limiter_slot.second.y() = -ego_slot_info.slot_width * 0.5;

    const double init_limiter_x =
        (limiter_slot.first.x() + limiter_slot.second.x()) * 0.5;

    limiter_slot.first.x() = init_limiter_x;
    limiter_slot.second.x() = init_limiter_x;

    if (apa_param.GetParam().limiter_length > 0.0) {
      double limiter_x = init_limiter_x + move_dist;

      limiter_x = std::min(
          limiter_x, std::min(ego_slot_info.pt_0.x(), ego_slot_info.pt_1.x()) -
                         apa_param.GetParam().wheel_base +
                         apa_param.GetParam().limiter_length);

      const double virtual_x =
          std::min(ego_slot_info.pt_0.x(), ego_slot_info.pt_1.x()) -
          apa_param.GetParam().front_overhanging -
          apa_param.GetParam().wheel_base - apa_param.GetParam().limiter_length;

      limiter_slot.first.x() = std::max(virtual_x, limiter_x);
      limiter_slot.second.x() = limiter_slot.first.x();
    } else {
      limiter_slot.first.x() += move_dist;
      limiter_slot.second.x() += move_dist;
    }

    frame_.limiter_point_window.Add(limiter_slot);
  }

  current_limiter_slot = frame_.limiter_point_window.GetFusedLimiterPoints();
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

  if (frame_.slot_management_info.limiter_points_size() == 0) {
    auto limiter = frame_.slot_management_info.add_limiter_points();
    *limiter = current_limiter_global_left_p;
    limiter = frame_.slot_management_info.add_limiter_points();
    *limiter = current_limiter_global_right_p;
  } else {
    auto limiter = frame_.slot_management_info.mutable_limiter_points(0);
    limiter->set_x(current_limiter_global_left_p.x());
    limiter->set_y(current_limiter_global_left_p.y());
    limiter = frame_.slot_management_info.mutable_limiter_points(1);
    limiter->set_x(current_limiter_global_right_p.x());
    limiter->set_y(current_limiter_global_right_p.y());
  }
}

void SlotManager::UpdateReleaseSlotIdVec() {
  frame_.release_slot_id_vec.clear();
  for (const auto &slot_info : frame_.slot_management_info.slot_info_vec()) {
    if (slot_info.is_release()) {
      frame_.release_slot_id_vec.emplace_back(static_cast<int>(slot_info.id()));
    } else {
      // if in parking, force set the slot release and send to hmi
      if ((state_machine_ptr_->IsParkingStatus()) &&
          (static_cast<int>(slot_info.id()) ==
           static_cast<int>(frame_.ego_slot_info.select_slot_id))) {
        frame_.release_slot_id_vec.emplace_back(
            static_cast<int>(slot_info.id()));
      }
    }
  }
}

void SlotManager::CopySlotReleaseInfo() {
  // for hmi
  UpdateReleaseSlotIdVec();

  // restore slot management info
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->clear_slot_management_info();
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_slot_management_info()
      ->CopyFrom(frame_.slot_management_info);

  ILOG_INFO << "select slot id = " << frame_.parking_slot_ptr->select_slot_id;

  return;
}

void SlotManager::SlotReleaseByScenarioTry(const bool release,
                                           const SlotReleaseMethod method) {
  for (int i = 0; i < frame_.slot_management_info.slot_info_vec_size(); i++) {
    common::SlotInfo *slot_info =
        frame_.slot_management_info.mutable_slot_info_vec(i);

    if (slot_info->has_id() &&
        slot_info->id() == frame_.ego_slot_info.select_slot_id) {
      if (!release) {
        slot_info->set_is_release(false);
        slot_info->set_is_occupied(true);
      } else {
        slot_info->set_is_release(true);
        slot_info->set_is_occupied(false);
      }

      break;
    }
  }

  if (!release) {
    frame_.ego_slot_info.release_info.release_state[method] =
        SlotReleaseState::NOT_RELEASE;
  } else {
    frame_.ego_slot_info.release_info.release_state[method] =
        SlotReleaseState::RELEASE;
  }

  CopySlotReleaseInfo();

  return;
}

const bool SlotManager::IsReleaseByRuleBased(const uint32_t select_slot_id) {
  for (int i = 0; i < frame_.slot_management_info.slot_info_vec_size(); i++) {
    const common::SlotInfo &slot_info =
        frame_.slot_management_info.slot_info_vec(i);

    if (slot_info.has_id() && slot_info.id() == select_slot_id) {
      if (slot_info.is_release()) {
        return true;
      }
      return false;
    }
  }

  return false;
}

}  // namespace apa_planner
}  // namespace planning
#include "rule_based_slot_release.h"

#include <cmath>
#include <cstddef>

#include "apa_state_machine_manager.h"
#include "ego_planning_config.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "point_cloud_obstacle.h"
#include "polygon_base.h"
#include "pose2d.h"
#include "slot_manager.h"

namespace planning {
namespace apa_planner {
void RuleBasedSlotRelease::Process(
    const LocalView *local_view, const MeasurementData *measures_ptr,
    const std::shared_ptr<ApaStateMachineManager> state_machine_manger_ptr,
    std::unordered_map<size_t, iflyauto::ParkingFusionSlot> &fusion_slot_map,
    apa_planner::SlotManager::Frame &frame) {
  frame_ = &frame;
  config_ = &apa_param.GetParam();
  local_view_ = local_view;
  measures_ptr_ = measures_ptr;
  state_machine_manger_ptr_ = state_machine_manger_ptr;

  // assemble slot_management_info
  frame_->slot_management_info.mutable_slot_info_vec()->Clear();
  if (state_machine_manger_ptr_->IsSeachingStatus()) {
    ParkingLotCruiseProcess(fusion_slot_map);
  } else {
    ParkingActivateProcess(fusion_slot_map);
  }

  return;
}

void RuleBasedSlotRelease::ParkingActivateProcess(
    std::unordered_map<size_t, iflyauto::ParkingFusionSlot> &fusion_slot_map) {
  return;
}

void RuleBasedSlotRelease::ParkingLotCruiseProcess(
    std::unordered_map<size_t, iflyauto::ParkingFusionSlot> &fusion_slot_map) {
  // Sort by the distance between the slot and the car
  const double time_start = IflyTime::Now_ms();
  std::map<double, apa_planner::SlotInfoWindow *> slot_map;
  for (auto &pair : frame_->slot_info_window_map) {
    const auto &slot = pair.second.GetFusedInfo();
    Eigen::Vector2d car_mirror = frame_->measurement_data_ptr->pos +
                                 frame_->measurement_data_ptr->heading_vec *
                                     config_->lon_dist_mirror_to_rear_axle;

    const double dist =
        (car_mirror - Eigen::Vector2d(slot.center().x(), slot.center().y()))
            .norm();
    slot_map[dist] = &pair.second;
  }

  bool is_ego_collision = IsEgoCloseToObs(local_view_, measures_ptr_);
  if (is_ego_collision) {
    ILOG_INFO << "ego is collision";
  }

  for (auto &pair : slot_map) {
    auto slot = frame_->slot_management_info.add_slot_info_vec();
    *slot = pair.second->GetFusedInfo();

    ILOG_INFO << "slot id = " << slot->id();

    if (!slot->is_release()) {
      continue;
    }

    // only extra protect
    if (fusion_slot_map.count(slot->id()) == 0 ||
        fusion_slot_map[static_cast<size_t>(slot->id())].allow_parking == 0) {
      slot->set_is_release(false);
      slot->set_is_occupied(true);
      continue;
    }

    if (is_ego_collision) {
      slot->set_is_release(false);
      slot->set_is_occupied(true);
      continue;
    }

    // distance is big
    if (pair.first > 16.8) {
      slot->set_is_release(false);
      slot->set_is_occupied(true);
      ILOG_INFO << "distance is big";
    }

    if (slot->corner_points().corner_point_size() != 4) {
      slot->set_is_release(false);
      slot->set_is_occupied(true);
      continue;
    }

    if ((slot->slot_type() ==
             Common::ParkingSlotType::PARKING_SLOT_TYPE_VERTICAL ||
         slot->slot_type() ==
             Common::ParkingSlotType::PARKING_SLOT_TYPE_SLANTING) &&
        slot->is_release() &&
        config_->path_generator_type ==
            apa_planner::ParkPathGenerationType::GEOMETRY_BASED &&
        (state_machine_manger_ptr_->GetStateMachine() ==
             ApaStateMachineT::ACTIVE_IN_CAR_FRONT ||
         state_machine_manger_ptr_->GetStateMachine() ==
             ApaStateMachineT::ACTIVE_IN_CAR_REAR)) {
      IsPerpendicularSlotCoarseRelease(slot, pair.second);

    } else if (slot->slot_type() ==
                   Common::ParkingSlotType::PARKING_SLOT_TYPE_HORIZONTAL &&
               slot->is_release()) {
      IsParallelSlotCoarseRelease(slot, pair.second);
    }
  }

  return;
}

const double RuleBasedSlotRelease::CalLonDistSlot2Car(
    const common::SlotInfo &new_slot_info) const {
  const auto slot_pts = new_slot_info.corner_points().corner_point();

  Eigen::Vector2d origin_pt_0 =
      Eigen::Vector2d(slot_pts[0].x(), slot_pts[0].y());
  Eigen::Vector2d origin_pt_1 =
      Eigen::Vector2d(slot_pts[1].x(), slot_pts[1].y());

  if (new_slot_info.slot_type() == Common::PARKING_SLOT_TYPE_SLANTING &&
      frame_->slot_info_corner_01.count(new_slot_info.id()) != 0) {
    origin_pt_0 = frame_->slot_info_corner_01.at(new_slot_info.id()).first;
    origin_pt_1 = frame_->slot_info_corner_01.at(new_slot_info.id()).second;
  }

  const Eigen::Vector2d ego_pos_to_pt0_vec =
      origin_pt_0 - frame_->measurement_data_ptr->pos;

  const Eigen::Vector2d ego_unit_heading =
      pnc::geometry_lib::GetUnitTangVecByHeading(
          frame_->measurement_data_ptr->heading);

  const double cross_product = pnc::geometry_lib::GetCrossFromTwoVec2d(
      ego_unit_heading, ego_pos_to_pt0_vec);

  Eigen::Vector2d mirror_pos;
  if (cross_product < -1e-5) {
    // right side slot
    mirror_pos = frame_->measurement_data_ptr->right_mirror_pos;
  } else if (cross_product > 1e-5) {
    // left side slot
    mirror_pos = frame_->measurement_data_ptr->left_mirror_pos;
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

  // when car is at outside, lon_dist should be negative
  if ((cross_product < 1e-5 && cros > 0.0) ||
      (cross_product > 1e-5 && cros < 0.0)) {
    // right side slot and car is at outside
    // left side slot and car is at outside
    lon_dist *= -1.0;
  }

  return lon_dist;
}

const bool RuleBasedSlotRelease::UpdateEgoParallelSlotInfoInSearching(
    apa_planner::SlotManager::EgoSlotInfo &ego_slot_info,
    const common::SlotInfo *slot_info) {
  if (!slot_info->has_corner_points()) {
    ILOG_INFO << "no selected corner pts in slm!";
    return false;
  }

  if (slot_info->corner_points().corner_point_size() != 4) {
    ILOG_INFO << "select slot in slm corner points size != 4!";
    return false;
  }

  if (!slot_info->has_slot_type() ||
      slot_info->slot_type() !=
          Common::ParkingSlotType::PARKING_SLOT_TYPE_HORIZONTAL) {
    ILOG_INFO << "not parallel slot!";
    return false;
  }

  ego_slot_info.Reset();
  ego_slot_info.select_slot = *slot_info;

  for (size_t i = 0; i < 4; ++i) {
    const Eigen::Vector2d pt(
        ego_slot_info.select_slot.corner_points().corner_point(i).x(),
        ego_slot_info.select_slot.corner_points().corner_point(i).y());

    ego_slot_info.slot_corner.emplace_back(std::move(pt));
  }

  const Eigen::Vector2d v_10_unit =
      (ego_slot_info.slot_corner[0] - ego_slot_info.slot_corner[1])
          .normalized();

  const double dot_ego_to_v10 =
      frame_->measurement_data_ptr->heading_vec.dot(v_10_unit);
  // ILOG_INFO <<"dot ego to v10 = " << dot_ego_to_v10);

  // judge slot side via slot pt3
  if (dot_ego_to_v10 < -1e-8) {
    ego_slot_info.slot_side = pnc::geometry_lib::SLOT_SIDE_LEFT;
    ILOG_INFO << "left!";
  } else if (dot_ego_to_v10 > 1e-8) {
    ego_slot_info.slot_side = pnc::geometry_lib::SLOT_SIDE_RIGHT;
    ILOG_INFO << "right!";
  } else {
    ego_slot_info.slot_side = pnc::geometry_lib::SLOT_SIDE_INVALID;
    ILOG_INFO << "calculate parallel slot side error ";
    return false;
  }

  ego_slot_info.slot_length =
      (ego_slot_info.slot_corner[0] - ego_slot_info.slot_corner[1]).norm();
  pnc::geometry_lib::LineSegment line_01(ego_slot_info.slot_corner[0],
                                         ego_slot_info.slot_corner[1]);

  ILOG_INFO << "slot side in slm = "
            << static_cast<int>(ego_slot_info.slot_side);

  ego_slot_info.slot_width =
      std::min(pnc::geometry_lib::CalPoint2LineDist(
                   ego_slot_info.slot_corner[2], line_01),
               pnc::geometry_lib::CalPoint2LineDist(
                   ego_slot_info.slot_corner[3], line_01));
  ILOG_INFO << "slot width =" << ego_slot_info.slot_width;

  Eigen::Vector2d n = Eigen::Vector2d::Zero();
  Eigen::Vector2d t = Eigen::Vector2d::Zero();
  // note: slot points' order is corrected in slot management
  if (ego_slot_info.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
    n = (ego_slot_info.slot_corner[0] - ego_slot_info.slot_corner[1])
            .normalized();
    t << -n.y(), n.x();
    ego_slot_info.slot_origin_pos = ego_slot_info.slot_corner[0] -
                                    ego_slot_info.slot_length * n -
                                    0.5 * ego_slot_info.slot_width * t;

  } else if (ego_slot_info.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
    n = -(ego_slot_info.slot_corner[0] - ego_slot_info.slot_corner[1])
             .normalized();
    t << -n.y(), n.x();
    ego_slot_info.slot_origin_pos = ego_slot_info.slot_corner[1] -
                                    ego_slot_info.slot_length * n +
                                    0.5 * ego_slot_info.slot_width * t;
  } else {
    ILOG_INFO << "side error";
    return false;
  }

  ego_slot_info.slot_origin_heading =
      pnc::geometry_lib::NormalizeAngle(std::atan2(n.y(), n.x()));

  ego_slot_info.slot_origin_heading_vec = n;

  ILOG_INFO << "origin heading ="
            << ego_slot_info.slot_origin_heading * kRad2Deg;

  ego_slot_info.g2l_tf.Init(ego_slot_info.slot_origin_pos,
                            ego_slot_info.slot_origin_heading);

  ego_slot_info.l2g_tf.Init(ego_slot_info.slot_origin_pos,
                            ego_slot_info.slot_origin_heading);

  ego_slot_info.ego_pos_slot =
      ego_slot_info.g2l_tf.GetPos(frame_->measurement_data_ptr->pos);

  ego_slot_info.ego_heading_slot =
      ego_slot_info.g2l_tf.GetHeading(frame_->measurement_data_ptr->heading);

  ego_slot_info.ego_heading_slot_vec
      << std::cos(ego_slot_info.ego_heading_slot),
      std::sin(ego_slot_info.ego_heading_slot);

  const double rac_to_center_dist =
      0.5 * config_->car_length - config_->rear_overhanging;

  ego_slot_info.target_ego_pos_slot
      << 0.5 * ego_slot_info.slot_length - rac_to_center_dist,
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

  const Eigen::Vector2d slot_center(ego_slot_info.select_slot.center().x(),
                                    ego_slot_info.select_slot.center().y());

  const double slot_side_sgn =
      ego_slot_info.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT ? 1.0
                                                                    : -1.0;
  const double max_obs_to_slot_dist_square = 12.0 * 12.0;

  int obs_in_slot = 0;
  for (const auto &obs_pt_g : frame_->obs_pt_vec) {
    const Eigen::Vector2d obs_to_sc = obs_pt_g - slot_center;

    const double obs_to_sc_dist_square =
        obs_to_sc.x() * obs_to_sc.x() + obs_to_sc.y() * obs_to_sc.y();

    if (obs_to_sc_dist_square > max_obs_to_slot_dist_square) {
      continue;
    }

    const auto obs_pt_slot = ego_slot_info.g2l_tf.GetPos(obs_pt_g);
    if (pnc::mathlib::IsInBound(obs_pt_slot.x(), 0.5,
                                ego_slot_info.slot_length - 0.5) &&
        pnc::mathlib::IsInBound(obs_pt_slot.y(), -0.4 * slot_side_sgn,
                                1.2 * slot_side_sgn)) {
      obs_in_slot++;
    }

    if (obs_in_slot > 2) {
      ILOG_INFO << "too many obs in slot";
      return false;
    }

    ego_slot_info.obs_pt_vec_slot.emplace_back(std::move(obs_pt_slot));
  }

  return true;
}

const bool RuleBasedSlotRelease::IsParallelSlotCoarseRelease(
    common::SlotInfo *slot, apa_planner::SlotInfoWindow *slot_history) {
  apa_planner::SlotManager::EgoSlotInfo ego_slot_info;
  const double lon_dist = CalLonDistSlot2Car(*slot);

  if (!UpdateEgoParallelSlotInfoInSearching(ego_slot_info, slot)) {
    slot->set_is_release(false);
    slot->set_is_occupied(true);
    ILOG_INFO << "Update Parallel In Searching, slot id = " << slot->id()
              << "  slot type = " << slot->slot_type()
              << "  is_release = " << slot->is_release();
    return false;
  }

  ILOG_INFO << "use fusion obs, lon_dist = " << lon_dist;
  // const size_t slot_id = static_cast<size_t>(slot->id());
  // frame_->obs_pt_map[slot_id] = frame_->obs_pt_vec;

  // ILOG_INFO <<"frame_->obs_pt_map[slot_id] size = "
  //             << frame_->obs_pt_map[slot_id].size());

  if (lon_dist < config_->min_parallel_vis_slot_release_long_dist_slot2mirror &&
      slot_history->GetOccupied()) {
    slot->set_is_release(false);
    slot->set_is_occupied(true);
  } else {
    slot_history->SetOccupied(false);
    slot->set_is_release(true);
    slot->set_is_occupied(false);
  }

  ILOG_INFO << "Parallel slot id = " << slot->id()
            << "  is_release = " << slot->is_release()
            << "  is_occupied = " << slot->is_occupied();

  return true;
}

const bool RuleBasedSlotRelease::IsPerpendicularSlotCoarseRelease(
    common::SlotInfo *slot, apa_planner::SlotInfoWindow *slot_history) {
  if (slot->slot_type() ==
          Common::ParkingSlotType::PARKING_SLOT_TYPE_SLANTING &&
      frame_->slot_info_direction.count(slot->id()) != 0) {
    if (!frame_->slot_info_direction[slot->id()]) {
      slot->set_is_release(false);
      slot->set_is_occupied(true);
      ILOG_INFO << "car and slot is no same direction slot id = " << slot->id()
                << "  slot type = " << slot->slot_type()
                << "  is_release = " << slot->is_release();
    }
  }

  const double lon_dist = CalLonDistSlot2Car(*slot);
  // ILOG_INFO <<"lon_dist = " << lon_dist);
  // ILOG_INFO <<"angle = " << CalAngleSlot2Car(*slot) * kRad2Deg);
  const double min_slot_release_long_dist =
      frame_->fus_obj_valid_flag
          ? -1.68
          : config_->min_slot_release_long_dist_slot2mirror;

  if (lon_dist < min_slot_release_long_dist && slot_history->GetOccupied()) {
    slot->set_is_release(false);
    slot->set_is_occupied(true);
    ILOG_INFO << "CalLonDistSlot2Car slot id = " << slot->id()
              << "  slot type = " << slot->slot_type()
              << "  is_release = " << slot->is_release();
    return false;
  } else {
    slot_history->SetOccupied(false);
  }

  if (IsSlotOccupied(slot)) {
    slot->set_is_release(false);
    slot->set_is_occupied(true);

    ILOG_INFO << "slot id = " << slot->id()
              << "  slot type = " << slot->slot_type() << "slot is occupied";
    return false;
  }

  if (!IsPassageAreaEnough(slot)) {
    slot->set_is_release(false);
    slot->set_is_occupied(true);

    ILOG_INFO << "slot id = " << slot->id()
              << "  slot type = " << slot->slot_type() << "passage is small";
    return false;
  }

  ILOG_INFO << "slot id = " << slot->id()
            << "  slot type = " << slot->slot_type()
            << "  is_release = " << slot->is_release();

  return true;
}

const bool RuleBasedSlotRelease::IsEgoCloseToObs(
    const LocalView *local_view, const MeasurementData *measures_ptr) {
  // move all obstacle related code to one place.
  PointCloudObstacleTransform transform;

  obs_list_.Clear();
  transform.GenerateGlobalObstacle(obs_list_, local_view, false);

  PathSafeChecker safe_check;
  Pose2D ego =
      Pose2D(measures_ptr->pos[0], measures_ptr->pos[1], measures_ptr->heading);
  bool collision = safe_check.CalcEgoCollision(&obs_list_, ego, 0.1, 0.1);

  return collision;
}

const bool RuleBasedSlotRelease::IsSlotOccupied(const common::SlotInfo *slot) {
  const auto slot_pts = slot->corner_points().corner_point();

  Eigen::Vector2d pt_0 = Eigen::Vector2d(slot_pts[0].x(), slot_pts[0].y());
  Eigen::Vector2d pt_1 = Eigen::Vector2d(slot_pts[1].x(), slot_pts[1].y());

  const Eigen::Vector2d pt_01_vec = pt_1 - pt_0;
  Eigen::Vector2d pt_01_vec_n(pt_01_vec.y(), -pt_01_vec.x());
  pt_01_vec_n.normalize();

  const Eigen::Vector2d pt_01_mid = (pt_0 + pt_1) * 0.5;

  Eigen::Vector2d target_pos =
      pt_01_mid -
      pt_01_vec_n * (config_->wheel_base + config_->front_overhanging);

  Pose2D target_pose;
  target_pose.x = target_pos[0];
  target_pose.y = target_pos[1];
  target_pose.theta = std::atan2(pt_01_vec_n[1], pt_01_vec_n[0]);

  PathSafeChecker safe_check;
  bool collision =
      safe_check.CalcEgoCollision(&obs_list_, target_pose, 0.05, 0.1);

  return collision ? true : false;
}

bool RuleBasedSlotRelease::IsPassageAreaEnough(const common::SlotInfo *slot) {
  // 暂时使用库口3米内没有障碍物判断.
  // 未来可以使用最大子矩阵和算法，来判定一个通道的大小.
  const auto slot_pts = slot->corner_points().corner_point();

  Eigen::Vector2d pt_0 = Eigen::Vector2d(slot_pts[0].x(), slot_pts[0].y());
  Eigen::Vector2d pt_1 = Eigen::Vector2d(slot_pts[1].x(), slot_pts[1].y());

  const Eigen::Vector2d pt_01_vec = pt_1 - pt_0;
  Eigen::Vector2d pt_01_vec_n(pt_01_vec.y(), -pt_01_vec.x());
  pt_01_vec_n.normalize();

  const Eigen::Vector2d pt_01_mid = (pt_0 + pt_1) * 0.5;

  // check length
  Polygon2D polygon;
  polygon.vertexes[0].x = (pt_0)[0];
  polygon.vertexes[0].y = (pt_0)[1];
  polygon.vertexes[1].x = (pt_0 + pt_01_vec_n * 3.0)[0];
  polygon.vertexes[1].y = (pt_0 + pt_01_vec_n * 3.0)[1];

  polygon.vertexes[2].x = (pt_1 + pt_01_vec_n * 3.0)[0];
  polygon.vertexes[2].y = (pt_1 + pt_01_vec_n * 3.0)[1];
  polygon.vertexes[3].x = (pt_1)[0];
  polygon.vertexes[3].y = (pt_1)[1];

  polygon.vertex_num = 4;
  polygon.shape = PolygonShape::box;
  UpdatePolygonValue(&polygon, NULL, 0, false, POLYGON_MAX_RADIUS);

  polygon.min_tangent_radius = 0.6;

  PathSafeChecker safe_check;

  safe_check.SetObstacle(&obs_list_);
  bool collision = safe_check.IsPolygonCollision(&polygon);

  return collision ? false : true;
}

}  // namespace apa_planner
}  // namespace planning
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

    // only extra protect, it can delete to be fast
    if (fusion_slot_map.count(slot->id()) == 0) {
      slot->set_is_release(false);
      slot->set_is_occupied(true);
      continue;
    }

    if (is_ego_collision) {
      slot->set_is_release(false);
      slot->set_is_occupied(true);
      continue;
    }

    if (fusion_slot_map[static_cast<size_t>(slot->id())].allow_parking == 1) {
      slot->set_is_release(true);
    } else {
      slot->set_is_release(false);
    }
    slot->set_is_occupied(!slot->is_release());

    // distance is big
    if (pair.first > 100.0) {
      slot->set_is_release(false);
      slot->set_is_occupied(true);
      ILOG_INFO << "distance is big";
    }

    if (slot->corner_points().corner_point_size() != 4) {
      slot->set_is_release(false);
      slot->set_is_occupied(true);
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
      if (IflyTime::Now_ms() - time_start >
          config_->prepare_single_max_allow_time) {
        slot->set_is_release(false);
        slot->set_is_occupied(true);
        continue;
      }

      IsPerpendicularSlotCoarseRelease(slot, pair.second);

    } else if (slot->slot_type() ==
                   Common::ParkingSlotType::PARKING_SLOT_TYPE_HORIZONTAL &&
               slot->is_release()) {
      IsParallelSlotCoarseRelease(slot, pair.second);

    } else {
      ILOG_INFO << "use uss obs";
      // select nearby obs pt from ori USS pt for given slot
      AddUssPerceptObstacles(*slot);
      const double lon_dist = CalLonDistSlot2Car(*slot);

      if (lon_dist <
              config_->min_parallel_uss_slot_release_long_dist_slot2mirror &&
          pair.second->GetOccupied()) {
        slot->set_is_release(false);
        slot->set_is_occupied(true);
      } else {
        pair.second->SetOccupied(false);
        slot->set_is_release(true);
        slot->set_is_occupied(false);
      }
    }
  }

  return;
}

// select nearby obs pt from ori USS pt for given slot
const bool RuleBasedSlotRelease::AddUssPerceptObstacles(
    const common::SlotInfo &slot_info) {
  // tmp: no consider obs point
  if (config_->force_both_side_occupied ||
      frame_->uss_percept_info_ptr == NULL) {
    frame_->obs_pt_map.clear();
    return false;
  }
  if (frame_->uss_percept_info_ptr->out_line_dataori[0].obj_pt_cnt == 0) {
    ILOG_INFO << "obs is empty";
    frame_->obs_pt_map.clear();
    return false;
  }
  const auto &obj_info_desample =
      frame_->uss_percept_info_ptr->out_line_dataori[0];  // 0 means desample
                                                          // while 1 means
                                                          // raw model output

  const size_t selected_id = slot_info.id();
  frame_->obs_pt_map.erase(selected_id);

  const Eigen::Vector2d slot_center(slot_info.center().x(),
                                    slot_info.center().y());

  double filtered_obs_dis = config_->obs2slot_max_dist;
  if (slot_info.slot_type() == Common::PARKING_SLOT_TYPE_HORIZONTAL) {
    filtered_obs_dis = config_->parallel_obs2slot_max_dist;
  } else if (slot_info.slot_type() == Common::PARKING_SLOT_TYPE_SLANTING &&
             frame_->slot_info_angle.count(slot_info.id()) != 0) {
    filtered_obs_dis = config_->obs2slot_max_dist /
                       frame_->slot_info_angle[slot_info.id()].second;
  }

  Eigen::Vector2d obs_pt;
  std::vector<Eigen::Vector2d> slot_obs_vec;
  for (uint32 i = 0; i < obj_info_desample.obj_pt_cnt; ++i) {
    obs_pt << obj_info_desample.obj_pt_global[i].x,
        obj_info_desample.obj_pt_global[i].y;
    const double dist = (slot_center - obs_pt).norm();
    // todo: consider dist from ego to obs
    if (dist < filtered_obs_dis) {
      slot_obs_vec.emplace_back(obs_pt);
    }
  }
  if (!slot_obs_vec.empty()) {
    ILOG_INFO << "there are obs around slot " << selected_id;
  } else {
    ILOG_INFO << "there are no obs around slot " << selected_id;
  }
  frame_->obs_pt_map[selected_id] = slot_obs_vec;
  return true;
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

// used in searching state
const bool RuleBasedSlotRelease::UpdateEgoSlotInfo(
    apa_planner::SlotManager::EgoSlotInfo &ego_slot_info,
    const common::SlotInfo *slot_info) {
  const auto &slot_points = slot_info->corner_points().corner_point();
  if (slot_points.size() < 4) {
    ILOG_INFO << "slot_points size is not normal, quit";
    return false;
  }

  ego_slot_info.slot_type = slot_info->slot_type();
  ego_slot_info.select_slot_id = slot_info->id();

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

  ego_slot_info.slot_corner = pt;

  // const double virtual_slot_length =
  //     config_->car_length +
  //     config_->slot_compare_to_car_length;

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

  ego_slot_info.ego_pos_slot =
      ego_slot_info.g2l_tf.GetPos(frame_->measurement_data_ptr->pos);

  ego_slot_info.ego_heading_slot =
      ego_slot_info.g2l_tf.GetHeading(frame_->measurement_data_ptr->heading);

  ego_slot_info.ego_heading_slot_vec
      << std::cos(ego_slot_info.ego_heading_slot),
      std::sin(ego_slot_info.ego_heading_slot);

  ego_slot_info.target_ego_pos_slot << config_->terminal_target_x,
      config_->terminal_target_y;

  ego_slot_info.target_ego_heading_slot = config_->terminal_target_heading;

  ego_slot_info.sin_angle = 1.0;
  ego_slot_info.origin_pt_0_heading = 0.0;
  ego_slot_info.pt_0 = ego_slot_info.g2l_tf.GetPos(pt[0]);
  ego_slot_info.pt_1 = ego_slot_info.g2l_tf.GetPos(pt[1]);
  if (slot_info->slot_type() == Common::PARKING_SLOT_TYPE_SLANTING) {
    if (frame_->slot_info_angle.count(slot_info->id()) != 0) {
      ego_slot_info.sin_angle = frame_->slot_info_angle[slot_info->id()].second;
      ego_slot_info.origin_pt_0_heading =
          90.0 - frame_->slot_info_angle[slot_info->id()].first;
      ego_slot_info.pt_0 = ego_slot_info.g2l_tf.GetPos(
          frame_->slot_info_corner_01[slot_info->id()].first);
      ego_slot_info.pt_1 = ego_slot_info.g2l_tf.GetPos(
          frame_->slot_info_corner_01[slot_info->id()].second);
    }
  }
  if (ego_slot_info.pt_0.y() > ego_slot_info.pt_1.y()) {
    std::swap(ego_slot_info.pt_0, ego_slot_info.pt_1);
  }

  ego_slot_info.obs_pt_vec_slot.clear();
  ego_slot_info.fus_obj_valid_flag = frame_->fus_obj_valid_flag;
  if (!ego_slot_info.fus_obj_valid_flag) {
    // use uss obs
    if (frame_->obs_pt_map.count(slot_info->id()) == 0) {
      return true;
    }
    const auto &obs_pt_vec = frame_->obs_pt_map[slot_info->id()];
    ego_slot_info.obs_pt_vec_slot.reserve(obs_pt_vec.size());
    for (const auto &obs_pt : obs_pt_vec) {
      const auto obs_pt_slot = ego_slot_info.g2l_tf.GetPos(obs_pt);
      ego_slot_info.obs_pt_vec_slot.emplace_back(std::move(obs_pt_slot));
    }
  }

  else {
    // use fus obj and ground line
    ego_slot_info.obs_pt_vec_slot.reserve(frame_->obs_pt_vec.size());
    // obs global coord transform to local coord
    uint8_t obs_in_slot_count = 0;
    const uint8_t max_obs_in_slot_count = 5;
    for (const auto &obs_pt : frame_->obs_pt_vec) {
      const Eigen::Vector2d obs_pt_slot = ego_slot_info.g2l_tf.GetPos(obs_pt);
      if (std::fabs(obs_pt_slot.y()) < 0.98 &&
          obs_pt_slot.x() >
              std::min(ego_slot_info.pt_0.x(), ego_slot_info.pt_1.x()) -
                  config_->car_length + 0.86 &&
          obs_pt_slot.x() <
              std::min(ego_slot_info.pt_0.x(), ego_slot_info.pt_1.x()) +
                  0.168) {
        obs_in_slot_count++;
      }
      ego_slot_info.obs_pt_vec_slot.emplace_back(std::move(obs_pt_slot));
    }
    if (obs_in_slot_count > max_obs_in_slot_count) {
      ILOG_INFO << "there are too obs in slot, no release the slot";
      return false;
    }
  }

  return true;
}

const bool RuleBasedSlotRelease::GenTLane(
    apa_planner::SlotManager::EgoSlotInfo &ego_slot_info,
    apa_planner::PerpendicularTailInPathGenerator::Tlane &slot_tlane,
    apa_planner::PerpendicularTailInPathGenerator::Tlane &obs_tlane) {
  using namespace pnc::geometry_lib;

  const Eigen::Vector2d pM01 =
      0.5 * (ego_slot_info.slot_corner[0] + ego_slot_info.slot_corner[1]);

  const Eigen::Vector2d pM23 =
      0.5 * (ego_slot_info.slot_corner[2] + ego_slot_info.slot_corner[3]);

  Eigen::Vector2d ego_to_slot_center_vec =
      0.5 * (pM01 + pM23) - frame_->measurement_data_ptr->pos;

  const double cross_ego_to_slot_center =
      pnc::geometry_lib::GetCrossFromTwoVec2d(
          frame_->measurement_data_ptr->heading_vec, ego_to_slot_center_vec);

  const double cross_ego_to_slot_heading =
      pnc::geometry_lib::GetCrossFromTwoVec2d(
          frame_->measurement_data_ptr->heading_vec,
          ego_slot_info.slot_origin_heading_vec);

  if (cross_ego_to_slot_heading > 0.0 && cross_ego_to_slot_center < 0.0) {
    slot_tlane.slot_side = pnc::geometry_lib::SLOT_SIDE_RIGHT;
  } else if (cross_ego_to_slot_heading < 0.0 &&
             cross_ego_to_slot_center > 0.0) {
    slot_tlane.slot_side = pnc::geometry_lib::SLOT_SIDE_LEFT;
  } else {
    slot_tlane.slot_side = pnc::geometry_lib::SLOT_SIDE_INVALID;
    return false;
  }

  const Eigen::Vector2d pt_01_norm_vec =
      (ego_slot_info.pt_1 - ego_slot_info.pt_0).normalized();
  const Eigen::Vector2d pt_10_norm_vec =
      (ego_slot_info.pt_0 - ego_slot_info.pt_1).normalized();
  const Eigen::Vector2d pt_01_norm_up_vec(pt_01_norm_vec.y(),
                                          -pt_01_norm_vec.x());
  const Eigen::Vector2d pt_01_norm_down_vec(-pt_01_norm_vec.y(),
                                            pt_01_norm_vec.x());

  const double x_max =
      ((ego_slot_info.pt_1 + ego_slot_info.pt_0) * 0.5 +
       config_->obs_consider_long_threshold * pt_01_norm_up_vec)
          .x();

  const double y_max =
      std::fabs((ego_slot_info.pt_1 +
                 config_->obs_consider_lat_threshold * pt_01_norm_vec)
                    .y());

  // construct channel width and length, only for uss obs
  if (!ego_slot_info.fus_obj_valid_flag) {
    std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>, Compare>
        channel_width_pq_for_x(Compare(1));
    for (const auto &obstacle_point_slot : ego_slot_info.obs_pt_vec_slot) {
      if (obstacle_point_slot.x() < x_max ||
          std::fabs(obstacle_point_slot.y()) > y_max) {
        continue;
      }
      channel_width_pq_for_x.emplace(obstacle_point_slot);
    }
    if (channel_width_pq_for_x.empty()) {
      channel_width_pq_for_x.emplace(Eigen::Vector2d(
          ((ego_slot_info.pt_1 + ego_slot_info.pt_0) * 0.5).x() +
              config_->channel_width,
          0.0));
    }

    const double channel_width =
        channel_width_pq_for_x.top().x() -
        ((ego_slot_info.pt_1 + ego_slot_info.pt_0) * 0.5).x();

    ego_slot_info.channel_width = pnc::mathlib::DoubleConstrain(
        channel_width, config_->min_channel_width, config_->channel_width);
  } else {
    // use fus obs
    const double channel_width =
        frame_->collision_detector_ptr->GetCarMaxX(pnc::geometry_lib::PathPoint(
            ego_slot_info.ego_pos_slot, ego_slot_info.ego_heading_slot)) +
        3.068 - std::max(ego_slot_info.pt_0.x(), ego_slot_info.pt_1.x());
    ego_slot_info.channel_width =
        std::max(channel_width, config_->channel_width);
  }

  // ILOG_INFO <<"channel_width = " << ego_slot_info.channel_width);

  // construct tlane pq
  // left y is positive, right y is negative
  // left y should be smallest, right y should be largest
  // all x should be largest
  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>, Compare>
      left_pq_for_y(Compare(3));
  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>, Compare>
      left_pq_for_x(Compare(0));
  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>, Compare>
      right_pq_for_y(Compare(2));
  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>, Compare>
      right_pq_for_x(Compare(0));

  // only hack for obs is not accurate
  // const double y_min = ego_slot_info.slot_width * 0.5 - 0.068;
  // const double x_min = ((ego_slot_info.pt_1 + ego_slot_info.pt_0) * 0.5 +
  //                       4.28 * pt_01_norm_down_vec)
  //                          .x();

  apa_planner::CollisionDetector::ObsSlotType obs_slot_type;
  const std::pair<Eigen::Vector2d, Eigen::Vector2d> slot_pt =
      std::make_pair(ego_slot_info.pt_1, ego_slot_info.pt_0);
  const bool is_left_side =
      (slot_tlane.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT);
  double max_obs_lat_invasion_slot_dist = 0.0;
  const double mir_width = (config_->max_car_width - config_->car_width) * 0.5;

  const double mir_x = ego_slot_info.target_ego_pos_slot.x() +
                       config_->lon_dist_mirror_to_rear_axle - 0.368;
  // sift obstacles that meet requirement
  for (Eigen::Vector2d obstacle_point_slot : ego_slot_info.obs_pt_vec_slot) {
    // if (std::fabs(obstacle_point_slot.x()) > x_max ||
    //     obstacle_point_slot.x() < x_min ||
    //     std::fabs(obstacle_point_slot.y()) > y_max ||
    //     std::fabs(obstacle_point_slot.y()) < y_min) {
    //   continue;
    // }
    obs_slot_type = frame_->collision_detector_ptr->GetObsSlotType(
        obstacle_point_slot, slot_pt, is_left_side, frame_->replan_flag);

    if (obs_slot_type ==
            apa_planner::CollisionDetector::ObsSlotType::SLOT_IN_OBS &&
        !config_->believe_in_fus_obs) {
      max_obs_lat_invasion_slot_dist =
          frame_->replan_flag
              ? config_->max_obs_lat_invasion_slot_dist
              : config_->max_obs_lat_invasion_slot_dist_dynamic_col;
      const double min_left_y =
          0.5 * ego_slot_info.slot_width - max_obs_lat_invasion_slot_dist;
      const double max_right_y =
          -0.5 * ego_slot_info.slot_width + max_obs_lat_invasion_slot_dist;
      // obs is in slot, temp hack, when believe_in_fus_obs is false,
      // force move obs to out slot
      if (obstacle_point_slot.y() < min_left_y &&
          obstacle_point_slot.y() > 0.0) {
        obstacle_point_slot.y() = min_left_y;
      }
      if (obstacle_point_slot.y() > max_right_y &&
          obstacle_point_slot.y() < 0.0) {
        obstacle_point_slot.y() = max_right_y;
      }
    } else if (obs_slot_type != apa_planner::CollisionDetector::ObsSlotType::
                                    SLOT_INSIDE_OBS &&
               obs_slot_type != apa_planner::CollisionDetector::ObsSlotType::
                                    SLOT_OUTSIDE_OBS) {
      continue;
    }
    // the obs lower mir can relax requirements
    if (obstacle_point_slot.x() < mir_x) {
      if (obstacle_point_slot.y() > 1e-6) {
        obstacle_point_slot.y() += mir_width;
      } else {
        obstacle_point_slot.y() -= mir_width;
      }
    }
    // the obs far from slot can relax requirements
    if (std::fabs(obstacle_point_slot.y()) >
        ego_slot_info.slot_width * 0.5 + 0.468) {
      obstacle_point_slot.x() -= 0.268;
    }
    if (obstacle_point_slot.y() > 1e-6) {
      left_pq_for_y.emplace(std::move(obstacle_point_slot));
      left_pq_for_x.emplace(std::move(obstacle_point_slot));
    } else {
      right_pq_for_y.emplace(std::move(obstacle_point_slot));
      right_pq_for_x.emplace(std::move(obstacle_point_slot));
    }
  }

  apa_param.SetPram().actual_mono_plan_enable = config_->mono_plan_enable;
  if (config_->conservative_mono_enable) {
    if (!left_pq_for_x.empty() || !right_pq_for_x.empty()) {
      apa_param.SetPram().actual_mono_plan_enable = false;
    }
  }

  // If there are no obstacles on either side, set up a virtual obstacle
  // that is farther away
  const double virtual_x = ((ego_slot_info.pt_1 + ego_slot_info.pt_0) * 0.5 +
                            config_->virtual_obs_x_pos * pt_01_norm_down_vec)
                               .x();

  const double virtual_left_y =
      (ego_slot_info.pt_1 + config_->virtual_obs_y_pos * pt_01_norm_vec).y();
  const double virtual_right_y =
      (ego_slot_info.pt_0 + config_->virtual_obs_y_pos * pt_10_norm_vec).y();

  bool left_empty = false;
  bool right_empty = false;

  if (left_pq_for_x.empty()) {
    ILOG_INFO << "left space is empty";
    left_empty = true;
    left_pq_for_x.emplace(Eigen::Vector2d(virtual_x, 0.0));
    left_pq_for_y.emplace(Eigen::Vector2d(0.0, virtual_left_y));
  }
  if (right_pq_for_x.empty()) {
    ILOG_INFO << "right space is empty";
    right_empty = true;
    right_pq_for_x.emplace(Eigen::Vector2d(virtual_x, 0.0));
    right_pq_for_y.emplace(Eigen::Vector2d(0.0, virtual_right_y));
  }

  // If the car is parked according to the actual slot, its leftmost and
  // rightmost coordinates which includes rearview mirror are as follows
  const double car_half_width_with_mirror = config_->max_car_width * 0.5;

  const double virtual_slot_width =
      config_->max_car_width + config_->slot_compare_to_car_width;

  const double real_slot_width = ego_slot_info.slot_width;

  ILOG_INFO << "max_car_width = " << config_->max_car_width
            << "  virtual slot width = " << virtual_slot_width
            << "  real slot width = " << real_slot_width;

  double left_y = left_pq_for_y.top().y();
  double real_left_y = left_y;

  double left_x = left_pq_for_x.top().x();
  double real_left_x = left_x;

  double right_y = right_pq_for_y.top().y();
  double real_right_y = right_y;

  double right_x = right_pq_for_x.top().x();
  double real_right_x = right_x;

  // temp hack, when use uss obs, obs lat y is set virtual value
  if (config_->tmp_no_consider_obs_dy && !ego_slot_info.fus_obj_valid_flag) {
    if (!left_empty) {
      left_y = real_slot_width * 0.5 + config_->tmp_virtual_obs_dy;
    }
    if (!right_empty) {
      right_y = -real_slot_width * 0.5 - config_->tmp_virtual_obs_dy;
    }
  }

  // set t lane area
  const double threshold = 0.6868;
  if (ego_slot_info.fus_obj_valid_flag) {
    left_y = std::max(left_y, car_half_width_with_mirror + threshold);
    left_y = std::min(left_y, virtual_left_y);
    left_x = std::min(left_x, ego_slot_info.pt_1.x() - 1.68 * threshold);
    left_x = std::max(left_x, virtual_x);
    right_y = std::min(right_y, -car_half_width_with_mirror - threshold);
    right_y = std::max(right_y, virtual_right_y);
    right_x = std::min(right_x, ego_slot_info.pt_0.x() - 1.68 * threshold);
    right_x = std::max(right_x, virtual_x);
  }

  // ILOG_INFO <<"real_left_y = " << real_left_y
  //                              << "  real_right_y = " << real_right_y);

  // ILOG_INFO <<"left_y = " << left_y << "  right_y = " << right_y
  //                         << "  left_x = " << left_x
  //                         << "  right_x = " << right_x);

  // todo: consider actual obs pos to let slot release or not release or
  // move target pose
  double left_dis_obs_car = 0.0;
  double right_dis_obs_car = 0.0;
  if (ego_slot_info.fus_obj_valid_flag) {
    // use fus obj
    left_dis_obs_car = real_left_y - car_half_width_with_mirror;
    right_dis_obs_car = -car_half_width_with_mirror - real_right_y;
  } else {
    // use uss obj
    left_dis_obs_car = left_y - car_half_width_with_mirror;
    right_dis_obs_car = -car_half_width_with_mirror - right_y;
  }

  ILOG_INFO << "left_dis_obs_car = " << left_dis_obs_car
            << "  right_dis_obs_car = " << right_dis_obs_car;

  const double safe_threshold =
      config_->car_lat_inflation_normal + config_->safe_threshold;

  // ensure it can move slot to make both side safe
  if (left_dis_obs_car + right_dis_obs_car < 2.0 * safe_threshold) {
    ILOG_INFO << "obs to slot safe dist doesnot meet safe_threshold";
    return false;
  }
  bool left_obs_meet_safe_require = false;
  bool right_obs_meet_safe_require = false;
  left_obs_meet_safe_require = left_dis_obs_car > safe_threshold ? true : false;
  right_obs_meet_safe_require =
      right_dis_obs_car > safe_threshold ? true : false;

  bool need_move_slot = false;
  double move_slot_dist = 0.0;
  if (!left_obs_meet_safe_require) {
    // left side is dangerous, should move toward right
    move_slot_dist = safe_threshold - left_dis_obs_car;
    move_slot_dist *= -1.0;
    need_move_slot = true;
  } else if (!right_obs_meet_safe_require) {
    // right side is dangerous, should move toward left
    move_slot_dist = safe_threshold - right_dis_obs_car;
    need_move_slot = true;
  }

  if (need_move_slot) {
    // cal max_move_slot_dist to avoid car press line
    // no consider mirror
    const double half_car_width = config_->car_width * 0.5;
    const double half_slot_width = ego_slot_info.slot_width * 0.5;
    const double car2line_dist_threshold = config_->car2line_dist_threshold;

    // first sure if car is parked in the center, does it meet the slot line
    // distance requirement
    const double max_move_slot_dist =
        half_slot_width - half_car_width - car2line_dist_threshold;
    if (max_move_slot_dist > 0.0 &&
        (std::fabs(move_slot_dist) > max_move_slot_dist)) {
      if (move_slot_dist > 0.0) {
        move_slot_dist = max_move_slot_dist;
      }
      if (move_slot_dist < 0.0) {
        move_slot_dist = -max_move_slot_dist;
      }
    }
  }

  // construct slot_t_lane_, left is positive, right is negative
  const double slot_width = std::min(virtual_slot_width, real_slot_width);

  Eigen::Vector2d corner_left_slot(ego_slot_info.slot_length, 0.5 * slot_width);

  Eigen::Vector2d corner_right_slot(ego_slot_info.slot_length,
                                    -0.5 * slot_width);

  const uint8_t &slot_side = slot_tlane.slot_side;
  if (slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
    // inside is right, outside is left
    slot_tlane.corner_outside_slot = corner_left_slot;
    slot_tlane.corner_inside_slot = corner_right_slot;
    slot_tlane.pt_outside = corner_left_slot;
    slot_tlane.pt_inside = corner_right_slot;
    slot_tlane.pt_inside.x() =
        std::min(real_right_x, ego_slot_info.pt_0.x() + 2.68) +
        config_->tlane_safe_dx;
    // slot_tlane.pt_inside.y() =
    //     std::max(real_right_y, ego_slot_info.pt_0.y() + 0.05);
  } else if (slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
    // outside is right, inside is left
    slot_tlane.corner_outside_slot = corner_right_slot;
    slot_tlane.corner_inside_slot = corner_left_slot;
    slot_tlane.pt_outside = corner_right_slot;
    slot_tlane.pt_inside = corner_left_slot;
    slot_tlane.pt_inside.x() =
        std::min(real_left_x, ego_slot_info.pt_1.x() + 2.68) +
        config_->tlane_safe_dx;
    // slot_tlane.pt_inside.y() =
    //     std::max(real_left_y, ego_slot_info.pt_1.y() - 0.05);
  }

  slot_tlane.pt_terminal_pos << ego_slot_info.target_ego_pos_slot.x(),
      ego_slot_info.target_ego_pos_slot.y();

  slot_tlane.pt_terminal_heading = ego_slot_info.target_ego_heading_slot;

  if (need_move_slot) {
    slot_tlane.pt_terminal_pos.y() += move_slot_dist;
    slot_tlane.pt_inside.y() += move_slot_dist;
    slot_tlane.pt_outside.y() += move_slot_dist;
    // ILOG_INFO <<
    //     "should move slot according to obs pt, move dist = " <<
    //     move_slot_dist);
  }

  slot_tlane.pt_lower_boundry_pos = slot_tlane.pt_terminal_pos;
  // subtrace 0.05 to avoid plan failure due to col det
  slot_tlane.pt_lower_boundry_pos.x() =
      slot_tlane.pt_lower_boundry_pos.x() - config_->rear_overhanging -
      config_->col_obs_safe_dist_normal - 0.05;

  // construct obstacle_t_lane_
  // for onstacle_t_lane    right is inside, left is outside
  obs_tlane.slot_side = slot_side;
  if (slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
    obs_tlane.pt_inside.x() = right_x + config_->obs_safe_dx;
    obs_tlane.pt_inside.y() = right_y;
    obs_tlane.pt_outside.x() = left_x + config_->obs_safe_dx;
    obs_tlane.pt_outside.y() = left_y;
  } else if (slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
    obs_tlane.pt_inside.x() = left_x + config_->obs_safe_dx;
    obs_tlane.pt_inside.y() = left_y;
    obs_tlane.pt_outside.x() = right_x + config_->obs_safe_dx;
    obs_tlane.pt_outside.y() = right_y;
  }

  obs_tlane.pt_terminal_pos = slot_tlane.pt_terminal_pos;
  obs_tlane.pt_terminal_heading = slot_tlane.pt_terminal_heading;
  obs_tlane.pt_lower_boundry_pos = slot_tlane.pt_lower_boundry_pos;

  // tmp method, obstacle is temporarily unavailable, force lift slot_t_lane
  // pt_inside and pt_outside
  if (config_->force_both_side_occupied) {
    slot_tlane.pt_inside.x() = corner_right_slot.x();
    slot_tlane.pt_outside.x() = corner_left_slot.x();

    slot_tlane.pt_inside += Eigen::Vector2d(config_->occupied_pt_inside_dx,
                                            config_->occupied_pt_inside_dy);

    slot_tlane.pt_outside += Eigen::Vector2d(config_->occupied_pt_outside_dx,
                                             config_->occupied_pt_outside_dy);
  }

  ILOG_INFO << "t_lane.pt_inside.x() = " << slot_tlane.pt_inside.x();

  return true;
}

const bool RuleBasedSlotRelease::GenObstacles(
    const apa_planner::PerpendicularTailInPathGenerator::Tlane &slot_tlane,
    apa_planner::PerpendicularTailInPathGenerator::Tlane &obs_tlane,
    const apa_planner::SlotManager::EgoSlotInfo &ego_slot_info) {
  if (!frame_->collision_detector_ptr) {
    return false;
  }
  frame_->collision_detector_ptr->ClearObstacles();
  // set obstacles
  double channel_width = ego_slot_info.channel_width;
  double channel_length = config_->channel_length;

  if (config_->force_both_side_occupied) {
    obs_tlane = slot_tlane;
  }

  // add tlane obstacle
  //  B is always outside
  int slot_side = 1;
  bool is_left_side = false;
  if (obs_tlane.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
    slot_side = -1;
    is_left_side = false;
  } else if (obs_tlane.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
    slot_side = 1;
    is_left_side = true;
  }
  Eigen::Vector2d B(obs_tlane.pt_outside);
  const Eigen::Vector2d pt_01_vec = ego_slot_info.pt_1 - ego_slot_info.pt_0;
  const Eigen::Vector2d pt_01_norm_vec = pt_01_vec.normalized();
  const double obs_length = (channel_length - pt_01_vec.norm()) * 0.5;
  Eigen::Vector2d A = B - slot_side * pt_01_norm_vec * obs_length;

  Eigen::Vector2d C(obs_tlane.pt_lower_boundry_pos);
  C.y() = B.y();

  Eigen::Vector2d E(obs_tlane.pt_inside);
  Eigen::Vector2d D(obs_tlane.pt_lower_boundry_pos);
  D.y() = E.y();

  Eigen::Vector2d F = E + slot_side * pt_01_norm_vec * obs_length;

  // add channel obstacle
  const double pt_01_x = ((ego_slot_info.pt_0 + ego_slot_info.pt_1) * 0.5).x();
  const double top_x = pt_01_x + channel_width / ego_slot_info.sin_angle;
  Eigen::Vector2d channel_point_1 =
      Eigen::Vector2d(top_x, 0.0) -
      slot_side * pt_01_norm_vec * channel_length * 0.5;
  Eigen::Vector2d channel_point_2 =
      Eigen::Vector2d(top_x, 0.0) +
      slot_side * pt_01_norm_vec * channel_length * 0.5;

  Eigen::Vector2d channel_point_3;
  pnc::geometry_lib::LineSegment channel_line;
  std::vector<pnc::geometry_lib::LineSegment> channel_line_vec;
  channel_point_3 = F;
  channel_point_2.y() = channel_point_3.y();
  channel_line.SetPoints(channel_point_2, channel_point_3);
  channel_line_vec.emplace_back(channel_line);
  channel_point_3 = A;
  channel_point_1.y() = channel_point_3.y();
  channel_line.SetPoints(channel_point_1, channel_point_3);
  channel_line_vec.emplace_back(channel_line);
  channel_line.SetPoints(channel_point_1, channel_point_2);
  channel_line_vec.emplace_back(channel_line);

  const double ds = config_->obstacle_ds;
  std::vector<Eigen::Vector2d> point_set;
  std::vector<Eigen::Vector2d> channel_obstacle_vec;
  channel_obstacle_vec.clear();
  channel_obstacle_vec.reserve(68);
  for (const auto &line : channel_line_vec) {
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, line, ds);
    channel_obstacle_vec.insert(channel_obstacle_vec.end(), point_set.begin(),
                                point_set.end());
  }

  frame_->collision_detector_ptr->SetObstacles(
      channel_obstacle_vec, apa_planner::CollisionDetector::CHANNEL_OBS);

  pnc::geometry_lib::LineSegment tlane_line;
  std::vector<pnc::geometry_lib::LineSegment> tlane_line_vec;
  tlane_line.SetPoints(A, B);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(B, C);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(C, D);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(D, E);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(E, F);
  tlane_line_vec.emplace_back(tlane_line);

  // tmp method, should modify
  std::vector<Eigen::Vector2d> tlane_obstacle_vec;
  tlane_obstacle_vec.clear();
  tlane_obstacle_vec.reserve(88);
  for (const auto &line : tlane_line_vec) {
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, line, ds);
    tlane_obstacle_vec.insert(tlane_obstacle_vec.end(), point_set.begin(),
                              point_set.end());
  }
  pnc::geometry_lib::PathPoint ego_pose;
  ego_pose.Set(frame_->ego_slot_info.ego_pos_slot,
               frame_->ego_slot_info.ego_heading_slot);

  if (!frame_->fus_obj_valid_flag) {
    // when no fus obj, temp hack, only increase plan success ratio
    double safe_dist = config_->max_obs2car_dist_out_slot;
    if (frame_->ego_slot_info.slot_occupied_ratio >
            config_->max_obs2car_dist_slot_occupied_ratio &&
        std::fabs(frame_->ego_slot_info.terminal_err.heading) * kRad2Deg <
            36.6) {
      safe_dist = config_->max_obs2car_dist_in_slot;
    }
    for (const auto &obs_pos : tlane_obstacle_vec) {
      if (!frame_->collision_detector_ptr->IsObstacleInCar(obs_pos, ego_pose,
                                                           safe_dist)) {
        frame_->collision_detector_ptr->AddObstacles(
            obs_pos, apa_planner::CollisionDetector::TLANE_OBS);
      }
    }
  }

  else {
    const double safe_dist = 0.0268;
    // add virtual tlane obs
    std::vector<Eigen::Vector2d> tlane_obs_vec;
    tlane_obs_vec.reserve(tlane_obstacle_vec.size());
    for (const Eigen::Vector2d &obs_pos : tlane_obstacle_vec) {
      if (!frame_->collision_detector_ptr->IsObstacleInCar(obs_pos, ego_pose,
                                                           safe_dist)) {
        tlane_obs_vec.emplace_back(obs_pos);
      }
    }
    frame_->collision_detector_ptr->AddObstacles(
        tlane_obs_vec, apa_planner::CollisionDetector::TLANE_OBS);

    // add actual fus obs
    Eigen::Vector2d pt_left = obs_tlane.pt_outside;
    Eigen::Vector2d pt_right = obs_tlane.pt_inside;
    if (obs_tlane.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
      std::swap(pt_left, pt_right);
    }
    double max_obs_lat_invasion_slot_dist = 0.0;
    std::vector<Eigen::Vector2d> fus_obs_vec;
    std::pair<Eigen::Vector2d, Eigen::Vector2d> slot_pt =
        std::make_pair(ego_slot_info.pt_1, ego_slot_info.pt_0);
    apa_planner::CollisionDetector::ObsSlotType obs_slot_type;
    std::vector<Eigen::Vector2d> tlane_vec;
    tlane_vec.emplace_back(A);
    tlane_vec.emplace_back(B);
    tlane_vec.emplace_back(C);
    tlane_vec.emplace_back(D);
    tlane_vec.emplace_back(E);
    tlane_vec.emplace_back(F);
    tlane_vec.emplace_back(channel_point_2);
    tlane_vec.emplace_back(channel_point_1);

    if (!pnc::geometry_lib::IsPointInPolygon(tlane_vec,
                                             ego_slot_info.ego_pos_slot)) {
      ILOG_INFO << "ego pos is not in tlane area, the slot should not release.";
      return false;
    }

    for (Eigen::Vector2d obs_pos : ego_slot_info.obs_pt_vec_slot) {
      obs_slot_type = frame_->collision_detector_ptr->GetObsSlotType(
          obs_pos, slot_pt, is_left_side, frame_->replan_flag);

      if (!config_->believe_in_fus_obs) {
        if (obs_slot_type == apa_planner::CollisionDetector::ObsSlotType::
                                 SLOT_ENTRANCE_OBS &&
            frame_->replan_flag) {
          // obs is slot entrance, when replan, no conside it, but when dynamic
          // move and col det, conside it
          continue;
        }

        if (obs_slot_type ==
            apa_planner::CollisionDetector::ObsSlotType::SLOT_IN_OBS) {
          max_obs_lat_invasion_slot_dist =
              frame_->replan_flag
                  ? config_->max_obs_lat_invasion_slot_dist
                  : config_->max_obs_lat_invasion_slot_dist_dynamic_col;
          const double min_left_y =
              0.5 * ego_slot_info.slot_width - max_obs_lat_invasion_slot_dist;
          const double max_right_y =
              -0.5 * ego_slot_info.slot_width + max_obs_lat_invasion_slot_dist;
          // obs is in slot, temp hack, when believe_in_fus_obs is false,
          // force move obs to out slot
          if (obs_pos.y() < min_left_y && obs_pos.y() > 0.0) {
            obs_pos.y() = min_left_y;
          }
          if (obs_pos.y() > max_right_y && obs_pos.y() < 0.0) {
            obs_pos.y() = max_right_y;
          }
        }

        if (obs_slot_type == apa_planner::CollisionDetector::ObsSlotType::
                                 SLOT_DIRECTLY_BEHIND_OBS) {
          if (frame_->replan_flag) {
            // when replan, temp del obs directly behind slot
            continue;
          }
          // obs is directly behind slot, temp hack, when believe_in_fus_obs is
          // false, force move obs to out slot
          // if (obs_pos.y() > 0.0) {
          //   obs_pos.y() =
          //       0.5 * ego_slot_info.slot_width -
          //       max_obs_lat_invasion_slot_dist;
          // } else {
          //   obs_pos.y() = -0.5 * ego_slot_info.slot_width +
          //                 max_obs_lat_invasion_slot_dist;
          // }
        }
      }

      // if obs is in tlane area lose it
      if (!pnc::geometry_lib::IsPointInPolygon(tlane_vec, obs_pos)) {
        continue;
      }

      fus_obs_vec.emplace_back(std::move(obs_pos));
    }
    frame_->collision_detector_ptr->AddObstacles(
        fus_obs_vec, apa_planner::CollisionDetector::FUSION_OBS);
  }

  // first check ego pose if collision
  frame_->collision_detector_ptr->SetParam(
      apa_planner::CollisionDetector::Paramters(
          config_->car_lat_inflation_strict + 0.068));
  if (frame_->collision_detector_ptr->IsObstacleInCar(
          pnc::geometry_lib::PathPoint(ego_slot_info.ego_pos_slot,
                                       ego_slot_info.ego_heading_slot))) {
    ILOG_INFO << "ego pose has obs, force quit PreparePathPlan, fail";
    return false;
  }

  // const double time1 = IflyTime::Now_ms();
  // frame_->collision_detector_ptr->TransObsMapToOccupancyGridMap();
  // ILOG_INFO <<"construct map time = " << IflyTime::Now_ms() - time1 << "ms";

  OccupancyGridBound bound(
      std::min(C.x(), D.x()) - 0.0168,
      std::min({channel_point_1.y(), channel_point_2.y(), A.y(), F.y()}) -
          0.0168,
      std::max(channel_point_1.x(), channel_point_2.x()) + 0.0168,
      std::max({channel_point_1.y(), channel_point_2.y(), A.y(), F.y()}) +
          0.0168);
  frame_->collision_detector_ptr->TransObsMapToOccupancyGridMap(bound);

  return true;
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

  if (frame_->fus_obj_valid_flag) {
    ILOG_INFO << "use fusion obs, lon_dist = " << lon_dist;
    // const size_t slot_id = static_cast<size_t>(slot->id());
    // frame_->obs_pt_map[slot_id] = frame_->obs_pt_vec;

    // ILOG_INFO <<"frame_->obs_pt_map[slot_id] size = "
    //             << frame_->obs_pt_map[slot_id].size());

    if (lon_dist <
            config_->min_parallel_vis_slot_release_long_dist_slot2mirror &&
        slot_history->GetOccupied()) {
      slot->set_is_release(false);
      slot->set_is_occupied(true);
    } else {
      slot_history->SetOccupied(false);
      slot->set_is_release(true);
      slot->set_is_occupied(false);
    }
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

  if (!frame_->fus_obj_valid_flag) {
    // no fus obs, should consider uss obs
    AddUssPerceptObstacles(*slot);
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

const bool RuleBasedSlotRelease::GeometryPathForTailInScenario(
    common::SlotInfo *slot, apa_planner::SlotInfoWindow *slot_history) {
  apa_planner::SlotManager::EgoSlotInfo ego_slot_info;
  // get ego slot info
  if (!UpdateEgoSlotInfo(ego_slot_info, slot)) {
    slot->set_is_release(false);
    slot->set_is_occupied(true);
    ILOG_INFO << "UpdateEgoSlotInfo slot id = " << slot->id()
              << "  slot type = " << slot->slot_type()
              << "  is_release = " << slot->is_release();

    return false;
  }

  // gen T_Lane
  apa_planner::PerpendicularTailInPathGenerator::Tlane slot_tlane;
  apa_planner::PerpendicularTailInPathGenerator::Tlane obs_tlane;
  if (!GenTLane(ego_slot_info, slot_tlane, obs_tlane)) {
    slot->set_is_release(false);
    slot->set_is_occupied(true);
    ILOG_INFO << "GenTLane slot id = " << slot->id()
              << "  slot type = " << slot->slot_type()
              << "  is_release = " << slot->is_release();
    return false;
  }

  if (!config_->release_slot_by_prepare) {
    return false;
  }

  if (!GenObstacles(slot_tlane, obs_tlane, ego_slot_info)) {
    slot->set_is_release(false);
    slot->set_is_occupied(true);
    ILOG_INFO << "GenObstacles slot id = " << slot->id()
              << "  slot type = " << slot->slot_type()
              << "  is_release = " << slot->is_release();
    return false;
  }

  apa_planner::PerpendicularTailInPathGenerator::Input path_planner_input;
  path_planner_input.pt_0 = ego_slot_info.pt_0;
  path_planner_input.pt_1 = ego_slot_info.pt_1;
  path_planner_input.sin_angle = ego_slot_info.sin_angle;
  path_planner_input.origin_pt_0_heading = ego_slot_info.origin_pt_0_heading;
  path_planner_input.tlane = slot_tlane;
  path_planner_input.ego_pose.Set(ego_slot_info.ego_pos_slot,
                                  ego_slot_info.ego_heading_slot);
  apa_planner::PerpendicularTailInPathGenerator path_planner;
  path_planner.SetInput(path_planner_input);
  path_planner.SetColPtr(frame_->collision_detector_ptr);
  if (!path_planner.UpdateByPrePlan()) {
    slot->set_is_release(false);
    slot->set_is_occupied(true);
    ILOG_INFO << "prepare slot id = " << slot->id()
              << "  slot type = " << slot->slot_type()
              << "  is_release = " << slot->is_release();
    return false;
  }

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
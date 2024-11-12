#include "perpendicular_head_out_scenario.h"

#include <queue>

#include "src/modules/apa_function/apa_param_config.h"
#include "debug_info_log.h"
#include "geometry_math.h"
#include "ifly_time.h"

namespace planning {
namespace apa_planner {

void PerpendicularHeadOutScenario::Reset() {
  frame_.Reset();
  slot_t_lane_.Reset();
  obstacle_t_lane_.Reset();
  perpendicular_path_planner_.Reset();
  current_path_point_global_vec_.clear();
  current_plan_path_vec_.clear();

  pt_center_replan_.setZero();
  pt_center_heading_replan_ = 0.0;
  pt_center_replan_jump_dist_ = 0.0;
  pt_center_replan_jump_heading_ = 0.0;

  ParkingScenario::Reset();
}

void PerpendicularHeadOutScenario::PlanCore() {
  // prepare simulation
  InitSimulation();

  // check planning status
  if (!apa_world_ptr_->GetApaDataPtr()->simu_param.force_plan &&
      CheckPlanSkip()) {
    return;
  }

  if (CheckPaused()) {
    SetParkingStatus(PARKING_PAUSED);
    if (frame_.pause_time > apa_param.GetParam().pause_failed_time) {
      SetParkingStatus(PARKING_FAILED);
      frame_.plan_fail_reason = PAUSE_FAILED_TIME;
    }
    return;
  }

  const double safe_uss_remain_dist =
      (frame_.ego_slot_info.slot_occupied_ratio < 0.05)
          ? apa_param.GetParam().safe_uss_remain_dist_out_slot
          : apa_param.GetParam().safe_uss_remain_dist_in_slot;
  // update remain dist
  UpdateRemainDist(safe_uss_remain_dist);

  // update ego slot info
  if (!UpdateEgoSlotInfo()) {
    ILOG_INFO << "update ego slot info";
    SetParkingStatus(PARKING_FAILED);
    frame_.plan_fail_reason = UPDATE_EGO_SLOT_INFO;
    return;
  }

  // check finish
  if (CheckFinished()) {
    ILOG_INFO << "check apa finished!";
    SetParkingStatus(PARKING_FINISHED);
    return;
  }

  // check failed
  if (CheckStuckFailed()) {
    ILOG_INFO << "check stuck failed!";
    SetParkingStatus(PARKING_FAILED);
    frame_.plan_fail_reason = STUCK_FAILED_TIME;
    return;
  }

  // check replan
  if (apa_world_ptr_->GetApaDataPtr()->simu_param.force_plan || CheckReplan()) {
    ILOG_INFO << "replan is required!";

    frame_.replan_flag = true;
    pt_center_replan_ = frame_.ego_slot_info.slot_center;
    pt_center_heading_replan_ = frame_.ego_slot_info.slot_origin_heading;
    frame_.dynamic_plan_fail_flag = false;

    const double start_time = IflyTime::Now_ms();

    GenTlane();

    GenObstacles();

    frame_.total_plan_count++;

    uint8_t pathplan_result = PathPlannerResult::PLAN_FAILED;

    if (frame_.total_plan_count <= apa_param.GetParam().max_replan_count) {
      pathplan_result = PathPlanOnce();
      ILOG_INFO << "generate path by geometry";
    } else {
      ILOG_INFO << "replan count is exceed max count, fail, directly quit apa";
      frame_.plan_fail_reason = PLAN_COUNT_EXCEED_LIMIT;
    }

    if (!frame_.dynamic_plan_fail_flag) {
      frame_.car_already_move_dist = 0.0;
    }

    JSON_DEBUG_VALUE("replan_count", frame_.total_plan_count)

    JSON_DEBUG_VALUE("dynamic_plan_fail_flag", frame_.dynamic_plan_fail_flag)

    ILOG_INFO << "replan_consume_time = " << IflyTime::Now_ms() - start_time
              << " ms";
    JSON_DEBUG_VALUE("replan_consume_time", IflyTime::Now_ms() - start_time)

    frame_.pathplan_result = pathplan_result;

    if (pathplan_result == PathPlannerResult::PLAN_HOLD) {
      if (PostProcessPath()) {
        SetParkingStatus(PARKING_GEARCHANGE);
        ILOG_INFO << "replan from PARKING_GEARCHANGE!";
      } else {
        SetParkingStatus(PARKING_FAILED);
        ILOG_INFO << "replan failed from PLAN_HOLD!";
      }
    } else if (pathplan_result == PathPlannerResult::PLAN_UPDATE) {
      if (PostProcessPath()) {
        SetParkingStatus(PARKING_PLANNING);
        ILOG_INFO << "replan from PARKING_PLANNING!";
      } else {
        SetParkingStatus(PARKING_FAILED);
        ILOG_INFO << "replan failed from PARKING_PLANNING!";
      }
    } else if (pathplan_result == PathPlannerResult::PLAN_FAILED) {
      SetParkingStatus(PARKING_FAILED);
    }

    ILOG_INFO << "pathplan_result = " << static_cast<int>(pathplan_result);
  } else {
    ILOG_INFO << "replan is not required!";
    SetParkingStatus(PARKING_RUNNING);
  }

  // check planning status
  ILOG_INFO << "parking status = "
            << static_cast<int>(GetPlannerStates().planning_status);
}

const bool PerpendicularHeadOutScenario::UpdateEgoSlotInfo() {
  const auto* measures_ptr = &apa_world_ptr_->GetApaDataPtr()->measurement_data;
  const auto slot_manager_ptr_ = apa_world_ptr_->GetSlotManagerPtr();

  frame_.correct_path_for_limiter = false;
  frame_.replan_flag = false;

  EgoSlotInfo& ego_slot_info = frame_.ego_slot_info;

  if (!frame_.is_fix_slot) {
    ego_slot_info.target_managed_slot =
        slot_manager_ptr_->GetEgoSlotInfo().select_slot_filter;

    const auto& slot_points =
        ego_slot_info.target_managed_slot.corner_points().corner_point();
    ILOG_INFO << "*************** slot_points size **************  "
              << slot_points.size();
    std::vector<Eigen::Vector2d> pt;
    pt.resize(4);
    for (size_t i = 0; i < 4; ++i) {
      if (apa_world_ptr_->GetApaDataPtr()->simu_param.is_simulation &&
          apa_world_ptr_->GetApaDataPtr()
                  ->simu_param.target_managed_slot_x_vec.size() == 4 &&
          apa_world_ptr_->GetApaDataPtr()->simu_param.use_slot_in_bag) {
        pt[i] << apa_world_ptr_->GetApaDataPtr()
                     ->simu_param.target_managed_slot_x_vec[i],
            apa_world_ptr_->GetApaDataPtr()
                ->simu_param.target_managed_slot_y_vec[i];
      } else {
        pt[i] << slot_points[i].x(), slot_points[i].y();
      }
    }
    const Eigen::Vector2d pM01 = 0.5 * (pt[0] + pt[1]);
    const Eigen::Vector2d pM23 = 0.5 * (pt[2] + pt[3]);
    const double real_slot_length = (pM01 - pM23).norm();
    const Eigen::Vector2d t = (pt[1] - pt[0]).normalized();
    // n is vec that slot opening orientation
    const Eigen::Vector2d n = Eigen::Vector2d(t.y(), -t.x());
    // const Eigen::Vector2d n = (pM01 - pM23).normalized();
    pt[2] = pt[0] - real_slot_length * n;
    pt[3] = pt[1] - real_slot_length * n;

    ego_slot_info.slot_corner = pt;

    ego_slot_info.slot_center = (pt[0] + pt[1] + pt[2] + pt[3]) * 0.25;

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

    if (ego_slot_info.target_managed_slot.slot_type() ==
        Common::PARKING_SLOT_TYPE_SLANTING) {
      const Eigen::Vector2d origin_pt_0 =
          Eigen::Vector2d(slot_manager_ptr_->GetEgoSlotInfo()
                              .select_fusion_slot.corner_points[0]
                              .x,
                          slot_manager_ptr_->GetEgoSlotInfo()
                              .select_fusion_slot.corner_points[0]
                              .y);

      const Eigen::Vector2d origin_pt_1 =
          Eigen::Vector2d(slot_manager_ptr_->GetEgoSlotInfo()
                              .select_fusion_slot.corner_points[1]
                              .x,
                          slot_manager_ptr_->GetEgoSlotInfo()
                              .select_fusion_slot.corner_points[1]
                              .y);

      ego_slot_info.pt_0 = ego_slot_info.g2l_tf.GetPos(origin_pt_0);
      ego_slot_info.pt_1 = ego_slot_info.g2l_tf.GetPos(origin_pt_1);

      if (ego_slot_info.pt_0.y() > ego_slot_info.pt_1.y()) {
        std::swap(ego_slot_info.pt_0, ego_slot_info.pt_1);
      }

      const Eigen::Vector2d pt_01_vec = ego_slot_info.pt_1 - ego_slot_info.pt_0;

      double angle = std::fabs(pnc::geometry_lib::GetAngleFromTwoVec(
                         Eigen::Vector2d(real_slot_length, 0.0), pt_01_vec)) *
                     kRad2Deg;

      if (angle > 90.0) {
        angle = 180.0 - angle;
      }

      angle = pnc::mathlib::DoubleConstrain(angle, 10.0, 80.0);
      ego_slot_info.sin_angle = std::sin(angle * kDeg2Rad);
      ego_slot_info.origin_pt_0_heading = 90.0 - angle;
    } else {
      ego_slot_info.pt_0 = ego_slot_info.g2l_tf.GetPos(pt[0]);
      ego_slot_info.pt_1 = ego_slot_info.g2l_tf.GetPos(pt[1]);
      if (ego_slot_info.pt_0.y() > ego_slot_info.pt_1.y()) {
        std::swap(ego_slot_info.pt_0, ego_slot_info.pt_1);
      }
      ego_slot_info.sin_angle = 1.0;
      ego_slot_info.origin_pt_0_heading = 0.0;
    }
  }

  ego_slot_info.ego_pos_slot = ego_slot_info.g2l_tf.GetPos(measures_ptr->pos);

  ego_slot_info.ego_heading_slot =
      ego_slot_info.g2l_tf.GetHeading(measures_ptr->heading);

  ego_slot_info.ego_heading_slot_vec
      << std::cos(ego_slot_info.ego_heading_slot),
      std::sin(ego_slot_info.ego_heading_slot);

  ego_slot_info.target_ego_pos_slot << apa_param.GetParam().terminal_target_x,
      apa_param.GetParam().terminal_target_y;

  ego_slot_info.target_ego_heading_slot =
      apa_param.GetParam().terminal_target_heading;

  ego_slot_info.fus_obj_valid_flag =
      slot_manager_ptr_->GetEgoSlotInfo().fus_obj_valid_flag;
  ego_slot_info.obs_pt_vec_slot.clear();
  ego_slot_info.obs_pt_vec_slot.reserve(
      slot_manager_ptr_->GetEgoSlotInfo().obs_pt_vec_slot.size());

  uint8_t obs_in_slot_count = 0;
  const uint8_t max_obs_in_slot_count = 5;

  for (const Eigen::Vector2d& obs_pt :
       slot_manager_ptr_->GetEgoSlotInfo().obs_pt_vec_slot) {
    const Eigen::Vector2d obs_pt_slot = ego_slot_info.g2l_tf.GetPos(obs_pt);
    if (ego_slot_info.fus_obj_valid_flag) {
      if (std::fabs(obs_pt_slot.y()) < 0.6 &&
          obs_pt_slot.x() > ego_slot_info.pt_0.x() -
                                apa_param.GetParam().car_length + 0.168 &&
          obs_pt_slot.x() < ego_slot_info.pt_0.x() + 0.168) {
        obs_in_slot_count++;
      }
    }

    ego_slot_info.obs_pt_vec_slot.emplace_back(std::move(obs_pt_slot));
  }

  if (obs_in_slot_count > max_obs_in_slot_count) {
    ILOG_INFO << "obs_in_slot_count > max_obs_in_slot_count";
    return false;
  }

  //**************************
  // limiter,cal target pos,cal terminal error,cal slot occupied ratio
  //**************************

  // calc slot side and init gear and init steer at first
  if (frame_.is_replan_first) {
    pt_center_replan_ = ego_slot_info.slot_center;
    pt_center_heading_replan_ = ego_slot_info.slot_origin_heading;
    frame_.car_already_move_dist = 0.0;

    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_DRIVE;
    if (apa_param.GetParam().perpendicular_parking_out_right) {
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_RIGHT;
      slot_t_lane_.slot_side = pnc::geometry_lib::SLOT_SIDE_RIGHT;
    } else if (!apa_param.GetParam().perpendicular_parking_out_right) {
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_LEFT;
      slot_t_lane_.slot_side = pnc::geometry_lib::SLOT_SIDE_LEFT;
    }

    ILOG_INFO << "Now default to turn right at first";
    // judge slot side via slot center and heading
    // ...
  }

  // update stuck by uss time
  if (frame_.plan_stm.planning_status == PARKING_RUNNING &&
      measures_ptr->static_flag &&
      apa_world_ptr_->GetApaDataPtr()->cur_state ==
          ApaStateMachine::ACTIVE_OUT) {
    frame_.stuck_uss_time += apa_param.GetParam().plan_time;
  } else {
    frame_.stuck_uss_time = 0.0;
  }

  // update stuck time
  if ((frame_.plan_stm.planning_status == PARKING_RUNNING ||
       frame_.plan_stm.planning_status == PARKING_PLANNING) &&
      measures_ptr->static_flag &&
      apa_world_ptr_->GetApaDataPtr()->cur_state ==
          ApaStateMachine::ACTIVE_OUT) {
    frame_.stuck_time += apa_param.GetParam().plan_time;
  } else {
    frame_.stuck_time = 0.0;
  }

  // update pause time
  if (frame_.plan_stm.planning_status == PARKING_PAUSED) {
    frame_.pause_time += apa_param.GetParam().plan_time;
  } else {
    frame_.pause_time = 0.0;
  }

  // fix slot
  if (ego_slot_info.slot_occupied_ratio >
          apa_param.GetParam().fix_slot_occupied_ratio &&
      !frame_.is_fix_slot && measures_ptr->static_flag) {
    frame_.is_fix_slot = true;
  }

  ILOG_INFO << "frame_.current_gear = "
            << static_cast<int>(frame_.current_gear);
  ILOG_INFO << "frame_.current_arc_steer = "
            << static_cast<int>(frame_.current_arc_steer);

  return true;
}

void PerpendicularHeadOutScenario::GenTlane() {
  using namespace pnc::geometry_lib;
  EgoSlotInfo& ego_slot_info = frame_.ego_slot_info;

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
       apa_param.GetParam().obs_consider_long_threshold * pt_01_norm_up_vec)
          .x();

  const double y_max = std::fabs(
      (ego_slot_info.pt_1 +
       apa_param.GetParam().obs_consider_lat_threshold * pt_01_norm_vec)
          .y());

  // construct channel width and length
  if (!ego_slot_info.fus_obj_valid_flag) {
    // use uss obs
    std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>, Compare>
        channel_width_pq_for_x(Compare(1));
    for (const auto& obstacle_point_slot : ego_slot_info.obs_pt_vec_slot) {
      if (obstacle_point_slot.x() < x_max ||
          std::fabs(obstacle_point_slot.y()) > y_max) {
        continue;
      }
      channel_width_pq_for_x.emplace(obstacle_point_slot);
    }

    if (channel_width_pq_for_x.empty()) {
      channel_width_pq_for_x.emplace(Eigen::Vector2d(
          apa_param.GetParam().channel_width +
              ((ego_slot_info.pt_1 + ego_slot_info.pt_0) * 0.5).x(),
          0.0));
    }

    const double channel_width =
        channel_width_pq_for_x.top().x() -
        ((ego_slot_info.pt_1 + ego_slot_info.pt_0) * 0.5).x();

    ego_slot_info.channel_width = pnc::mathlib::DoubleConstrain(
        channel_width, apa_param.GetParam().min_channel_width,
        apa_param.GetParam().channel_width);
  } else {
    // use fus obs
    ego_slot_info.channel_width = apa_param.GetParam().channel_width;
  }

  ILOG_INFO << "channel_width = " << ego_slot_info.channel_width;

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

  CollisionDetector::ObsSlotType obs_slot_type;
  const std::pair<Eigen::Vector2d, Eigen::Vector2d> slot_pt =
      std::make_pair(ego_slot_info.pt_1, ego_slot_info.pt_0);
  const bool is_left_side =
      (slot_t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT);
  double max_obs_lat_invasion_slot_dist = 0.0;
  const double mir_width =
      (apa_param.GetParam().max_car_width - apa_param.GetParam().car_width) *
      0.5;

  const double mir_x = ego_slot_info.target_ego_pos_slot.x() +
                       apa_param.GetParam().lon_dist_mirror_to_rear_axle -
                       0.368;
  // sift obstacles that meet requirement
  for (Eigen::Vector2d obstacle_point_slot : ego_slot_info.obs_pt_vec_slot) {
    // if (std::fabs(obstacle_point_slot.x()) > x_max ||
    //     obstacle_point_slot.x() < x_min ||
    //     std::fabs(obstacle_point_slot.y()) > y_max ||
    //     std::fabs(obstacle_point_slot.y()) < y_min) {
    //   continue;
    // }
    obs_slot_type = apa_world_ptr_->GetCollisionDetectorPtr()->GetObsSlotType(
        obstacle_point_slot, slot_pt, is_left_side, frame_.replan_flag);

    if (obs_slot_type == CollisionDetector::ObsSlotType::SLOT_IN_OBS &&
        !apa_param.GetParam().believe_in_fus_obs) {
      max_obs_lat_invasion_slot_dist =
          frame_.replan_flag
              ? apa_param.GetParam().max_obs_lat_invasion_slot_dist
              : apa_param.GetParam().max_obs_lat_invasion_slot_dist_dynamic_col;
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
    } else if (obs_slot_type !=
                   CollisionDetector::ObsSlotType::SLOT_INSIDE_OBS &&
               obs_slot_type !=
                   CollisionDetector::ObsSlotType::SLOT_OUTSIDE_OBS) {
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

  if (apa_param.GetParam().conservative_mono_enable) {
    if (!left_pq_for_x.empty() || !right_pq_for_x.empty()) {
      apa_param.SetPram().mono_plan_enable = false;
    }
  }

  // If there are no obstacles on either side, set up a virtual obstacle that is
  // farther away
  const double virtual_x =
      ((ego_slot_info.pt_1 + ego_slot_info.pt_0) * 0.5 +
       apa_param.GetParam().virtual_obs_x_pos * pt_01_norm_down_vec)
          .x();

  const double virtual_left_y =
      (ego_slot_info.pt_1 +
       apa_param.GetParam().virtual_obs_y_pos * pt_01_norm_vec)
          .y();
  const double virtual_right_y =
      (ego_slot_info.pt_0 +
       apa_param.GetParam().virtual_obs_y_pos * pt_10_norm_vec)
          .y();

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

  const double car_half_width_with_mirror =
      apa_param.GetParam().max_car_width * 0.5;

  const double virtual_slot_width =
      apa_param.GetParam().max_car_width +
      apa_param.GetParam().slot_compare_to_car_width;

  const double real_slot_width = ego_slot_info.slot_width;

  ILOG_INFO << "max_car_width = " << apa_param.GetParam().max_car_width
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
  if (apa_param.GetParam().tmp_no_consider_obs_dy &&
      !ego_slot_info.fus_obj_valid_flag) {
    if (!left_empty) {
      left_y = real_slot_width * 0.5 + apa_param.GetParam().tmp_virtual_obs_dy;
    }
    if (!right_empty) {
      right_y =
          -real_slot_width * 0.5 - apa_param.GetParam().tmp_virtual_obs_dy;
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

  ILOG_INFO << "real_left_y = " << real_left_y
            << "  real_right_y = " << real_right_y;

  ILOG_INFO << "left_y = " << left_y << "  right_y = " << right_y
            << "  left_x = " << left_x << "  right_x = " << right_x;

  // todo: consider actual obs pos to let slot release or not release or move
  // target pose
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

  bool left_obs_meet_safe_require = false;
  bool right_obs_meet_safe_require = false;

  const double safe_threshold = apa_param.GetParam().car_lat_inflation_normal +
                                apa_param.GetParam().safe_threshold;

  left_obs_meet_safe_require = left_dis_obs_car > safe_threshold ? true : false;
  right_obs_meet_safe_require =
      right_dis_obs_car > safe_threshold ? true : false;

  if (ego_slot_info.slot_occupied_ratio < 0.618 && frame_.replan_flag) {
    // if two side is all no safe, the slot would not release in slot managment,
    // and should not move target pose
    if (!left_obs_meet_safe_require && right_obs_meet_safe_require) {
      // left side is dangerous, should move toward right
      ego_slot_info.move_slot_dist = safe_threshold - left_dis_obs_car;
      ego_slot_info.move_slot_dist *= -1.0;
    } else if (left_obs_meet_safe_require && !right_obs_meet_safe_require) {
      // right side is dangerous, should move toward left
      ego_slot_info.move_slot_dist = safe_threshold - right_dis_obs_car;
    }
    ILOG_INFO << "left_dis_obs_car = " << left_dis_obs_car
              << "  right_dis_obs_car = " << right_dis_obs_car;
  }

  const bool need_move_slot =
      (pnc::mathlib::IsDoubleEqual(ego_slot_info.move_slot_dist, 0.0)) ? false
                                                                       : true;

  if (need_move_slot) {
    // cal max_move_slot_dist to avoid car press line
    // no consider mirror
    const double half_car_width = apa_param.GetParam().car_width * 0.5;
    const double half_slot_width = ego_slot_info.slot_width * 0.5;
    const double car2line_dist_threshold =
        apa_param.GetParam().car2line_dist_threshold;

    // first sure if car is parked in the center, does it meet the slot line
    // distance requirement
    const double max_move_slot_dist =
        half_slot_width - half_car_width - car2line_dist_threshold;
    if (max_move_slot_dist > 0.0 &&
        (std::fabs(ego_slot_info.move_slot_dist) > max_move_slot_dist)) {
      if (ego_slot_info.move_slot_dist > 0.0) {
        ego_slot_info.move_slot_dist = max_move_slot_dist;
      }
      if (ego_slot_info.move_slot_dist < 0.0) {
        ego_slot_info.move_slot_dist = -max_move_slot_dist;
      }
    }
  }

  JSON_DEBUG_VALUE("move_slot_dist", ego_slot_info.move_slot_dist)

  // construct slot_t_lane_, left is positive, right is negative
  const double slot_width = std::min(virtual_slot_width, real_slot_width);

  Eigen::Vector2d corner_left_slot(ego_slot_info.slot_length, 0.5 * slot_width);

  Eigen::Vector2d corner_right_slot(ego_slot_info.slot_length,
                                    -0.5 * slot_width);

  const auto& slot_side = slot_t_lane_.slot_side;
  if (slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
    // inside is right, outside is left
    slot_t_lane_.corner_outside_slot = corner_left_slot;
    slot_t_lane_.corner_inside_slot = corner_right_slot;
    slot_t_lane_.pt_outside = corner_left_slot;
    slot_t_lane_.pt_inside = corner_right_slot;
    slot_t_lane_.pt_inside.x() =
        std::min(real_right_x, ego_slot_info.pt_0.x()) +
        apa_param.GetParam().tlane_safe_dx;
    slot_t_lane_.pt_inside.y() =
        std::min(real_right_y, ego_slot_info.pt_0.y() + 0.05);
  } else if (slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
    // outside is right, inside is left
    slot_t_lane_.corner_outside_slot = corner_right_slot;
    slot_t_lane_.corner_inside_slot = corner_left_slot;
    slot_t_lane_.pt_outside = corner_right_slot;
    slot_t_lane_.pt_inside = corner_left_slot;
    slot_t_lane_.pt_inside.x() = std::min(real_left_x, ego_slot_info.pt_1.x()) +
                                 apa_param.GetParam().tlane_safe_dx;
    slot_t_lane_.pt_inside.y() =
        std::max(real_left_y, ego_slot_info.pt_1.y() - 0.05);
  }

  slot_t_lane_.pt_terminal_pos << ego_slot_info.target_ego_pos_slot.x(),
      ego_slot_info.target_ego_pos_slot.y();

  slot_t_lane_.pt_terminal_heading = ego_slot_info.target_ego_heading_slot;

  if (need_move_slot) {
    slot_t_lane_.pt_terminal_pos.y() += ego_slot_info.move_slot_dist;
    slot_t_lane_.pt_inside.y() += ego_slot_info.move_slot_dist;
    slot_t_lane_.pt_outside.y() += ego_slot_info.move_slot_dist;
    std::cout << "should move slot according to obs pt, move dist = "
              << ego_slot_info.move_slot_dist << std::endl;

    ego_slot_info.terminal_err.Set(
        ego_slot_info.ego_pos_slot - slot_t_lane_.pt_terminal_pos,
        ego_slot_info.ego_heading_slot - slot_t_lane_.pt_terminal_heading);
  }

  slot_t_lane_.pt_lower_boundry_pos = slot_t_lane_.pt_terminal_pos;
  slot_t_lane_.pt_lower_boundry_pos.x() =
      slot_t_lane_.pt_lower_boundry_pos.x() -
      apa_param.GetParam().rear_overhanging -
      apa_param.GetParam().col_obs_safe_dist_normal - 0.05;

  // construct obstacle_t_lane_
  // for onstacle_t_lane    right is inside, left is outside
  obstacle_t_lane_.slot_side = slot_side;
  if (slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
    obstacle_t_lane_.pt_inside.x() = right_x + apa_param.GetParam().obs_safe_dx;
    obstacle_t_lane_.pt_inside.y() = right_y;
    obstacle_t_lane_.pt_outside.x() = left_x + apa_param.GetParam().obs_safe_dx;
    obstacle_t_lane_.pt_outside.y() = left_y;
  } else if (slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
    obstacle_t_lane_.pt_inside.x() = left_x + apa_param.GetParam().obs_safe_dx;
    obstacle_t_lane_.pt_inside.y() = left_y;
    obstacle_t_lane_.pt_outside.x() =
        right_x + apa_param.GetParam().obs_safe_dx;
    obstacle_t_lane_.pt_outside.y() = right_y;
  }

  obstacle_t_lane_.pt_terminal_pos = slot_t_lane_.pt_terminal_pos;
  obstacle_t_lane_.pt_terminal_heading = slot_t_lane_.pt_terminal_heading;
  obstacle_t_lane_.pt_lower_boundry_pos = slot_t_lane_.pt_lower_boundry_pos;

  // tmp method, obstacle is temporarily unavailable, force lift slot_t_lane
  // pt_inside and pt_outside
  if (apa_param.GetParam().force_both_side_occupied) {
    slot_t_lane_.pt_inside.x() = corner_right_slot.x();
    slot_t_lane_.pt_outside.x() = corner_left_slot.x();

    slot_t_lane_.pt_inside +=
        Eigen::Vector2d(apa_param.GetParam().occupied_pt_inside_dx,
                        apa_param.GetParam().occupied_pt_inside_dy);

    slot_t_lane_.pt_outside +=
        Eigen::Vector2d(apa_param.GetParam().occupied_pt_outside_dx,
                        apa_param.GetParam().occupied_pt_outside_dy);
  }

  ILOG_INFO << "-- slot_t_lane_ --";
  ILOG_INFO << "pt_outside = " << slot_t_lane_.pt_outside.transpose();
  ILOG_INFO << "pt_inside = " << slot_t_lane_.pt_inside.transpose();
  ILOG_INFO << "pt_terminal_pos = " << slot_t_lane_.pt_terminal_pos.transpose();
  ILOG_INFO << "pt_terminal_heading = " << slot_t_lane_.pt_terminal_heading;
  ILOG_INFO << "pt_lower_boundry_pos = "
            << slot_t_lane_.pt_lower_boundry_pos.transpose();

  ILOG_INFO << "-- obstacle_t_lane_ --";
  ILOG_INFO << "pt_outside = " << obstacle_t_lane_.pt_outside.transpose();
  ILOG_INFO << "pt_inside = " << obstacle_t_lane_.pt_inside.transpose();
  ILOG_INFO << "pt_terminal_pos = "
            << obstacle_t_lane_.pt_terminal_pos.transpose();
  ILOG_INFO << "pt_terminal_heading = " << obstacle_t_lane_.pt_terminal_heading;
  ILOG_INFO << "pt_lower_boundry_pos = "
            << obstacle_t_lane_.pt_lower_boundry_pos.transpose();

  std::vector<double> x_vec = {0.0};
  std::vector<double> y_vec = {0.0};
  std::vector<double> phi_vec = {0.0};

  JSON_DEBUG_VECTOR("col_det_path_x", x_vec, 2)
  JSON_DEBUG_VECTOR("col_det_path_y", y_vec, 2)
  JSON_DEBUG_VECTOR("col_det_path_phi", phi_vec, 2)
}

void PerpendicularHeadOutScenario::GenObstacles() {
  apa_world_ptr_->GetCollisionDetectorPtr()->ClearObstacles();
  // set obstacles
  double channel_width = frame_.ego_slot_info.channel_width;
  double channel_length = apa_param.GetParam().channel_length + 3.0;

  if (apa_param.GetParam().force_both_side_occupied) {
    obstacle_t_lane_ = slot_t_lane_;
  }

  // add tlane obstacle
  //  B is always outside
  int slot_side = 1;
  bool is_left_side = false;
  if (obstacle_t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
    slot_side = -1;
    is_left_side = false;
  } else if (obstacle_t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
    slot_side = 1;
    is_left_side = true;
  }
  Eigen::Vector2d B(obstacle_t_lane_.pt_outside);
  const EgoSlotInfo& ego_slot_info = frame_.ego_slot_info;
  const Eigen::Vector2d pt_01_vec = ego_slot_info.pt_1 - ego_slot_info.pt_0;
  const Eigen::Vector2d pt_01_norm_vec = pt_01_vec.normalized();
  const double obs_length = (channel_length - pt_01_vec.norm()) * 0.5;
  Eigen::Vector2d A = B - slot_side * pt_01_norm_vec * obs_length;

  Eigen::Vector2d C(obstacle_t_lane_.pt_lower_boundry_pos);
  C.y() = B.y();

  Eigen::Vector2d E(obstacle_t_lane_.pt_inside);
  Eigen::Vector2d D(obstacle_t_lane_.pt_lower_boundry_pos);
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

  const double ds = apa_param.GetParam().obstacle_ds;
  std::vector<Eigen::Vector2d> point_set;
  std::vector<Eigen::Vector2d> channel_obstacle_vec;
  channel_obstacle_vec.clear();
  channel_obstacle_vec.reserve(68);
  for (const auto& line : channel_line_vec) {
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, line, ds);
    channel_obstacle_vec.insert(channel_obstacle_vec.end(), point_set.begin(),
                                point_set.end());
  }

  apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(
      channel_obstacle_vec, CollisionDetector::CHANNEL_OBS);
  // apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(channel_obstacle_vec);

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
  for (const auto& line : tlane_line_vec) {
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, line, ds);
    tlane_obstacle_vec.insert(tlane_obstacle_vec.end(), point_set.begin(),
                              point_set.end());
  }
  pnc::geometry_lib::PathPoint ego_pose;
  ego_pose.Set(frame_.ego_slot_info.ego_pos_slot,
               frame_.ego_slot_info.ego_heading_slot);

  // if (apa_world_ptr_->GetApaDataPtr()->simu_param.is_simulation &&
  //     apa_world_ptr_->GetApaDataPtr()->simu_param.use_obs_in_bag &&
  //     apa_world_ptr_->GetApaDataPtr()->simu_param.obs_x_vec.size() > 0) {
  //   std::vector<Eigen::Vector2d> obs_vec;
  //   obs_vec.reserve(
  //       apa_world_ptr_->GetApaDataPtr()->simu_param.obs_x_vec.size());
  //   Eigen::Vector2d obs;
  //   for (size_t i = 0;
  //        i < apa_world_ptr_->GetApaDataPtr()->simu_param.obs_x_vec.size();
  //        ++i) {
  //     obs = ego_slot_info.g2l_tf.GetPos(Eigen::Vector2d(
  //         apa_world_ptr_->GetApaDataPtr()->simu_param.obs_x_vec[i],
  //         apa_world_ptr_->GetApaDataPtr()->simu_param.obs_y_vec[i]));
  //     obs_vec.emplace_back(obs);
  //   }
  //   apa_world_ptr_->GetCollisionDetectorPtr()->AddObstacles(
  //       obs_vec, CollisionDetector::RECORD_OBS);
  //   return;
  // }

  if (!ego_slot_info.fus_obj_valid_flag) {
    // when no fus obj, temp hack, only increase plan success ratio
    double safe_dist = apa_param.GetParam().max_obs2car_dist_out_slot;
    if (frame_.ego_slot_info.slot_occupied_ratio >
            apa_param.GetParam().max_obs2car_dist_slot_occupied_ratio &&
        std::fabs(frame_.ego_slot_info.terminal_err.heading) * kRad2Deg <
            36.6) {
      safe_dist = apa_param.GetParam().max_obs2car_dist_in_slot;
    }
    for (const Eigen::Vector2d& obs_pos : tlane_obstacle_vec) {
      if (!apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
              obs_pos, ego_pose, safe_dist)) {
        apa_world_ptr_->GetCollisionDetectorPtr()->AddObstacles(
            obs_pos, CollisionDetector::TLANE_OBS);
      }
    }
  } else {
    const double safe_dist = 0.0268;
    // add virtual tlane obs
    std::vector<Eigen::Vector2d> tlane_obs_vec;
    tlane_obs_vec.reserve(tlane_obstacle_vec.size());
    for (const Eigen::Vector2d& obs_pos : tlane_obstacle_vec) {
      if (!apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
              obs_pos, ego_pose, safe_dist)) {
        tlane_obs_vec.emplace_back(obs_pos);
      }
    }
    apa_world_ptr_->GetCollisionDetectorPtr()->AddObstacles(
        tlane_obs_vec, CollisionDetector::TLANE_OBS);

    // add actual fus obs
    Eigen::Vector2d pt_left = obstacle_t_lane_.pt_outside;
    Eigen::Vector2d pt_right = obstacle_t_lane_.pt_inside;
    if (obstacle_t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
      std::swap(pt_left, pt_right);
    }
    double max_obs_lat_invasion_slot_dist = 0.0;
    std::vector<Eigen::Vector2d> fus_obs_vec;
    std::pair<Eigen::Vector2d, Eigen::Vector2d> slot_pt =
        std::make_pair(ego_slot_info.pt_1, ego_slot_info.pt_0);
    CollisionDetector::ObsSlotType obs_slot_type;
    std::vector<Eigen::Vector2d> tlane_vec;
    tlane_vec.emplace_back(A);
    tlane_vec.emplace_back(B);
    tlane_vec.emplace_back(C);
    tlane_vec.emplace_back(D);
    tlane_vec.emplace_back(E);
    tlane_vec.emplace_back(F);
    tlane_vec.emplace_back(channel_point_2);
    tlane_vec.emplace_back(channel_point_1);

    for (Eigen::Vector2d obs_pos : ego_slot_info.obs_pt_vec_slot) {
      obs_slot_type = apa_world_ptr_->GetCollisionDetectorPtr()->GetObsSlotType(
          obs_pos, slot_pt, is_left_side, frame_.replan_flag);

      if (apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
              obs_pos, ego_pose, safe_dist)) {
        // temp hack, when obs is in car, lose it, only increase plan success
        // ratio, To Do, when obs change accurately, should not del any obs
        // ILOG_INFO << "obs is in car, lost it, obs = "
        //             << ego_slot_info.l2g_tf.GetPos(obs_pos).transpose())
        continue;
      }

      if (!apa_param.GetParam().believe_in_fus_obs) {
        if (obs_slot_type ==
                CollisionDetector::ObsSlotType::SLOT_ENTRANCE_OBS &&
            frame_.replan_flag) {
          // obs is slot entrance, when replan, no conside it, but when dynamic
          // move and col det, conside it
          continue;
        }

        if (obs_slot_type == CollisionDetector::ObsSlotType::SLOT_IN_OBS) {
          max_obs_lat_invasion_slot_dist =
              frame_.replan_flag
                  ? apa_param.GetParam().max_obs_lat_invasion_slot_dist
                  : apa_param.GetParam()
                        .max_obs_lat_invasion_slot_dist_dynamic_col;
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

        if (obs_slot_type ==
            CollisionDetector::ObsSlotType::SLOT_DIRECTLY_BEHIND_OBS) {
          if (frame_.replan_flag) {
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

      // if obs is not in tlane area lose it
      if (!pnc::geometry_lib::IsPointInPolygon(tlane_vec, obs_pos)) {
        continue;
      }

      fus_obs_vec.emplace_back(std::move(obs_pos));
    }

    apa_world_ptr_->GetCollisionDetectorPtr()->AddObstacles(
        fus_obs_vec, CollisionDetector::FUSION_OBS);
  }
}

const uint8_t PerpendicularHeadOutScenario::PathPlanOnce() {
  ILOG_INFO << "-------------- PathPlanOnce --------------";
  // construct input
  const auto& ego_slot_info = frame_.ego_slot_info;
  PerpendicularTailInPathGenerator::Input path_planner_input;
  path_planner_input.is_simulation =
      apa_world_ptr_->GetApaDataPtr()->simu_param.is_simulation;
  path_planner_input.pt_0 = ego_slot_info.pt_0;
  path_planner_input.pt_1 = ego_slot_info.pt_1;
  path_planner_input.sin_angle = ego_slot_info.sin_angle;
  path_planner_input.origin_pt_0_heading = ego_slot_info.origin_pt_0_heading;
  // path_planner_input.slot_occupied_ratio = ego_slot_info.slot_occupied_ratio;
  path_planner_input.tlane = slot_t_lane_;
  path_planner_input.is_complete_path =
      apa_world_ptr_->GetApaDataPtr()->simu_param.is_complete_path;
  path_planner_input.sample_ds =
      apa_world_ptr_->GetApaDataPtr()->simu_param.sample_ds;
  path_planner_input.ref_arc_steer = frame_.current_arc_steer;
  path_planner_input.ref_gear = frame_.current_gear;
  path_planner_input.is_replan_first = frame_.is_replan_first;
  path_planner_input.is_replan_second = frame_.is_replan_second;
  // path_planner_input.is_replan_dynamic = frame_.is_replan_dynamic;
  path_planner_input.ego_pose.Set(ego_slot_info.ego_pos_slot,
                                  ego_slot_info.ego_heading_slot);

  perpendicular_path_planner_.SetInput(path_planner_input);

  // need replan all path
  const bool path_plan_success =
      perpendicular_path_planner_.GeometryPathGenerator::Update(
          apa_world_ptr_->GetCollisionDetectorPtr());

  uint8_t plan_result = 0;
  if (!path_plan_success &&
      !apa_world_ptr_->GetApaDataPtr()->simu_param.is_simulation) {
    ILOG_INFO << "path plan fail";
    plan_result = PathPlannerResult::PLAN_FAILED;
    frame_.plan_fail_reason = PATH_PLAN_FAILED;
    current_plan_path_vec_.clear();
    current_path_point_global_vec_.clear();
    return plan_result;
  }

  plan_result = PathPlannerResult::PLAN_UPDATE;

  if (!perpendicular_path_planner_.SetCurrentPathSegIndex()) {
    ILOG_INFO << "path plan fail";
    plan_result = PathPlannerResult::PLAN_FAILED;
    frame_.plan_fail_reason = SET_SEG_INDEX;
    return plan_result;
  }

  // retired
  // perpendicular_path_planner_.SetLineSegmentHeading();

  // perpendicular_path_planner_.InsertLineSegAfterCurrentFollowLastPath(
  //     apa_param.GetParam().path_extend_distance);

  if (!perpendicular_path_planner_.CheckCurrentGearLength()) {
    ILOG_INFO << "path plan fail";
    plan_result = PathPlannerResult::PLAN_FAILED;
    frame_.plan_fail_reason = CHECK_GEAR_LENGTH;
    return plan_result;
  }

  perpendicular_path_planner_.SampleCurrentPathSeg();

  perpendicular_path_planner_.PrintOutputSegmentsInfo();

  const auto& planner_output = perpendicular_path_planner_.GetOutput();
  current_plan_path_vec_.clear();
  current_plan_path_vec_.reserve(5);

  if (apa_param.GetParam().dynamic_col_det_enable) {
    for (size_t i = planner_output.path_seg_index.first;
         i <= planner_output.path_seg_index.second; ++i) {
      const auto& path_seg_local = planner_output.path_segment_vec[i];
      pnc::geometry_lib::PathSegment path_seg_global = path_seg_local;
      if (path_seg_local.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
        path_seg_global.line_seg.pA =
            ego_slot_info.l2g_tf.GetPos(path_seg_local.GetLineSeg().pA);
        path_seg_global.line_seg.pB =
            ego_slot_info.l2g_tf.GetPos(path_seg_local.GetLineSeg().pB);
        path_seg_global.line_seg.heading = ego_slot_info.l2g_tf.GetHeading(
            path_seg_local.GetLineSeg().heading);
      } else if (path_seg_local.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
        path_seg_global.arc_seg.pA =
            ego_slot_info.l2g_tf.GetPos(path_seg_local.GetArcSeg().pA);
        path_seg_global.arc_seg.pB =
            ego_slot_info.l2g_tf.GetPos(path_seg_local.GetArcSeg().pB);
        path_seg_global.arc_seg.circle_info.center =
            ego_slot_info.l2g_tf.GetPos(
                path_seg_local.GetArcSeg().circle_info.center);
        path_seg_global.arc_seg.headingA = ego_slot_info.l2g_tf.GetHeading(
            path_seg_local.GetArcSeg().headingA);
        path_seg_global.arc_seg.headingB = ego_slot_info.l2g_tf.GetHeading(
            path_seg_local.GetArcSeg().headingB);
      }
      current_plan_path_vec_.emplace_back(std::move(path_seg_global));
    }
  }

  // const bool gear_steer_shift =
  //     (frame_.is_replan_first && planner_output.gear_shift) ||
  //     (frame_.is_replan_second && planner_output.gear_shift) ||
  //     (!frame_.is_replan_first && !frame_.is_replan_second &&
  //      frame_.replan_reason != DYNAMIC);

  if (!frame_.is_replan_first) {
    ILOG_INFO << "next plan should shift gear";
    // set current arc steer
    if (frame_.current_arc_steer == pnc::geometry_lib::SEG_STEER_RIGHT) {
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_LEFT;
    } else if (frame_.current_arc_steer == pnc::geometry_lib::SEG_STEER_LEFT) {
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_RIGHT;
    } else {
      ILOG_INFO << "fault ref_arc_steer state!";
      return false;
    }

    // set current gear
    if (frame_.current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
      frame_.current_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
    } else if (frame_.current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
      frame_.current_gear = pnc::geometry_lib::SEG_GEAR_DRIVE;
    } else {
      ILOG_INFO << "fault ref_gear state!";
      return false;
    }
  }

  if (frame_.is_replan_first) {
    frame_.is_replan_first = false;
    if (planner_output.gear_shift) {
      frame_.is_replan_second = false;
    } else {
      frame_.is_replan_second = true;
    }
  } else {
    if (frame_.is_replan_second) {
      frame_.is_replan_second = false;
    }
  }

  if (!path_plan_success) {
    return plan_result;
  }

  frame_.gear_command = planner_output.current_gear;

  // lateral path optimization
  bool is_use_optimizer = true;

  // refuse optimizer
  if (planner_output.path_point_vec.size() < 3) {
    ILOG_INFO << " input size is too small";
    is_use_optimizer = false;
  } else {
    const auto path_length = (planner_output.path_point_vec.front().pos -
                              planner_output.path_point_vec.back().pos)
                                 .norm();
    if (path_length < apa_param.GetParam().min_opt_path_length) {
      ILOG_INFO << "path length is too short, optimizer is closed ";
      is_use_optimizer = false;
    }
  }

  bool cilqr_optimization_enable = true;
  bool perpendicular_optimization_enable = true;

  if (!apa_world_ptr_->GetApaDataPtr()->simu_param.is_simulation) {
    perpendicular_optimization_enable =
        apa_param.GetParam().perpendicular_lat_opt_enable;

    cilqr_optimization_enable =
        apa_param.GetParam().cilqr_path_optimization_enable;

  } else {
    perpendicular_optimization_enable =
        apa_world_ptr_->GetApaDataPtr()->simu_param.is_path_optimization;
    cilqr_optimization_enable =
        apa_world_ptr_->GetApaDataPtr()->simu_param.is_cilqr_optimization;
  }

  double lat_path_opt_cost_time_ms = 0.0;
  if (perpendicular_optimization_enable && is_use_optimizer) {
    ILOG_INFO << "------------------------ lateral path optimization "
                 "------------------------";
    ILOG_INFO << "frame_.gear_command= "
              << static_cast<int>(frame_.gear_command);
    ILOG_INFO << "origin path size= " << planner_output.path_point_vec.size();

    LateralPathOptimizer::Parameter param;
    param.sample_ds = apa_world_ptr_->GetApaDataPtr()->simu_param.sample_ds;
    param.q_ref_xy = apa_world_ptr_->GetApaDataPtr()->simu_param.q_ref_xy;
    param.q_ref_theta = apa_world_ptr_->GetApaDataPtr()->simu_param.q_ref_theta;
    param.q_terminal_xy =
        apa_world_ptr_->GetApaDataPtr()->simu_param.q_terminal_xy;
    param.q_terminal_theta =
        apa_world_ptr_->GetApaDataPtr()->simu_param.q_terminal_theta;
    param.q_k = apa_world_ptr_->GetApaDataPtr()->simu_param.q_k;
    param.q_u = apa_world_ptr_->GetApaDataPtr()->simu_param.q_u;
    param.q_k_bound = apa_world_ptr_->GetApaDataPtr()->simu_param.q_k_bound;
    param.q_u_bound = apa_world_ptr_->GetApaDataPtr()->simu_param.q_u_bound;

    apa_world_ptr_->GetLateralPathOptimizerPtr()->Init(
        cilqr_optimization_enable);
    apa_world_ptr_->GetLateralPathOptimizerPtr()->SetParam(param);
    auto time_start = IflyTime::Now_ms();
    apa_world_ptr_->GetLateralPathOptimizerPtr()->Update(
        planner_output.path_point_vec, frame_.gear_command);

    auto time_end = IflyTime::Now_ms();
    lat_path_opt_cost_time_ms = time_end - time_start;

    const auto& optimized_path_vec =
        apa_world_ptr_->GetLateralPathOptimizerPtr()->GetOutputPathVec();
    // TODO: longitudinal path optimization
    current_path_point_global_vec_.clear();
    current_path_point_global_vec_.reserve(optimized_path_vec.size());

    pnc::geometry_lib::PathPoint global_point;
    for (const auto& path_point : optimized_path_vec) {
      global_point.Set(ego_slot_info.l2g_tf.GetPos(path_point.pos),
                       ego_slot_info.l2g_tf.GetHeading(path_point.heading));

      current_path_point_global_vec_.emplace_back(global_point);
    }
    const auto plan_debug_info =
        apa_world_ptr_->GetLateralPathOptimizerPtr()->GetOutputDebugInfo();

    ILOG_INFO << "lat_path_opt_cost_time_ms = " << lat_path_opt_cost_time_ms;

    ILOG_INFO << "terminal point error = "
              << plan_debug_info.terminal_pos_error();

    ILOG_INFO << "terminal heading error = "
              << plan_debug_info.terminal_heading_error();
  } else {
    current_path_point_global_vec_.clear();
    current_path_point_global_vec_.reserve(
        planner_output.path_point_vec.size());

    pnc::geometry_lib::PathPoint global_point;
    for (const auto& path_point : planner_output.path_point_vec) {
      global_point.Set(ego_slot_info.l2g_tf.GetPos(path_point.pos),
                       ego_slot_info.l2g_tf.GetHeading(path_point.heading));

      current_path_point_global_vec_.emplace_back(global_point);
    }
  }

  JSON_DEBUG_VALUE("cilqr_optimization_enable", cilqr_optimization_enable);
  JSON_DEBUG_VALUE("lat_path_opt_cost_time_ms", lat_path_opt_cost_time_ms);

  ILOG_INFO << "current_path_point_global_vec_.size() = "
            << current_path_point_global_vec_.size();

  return plan_result;
}

const bool PerpendicularHeadOutScenario::CheckSegCompleted() {
  bool is_seg_complete = false;
  if (frame_.spline_success) {
    if (frame_.remain_dist < apa_param.GetParam().max_replan_remain_dist &&
        apa_world_ptr_->GetApaDataPtr()->measurement_data.static_flag) {
      ILOG_INFO << "close to target, need wait a certain time!";
      if (frame_.stuck_uss_time > 0.068) {
        ILOG_INFO << "wait a certain time, start plan";
        is_seg_complete = true;
      }
    }
  }

  return is_seg_complete;
}

const bool PerpendicularHeadOutScenario::CheckUssStucked() {
  if (frame_.remain_dist_uss < apa_param.GetParam().max_replan_remain_dist &&
      apa_world_ptr_->GetApaDataPtr()->measurement_data.static_flag) {
    ILOG_INFO << "close to obstacle by uss!, need wait a certain time!";
    if (frame_.stuck_uss_time >
        apa_param.GetParam().uss_stuck_replan_wait_time) {
      ILOG_INFO << "wait a certain time, start plan";
      frame_.is_replan_by_uss = true;
      return true;
    }
  }

  return false;
}

const bool PerpendicularHeadOutScenario::CheckColDetStucked() {
  if (frame_.remain_dist_col_det <
          apa_param.GetParam().max_replan_remain_dist &&
      apa_world_ptr_->GetApaDataPtr()->measurement_data.static_flag) {
    ILOG_INFO << "close to obstacle by col det!, need wait a certain time!";
    if (frame_.stuck_uss_time >
        apa_param.GetParam().uss_stuck_replan_wait_time) {
      ILOG_INFO << "wait a certain time, start plan";
      return true;
    }
  }
  return false;
}

const bool PerpendicularHeadOutScenario::CheckReplan() {
  pt_center_replan_jump_dist_ =
      (pt_center_replan_ - frame_.ego_slot_info.slot_center).norm();
  pt_center_replan_jump_heading_ =
      std::fabs(pnc::geometry_lib::NormalizeAngle(
          pt_center_heading_replan_ -
          frame_.ego_slot_info.slot_origin_heading)) *
      kRad2Deg;
  ILOG_INFO << "replan slot_jump_dist = " << pt_center_replan_jump_dist_;
  ILOG_INFO << "replan slot_jump_heading = " << pt_center_replan_jump_heading_;

  if (frame_.is_replan_first == true) {
    ILOG_INFO << "first plan";
    frame_.replan_reason = FIRST_PLAN;
    return true;
  }

  frame_.is_replan_by_uss = false;
  // frame_.is_replan_dynamic = false;

  if (CheckSegCompleted()) {
    ILOG_INFO << "replan by current segment completed!";
    frame_.replan_reason = SEG_COMPLETED_PATH;
    return true;
  }

  if (CheckUssStucked()) {
    ILOG_INFO << "replan by uss stucked!";
    frame_.replan_reason = SEG_COMPLETED_USS;
    return true;
  }

  if (CheckColDetStucked()) {
    ILOG_INFO << "replan by col det stucked!";
    frame_.replan_reason = SEG_COMPLETED_COL_DET;
    return true;
  }

  if (frame_.stuck_uss_time > apa_param.GetParam().stuck_replan_time) {
    // if plan once, the stuck_uss_time is clear and accumlate again
    ILOG_INFO << "replan by stuck!";
    frame_.replan_reason = STUCKED;
    return true;
  }

  // if (!apa_world_ptr_->GetApaDataPtr()->simu_param.sim_to_target &&
  //     CheckDynamicUpdate()) {
  //   ILOG_INFO << "replan by dynamic!";
  //   frame_.replan_reason = DYNAMIC;
  //   return true;
  // }

  frame_.replan_reason = NOT_REPLAN;

  return false;
}

const bool PerpendicularHeadOutScenario::CheckFinished() {
  const auto& ego_slot_info = frame_.ego_slot_info;

  const bool heading_condition_1 =
      std::fabs(ego_slot_info.ego_heading_slot) <= 110.0 / 57.3;  // TODU::

  const bool heading_condition_2 =
      std::fabs(ego_slot_info.ego_heading_slot) >= 80.0 / 57.3;

  const bool lat_condition = heading_condition_1 && heading_condition_2;

  const bool static_condition =
      apa_world_ptr_->GetApaDataPtr()->measurement_data.static_flag;

  const bool remain_s_condition =
      frame_.remain_dist < apa_param.GetParam().max_replan_remain_dist;

  bool parking_finish = lat_condition && static_condition && remain_s_condition;

  if (parking_finish) {
    return true;
  }

  // stucked by directly behind uss
  const auto& uss_obstacle_avoider_ptr =
      apa_world_ptr_->GetUssObstacleAvoidancePtr();
  const bool enter_slot_condition =
      frame_.ego_slot_info.slot_occupied_ratio >
      apa_param.GetParam().finish_uss_slot_occupied_ratio;
  const bool remain_uss_condition =
      frame_.remain_dist_uss < apa_param.GetParam().max_replan_remain_dist;
  if (uss_obstacle_avoider_ptr->CheckIsDirectlyBehindUss()) {
    parking_finish = lat_condition && static_condition &&
                     enter_slot_condition && remain_uss_condition;
  }

  return parking_finish;
}

const bool PerpendicularHeadOutScenario::PostProcessPathAccordingObs(
    const double car_remain_dist) {
  size_t origin_traj_size = current_path_point_global_vec_.size();

  if (origin_traj_size < 2) {
    frame_.spline_success = false;
    ILOG_INFO << "error: origin_traj_size = " << origin_traj_size;
    return false;
  }

  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> s_vec;
  std::vector<double> heading_vec;
  x_vec.clear();
  y_vec.clear();
  s_vec.clear();
  heading_vec.clear();

  x_vec.reserve(origin_traj_size);
  y_vec.reserve(origin_traj_size);
  s_vec.reserve(origin_traj_size);
  heading_vec.reserve(origin_traj_size);

  double ds = 0.0;
  double s = 0.0;
  for (size_t i = 0; i < origin_traj_size; ++i) {
    if (i > 0) {
      ds = std::hypot(current_path_point_global_vec_[i].pos.x() -
                          current_path_point_global_vec_[i - 1].pos.x(),
                      current_path_point_global_vec_[i].pos.y() -
                          current_path_point_global_vec_[i - 1].pos.y());
      s += std::max(ds, 1e-3);
    }
    if (s > car_remain_dist) {
      ILOG_INFO << "path shoule be shorten because of obs";
      if (car_remain_dist - s_vec.back() > 0.036 && frame_.spline_success) {
        x_vec.emplace_back(frame_.x_s_spline(car_remain_dist));
        y_vec.emplace_back(frame_.y_s_spline(car_remain_dist));
        heading_vec.emplace_back(heading_vec.back());
        s_vec.emplace_back(car_remain_dist);
      }
      break;
    }
    x_vec.emplace_back(current_path_point_global_vec_[i].pos.x());
    y_vec.emplace_back(current_path_point_global_vec_[i].pos.y());
    heading_vec.emplace_back(current_path_point_global_vec_[i].heading);
    s_vec.emplace_back(s);
  }

  frame_.current_path_length = s_vec.back();
  const size_t N = x_vec.size();
  if (N < 2) {
    frame_.spline_success = false;
    ILOG_INFO << "error: no enough point = " << x_vec.size();
    return false;
  }
  current_path_point_global_vec_.clear();
  current_path_point_global_vec_.reserve(N);
  pnc::geometry_lib::PathPoint path_point;
  for (size_t i = 0; i < N; ++i) {
    path_point.Set(Eigen::Vector2d(x_vec[i], y_vec[i]), heading_vec[i]);
    current_path_point_global_vec_.emplace_back(path_point);
  }

  // need extend by cal proj point
  Eigen::Vector2d extended_point;
  bool success = pnc::geometry_lib::CalExtendedPointByTwoPoints(
      Eigen::Vector2d(x_vec[N - 2], y_vec[N - 2]),
      Eigen::Vector2d(x_vec[N - 1], y_vec[N - 1]), extended_point,
      frame_.path_extended_dist);

  if (!success) {
    frame_.spline_success = false;
    ILOG_INFO << "limit need extend fit line by spline error!";
    return false;
  }

  x_vec.emplace_back(extended_point.x());
  y_vec.emplace_back(extended_point.y());
  heading_vec.emplace_back(heading_vec.back());
  s_vec.emplace_back(frame_.current_path_length + frame_.path_extended_dist);

  frame_.x_s_spline.set_points(s_vec, x_vec);
  frame_.y_s_spline.set_points(s_vec, y_vec);

  frame_.spline_success = true;

  return true;
}

void PerpendicularHeadOutScenario::Log() const {
  const EgoSlotInfo& ego_slot_info = frame_.ego_slot_info;
  const auto& l2g_tf = ego_slot_info.l2g_tf;
  const auto p0_g = l2g_tf.GetPos(slot_t_lane_.pt_outside);
  const auto p1_g = l2g_tf.GetPos(slot_t_lane_.pt_inside);
  const auto pt_g = l2g_tf.GetPos(slot_t_lane_.pt_terminal_pos);

  JSON_DEBUG_VALUE("tlane_p0_x", p0_g.x())
  JSON_DEBUG_VALUE("tlane_p0_y", p0_g.y())
  JSON_DEBUG_VALUE("tlane_p1_x", p1_g.x())
  JSON_DEBUG_VALUE("tlane_p1_y", p1_g.y())
  JSON_DEBUG_VALUE("tlane_pt_x", pt_g.x())
  JSON_DEBUG_VALUE("tlane_pt_y", pt_g.y())
  JSON_DEBUG_VALUE("slot_side", slot_t_lane_.slot_side)

  JSON_DEBUG_VALUE("correct_path_for_limiter", frame_.correct_path_for_limiter)
  JSON_DEBUG_VALUE("replan_flag", frame_.replan_flag)
  JSON_DEBUG_VALUE("slot_replan_jump_dist", pt_center_replan_jump_dist_)
  JSON_DEBUG_VALUE("slot_replan_jump_heading", pt_center_replan_jump_heading_)

  const std::vector<Eigen::Vector2d>& obstacles =
      apa_world_ptr_->GetCollisionDetectorPtr()->GetObstacles();

  std::vector<double> obstaclesX;
  obstaclesX.clear();
  obstaclesX.reserve(obstacles.size());
  std::vector<double> obstaclesY;
  obstaclesY.clear();
  obstaclesY.reserve(obstacles.size());
  for (const Eigen::Vector2d& obstacle : obstacles) {
    const Eigen::Vector2d tmp_obstacle = l2g_tf.GetPos(obstacle);
    obstaclesX.emplace_back(tmp_obstacle.x());
    obstaclesY.emplace_back(tmp_obstacle.y());
  }

  const std::unordered_map<size_t, std::vector<Eigen::Vector2d>>&
      obstacles_map =
          apa_world_ptr_->GetCollisionDetectorPtr()->GetObstaclesMap();

  for (const auto& obs_pair : obstacles_map) {
    if (obs_pair.first == CollisionDetector::RECORD_OBS) {
      continue;
    }
    for (const auto& obstacle : obs_pair.second) {
      if (obs_pair.first == CollisionDetector::FUSION_OBS &&
          !(std::fabs(obstacle.y()) < 1.568 && obstacle.x() < 5.568 &&
            obstacle.x() > -0.468)) {
        continue;
      }
      const Eigen::Vector2d tmp_obstacle = l2g_tf.GetPos(obstacle);
      obstaclesX.emplace_back(tmp_obstacle.x());
      obstaclesY.emplace_back(tmp_obstacle.y());
    }
  }

  JSON_DEBUG_VECTOR("obstaclesX", obstaclesX, 6)
  JSON_DEBUG_VECTOR("obstaclesY", obstaclesY, 6)

  std::vector<double> slot_corner_X;
  const size_t corner_size = ego_slot_info.slot_corner.size();
  slot_corner_X.clear();
  slot_corner_X.reserve(4 * corner_size);
  std::vector<double> slot_corner_Y;
  slot_corner_Y.clear();
  slot_corner_Y.reserve(4 * corner_size);
  std::vector<Eigen::Vector2d> pt_vec;
  pt_vec.clear();
  pt_vec.reserve(corner_size);
  for (const auto& corner : ego_slot_info.slot_corner) {
    slot_corner_X.emplace_back(corner.x());
    slot_corner_Y.emplace_back(corner.y());
    pt_vec.emplace_back(corner);
  }

  if (corner_size == 4) {
    slot_corner_X.emplace_back(((pt_vec[0] + pt_vec[1]) * 0.5).x());
    slot_corner_Y.emplace_back(((pt_vec[0] + pt_vec[1]) * 0.5).y());
    slot_corner_X.emplace_back(((pt_vec[2] + pt_vec[3]) * 0.5).x());
    slot_corner_Y.emplace_back(((pt_vec[2] + pt_vec[3]) * 0.5).y());
    const Eigen::Vector2d vec_01 = (pt_vec[1] - pt_vec[0]).normalized();
    const Eigen::Vector2d vec_23 = (pt_vec[3] - pt_vec[2]).normalized();
    pt_vec[0] = pt_vec[0] + ego_slot_info.move_slot_dist * vec_01;
    pt_vec[1] = pt_vec[1] + ego_slot_info.move_slot_dist * vec_01;
    pt_vec[2] = pt_vec[2] + ego_slot_info.move_slot_dist * vec_23;
    pt_vec[3] = pt_vec[3] + ego_slot_info.move_slot_dist * vec_23;
    for (const Eigen::Vector2d& pt : pt_vec) {
      slot_corner_X.emplace_back(pt.x());
      slot_corner_Y.emplace_back(pt.y());
    }
    slot_corner_X.emplace_back(((pt_vec[0] + pt_vec[1]) * 0.5).x());
    slot_corner_Y.emplace_back(((pt_vec[0] + pt_vec[1]) * 0.5).y());
    slot_corner_X.emplace_back(((pt_vec[2] + pt_vec[3]) * 0.5).x());
    slot_corner_Y.emplace_back(((pt_vec[2] + pt_vec[3]) * 0.5).y());
  }
  slot_corner_X.emplace_back(l2g_tf.GetPos(slot_t_lane_.pt_terminal_pos).x());
  slot_corner_Y.emplace_back(l2g_tf.GetPos(slot_t_lane_.pt_terminal_pos).y());

  JSON_DEBUG_VECTOR("slot_corner_X", slot_corner_X, 6)
  JSON_DEBUG_VECTOR("slot_corner_Y", slot_corner_Y, 6)

  std::vector<double> limiter_corner_X;
  limiter_corner_X.clear();
  limiter_corner_X.reserve(frame_.ego_slot_info.limiter_corner.size());
  std::vector<double> limiter_corner_Y;
  limiter_corner_Y.clear();
  limiter_corner_Y.reserve(frame_.ego_slot_info.limiter_corner.size());
  for (const auto& corner : frame_.ego_slot_info.limiter_corner) {
    const auto tmp_corner = l2g_tf.GetPos(corner);
    limiter_corner_X.emplace_back(tmp_corner.x());
    limiter_corner_Y.emplace_back(tmp_corner.y());
  }
  JSON_DEBUG_VECTOR("limiter_corner_X", limiter_corner_X, 2)
  JSON_DEBUG_VECTOR("limiter_corner_Y", limiter_corner_Y, 2)

  JSON_DEBUG_VALUE("terminal_error_x",
                   frame_.ego_slot_info.terminal_err.pos.x())
  JSON_DEBUG_VALUE("terminal_error_y",
                   frame_.ego_slot_info.terminal_err.pos.y())
  JSON_DEBUG_VALUE("terminal_error_heading",
                   frame_.ego_slot_info.terminal_err.heading)

  JSON_DEBUG_VALUE("is_replan", frame_.is_replan)
  JSON_DEBUG_VALUE("is_finished", frame_.is_finished)
  JSON_DEBUG_VALUE("is_replan_first", frame_.is_replan_first)
  JSON_DEBUG_VALUE("is_replan_by_uss", frame_.is_replan_by_uss)
  JSON_DEBUG_VALUE("current_path_length", frame_.current_path_length)
  JSON_DEBUG_VALUE("gear_change_count", frame_.gear_change_count)
  JSON_DEBUG_VALUE("path_plan_success", frame_.plan_stm.path_plan_success)
  JSON_DEBUG_VALUE("planning_status", frame_.plan_stm.planning_status)
  JSON_DEBUG_VALUE("spline_success", frame_.spline_success)
  JSON_DEBUG_VALUE("remain_dist", frame_.remain_dist)
  JSON_DEBUG_VALUE("remain_dist_col_det", frame_.remain_dist_col_det)
  JSON_DEBUG_VALUE("remain_dist_uss", frame_.remain_dist_uss)
  JSON_DEBUG_VALUE("stuck_time", frame_.stuck_time)
  JSON_DEBUG_VALUE("replan_reason", frame_.replan_reason)
  JSON_DEBUG_VALUE("plan_fail_reason", frame_.plan_fail_reason)
  JSON_DEBUG_VALUE("dynamic_replan_count", frame_.dynamic_replan_count)
  JSON_DEBUG_VALUE("ego_heading_slot", frame_.ego_slot_info.ego_heading_slot)

  JSON_DEBUG_VALUE("selected_slot_id", frame_.ego_slot_info.selected_slot_id)
  JSON_DEBUG_VALUE("slot_length", frame_.ego_slot_info.slot_length)
  JSON_DEBUG_VALUE("slot_width", frame_.ego_slot_info.slot_width)

  JSON_DEBUG_VALUE("slot_origin_pos_x",
                   frame_.ego_slot_info.slot_origin_pos.x())

  JSON_DEBUG_VALUE("slot_origin_pos_y",
                   frame_.ego_slot_info.slot_origin_pos.y())

  JSON_DEBUG_VALUE("slot_origin_heading",
                   frame_.ego_slot_info.slot_origin_heading)

  JSON_DEBUG_VALUE("slot_occupied_ratio",
                   frame_.ego_slot_info.slot_occupied_ratio)

  std::vector<double> target_ego_pos_slot = {
      frame_.ego_slot_info.target_ego_pos_slot.x(),
      frame_.ego_slot_info.target_ego_pos_slot.y()};

  JSON_DEBUG_VALUE("pathplan_result", frame_.pathplan_result)
  JSON_DEBUG_VECTOR("target_ego_pos_slot", target_ego_pos_slot, 2)

  const auto& path_plan_output = perpendicular_path_planner_.GetOutput();
  JSON_DEBUG_VALUE("path_start_seg_index",
                   path_plan_output.path_seg_index.first)
  JSON_DEBUG_VALUE("path_end_seg_index", path_plan_output.path_seg_index.second)
  JSON_DEBUG_VALUE("path_length", path_plan_output.length)

  const auto uss_info =
      apa_world_ptr_->GetUssObstacleAvoidancePtr()->GetRemainDistInfo();
  JSON_DEBUG_VALUE("uss_available", uss_info.is_available)
  JSON_DEBUG_VALUE("uss_remain_dist", uss_info.remain_dist)
  JSON_DEBUG_VALUE("uss_index", uss_info.uss_index)
  JSON_DEBUG_VALUE("uss_car_index", uss_info.car_index)

  // lateral optimization
  const auto plan_debug_info =
      apa_world_ptr_->GetLateralPathOptimizerPtr()->GetOutputDebugInfo();

  if (plan_debug_info.has_terminal_pos_error()) {
    JSON_DEBUG_VALUE("optimization_terminal_pose_error",
                     plan_debug_info.terminal_pos_error())
    JSON_DEBUG_VALUE("optimization_terminal_heading_error",
                     plan_debug_info.terminal_heading_error())
  } else {
    JSON_DEBUG_VALUE("optimization_terminal_pose_error", 0.0)
    JSON_DEBUG_VALUE("optimization_terminal_heading_error", 0.0)
  }
}

}  // namespace apa_planner

}  // namespace planning
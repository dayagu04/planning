#include "perpendicular_head_out_scenario.h"

#include <math.h>

#include <cstddef>
#include <queue>

#include "apa_state_machine_manager.h"
#include "debug_info_log.h"
#include "geometry_math.h"
#include "ifly_time.h"
#include "src/modules/apa_function/apa_param_config.h"

namespace planning {
namespace apa_planner {

void PerpendicularHeadOutScenario::Reset() {
  frame_.Reset();
  perpendicular_path_planner_.Reset();
  current_plan_path_vec_.clear();
  path_trim_flag_ = false;
  end_position_correction_flag_ = false;

  ParkingScenario::Reset();
}

void PerpendicularHeadOutScenario::ExcutePathPlanningTask() {
  // prepare simulation
  InitSimulation();

  // check planning status
  if (CheckPlanSkip()) {
    return;
  }

  UpdateStuckTime();

  if (CheckPaused()) {
    SetParkingStatus(PARKING_PAUSED);
    if (frame_.pause_time > apa_param.GetParam().pause_failed_time) {
      SetParkingStatus(PARKING_FAILED);
      frame_.plan_fail_reason = PAUSE_FAILED_TIME;
    }
    return;
  }

  // const double safe_uss_remain_dist =
  //     (frame_.ego_slot_info.slot_occupied_ratio < 0.05)
  //         ? apa_param.GetParam().safe_uss_remain_dist_out_slot
  //         : apa_param.GetParam().safe_uss_remain_dist_in_slot;

  const double safe_uss_remain_dist =
      apa_param.GetParam().safe_uss_remain_dist_in_slot;
  // update remain dist
  frame_.remain_dist_path = CalRemainDistFromPath();
  frame_.remain_dist_obs = CalRemainDistFromObs(safe_uss_remain_dist);

  // update ego slot info
  UpdateEgoSlotInfo();

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

  CheckReplanParams replan_params;
  frame_.replan_flag = CheckReplan(replan_params);

  GenTlane();
  GenObstacles();

  // check replan
  if (frame_.replan_flag) {
    ILOG_INFO << "replan is required!";

    frame_.dynamic_plan_fail_flag = false;

    const double start_time = IflyTime::Now_ms();

    if (frame_.replan_reason != FORCE_PLAN && frame_.replan_reason != DYNAMIC) {
      frame_.total_plan_count++;
    }

    uint8_t pathplan_result = PathPlannerResult::PLAN_FAILED;

    if (frame_.total_plan_count <= apa_param.GetParam().max_replan_count) {
      pathplan_result = PathPlanOnce();
      ILOG_INFO << "generate path by geometry";
    } else {
      ILOG_INFO << "replan count is exceed max count, fail, directly quit apa";
      frame_.plan_fail_reason = PLAN_COUNT_EXCEED_LIMIT;
    }

    // if (!frame_.dynamic_plan_fail_flag) {
    //   frame_.car_already_move_dist = 0.0;
    // }

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

  // check if the current path is safe
  if (CheckSecurityCurrentpath()) {
    if (CurrentPathTrimmed()) {
      path_trim_flag_ = true;
    }
  }

  // check if the key positions need to be corrected
  if (CheckRationalityEndpointPosition()) {
    if (CurrentPathTrimmed()) {
      end_position_correction_flag_ = true;
    }
  }

  // check planning status
  ILOG_INFO << "parking status = "
            << static_cast<int>(GetPlannerStates().planning_status);
}

const bool PerpendicularHeadOutScenario::UpdateEgoSlotInfo() {
  const std::shared_ptr<ApaMeasureDataManager> measures_ptr =
      apa_world_ptr_->GetMeasureDataManagerPtr();

  const ApaParameters& param = apa_param.GetParam();

  // 建立车位坐标系 根据23角点或者限位器角点确定规划终点位姿
  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  ego_info_under_slot.origin_pose_global.heading_vec =
      ego_info_under_slot.slot.processed_corner_coord_global_
          .pt_23mid_01mid_unit_vec;

  ego_info_under_slot.origin_pose_global.heading =
      std::atan2(ego_info_under_slot.origin_pose_global.heading_vec.y(),
                 ego_info_under_slot.origin_pose_global.heading_vec.x());

  ego_info_under_slot.origin_pose_global.pos =
      ego_info_under_slot.slot.processed_corner_coord_global_.pt_01_mid -
      ego_info_under_slot.slot.slot_length_ *
          ego_info_under_slot.origin_pose_global.heading_vec;

  ego_info_under_slot.g2l_tf = geometry_lib::GlobalToLocalTf(
      ego_info_under_slot.origin_pose_global.pos,
      ego_info_under_slot.origin_pose_global.heading);

  ego_info_under_slot.l2g_tf = geometry_lib::LocalToGlobalTf(
      ego_info_under_slot.origin_pose_global.pos,
      ego_info_under_slot.origin_pose_global.heading);

  ego_info_under_slot.origin_pose_local.pos = ego_info_under_slot.g2l_tf.GetPos(
      ego_info_under_slot.origin_pose_global.pos);

  ego_info_under_slot.origin_pose_local.heading =
      ego_info_under_slot.g2l_tf.GetHeading(
          ego_info_under_slot.origin_pose_global.heading);

  ego_info_under_slot.origin_pose_local.heading_vec =
      geometry_lib::GenHeadingVec(
          ego_info_under_slot.origin_pose_local.heading);

  ego_info_under_slot.slot.TransformCoordFromGlobalToLocal(
      ego_info_under_slot.g2l_tf);

  ego_info_under_slot.cur_pose.pos =
      ego_info_under_slot.g2l_tf.GetPos(measures_ptr->GetPos());
  ego_info_under_slot.cur_pose.heading =
      ego_info_under_slot.g2l_tf.GetHeading(measures_ptr->GetHeading());
  ego_info_under_slot.cur_pose.heading_vec =
      geometry_lib::GenHeadingVec(ego_info_under_slot.cur_pose.heading);

  if (std::fabs(ego_info_under_slot.cur_pose.pos.y()) <
          param.slot_occupied_ratio_max_lat_err &&
      std::fabs(ego_info_under_slot.cur_pose.heading) <
          param.slot_occupied_ratio_max_heading_err * kDeg2Rad) {
    const std::vector<double> x_tab = {
        ego_info_under_slot.target_pose.pos.x(),
        ego_info_under_slot.slot.slot_length_ + param.rear_overhanging};

    const std::vector<double> occupied_ratio_tab = {1.0, 0.0};
    ego_info_under_slot.slot_occupied_ratio = mathlib::Interp1(
        x_tab, occupied_ratio_tab, ego_info_under_slot.cur_pose.pos.x());
  } else {
    ego_info_under_slot.slot_occupied_ratio = 0.0;
  }

  // calc init gear and init steer at first
  if (frame_.is_replan_first) {
    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_DRIVE;
    if (apa_world_ptr_->GetStateMachineManagerPtr()->GetParkOutDirection() ==
        ApaParkOutDirection::RIGHT_FRONT) {
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_RIGHT;
    } else if (apa_world_ptr_->GetStateMachineManagerPtr()
                   ->GetParkOutDirection() == ApaParkOutDirection::LEFT_FRONT) {
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_LEFT;
    } else if (apa_world_ptr_->GetStateMachineManagerPtr()
                   ->GetParkOutDirection() == ApaParkOutDirection::FRONT) {
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_STRAIGHT;
    }

    ILOG_INFO << "Now default to turn right at first";
    // judge slot side via slot center and heading
    // ...
  }

  ILOG_INFO << "frame_.current_gear = "
            << static_cast<int>(frame_.current_gear);
  ILOG_INFO << "frame_.current_arc_steer = "
            << static_cast<int>(frame_.current_arc_steer);

  ILOG_INFO << "slot_occupied_ratio = "
            << ego_info_under_slot.slot_occupied_ratio;

  return true;
}

const bool PerpendicularHeadOutScenario::GenTlane() {
  // construct tlane pq
  // left y is positive, right y is negative
  // left y should be smallest, right y should be largest
  // all x should be largest
  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>,
                      geometry_lib::Compare>
      left_pq_for_y(geometry_lib::Compare(3));
  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>,
                      geometry_lib::Compare>
      left_pq_for_x(geometry_lib::Compare(0));
  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>,
                      geometry_lib::Compare>
      right_pq_for_y(geometry_lib::Compare(2));
  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>,
                      geometry_lib::Compare>
      right_pq_for_x(geometry_lib::Compare(0));

  const auto& param = apa_param.GetParam();

  const double mir_width =
      (param.max_car_width - param.car_width) * 0.5 - 0.0168;

  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  const double mir_x = ego_info_under_slot.target_pose.pos.x() +
                       param.lon_dist_mirror_to_rear_axle - 0.368;

  apa_world_ptr_->GetObstacleManagerPtr()->TransformCoordFromGlobalToLocal(
      ego_info_under_slot.g2l_tf);

  const auto& obstacles =
      apa_world_ptr_->GetObstacleManagerPtr()->GetObstacles();

  Eigen::Vector2d obs_pt_slot;
  size_t obs_count_in_slot = 0;
  for (const auto& pair : obstacles) {
    for (const auto& obs : pair.second.GetPtClout2dLocal()) {
      obs_pt_slot = obs;
      SlotObsType obs_slot_type = CalSlotObsType(obs_pt_slot);
      if (obs_slot_type == SlotObsType::IN_OBS) {
        obs_count_in_slot++;
      }
      if (obs_slot_type != SlotObsType::INSIDE_OBS &&
          obs_slot_type != SlotObsType::OUTSIDE_OBS) {
        continue;
      }

      // the obs lower mir can relax lat requirements
      if (obs_pt_slot.x() < mir_x) {
        if (obs_pt_slot.y() > 1e-6) {
          obs_pt_slot.y() += mir_width;
        } else {
          obs_pt_slot.y() -= mir_width;
        }
      }

      // the obs far from slot can relax lon equirements
      if (std::fabs(obs_pt_slot.y()) >
          ego_info_under_slot.slot.slot_width_ * 0.5 + 0.468) {
        obs_pt_slot.x() -= 0.268;
      }

      if (obs_pt_slot.y() > 1e-6) {
        left_pq_for_y.emplace(std::move(obs_pt_slot));
        left_pq_for_x.emplace(std::move(obs_pt_slot));
      } else {
        right_pq_for_y.emplace(std::move(obs_pt_slot));
        right_pq_for_x.emplace(std::move(obs_pt_slot));
      }
    }
  }

  if (apa_world_ptr_->GetStateMachineManagerPtr()->IsSeachingStatus()) {
    if (obs_count_in_slot > 0) {
      ILOG_INFO << "there are obs in slot";
      return false;
    }
  }

  // apa_param.SetPram().actual_mono_plan_enable = param.mono_plan_enable;
  // // 如果保守的话  两侧全空才开启一把进 无意义 这个保守泊入已关闭
  // const bool left_empty = left_pq_for_x.empty();
  // const bool right_empty = right_pq_for_x.empty();
  // if (param.conservative_mono_enable && (!left_empty || !right_empty)) {
  //   apa_param.SetPram().actual_mono_plan_enable = false;
  // }

  // 加入左右侧的虚拟障碍物
  const Eigen::Vector2d pt_01_unit_vec =
      ego_info_under_slot.slot.origin_corner_coord_local_.pt_01_unit_vec;

  const Eigen::Vector2d pt_01_mid =
      ego_info_under_slot.slot.origin_corner_coord_local_.pt_01_mid;

  double half_origin_slot_width =
      ego_info_under_slot.slot.origin_corner_coord_local_.pt_01_vec.norm() *
      0.5;
  half_origin_slot_width =
      std::max(half_origin_slot_width, param.max_car_width * 0.5 + 0.168);

  const Eigen::Vector2d virtual_left_obs =
      pt_01_mid -
      param.virtual_obs_left_x_pos *
          ego_info_under_slot.origin_pose_local.heading_vec +
      (half_origin_slot_width + param.virtual_obs_left_y_pos) * pt_01_unit_vec;

  const Eigen::Vector2d virtual_right_obs =
      pt_01_mid -
      param.virtual_obs_right_x_pos *
          ego_info_under_slot.origin_pose_local.heading_vec -
      (half_origin_slot_width + param.virtual_obs_right_y_pos) * pt_01_unit_vec;

  left_pq_for_y.emplace(virtual_left_obs);
  left_pq_for_x.emplace(virtual_left_obs);
  right_pq_for_y.emplace(virtual_right_obs);
  right_pq_for_x.emplace(virtual_right_obs);

  // 找到左侧和右侧障碍物极限位置
  const Eigen::Vector2d left_obs(left_pq_for_x.top().x(),
                                 left_pq_for_y.top().y());
  const Eigen::Vector2d right_obs(right_pq_for_x.top().x(),
                                  right_pq_for_y.top().y());

  if (apa_world_ptr_->GetStateMachineManagerPtr()->GetParkOutDirection() ==
      ApaParkOutDirection::LEFT_FRONT) {
    ego_info_under_slot.pt_inside = left_obs;
  } else if (apa_world_ptr_->GetStateMachineManagerPtr()
                 ->GetParkOutDirection() == ApaParkOutDirection::RIGHT_FRONT) {
    ego_info_under_slot.pt_inside = right_obs;
  }

  const double car_width_fold_mirror = apa_param.GetParam().car_width + 0.06;
  const double car_width_no_fold_mirror = apa_param.GetParam().max_car_width;
  double car_width = car_width_no_fold_mirror;

  if (apa_world_ptr_->GetMeasureDataManagerPtr()->GetFoldMirrorFlag()) {
    car_width = car_width_fold_mirror;
  }

  const Eigen::Vector2d left_car(2.168, car_width * 0.5);
  const Eigen::Vector2d right_car(2.168, -car_width * 0.5);

  const double left_dis_obs_car =
      (left_obs - left_car).y() / ego_info_under_slot.slot.sin_angle_;
  const double right_dis_obs_car =
      (right_car - right_obs).y() / ego_info_under_slot.slot.sin_angle_;

  ILOG_INFO << "left_dis_obs_car = " << left_dis_obs_car
            << "  right_dis_obs_car = " << right_dis_obs_car;

  const double suitable_safe_threshold =
      apa_param.GetParam().car_lat_inflation_normal +
      apa_param.GetParam().safe_threshold;

  const double min_safe_threshold =
      apa_param.GetParam().car_lat_inflation_normal + 0.0168;

  if (apa_world_ptr_->GetStateMachineManagerPtr()->IsSeachingStatus()) {
    if (left_dis_obs_car + right_dis_obs_car <
        2.0 * suitable_safe_threshold + 0.01) {
      ILOG_INFO << "left and right obs dis is not meed need";
      return false;
    }
  }

  // const bool left_obs_meet_safe_require =
  //     left_dis_obs_car > suitable_safe_threshold ? true : false;

  // const bool right_obs_meet_safe_require =
  //     right_dis_obs_car > suitable_safe_threshold ? true : false;

  // // 一旦重规划失败 用上次重规划的车位移动距离
  // ego_info_under_slot.last_move_slot_dist =
  // ego_info_under_slot.move_slot_dist;
  // //
  // 只有在重规划且没有完全入库的时候才需要根据障碍物来改变是否移动车位或者移动车位的距离
  // if (frame_.replan_flag) {
  //   if (!left_obs_meet_safe_require && right_obs_meet_safe_require) {
  //     ego_info_under_slot.move_slot_dist =
  //         -1.0 * std::min(right_dis_obs_car - min_safe_threshold,
  //                         suitable_safe_threshold - left_dis_obs_car);
  //     ILOG_INFO << "left side is dangerous, should move toward right";
  //   } else if (left_obs_meet_safe_require && !right_obs_meet_safe_require) {
  //     ego_info_under_slot.move_slot_dist =
  //         std::min(left_dis_obs_car - min_safe_threshold,
  //                  suitable_safe_threshold - right_dis_obs_car);
  //     ILOG_INFO << "right side is dangerous, should move toward left";
  //   } else if (left_obs_meet_safe_require && right_obs_meet_safe_require) {
  //     ego_info_under_slot.move_slot_dist = 0.0;
  //     ILOG_INFO << "two side is safe, should not move slot";
  //   } else if (!left_obs_meet_safe_require && !right_obs_meet_safe_require) {
  //     // 往两侧障碍物正中停
  //     const double mid_dist = (left_dis_obs_car + right_dis_obs_car) * 0.5;
  //     if (mid_dist < left_dis_obs_car) {
  //       // 往右移
  //       ego_info_under_slot.move_slot_dist = mid_dist - left_dis_obs_car;
  //     } else if (mid_dist < right_dis_obs_car) {
  //       // 往左移
  //       ego_info_under_slot.move_slot_dist = right_dis_obs_car - mid_dist;
  //     }
  //     ILOG_INFO << "two side is dangerous, park in middle of two side obs";
  //   }

  //   // 计算最大移动车位距离 因为不仅要考虑安全
  //   // 也要考虑到底自车车体能侵入旁边空间多少
  //   // 车体能压线的距离 正数表示不能越过线 负数表示可以越过
  //   const double max_move_slot_dist =
  //       ego_info_under_slot.slot.slot_width_ * 0.5 -
  //       car_width_fold_mirror * 0.5 -
  //       apa_param.GetParam().car2line_dist_threshold;
  //   if (ego_info_under_slot.move_slot_dist > 0.0) {
  //     ego_info_under_slot.move_slot_dist =
  //         std::min(ego_info_under_slot.move_slot_dist, max_move_slot_dist);
  //   } else {
  //     ego_info_under_slot.move_slot_dist =
  //         std::max(ego_info_under_slot.move_slot_dist, -max_move_slot_dist);
  //   }
  // }

  // ILOG_INFO << "move_slot_dist = " << ego_info_under_slot.move_slot_dist
  //           << "  last move_slot_dist = "
  //           << ego_info_under_slot.last_move_slot_dist;
  // JSON_DEBUG_VALUE("move_slot_dist", ego_info_under_slot.move_slot_dist)

  // // 根据车位移动距离更新终点位置
  // ego_info_under_slot.target_pose.pos +=
  //     ego_info_under_slot.move_slot_dist * pt_01_unit_vec;

  // ego_info_under_slot.terminal_err.Set(
  //     ego_info_under_slot.cur_pose.pos - ego_info_under_slot.target_pose.pos,
  //     geometry_lib::NormalizeAngle(ego_info_under_slot.cur_pose.heading -
  //                                  ego_info_under_slot.target_pose.heading));

  // 根据自车位置制定通道宽
  double channel_width =
      apa_world_ptr_->GetCollisionDetectorPtr()->GetCarMaxX(
          ego_info_under_slot.cur_pose) +
      3.168 -
      std::max(ego_info_under_slot.slot.origin_corner_coord_local_.pt_0.x(),
               ego_info_under_slot.slot.origin_corner_coord_local_.pt_1.x());

  channel_width = std::max(channel_width, apa_param.GetParam().channel_width);
  // if (ego_info_under_slot.slot_occupied_ratio > 0.768 &&
  //     channel_width > apa_param.GetParam().min_channel_width) {
  //   channel_width = apa_param.GetParam().min_channel_width;
  // }

  // 根据实际障碍物计算一个宽泛的障碍物tlane 为了后续产生虚拟障碍物
  TLane& obs_tlane = ego_info_under_slot.obs_tlane;
  obs_tlane.B = virtual_left_obs;
  obs_tlane.E = virtual_right_obs;
  if (ego_info_under_slot.slot_occupied_ratio < 1e-3) {
    obs_tlane.B.x() += 2.0;
    obs_tlane.E.x() += 2.0;
  }

  const double area_length = 12.0 / ego_info_under_slot.slot.sin_angle_;
  const double area_width =
      channel_width +
      std::max(param.virtual_obs_left_x_pos, param.virtual_obs_right_x_pos);

  obs_tlane.A = obs_tlane.B + pt_01_unit_vec * area_length;
  obs_tlane.H = obs_tlane.A +
                ego_info_under_slot.origin_pose_local.heading_vec * area_width;

  const double origin_slot_length =
      (ego_info_under_slot.slot.origin_corner_coord_local_.pt_0 -
       ego_info_under_slot.slot.origin_corner_coord_local_.pt_2)
          .norm();

  obs_tlane.C = obs_tlane.B -
                ego_info_under_slot.origin_pose_local.heading_vec *
                    (origin_slot_length - param.virtual_obs_left_x_pos + 0.68);

  obs_tlane.D = obs_tlane.E -
                ego_info_under_slot.origin_pose_local.heading_vec *
                    (origin_slot_length - param.virtual_obs_right_x_pos + 0.68);

  obs_tlane.F = obs_tlane.E - pt_01_unit_vec * area_length;
  obs_tlane.G = obs_tlane.F +
                ego_info_under_slot.origin_pose_local.heading_vec * area_width;

  obs_tlane.CalcBound();

  ILOG_INFO
      << "cur pose = " << ego_info_under_slot.cur_pose.pos.transpose() << "  "
      << ego_info_under_slot.cur_pose.heading * kRad2Deg
      << "  tar_pose = " << ego_info_under_slot.target_pose.pos.transpose()
      << "  " << ego_info_under_slot.target_pose.heading * kRad2Deg
      << "  terminal_err = " << ego_info_under_slot.terminal_err.pos.transpose()
      << "  " << ego_info_under_slot.terminal_err.heading * kRad2Deg
      << "  slot occupied ratio = " << ego_info_under_slot.slot_occupied_ratio
      << "  pt_inside = " << ego_info_under_slot.pt_inside.transpose()
      << "  stuck time(s) = " << frame_.stuck_time
      << "  stuck_obs_time(s) = " << frame_.stuck_obs_time << "  slod side = "
      << geometry_lib::GetSlotSideString(ego_info_under_slot.slot_side);

  return true;
}

const bool PerpendicularHeadOutScenario::GenObstacles() {
  apa_world_ptr_->GetCollisionDetectorPtr()->ClearObstacles();

  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const TLane& obs_tlane = ego_info_under_slot.obs_tlane;

  geometry_lib::LineSegment tlane_line;
  std::vector<geometry_lib::LineSegment> tlane_line_vec;
  tlane_line.SetPoints(obs_tlane.A, obs_tlane.B);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(obs_tlane.B, obs_tlane.C);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(obs_tlane.C, obs_tlane.D);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(obs_tlane.D, obs_tlane.E);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(obs_tlane.E, obs_tlane.F);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(obs_tlane.F, obs_tlane.G);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(obs_tlane.G, obs_tlane.H);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(obs_tlane.H, obs_tlane.A);
  tlane_line_vec.emplace_back(tlane_line);

  std::vector<Eigen::Vector2d> tlane_obstacle_vec;
  tlane_obstacle_vec.clear();
  tlane_obstacle_vec.reserve(188);
  std::vector<Eigen::Vector2d> point_set;
  for (const auto& line : tlane_line_vec) {
    geometry_lib::SamplePointSetInLineSeg(point_set, line,
                                          apa_param.GetParam().obstacle_ds);
    tlane_obstacle_vec.insert(tlane_obstacle_vec.end(), point_set.begin(),
                              point_set.end());
  }

  apa_world_ptr_->GetCollisionDetectorPtr()->SetParam(
      CollisionDetector::Paramters(
          apa_param.GetParam().car_lat_inflation_normal));

  // 临时做法 后面障碍物不应该在这里做处理 并赋值新的类型
  // 应该统一使用障碍物管理器中的类型 对于虚拟障碍物
  // 自车位置附近的虚拟障碍物可以删除 避免无效提升规划难度 add virtual tlane obs
  std::vector<Eigen::Vector2d> tlane_obs_vec;
  tlane_obs_vec.reserve(tlane_obstacle_vec.size());
  for (const Eigen::Vector2d& obs_pos : tlane_obstacle_vec) {
    if (!apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
            obs_pos, ego_info_under_slot.cur_pose, 0.0168)) {
      tlane_obs_vec.emplace_back(obs_pos);
    }
  }
  apa_world_ptr_->GetCollisionDetectorPtr()->AddObstacles(
      tlane_obs_vec, CollisionDetector::TLANE_OBS);

  // add actual obs
  const std::vector<Eigen::Vector2d> tlane_vec{
      obs_tlane.A, obs_tlane.B, obs_tlane.C, obs_tlane.D,
      obs_tlane.E, obs_tlane.F, obs_tlane.G, obs_tlane.H};

  std::vector<Eigen::Vector2d> fus_obs_vec;
  const auto& obstacles =
      apa_world_ptr_->GetObstacleManagerPtr()->GetObstacles();

  Eigen::Vector2d obs_pt_slot;
  for (const auto& pair : obstacles) {
    for (const auto& obs : pair.second.GetPtClout2dLocal()) {
      if (apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
              obs, ego_info_under_slot.cur_pose, 0.0168)) {
        // temp hack, when obs is in car, lose it, only increase plan success
        // ratio, To Do, when obs change accurately, should not del any obs
        if (apa_world_ptr_->GetStateMachineManagerPtr()->IsSeachingStatus()) {
          return false;
        }
        continue;
      }
      SlotObsType obs_type = CalSlotObsType(obs);

      if (obs_type == SlotObsType::DISCARD_OBS ||
          obs_type == SlotObsType::IN_OBS) {
        continue;
      }

      // if obs is not in tlane area lose it
      if (!geometry_lib::IsPointInPolygon(tlane_vec, obs)) {
        continue;
      }

      fus_obs_vec.emplace_back(obs);
    }
  }

  apa_world_ptr_->GetCollisionDetectorPtr()->AddObstacles(
      fus_obs_vec, CollisionDetector::FUSION_OBS);

  const double bound_threshold = 0.68;

  OccupancyGridBound bound(
      obs_tlane.min_x - bound_threshold, obs_tlane.min_y - bound_threshold,
      obs_tlane.max_x + bound_threshold, obs_tlane.max_y + bound_threshold);

  bound.PrintInfo();

  // apa_world_ptr_->GetCollisionDetectorPtr()->TransObsMapToOccupancyGridMap(
  //     bound);

  return true;
}

const uint8_t PerpendicularHeadOutScenario::PathPlanOnce() {
  ILOG_INFO << "-------------- PathPlanOnce --------------";
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const bool conditions_1 =
      ego_info_under_slot.cur_pose.pos.x() < 5.5 && frame_.is_replan_second;

  const bool fix_shift_conditions = conditions_1 || frame_.is_replan_first;
  ILOG_INFO << "ego_pos_slot x " << ego_info_under_slot.cur_pose.pos.x();

  if (!fix_shift_conditions) {
    ILOG_INFO << "current plan should shift gear";
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

  GeometryPathInput input;
  input.ego_info_under_slot = ego_info_under_slot;
  input.is_complete_path = apa_world_ptr_->GetSimuParam().is_complete_path;
  input.sample_ds = apa_world_ptr_->GetSimuParam().sample_ds;
  input.ref_gear = frame_.current_gear;
  input.ref_arc_steer = frame_.current_arc_steer;
  input.is_replan_first = frame_.is_replan_first;
  input.is_replan_second = frame_.is_replan_second;
  input.is_replan_dynamic = frame_.is_replan_dynamic;
  input.is_searching_stage =
      apa_world_ptr_->GetStateMachineManagerPtr()->IsSeachingStatus();

  // if (frame_.replan_reason == DYNAMIC) {
  //   ILOG_INFO << "dynamic replan, gear should be reverse";
  //   input.ref_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
  // }

  perpendicular_path_planner_.SetInput(input);

  // need replan all path
  const bool path_plan_success =
      perpendicular_path_planner_.GeometryPathGenerator::Update(
          apa_world_ptr_->GetCollisionDetectorPtr());

  if (input.is_searching_stage) {
    if (path_plan_success) {
      return PathPlannerResult::PLAN_UPDATE;
    } else {
      return PathPlannerResult::PLAN_FAILED;
    }
  }

  uint8_t plan_result = 0;
  if (!path_plan_success && !apa_world_ptr_->GetSimuParam().is_simulation) {
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

  geometry_lib::PathSegment path_seg_global;
  for (size_t i = planner_output.path_seg_index.first;
       i < planner_output.path_segment_vec.size(); ++i) {
    path_seg_global = planner_output.path_segment_vec[i];
    path_seg_global.LocalToGlobal(ego_info_under_slot.l2g_tf);

    if (i <= planner_output.path_seg_index.second) {
      current_plan_path_vec_.emplace_back(path_seg_global);
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

  current_path_point_global_vec_.clear();
  current_path_point_global_vec_.reserve(planner_output.path_point_vec.size() +
                                         18);

  complete_path_point_global_vec_.clear();
  complete_path_point_global_vec_.reserve(
      planner_output.all_gear_path_point_vec.size() + 18);

  std::vector<pnc::geometry_lib::PathPoint> path_pt_vec;

  // if (LateralPathOptimize(path_pt_vec)) {
  //   JSON_DEBUG_VALUE("is_path_lateral_optimized", true);
  // } else {
  //   path_pt_vec = planner_output.path_point_vec;
  //   JSON_DEBUG_VALUE("is_path_lateral_optimized", false);
  // }

  path_pt_vec = planner_output.path_point_vec;

  geometry_lib::PathPoint global_point;
  for (const auto& path_point : path_pt_vec) {
    global_point.Set(ego_info_under_slot.l2g_tf.GetPos(path_point.pos),
                     ego_info_under_slot.l2g_tf.GetHeading(path_point.heading));
    global_point.lat_buffer = path_point.lat_buffer;
    global_point.s = path_point.s;
    global_point.kappa = path_point.kappa;
    current_path_point_global_vec_.emplace_back(global_point);
  }
  for (const auto& path_point : planner_output.all_gear_path_point_vec) {
    global_point.Set(ego_info_under_slot.l2g_tf.GetPos(path_point.pos),
                     ego_info_under_slot.l2g_tf.GetHeading(path_point.heading));
    global_point.lat_buffer = path_point.lat_buffer;
    global_point.s = path_point.s;
    global_point.kappa = path_point.kappa;
    complete_path_point_global_vec_.emplace_back(global_point);
  }

  JSON_DEBUG_VECTOR("plan_traj_x", std::vector<double>{0.0}, 3)
  JSON_DEBUG_VECTOR("plan_traj_y", std::vector<double>{0.0}, 3)
  JSON_DEBUG_VECTOR("plan_traj_heading", std::vector<double>{0.0}, 3)
  JSON_DEBUG_VECTOR("plan_traj_lat_buffer", std::vector<double>{0.0}, 3)

  // JSON_DEBUG_VALUE("cilqr_optimization_enable", cilqr_optimization_enable);
  // JSON_DEBUG_VALUE("lat_path_opt_cost_time_ms", lat_path_opt_cost_time_ms);

  ILOG_INFO << "current_path_point_global_vec_.size() = "
            << current_path_point_global_vec_.size();

  return plan_result;
}

const bool PerpendicularHeadOutScenario::CheckFinished() {
  bool parking_finish = false;
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const bool heading_condition_1 =
      std::fabs(ego_info_under_slot.cur_pose.heading) <=
      100.0 * kDeg2Rad;  // TODU::

  const bool heading_condition_2 =
      std::fabs(ego_info_under_slot.cur_pose.heading) >= 70.0 * kDeg2Rad;

  const bool lat_condition = heading_condition_1 && heading_condition_2;

  const bool static_condition =
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag();

  const bool remain_s_condition =
      frame_.remain_dist_path < apa_param.GetParam().max_replan_remain_dist;

  if (frame_.current_arc_steer == pnc::geometry_lib::SEG_STEER_STRAIGHT) {
    parking_finish = remain_s_condition && static_condition;
  } else {
    parking_finish = lat_condition && static_condition && remain_s_condition;
  }

  if (parking_finish) {
    return true;
  }

  // stucked by directly behind uss
  const std::shared_ptr<UssObstacleAvoidance>& uss_obstacle_avoider_ptr =
      apa_world_ptr_->GetCollisionDetectorInterfacePtr()
          ->GetUssObsAvoidancePtr();
  const bool enter_slot_condition =
      ego_info_under_slot.slot_occupied_ratio >
      apa_param.GetParam().finish_uss_slot_occupied_ratio;
  const bool remain_uss_condition =
      frame_.remain_dist_obs < apa_param.GetParam().max_replan_remain_dist;
  if (uss_obstacle_avoider_ptr->CheckIsDirectlyBehindUss()) {
    parking_finish = lat_condition && static_condition &&
                     enter_slot_condition && remain_uss_condition;
  }

  return parking_finish;
}

void PerpendicularHeadOutScenario::Log() const {
  JSON_DEBUG_VALUE("correct_path_for_limiter", frame_.correct_path_for_limiter)
  JSON_DEBUG_VALUE("replan_flag", frame_.replan_flag)

  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const geometry_lib::LocalToGlobalTf& l2g_tf = ego_info_under_slot.l2g_tf;

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
      if (obs_pair.first == CollisionDetector::FUSION_OBS) {
        continue;
      }
      const Eigen::Vector2d tmp_obstacle = l2g_tf.GetPos(obstacle);
      obstaclesX.emplace_back(tmp_obstacle.x());
      obstaclesY.emplace_back(tmp_obstacle.y());
    }
  }

  JSON_DEBUG_VECTOR("obstaclesX", obstaclesX, 2)
  JSON_DEBUG_VECTOR("obstaclesY", obstaclesY, 2)

  std::vector<double> slot_corner_X;
  slot_corner_X.clear();
  slot_corner_X.reserve(16);
  std::vector<double> slot_corner_Y;
  slot_corner_Y.clear();
  slot_corner_Y.reserve(16);

  const auto& origin_corner_coord_global =
      ego_info_under_slot.slot.origin_corner_coord_global_;

  std::vector<Eigen::Vector2d> pt_vec{
      origin_corner_coord_global.pt_0, origin_corner_coord_global.pt_1,
      origin_corner_coord_global.pt_2, origin_corner_coord_global.pt_3};

  for (const Eigen::Vector2d& pt : pt_vec) {
    slot_corner_X.emplace_back(pt.x());
    slot_corner_Y.emplace_back(pt.y());
  }

  slot_corner_X.emplace_back(((pt_vec[0] + pt_vec[1]) * 0.5).x());
  slot_corner_Y.emplace_back(((pt_vec[0] + pt_vec[1]) * 0.5).y());
  slot_corner_X.emplace_back(((pt_vec[2] + pt_vec[3]) * 0.5).x());
  slot_corner_Y.emplace_back(((pt_vec[2] + pt_vec[3]) * 0.5).y());

  pt_vec[0] = pt_vec[0] + ego_info_under_slot.move_slot_dist *
                              origin_corner_coord_global.pt_01_unit_vec;
  pt_vec[1] = pt_vec[1] + ego_info_under_slot.move_slot_dist *
                              origin_corner_coord_global.pt_01_unit_vec;
  pt_vec[2] = pt_vec[2] + ego_info_under_slot.move_slot_dist *
                              origin_corner_coord_global.pt_23_unit_vec;
  pt_vec[3] = pt_vec[3] + ego_info_under_slot.move_slot_dist *
                              origin_corner_coord_global.pt_23_unit_vec;

  for (const Eigen::Vector2d& pt : pt_vec) {
    slot_corner_X.emplace_back(pt.x());
    slot_corner_Y.emplace_back(pt.y());
  }

  slot_corner_X.emplace_back(((pt_vec[0] + pt_vec[1]) * 0.5).x());
  slot_corner_Y.emplace_back(((pt_vec[0] + pt_vec[1]) * 0.5).y());
  slot_corner_X.emplace_back(((pt_vec[2] + pt_vec[3]) * 0.5).x());
  slot_corner_Y.emplace_back(((pt_vec[2] + pt_vec[3]) * 0.5).y());

  slot_corner_X.emplace_back(
      l2g_tf.GetPos(ego_info_under_slot.target_pose.pos).x());
  slot_corner_Y.emplace_back(
      l2g_tf.GetPos(ego_info_under_slot.target_pose.pos).y());

  JSON_DEBUG_VECTOR("slot_corner_X", slot_corner_X, 3)
  JSON_DEBUG_VECTOR("slot_corner_Y", slot_corner_Y, 3)

  std::vector<double> limiter_corner_X;
  limiter_corner_X.clear();
  limiter_corner_X.reserve(3);
  std::vector<double> limiter_corner_Y;
  limiter_corner_Y.clear();
  limiter_corner_Y.reserve(3);

  limiter_corner_X.emplace_back(
      l2g_tf.GetPos(ego_info_under_slot.virtual_limiter.first).x());
  limiter_corner_X.emplace_back(
      l2g_tf.GetPos(ego_info_under_slot.virtual_limiter.second).x());
  limiter_corner_Y.emplace_back(
      l2g_tf.GetPos(ego_info_under_slot.virtual_limiter.first).y());
  limiter_corner_Y.emplace_back(
      l2g_tf.GetPos(ego_info_under_slot.virtual_limiter.second).y());

  JSON_DEBUG_VECTOR("limiter_corner_X", limiter_corner_X, 2)
  JSON_DEBUG_VECTOR("limiter_corner_Y", limiter_corner_Y, 2)

  JSON_DEBUG_VALUE("terminal_error_x", ego_info_under_slot.terminal_err.pos.x())
  JSON_DEBUG_VALUE("terminal_error_y", ego_info_under_slot.terminal_err.pos.y())
  JSON_DEBUG_VALUE("terminal_error_heading",
                   ego_info_under_slot.terminal_err.heading)

  JSON_DEBUG_VALUE("replan_flag", frame_.replan_flag)
  JSON_DEBUG_VALUE("is_replan_first", frame_.is_replan_first)
  JSON_DEBUG_VALUE("is_replan_by_uss", frame_.is_replan_by_obs)
  JSON_DEBUG_VALUE("current_path_length", frame_.current_path_length)
  JSON_DEBUG_VALUE("path_plan_success", frame_.plan_stm.path_plan_success)
  JSON_DEBUG_VALUE("planning_status", frame_.plan_stm.planning_status)
  JSON_DEBUG_VALUE("spline_success", frame_.spline_success)
  JSON_DEBUG_VALUE("remain_dist", frame_.remain_dist_path)
  JSON_DEBUG_VALUE("remain_dist_col_det", frame_.remain_dist_col_det)
  JSON_DEBUG_VALUE("remain_dist_uss", frame_.remain_dist_obs)
  JSON_DEBUG_VALUE("stuck_time", frame_.stuck_time)
  JSON_DEBUG_VALUE("replan_reason", frame_.replan_reason)
  JSON_DEBUG_VALUE("plan_fail_reason", frame_.plan_fail_reason)

  JSON_DEBUG_VALUE("selected_slot_id", ego_info_under_slot.id)
  JSON_DEBUG_VALUE("slot_length", ego_info_under_slot.slot.slot_length_)
  JSON_DEBUG_VALUE("slot_width", ego_info_under_slot.slot.slot_width_)

  JSON_DEBUG_VALUE("slot_origin_pos_x",
                   ego_info_under_slot.origin_pose_global.pos.x())

  JSON_DEBUG_VALUE("slot_origin_pos_y",
                   ego_info_under_slot.origin_pose_global.pos.y())

  JSON_DEBUG_VALUE("slot_origin_heading",
                   ego_info_under_slot.origin_pose_global.heading)

  JSON_DEBUG_VALUE("slot_occupied_ratio",
                   ego_info_under_slot.slot_occupied_ratio)

  std::vector<double> target_ego_pos_slot = {
      ego_info_under_slot.target_pose.pos.x(),
      ego_info_under_slot.target_pose.pos.y()};

  JSON_DEBUG_VALUE("pathplan_result", frame_.pathplan_result)
  JSON_DEBUG_VECTOR("target_ego_pos_slot", target_ego_pos_slot, 2)

  const auto& path_plan_output = perpendicular_path_planner_.GetOutput();
  JSON_DEBUG_VALUE("path_start_seg_index",
                   path_plan_output.path_seg_index.first)
  JSON_DEBUG_VALUE("path_end_seg_index", path_plan_output.path_seg_index.second)
  JSON_DEBUG_VALUE("path_length", path_plan_output.length)

  const UssObstacleAvoidance::RemainDistInfo uss_info =
      apa_world_ptr_->GetCollisionDetectorInterfacePtr()
          ->GetUssObsAvoidancePtr()
          ->GetRemainDistInfo();
  JSON_DEBUG_VALUE("uss_available", uss_info.is_available)
  JSON_DEBUG_VALUE("uss_remain_dist", uss_info.remain_dist)
  JSON_DEBUG_VALUE("uss_index", uss_info.uss_index)
  JSON_DEBUG_VALUE("uss_car_index", uss_info.car_index)
}

const PerpendicularHeadOutScenario::SlotObsType
PerpendicularHeadOutScenario::CalSlotObsType(const Eigen::Vector2d& obs_slot) {
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();
  // 2米2的车位重规划考虑的障碍物单侧最多入侵车位15厘米
  double dy1 = 0.15 / 1.1 * (ego_info_under_slot.slot.slot_width_ * 0.5);

  // 内外侧障碍物往远离车位的一遍考虑1.68米就可以
  double dy2 = apa_param.GetParam().x_max_internal_obstacles;

  // 最多高于车位3.468米的障碍物可以当做内外侧障碍物
  double dx1 = 3.468;
  // // 但是如果自车位置本身较低 那么内外侧障碍物考虑的x值也应该降低
  // dx1 = std::min(dx1, ego_info_under_slot.cur_pose.pos.x() -
  //                         apa_param.GetParam().car_width * 0.5 -
  //                         ego_info_under_slot.slot.slot_length_);
  // // 也需要有个最低考虑位置
  // dx1 = std::max(dx1, 0.368);

  // 对于5米长的车位 从车位线往内延长3.86米当做内外侧障碍物即可
  // 这个时候可以参考当做根据障碍物移动车位的标准， 再深就无需横向移动
  double dx2 = 4.86 / 5.0 * ego_info_under_slot.slot.slot_length_;

  // 对于5米长的车位 最多往后5.2米的障碍物可以在重规划的时候不考虑
  // 再往后就要考虑
  double dx3 = 5.2 / 5.0 * ego_info_under_slot.slot.slot_length_ - dx2;

  Eigen::Vector2d slot_left_pt =
      ego_info_under_slot.slot.origin_corner_coord_local_.pt_1;
  Eigen::Vector2d slot_right_pt =
      ego_info_under_slot.slot.origin_corner_coord_local_.pt_0;
  if (slot_left_pt.y() < slot_right_pt.y()) {
    std::swap(slot_left_pt, slot_right_pt);
  }

  bool is_left_side = false;
  if (ego_info_under_slot.slot_side == geometry_lib::SLOT_SIDE_LEFT) {
    is_left_side = true;
  }

  std::vector<Eigen::Vector2d> inside_area;
  std::vector<Eigen::Vector2d> outside_area;
  std::vector<Eigen::Vector2d> in_area;
  std::vector<Eigen::Vector2d> discard_area;
  inside_area.resize(4);
  outside_area.resize(4);
  in_area.resize(4);
  discard_area.resize(4);

  const Eigen::Vector2d unit_right2left_vec =
      (slot_left_pt - slot_right_pt).normalized();
  const Eigen::Vector2d unit_left2right_vec = -unit_right2left_vec;
  const Eigen::Vector2d unit_up2down_vec(-1.0, 0.0);
  const Eigen::Vector2d unit_down2up_vec = -unit_up2down_vec;

  // Firstly, the default right side is the inner side, and the left side is the
  // outer side
  Eigen::Vector2d pt;
  // cal inside area
  pt = slot_right_pt + dy1 * unit_right2left_vec + dx1 * unit_down2up_vec;
  inside_area[0] = pt;
  pt = slot_right_pt + dy2 * unit_left2right_vec + dx1 * unit_down2up_vec;
  inside_area[1] = pt;
  pt = slot_right_pt + dy2 * unit_left2right_vec + dx2 * unit_up2down_vec;
  inside_area[2] = pt;
  pt = slot_right_pt + dy1 * unit_right2left_vec + dx2 * unit_up2down_vec;
  inside_area[3] = pt;

  // cal outside area
  pt = slot_left_pt + dy2 * unit_right2left_vec + dx1 * unit_down2up_vec;
  outside_area[0] = pt;
  pt = slot_left_pt + dy1 * unit_left2right_vec + dx1 * unit_down2up_vec;
  outside_area[1] = pt;
  pt = slot_left_pt + dy1 * unit_left2right_vec + dx2 * unit_up2down_vec;
  outside_area[2] = pt;
  pt = slot_left_pt + dy2 * unit_right2left_vec + dx2 * unit_up2down_vec;
  outside_area[3] = pt;

  if (is_left_side) {
    std::swap(inside_area, outside_area);
  }

  // cal in_area
  pt = slot_left_pt + dy1 * unit_left2right_vec + dx1 * unit_down2up_vec;
  in_area[0] = pt;
  pt = slot_right_pt + dy1 * unit_right2left_vec + dx1 * unit_down2up_vec;
  in_area[1] = pt;
  pt = slot_right_pt + dy1 * unit_right2left_vec + dx2 * unit_up2down_vec;
  in_area[2] = pt;
  pt = slot_left_pt + dy1 * unit_left2right_vec + dx2 * unit_up2down_vec;
  in_area[3] = pt;

  // cal discard area
  pt = slot_left_pt + dy2 * unit_right2left_vec + dx2 * unit_up2down_vec;
  discard_area[0] = pt;
  pt = slot_right_pt + dy2 * unit_left2right_vec + dx2 * unit_up2down_vec;
  discard_area[1] = pt;
  pt = slot_right_pt + dy2 * unit_left2right_vec +
       (dx2 + dx3) * unit_up2down_vec;
  discard_area[2] = pt;
  pt =
      slot_left_pt + dy2 * unit_right2left_vec + (dx2 + dx3) * unit_up2down_vec;
  discard_area[3] = pt;

  if (geometry_lib::IsPointInPolygon(inside_area, obs_slot)) {
    return SlotObsType::INSIDE_OBS;
  } else if (geometry_lib::IsPointInPolygon(outside_area, obs_slot)) {
    return SlotObsType::OUTSIDE_OBS;
  } else if (geometry_lib::IsPointInPolygon(in_area, obs_slot)) {
    return SlotObsType::IN_OBS;
  } else if (geometry_lib::IsPointInPolygon(discard_area, obs_slot)) {
    return SlotObsType::DISCARD_OBS;
  } else {
    return SlotObsType::OTHER_OBS;
  }
}

const bool PerpendicularHeadOutScenario ::CheckSecurityCurrentpath() {
  return !path_trim_flag_ && apa_world_ptr_->GetSlotManagerPtr()
                                     ->GetEgoInfoUnderSlot()
                                     .slot_occupied_ratio < 0.15;
}

const bool PerpendicularHeadOutScenario ::CheckRationalityEndpointPosition() {
  if (current_path_point_global_vec_.size() > 0) {
    const pnc::geometry_lib::PathPoint& current_path_last_point =
        current_path_point_global_vec_.back();
    Eigen::Vector2d current_path_last_local_point =
        apa_world_ptr_->GetSlotManagerPtr()
            ->GetEgoInfoUnderSlot()
            .g2l_tf.GetPos(current_path_last_point.pos);

    frame_.current_path_last_point_heading =
        apa_world_ptr_->GetSlotManagerPtr()
            ->GetEgoInfoUnderSlot()
            .g2l_tf.GetHeading(current_path_last_point.heading);

    const bool conditions_endpoint_correction =
        current_path_last_local_point.x() < 7.0 &&
        frame_.current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE &&
        fabs(frame_.current_path_last_point_heading * kRad2Deg) > 60;
    return conditions_endpoint_correction;
  } else {
    return false;
  }
}

const bool PerpendicularHeadOutScenario::CurrentPathTrimmed() {
  if (frame_.current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
    const double current_car_s =
        frame_.current_path_length - frame_.remain_dist_path;
    // ILOG_INFO << "current_car_s " << current_car_s;
    const pnc::geometry_lib::GlobalToLocalTf& g2l_tf =
        apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot().g2l_tf;

    const pnc::geometry_lib::LocalToGlobalTf& l2g_tf =
        apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot().l2g_tf;

    std::vector<pnc::geometry_lib::PathPoint> path_point_local_vec;
    pnc::geometry_lib::PathPoint path_point_local;
    path_point_local_vec.clear();
    path_point_local_vec.reserve(200);
    for (const auto& point : current_path_point_global_vec_) {
      // *
      // move the starting point forward by 100 centimeters to ensure that the
      // collision detection path is outside the slot
      if (point.s >= current_car_s + 1.5) {
        path_point_local = point;
        path_point_local.GlobalToLocal(g2l_tf);
        path_point_local_vec.emplace_back(path_point_local);
      }
    }

    CollisionDetector::CollisionResult col_res;
    col_res = apa_world_ptr_->GetCollisionDetectorPtr()->UpdateByObsMap(
        path_point_local_vec, 0.35, 0.3);

    // ILOG_INFO << "col_pt_obs_global " << col_res.col_pt_obs_global.x() << ",
    // "
    //           << col_res.col_pt_obs_global.y();

    if (col_res.remain_dist == frame_.current_path_length) {
      ILOG_INFO << "at this time, there is no collision in the path";
      return true;
    }

    if (col_res.col_pt_obs_global.x() <= apa_world_ptr_->GetSlotManagerPtr()
                                                 ->GetEgoInfoUnderSlot()
                                                 .pt_inside.x() +
                                             1.0) {
      ILOG_INFO << "at this time, the collision occurred on the inner side";
      // ILOG_INFO << "current pos : "
      //           << path_point_local_vec.front().pos.transpose();
      return false;
    }

    const double lon_buffer = 0.4;
    const double safe_remain_dist = col_res.remain_dist - lon_buffer;

    // ILOG_INFO << "safe_remain_dist : " << safe_remain_dist;

    auto new_end = std::remove_if(
        current_path_point_global_vec_.begin(),
        current_path_point_global_vec_.end(),
        [this, safe_remain_dist](const pnc::geometry_lib::PathPoint& point) {
          return point.s > safe_remain_dist;
        });

    current_path_point_global_vec_.erase(new_end,
                                         current_path_point_global_vec_.end());

    PostProcessPath();
    ILOG_INFO << "the current path has been trimmed";

    return true;
  } else if (frame_.current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
    const double heading_condition =
        frame_.current_path_last_point_heading * kRad2Deg;
    ILOG_INFO << "current_path_last_point_heading : " << heading_condition;
    size_t size = current_path_point_global_vec_.size();
    if (size > 0) {
      if (fabs(heading_condition) > 70) {
        size_t half_size = static_cast<size_t>(size / 2);
        current_path_point_global_vec_.resize(half_size);
      } else if (fabs(heading_condition) > 60 &&
                 fabs(heading_condition) <= 70) {
        size_t half_size = static_cast<size_t>(size * 2 / 3);
        current_path_point_global_vec_.resize(half_size);
      }

    } else {
      return false;
    }

    PostProcessPath();
    ILOG_INFO << "the current path has been trimmed";

    return true;
  }
  return true;
}

}  // namespace apa_planner

}  // namespace planning
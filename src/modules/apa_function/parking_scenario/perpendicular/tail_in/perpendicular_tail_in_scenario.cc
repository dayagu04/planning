#include "perpendicular_tail_in_scenario.h"

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "apa_context.h"
#include "apa_obstacle.h"
#include "apa_param_config.h"
#include "apa_slot.h"
#include "apa_slot_manager.h"
#include "apa_state_machine_manager.h"
#include "collision_detection/collision_detection.h"
#include "collision_detection/gjk_collision_detector.h"
#include "debug_info_log.h"
#include "generate_obstacle_decider/generate_obstacle_decider.h"
#include "geometry_math.h"
#include "geometry_path_generator.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "math_lib.h"
#include "parking_scenario.h"
#include "perpendicular_tail_in_path_generator.h"

namespace planning {
namespace apa_planner {

void PerpendicularTailInScenario::Reset() {
  current_plan_path_vec_.clear();
  ParkingScenario::Reset();
}

void PerpendicularTailInScenario::ScenarioTry() {
  if (apa_world_ptr_->GetStateMachineManagerPtr()->IsParkingStatus()) {
    return;
  }
  Reset();
  frame_.replan_flag = true;
  SlotReleaseInfo& release_info = apa_world_ptr_->GetSlotManagerPtr()
                                      ->GetMutableEgoInfoUnderSlot()
                                      .slot.release_info_;
  if (!UpdateEgoSlotInfo()) {
    release_info.release_state[SlotReleaseMethod::GEOMETRY_PLANNING_RELEASE] =
        SlotReleaseState::NOT_RELEASE;
    return;
  }

  if (!GenTlane()) {
    release_info.release_state[SlotReleaseMethod::GEOMETRY_PLANNING_RELEASE] =
        SlotReleaseState::NOT_RELEASE;
    return;
  }

  if (PathPlanOnce() == PathPlannerResult::PLAN_UPDATE) {
    release_info.release_state[SlotReleaseMethod::GEOMETRY_PLANNING_RELEASE] =
        SlotReleaseState::RELEASE;
  } else {
    release_info.release_state[SlotReleaseMethod::GEOMETRY_PLANNING_RELEASE] =
        SlotReleaseState::NOT_RELEASE;
  }
  return;
}

void PerpendicularTailInScenario::ExcutePathPlanningTask() {
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

  // update ego slot info
  if (!UpdateEgoSlotInfo()) {
    SetParkingStatus(PARKING_FAILED);
    frame_.plan_fail_reason = UPDATE_EGO_SLOT_INFO;
    return;
  }

  CalSlotJumpErr();
  // update remain dist
  frame_.remain_dist_path = CalRemainDistFromPath();
  frame_.remain_dist_obs = CalRealTimeBrakeDist();
  frame_.remain_dist_slot_jump = CalRemainDistBySlotJump();

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

  PathPlan();

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

  // check planning status
  ILOG_INFO << "parking status = "
            << static_cast<int>(GetPlannerStates().planning_status);
}

const bool PerpendicularTailInScenario::UpdateEgoSlotInfo() {
  const std::shared_ptr<ApaMeasureDataManager> measures_ptr =
      apa_world_ptr_->GetMeasureDataManagerPtr();

  const ApaParameters& param = apa_param.GetParam();

  // 建立车位坐标系 根据23角点或者限位器角点确定规划终点位姿
  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  if (ego_info_under_slot.slot.slot_type_ != SlotType::SLANT &&
      ego_info_under_slot.slot.slot_type_ != SlotType::PERPENDICULAR) {
    return false;
  }

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

  ego_info_under_slot.cur_pose.pos =
      ego_info_under_slot.g2l_tf.GetPos(measures_ptr->GetPos());
  ego_info_under_slot.cur_pose.heading =
      ego_info_under_slot.g2l_tf.GetHeading(measures_ptr->GetHeading());
  ego_info_under_slot.cur_pose.heading_vec =
      geometry_lib::GenHeadingVec(ego_info_under_slot.cur_pose.heading);

  // transform slot and obs from global to slot
  ego_info_under_slot.slot.TransformCoordFromGlobalToLocal(
      ego_info_under_slot.g2l_tf);

  apa_world_ptr_->GetObstacleManagerPtr()->TransformCoordFromGlobalToLocal(
      ego_info_under_slot.g2l_tf);

  if (frame_.is_replan_first) {
    const Eigen::Vector2d ego_to_slot_center_vec =
        ego_info_under_slot.slot.origin_corner_coord_global_.pt_center -
        measures_ptr->GetPos();

    const double cross_ego_to_slot_center =
        pnc::geometry_lib::GetCrossFromTwoVec2d(measures_ptr->GetHeadingVec(),
                                                ego_to_slot_center_vec);

    const double cross_ego_to_slot_heading =
        pnc::geometry_lib::GetCrossFromTwoVec2d(
            measures_ptr->GetHeadingVec(),
            ego_info_under_slot.origin_pose_global.heading_vec);

    // 这个初始参考挡位对迭代式路径规划已经没有什么意义
    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
    if (cross_ego_to_slot_heading > 0.0 && cross_ego_to_slot_center < 0.0) {
      ego_info_under_slot.slot_side = geometry_lib::SLOT_SIDE_RIGHT;
    } else if (cross_ego_to_slot_heading < 0.0 &&
               cross_ego_to_slot_center > 0.0) {
      ego_info_under_slot.slot_side = geometry_lib::SLOT_SIDE_LEFT;
    } else {
      ego_info_under_slot.slot_side = geometry_lib::SLOT_SIDE_INVALID;
    }
  }

  // no consider obs target pose, real-time update
  TargetPoseDecider target_pose_decider(
      apa_world_ptr_->GetCollisionDetectorInterfacePtr());

  TargetPoseDeciderRequest tar_pose_decider_request(
      std::vector<double>{0.15}, 0.3,
      ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN, false, false);

  TargetPoseDeciderResult res = target_pose_decider.CalcTargetPose(
      ego_info_under_slot.slot, tar_pose_decider_request);

  ego_info_under_slot.origin_target_pose = res.target_pose_local;

  ego_info_under_slot.virtual_limiter.first
      << ego_info_under_slot.origin_target_pose.pos.x(),
      0.5 * ego_info_under_slot.slot.slot_width_;

  ego_info_under_slot.virtual_limiter.second
      << ego_info_under_slot.origin_target_pose.pos.x(),
      -0.5 * ego_info_under_slot.slot.slot_width_;

  if (std::fabs(ego_info_under_slot.cur_pose.pos.y()) <
          param.slot_occupied_ratio_max_lat_err &&
      std::fabs(ego_info_under_slot.cur_pose.heading) <
          param.slot_occupied_ratio_max_heading_err * kDeg2Rad) {
    const std::vector<double> x_tab = {
        ego_info_under_slot.origin_target_pose.pos.x(),
        ego_info_under_slot.slot.slot_length_ + param.rear_overhanging};

    const std::vector<double> x_postprocess_tab = {
        ego_info_under_slot.target_pose.pos.x(),
        ego_info_under_slot.slot.slot_length_ + param.rear_overhanging};

    const std::vector<double> occupied_ratio_tab = {1.0, 0.0};

    ego_info_under_slot.slot_occupied_ratio = mathlib::Interp1(
        x_tab, occupied_ratio_tab, ego_info_under_slot.cur_pose.pos.x());

    ego_info_under_slot.slot_occupied_ratio_postprocess =
        mathlib::Interp1(x_postprocess_tab, occupied_ratio_tab,
                         ego_info_under_slot.cur_pose.pos.x());

  } else {
    ego_info_under_slot.slot_occupied_ratio = 0.0;
    ego_info_under_slot.slot_occupied_ratio_postprocess = 0.0;
  }

  // trim or extend path according to limiter, only run once
  frame_.correct_path_for_limiter = false;
  if (frame_.gear_command == geometry_lib::SEG_GEAR_REVERSE &&
      !ego_info_under_slot.fix_slot) {
    const geometry_lib::LineSegment limiter_line(
        Eigen::Vector2d(ego_info_under_slot.origin_target_pose.pos.x(),
                        0.5 * ego_info_under_slot.slot.slot_width_),
        Eigen::Vector2d(ego_info_under_slot.origin_target_pose.pos.x(),
                        -0.5 * ego_info_under_slot.slot.slot_width_));

    const double dist_ego_limiter = geometry_lib::CalPoint2LineDist(
        ego_info_under_slot.cur_pose.pos, limiter_line);

    ILOG_INFO << "dist_ego_limiter = " << dist_ego_limiter;

    if (dist_ego_limiter < param.car_to_limiter_dis &&
        frame_.can_correct_path_for_limiter &&
        std::fabs(ego_info_under_slot.cur_pose.heading) * kRad2Deg <
            param.finish_heading_err * 1.05 &&
        std::fabs(ego_info_under_slot.cur_pose.pos.y()) <
            param.finish_lat_err_strict * 1.05) {
      ILOG_INFO << "should correct path according limiter";
      ego_info_under_slot.fix_slot = true;
      PostProcessPathAccordingLimiter();
      // 记录根据限位器裁剪路径时刻
      frame_.correct_path_for_limiter = true;
    }
  }

  // fix slot
  if (ego_info_under_slot.slot_occupied_ratio_postprocess >
          param.fix_slot_occupied_ratio &&
      !ego_info_under_slot.fix_slot && measures_ptr->GetStaticFlag()) {
    ego_info_under_slot.fix_slot = true;
  }

  return true;
}

const bool PerpendicularTailInScenario::CheckCanDelObsInSlot() {
  return (frame_.current_gear != geometry_lib::SEG_GEAR_REVERSE) ||
         !CheckEgoPoseInBelieveObsArea(
             0.2, apa_param.GetParam().believe_obs_ego_area);
}

const bool PerpendicularTailInScenario::CalcPtInside() {
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

  const auto& obstacles =
      apa_world_ptr_->GetObstacleManagerPtr()->GetObstacles();

  Eigen::Vector2d obs_pt_slot;
  for (const auto& pair : obstacles) {
    for (const auto& obs : pair.second.GetPtClout2dLocal()) {
      obs_pt_slot = obs;
      SlotObsType obs_slot_type = CalSlotObsType(obs_pt_slot);
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

  apa_param.SetPram().actual_mono_plan_enable = param.mono_plan_enable;
  // 如果保守的话  两侧全空才开启一把进 无意义 这个保守泊入已关闭
  frame_.is_left_empty = left_pq_for_x.empty();
  frame_.is_right_empty = right_pq_for_x.empty();
  if (param.conservative_mono_enable &&
      (!frame_.is_left_empty || !frame_.is_right_empty)) {
    apa_param.SetPram().actual_mono_plan_enable = false;
  }

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

  // 计算内侧安全圆切点
  if (ego_info_under_slot.slot_side == geometry_lib::SLOT_SIDE_RIGHT) {
    ego_info_under_slot.pt_inside = right_obs;
    ego_info_under_slot.pt_inside.x() =
        mathlib::Constrain(
            ego_info_under_slot.pt_inside.x(),
            ego_info_under_slot.slot.origin_corner_coord_local_.pt_0.x() -
                2.168,
            ego_info_under_slot.slot.origin_corner_coord_local_.pt_0.x() +
                2.68) +
        param.tlane_safe_dx;

    ego_info_under_slot.pt_inside.y() =
        mathlib::Constrain(
            ego_info_under_slot.pt_inside.y(),
            -0.5 * ego_info_under_slot.slot.slot_width_ - 0.128,
            -0.5 * ego_info_under_slot.slot.slot_width_ + 0.068) +
        0.0268;
  } else {
    ego_info_under_slot.pt_inside = left_obs;
    ego_info_under_slot.pt_inside.x() =
        mathlib::Constrain(
            ego_info_under_slot.pt_inside.x(),
            ego_info_under_slot.slot.origin_corner_coord_local_.pt_1.x() -
                2.168,
            ego_info_under_slot.slot.origin_corner_coord_local_.pt_1.x() +
                2.68) +
        param.tlane_safe_dx;

    ego_info_under_slot.pt_inside.y() =
        mathlib::Constrain(ego_info_under_slot.pt_inside.y(),
                           0.5 * ego_info_under_slot.slot.slot_width_ - 0.068,
                           0.5 * ego_info_under_slot.slot.slot_width_ + 0.128) -
        0.0268;
  }

  return true;
}

const bool PerpendicularTailInScenario::GenTlane() {
  // if safe target pose exist, return true, otherwise return false
  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  const geometry_lib::PathPoint& ego_pose = ego_info_under_slot.cur_pose;

  bool prohibit_move_slot = false;
  if (apa_param.GetParam().prohibit_move_slot &&
      !CheckEgoPoseInBelieveObsArea(
          0.2, apa_param.GetParam().believe_obs_ego_area, 60)) {
    ILOG_INFO << "prohibit move slot, and if have obs, move it";
    prohibit_move_slot = true;
    frame_.process_obs_method = ProcessObsMethod::MOVE_OBS_OUT_CAR_SAFE_POS;
  }

  bool move_slot_with_little_buffer = false;
  if (apa_param.GetParam().move_slot_with_little_buffer &&
      !CheckEgoPoseInBelieveObsArea(
          0.2, apa_param.GetParam().believe_obs_ego_area, 60)) {
    ILOG_INFO << "move_slot_with_little_buffer";
    move_slot_with_little_buffer = true;
  }

  GenerateObstacleRequest gen_obs_request(
      ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN,
      frame_.process_obs_method);

  apa_world_ptr_->GetParkingTaskInterfacePtr()
      ->GetGenerateObstacleDeciderPtr()
      ->GenObs(ego_info_under_slot, gen_obs_request);

  CalcPtInside();

  bool update_slot_move_dist = false;
  if (frame_.replan_flag &&
      (frame_.replan_reason != ReplanReason::DYNAMIC ||
       (frame_.replan_reason == ReplanReason::DYNAMIC &&
        ego_info_under_slot.slot_occupied_ratio < 0.0968))) {
    update_slot_move_dist = true;
  }

  ILOG_INFO << "move_slot_with_little_buffer = " << move_slot_with_little_buffer
            << "  can not move slot = " << prohibit_move_slot
            << "  update_slot_move_dist = " << update_slot_move_dist
            << "  process_obs_method = "
            << static_cast<int>(frame_.process_obs_method);

  if (update_slot_move_dist) {
    // 重规划时根据障碍物计算终点位置
    TargetPoseDecider target_pose_decider(
        apa_world_ptr_->GetCollisionDetectorInterfacePtr());

    double max_lat_buffer = 0.15;
    if (!frame_.is_replan_first &&
        apa_world_ptr_->GetStateMachineManagerPtr()->IsParkingStatus()) {
      max_lat_buffer =
          std::min(max_lat_buffer, ego_info_under_slot.safe_lat_buffer);
    }
    const double min_lat_buffer = 0.09;
    const double step = 0.01;
    max_lat_buffer = std::max(max_lat_buffer, min_lat_buffer + step + 1e-3);
    ILOG_INFO << "find target pose max lat buffer = " << max_lat_buffer;
    std::vector<double> lat_buffer_vec;
    for (double lat_buffer = max_lat_buffer; lat_buffer > min_lat_buffer - 1e-3;
         lat_buffer -= step) {
      lat_buffer_vec.emplace_back(lat_buffer);
    }
    if (apa_param.GetParam().force_use_little_buffer_move_slot ||
        prohibit_move_slot || move_slot_with_little_buffer) {
      ILOG_INFO << "force use little lat safe buffer";
      lat_buffer_vec = std::vector<double>{min_lat_buffer + step};
    }

    TargetPoseDeciderRequest tar_pose_decider_request(
        lat_buffer_vec, 0.3,
        ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN, true, true);

    TargetPoseDeciderResult res =
        apa_world_ptr_->GetParkingTaskInterfacePtr()
            ->GetTargetPoseDeciderPtr()
            ->CalcTargetPose(ego_info_under_slot.slot,
                             tar_pose_decider_request);

    if (!res.exist_target_pose) {
      ILOG_ERROR << "can not find target pose";
      return false;
    }

    ego_info_under_slot.target_pose = res.target_pose_local;

    if (frame_.replan_reason != ReplanReason::DYNAMIC && !prohibit_move_slot &&
        !move_slot_with_little_buffer) {
      ego_info_under_slot.safe_lat_buffer = res.safe_lat_buffer;
    }

    // 记录每次重规划移动距离
    ego_info_under_slot.lon_move_dist_every_replan = res.safe_lon_move_dist;
    ego_info_under_slot.lat_move_dist_every_replan = res.safe_lat_move_dist;
  } else {
    ego_info_under_slot.target_pose = ego_info_under_slot.origin_target_pose;
    // 根据上次重规划成功的移动距离来移动终点位置
    ego_info_under_slot.target_pose.pos.x() +=
        ego_info_under_slot.lon_move_dist_replan_success;
    ego_info_under_slot.target_pose.pos.y() +=
        ego_info_under_slot.lat_move_dist_replan_success;
  }

  ego_info_under_slot.terminal_err.Set(
      ego_info_under_slot.cur_pose.pos - ego_info_under_slot.target_pose.pos,
      geometry_lib::NormalizeAngle(ego_info_under_slot.cur_pose.heading -
                                   ego_info_under_slot.target_pose.heading));

  ILOG_INFO
      << "cur pose = " << ego_info_under_slot.cur_pose.pos.transpose() << "  "
      << ego_info_under_slot.cur_pose.heading * kRad2Deg
      << "  origin_tar_pose = "
      << ego_info_under_slot.origin_target_pose.pos.transpose() << "  "
      << ego_info_under_slot.origin_target_pose.heading * kRad2Deg
      << "  tar_pose = " << ego_info_under_slot.target_pose.pos.transpose()
      << "  " << ego_info_under_slot.target_pose.heading * kRad2Deg
      << "  terminal_err = " << ego_info_under_slot.terminal_err.pos.transpose()
      << "  " << ego_info_under_slot.terminal_err.heading * kRad2Deg
      << "  slot_occupied_ratio_postprocess = "
      << ego_info_under_slot.slot_occupied_ratio_postprocess
      << "  slot occupied ratio = " << ego_info_under_slot.slot_occupied_ratio
      << "  pt_inside = " << ego_info_under_slot.pt_inside.transpose()
      << "  stuck time(s) = " << frame_.stuck_time
      << "  stuck_obs_time(s) = " << frame_.stuck_obs_time << "  slod side = "
      << geometry_lib::GetSlotSideString(ego_info_under_slot.slot_side);

  return true;
}

const bool PerpendicularTailInScenario::GenObstacles() { return true; }

const uint8_t PerpendicularTailInScenario::PathPlanOnce() {
  const ApaParameters& param = apa_param.GetParam();
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const SimulationParam& simu_param = apa_world_ptr_->GetSimuParam();

  GeometryPathInput input;
  input.ego_info_under_slot = ego_info_under_slot;
  input.is_complete_path = simu_param.is_complete_path;
  input.sample_ds = simu_param.sample_ds;
  input.ref_gear = frame_.current_gear;
  input.ref_arc_steer = frame_.current_arc_steer;
  input.is_replan_first = frame_.is_replan_first;
  input.is_replan_second = frame_.is_replan_second;
  input.is_replan_dynamic = (frame_.replan_reason == ReplanReason::DYNAMIC);
  input.is_searching_stage =
      apa_world_ptr_->GetStateMachineManagerPtr()->IsSeachingStatus();

  input.force_mid_process_plan = simu_param.force_mid_process_plan;

  input.can_first_plan_again = frame_.can_first_plan_again;

  input.is_simulation = simu_param.is_simulation;

  if (input.is_simulation) {
    apa_param.SetPram().use_average_obs_dist =
        apa_world_ptr_->GetSimuParam().use_average_obs_dist;
  }

  input.is_left_empty = frame_.is_left_empty;
  input.is_right_empty = frame_.is_right_empty;

  if (frame_.replan_reason == ReplanReason::DYNAMIC) {
    ILOG_INFO << "dynamic replan, gear should be reverse";
    input.ref_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
  }

  std::vector<geometry_lib::PathPoint> splicing_point_global_vec;
  CalcProjPtForDynamicPlan(input.ego_info_under_slot.cur_pose,
                           splicing_point_global_vec);

  const std::shared_ptr<PerpendicularTailInPathGenerator>&
      per_path_planner_ptr = apa_world_ptr_->GetParkingTaskInterfacePtr()
                                 ->GetPerpendicularTailInPathGeneratorPtr();

  per_path_planner_ptr->SetInput(input);

  const bool path_plan_success = per_path_planner_ptr->Update();

  if (input.is_searching_stage) {
    if (path_plan_success) {
      return PathPlannerResult::PLAN_UPDATE;
    } else {
      return PathPlannerResult::PLAN_FAILED;
    }
  }

  PathPlannerResult plan_result = PathPlannerResult::PLAN_UPDATE;

  // dynamic replan
  if (frame_.replan_reason == ReplanReason::DYNAMIC) {
    if (!path_plan_success) {
      frame_.dynamic_plan_fail_flag = true;
      ILOG_INFO << "path dynamic plan fail";
      frame_.plan_fail_reason = ParkingFailReason::PATH_PLAN_FAILED;
      return plan_result;
    }
    ILOG_INFO << "path dynamic plan success";
    frame_.dynamic_plan_fail_flag = false;
    if (!CheckDynamicPlanPathOptimal()) {
      frame_.dynamic_plan_path_superior = false;
      frame_.plan_fail_reason = ParkingFailReason::DYNAMIC_PATH_NOT_SUPERIOR;
      ILOG_INFO << "path dynamic plan is not superior, should not replace "
                   "last path";
      return plan_result;
    }
    ILOG_INFO << "path dynamic plan is superior, should replace last path";
    frame_.dynamic_plan_path_superior = true;
  }
  // static replan
  else {
    if (path_plan_success) {
      ILOG_INFO << "path plan success";
      if (frame_.process_obs_method == ProcessObsMethod::DO_NOTHING &&
          !frame_.is_replan_first &&
          per_path_planner_ptr->GetOutput().current_gear !=
              frame_.current_gear) {
        ILOG_INFO
            << "when process_obs_method is do nothing and no first replan, "
               "plan gear should be same with ref gear, otherwise fail";
        frame_.plan_fail_reason = ParkingFailReason::PATH_PLAN_FAILED;
        return PathPlannerResult::PLAN_FAILED;
      }
    } else {
      ILOG_INFO << "path plan fail";
      const bool direct_fail_flag =
          frame_.process_obs_method == ProcessObsMethod::DO_NOTHING ||
          frame_.process_obs_method == ProcessObsMethod::MOVE_OBS_OUT_SLOT ||
          !frame_.can_first_plan_again || frame_.is_replan_first;

      if (direct_fail_flag) {
        ILOG_INFO << "no try first path plan, directly fail";
        frame_.plan_fail_reason = ParkingFailReason::PATH_PLAN_FAILED;
        return PathPlannerResult::PLAN_FAILED;
      }

      ILOG_INFO << "try first path plan again";
      frame_.is_replan_first = true;
      frame_.is_replan_second = false;
      frame_.can_first_plan_again = false;
      input.is_replan_first = frame_.is_replan_first;
      input.is_replan_second = frame_.is_replan_second;
      input.can_first_plan_again = frame_.can_first_plan_again;
      per_path_planner_ptr->SetInput(input);
      if (!per_path_planner_ptr->Update()) {
        ILOG_INFO << "try first path plan again also fail";
        frame_.plan_fail_reason = PATH_PLAN_FAILED;
        return PathPlannerResult::PLAN_FAILED;
      }
      // it can limit gear, but now not limit, try success possible
      if (per_path_planner_ptr->GetOutput().current_gear !=
          frame_.current_gear) {
        // return PathPlannerResult::PLAN_FAILED;
      }
      ILOG_INFO << "try first path plan again success";
    }
  }

  if (!per_path_planner_ptr->SetCurrentPathSegIndex()) {
    ILOG_INFO << "path plan fail";
    frame_.plan_fail_reason = ParkingFailReason::SET_SEG_INDEX;
    return PathPlannerResult::PLAN_FAILED;
  }

  if (!per_path_planner_ptr->CheckCurrentGearLength()) {
    ILOG_INFO << "path plan fail";
    frame_.plan_fail_reason = ParkingFailReason::CHECK_GEAR_LENGTH;
    return plan_result;
  }

  per_path_planner_ptr->SampleCurrentPathSeg();

  per_path_planner_ptr->PrintOutputSegmentsInfo();

  const GeometryPathOutput& planner_output = per_path_planner_ptr->GetOutput();
  current_plan_path_vec_.clear();
  current_plan_path_vec_.reserve(5);
  all_plan_path_vec_.clear();
  all_plan_path_vec_.reserve(8);

  frame_.is_last_path = planner_output.is_last_path;

  geometry_lib::PathSegment path_seg_global;
  for (size_t i = planner_output.path_seg_index.first;
       i < planner_output.path_segment_vec.size(); ++i) {
    path_seg_global = planner_output.path_segment_vec[i];
    path_seg_global.LocalToGlobal(ego_info_under_slot.l2g_tf);
    all_plan_path_vec_.emplace_back(path_seg_global);
    if (i <= planner_output.path_seg_index.second) {
      current_plan_path_vec_.emplace_back(path_seg_global);
    }
  }

  frame_.current_gear = geometry_lib::ReverseGear(planner_output.current_gear);

  if (frame_.is_replan_first) {
    frame_.is_replan_first = false;
    frame_.is_replan_second = true;
  } else {
    if (frame_.is_replan_second) {
      frame_.is_replan_second = false;
    }
  }

  frame_.gear_command = planner_output.current_gear;

  current_path_point_global_vec_.clear();
  current_path_point_global_vec_.reserve(planner_output.path_point_vec.size() +
                                         splicing_point_global_vec.size() + 18);

  complete_path_point_global_vec_.clear();
  complete_path_point_global_vec_.reserve(
      planner_output.all_gear_path_point_vec.size() +
      splicing_point_global_vec.size() + 18);

  std::vector<pnc::geometry_lib::PathPoint> path_pt_vec;
  if (LateralPathOptimize(path_pt_vec)) {
    JSON_DEBUG_VALUE("is_path_lateral_optimized", true);
  } else {
    path_pt_vec = planner_output.path_point_vec;
    JSON_DEBUG_VALUE("is_path_lateral_optimized", false);
  }

  for (const geometry_lib::PathPoint& pt : splicing_point_global_vec) {
    current_path_point_global_vec_.emplace_back(pt);
    complete_path_point_global_vec_.emplace_back(pt);
  }

  geometry_lib::PathPoint global_point;
  for (const geometry_lib::PathPoint& path_point : path_pt_vec) {
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

  perferred_geometry_path_vec_ = planner_output.perferred_geometry_path_vec;

  // record all_gear_path_segment_vec
  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> heading_vec;
  std::vector<double> lat_buffer_vec;
  const size_t interval_number = 2;
  x_vec.reserve(planner_output.all_gear_path_point_vec.size() /
                    interval_number +
                interval_number);
  y_vec.reserve(planner_output.all_gear_path_point_vec.size() /
                    interval_number +
                interval_number);
  heading_vec.reserve(planner_output.all_gear_path_point_vec.size() /
                          interval_number +
                      interval_number);
  lat_buffer_vec.reserve(planner_output.all_gear_path_point_vec.size() /
                             interval_number +
                         interval_number);
  for (size_t i = 0; i < complete_path_point_global_vec_.size(); ++i) {
    if (i % interval_number == 0) {
      x_vec.emplace_back(complete_path_point_global_vec_[i].pos.x());
      y_vec.emplace_back(complete_path_point_global_vec_[i].pos.y());
      heading_vec.emplace_back(complete_path_point_global_vec_[i].heading);
      lat_buffer_vec.emplace_back(
          complete_path_point_global_vec_[i].lat_buffer);
    }
  }

  JSON_DEBUG_VECTOR("plan_traj_x", x_vec, 3)
  JSON_DEBUG_VECTOR("plan_traj_y", y_vec, 3)
  JSON_DEBUG_VECTOR("plan_traj_heading", heading_vec, 3)
  JSON_DEBUG_VECTOR("plan_traj_lat_buffer", lat_buffer_vec, 3)

  ILOG_INFO << "current_path_point_global_vec_.size() = "
            << current_path_point_global_vec_.size()
            << "  complete_path_point_global_vec_.size() = "
            << complete_path_point_global_vec_.size();

  return plan_result;
}

void PerpendicularTailInScenario::PathPlan() {
  CheckReplanParams replan_params;
  frame_.replan_flag = CheckReplan(replan_params);
  frame_.pathplan_result = PathPlannerResult::PLAN_UPDATE;
  frame_.plan_fail_reason = ParkingFailReason::NOT_FAILED;
  const ApaParameters& param = apa_param.GetParam();
  if (!CheckCanDelObsInSlot()) {
    frame_.process_obs_method = ProcessObsMethod::DO_NOTHING;
  }

  if (apa_world_ptr_->GetSimuParam().is_simulation &&
      apa_world_ptr_->GetSimuParam().process_obs_method != -1) {
    frame_.process_obs_method = static_cast<ProcessObsMethod>(
        apa_world_ptr_->GetSimuParam().process_obs_method);
  }

  const bool exist_target_pose = GenTlane();
  if (!frame_.replan_flag) {
    ILOG_INFO << "replan is not required!";
    SetParkingStatus(ParkingStatus::PARKING_RUNNING);
    return;
  }
  SetParkingStatus(ParkingStatus::PARKING_PLANNING);

  if (!exist_target_pose) {
    frame_.pathplan_result = PathPlannerResult::PLAN_FAILED;
    frame_.plan_fail_reason = ParkingFailReason::NO_TARGET_POSE;
    if (frame_.replan_fail_time > apa_param.GetParam().max_replan_failed_time) {
      SetParkingStatus(ParkingStatus::PARKING_FAILED);
    }
    SwitchProcessObsMethod();
    return;
  }

  // if exist target pose, try to plan path
  ILOG_INFO << "target pose exists and replan is required!";
  const double start_time = IflyTime::Now_ms();

  if (frame_.replan_reason != ReplanReason::FORCE_PLAN &&
      frame_.replan_reason != ReplanReason::DYNAMIC) {
    frame_.total_plan_count++;
  }

  if (frame_.total_plan_count > param.max_replan_count) {
    ILOG_INFO << "replan count is exceed max count, fail, directly quit apa";
    frame_.pathplan_result = PathPlannerResult::PLAN_FAILED;
    frame_.plan_fail_reason = ParkingFailReason::PLAN_COUNT_EXCEED_LIMIT;
    SetParkingStatus(ParkingStatus::PARKING_FAILED);
    return;
  }

  // when dynamic plan, the pathplan_result is always PLAN_UPDATE
  frame_.pathplan_result = PathPlanOnce();

  ILOG_INFO << "try generate path, and replan_consume_time = "
            << IflyTime::Now_ms() - start_time << " ms, replan count = "
            << static_cast<int>(frame_.total_plan_count)
            << "  dynamic_plan_fail_flag = " << frame_.dynamic_plan_fail_flag
            << "  dynamic_plan_path_superior = "
            << frame_.dynamic_plan_path_superior;

  JSON_DEBUG_VALUE("replan_count", frame_.total_plan_count)
  JSON_DEBUG_VALUE("replan_consume_time", IflyTime::Now_ms() - start_time)
  JSON_DEBUG_VALUE("dynamic_plan_fail_flag", frame_.dynamic_plan_fail_flag)
  JSON_DEBUG_VALUE("dynamic_plan_path_superior",
                   frame_.dynamic_plan_path_superior)
  JSON_DEBUG_VALUE("process_obs_method",
                   static_cast<int>(frame_.process_obs_method))

  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  // dynamic plan fail
  if (frame_.replan_reason == ReplanReason::DYNAMIC &&
      (frame_.dynamic_plan_fail_flag || !frame_.dynamic_plan_path_superior)) {
    ILOG_INFO << "dynamic replan failed or path is not superior, use last path";
    frame_.dynamic_replan_fail_count++;

    // restore target pose and terminal err
    ego_info_under_slot.target_pose = ego_info_under_slot.origin_target_pose;
    ego_info_under_slot.target_pose.pos.y() +=
        ego_info_under_slot.lat_move_dist_replan_success;
    ego_info_under_slot.target_pose.pos.x() +=
        ego_info_under_slot.lon_move_dist_replan_success;
    ego_info_under_slot.terminal_err.Set(
        ego_info_under_slot.cur_pose.pos - ego_info_under_slot.target_pose.pos,
        geometry_lib::NormalizeAngle(ego_info_under_slot.cur_pose.heading -
                                     ego_info_under_slot.target_pose.heading));

    // const bool ego_should_stop_by_slot_jump =
    // CheckShouldStopWhenSlotJumpsMuch();
    // JSON_DEBUG_VALUE("ego_should_stop_by_slot_jump",
    // ego_should_stop_by_slot_jump) ILOG_INFO << " ego_should_stop_by_slot_jump
    // = " << ego_should_stop_by_slot_jump
    //           << "  dynamic_replan_fail_count = "
    //           << static_cast<int>(frame_.dynamic_replan_fail_count);

    return;
  }

  // static plan fail
  if (frame_.pathplan_result == PathPlannerResult::PLAN_FAILED) {
    ILOG_INFO << "static replan fail";
    if (frame_.total_plan_count > 0) {
      frame_.total_plan_count--;
    }
    if (frame_.replan_fail_time > param.max_replan_failed_time) {
      SetParkingStatus(PARKING_FAILED);
    }
    SwitchProcessObsMethod();
    return;
  }

  // static or dynamic plan success
  if (frame_.pathplan_result == PathPlannerResult::PLAN_UPDATE) {
    // update slot move dist replan success
    ego_info_under_slot.lat_move_dist_replan_success =
        ego_info_under_slot.lat_move_dist_every_replan;

    ego_info_under_slot.lon_move_dist_replan_success =
        ego_info_under_slot.lon_move_dist_every_replan;

    frame_.process_obs_method = ProcessObsMethod::DO_NOTHING;

    if (frame_.replan_reason == ReplanReason::DYNAMIC) {
      ILOG_INFO << "dynamic replan success and path is superior, update path";
      frame_.dynamic_replan_fail_count = 0;
    } else {
      ILOG_INFO << "static replan success, update path";
      frame_.can_correct_path_for_limiter = true;
    }

    if (PostProcessPath()) {
      SetParkingStatus(PARKING_PLANNING);
      ILOG_INFO << "postprocess path success!";
    } else {
      frame_.pathplan_result = PathPlannerResult::PLAN_FAILED;
      if (frame_.replan_fail_time > param.max_replan_failed_time) {
        SetParkingStatus(PARKING_FAILED);
      }
      ILOG_INFO << "postprocess path failed!";
    }
  }

  ILOG_INFO << "lat_move_dist_replan_success = "
            << ego_info_under_slot.lat_move_dist_replan_success
            << "  lon_move_dist_replan_success = "
            << ego_info_under_slot.lon_move_dist_replan_success
            << "  lat_move_dist_every_replan = "
            << ego_info_under_slot.lat_move_dist_every_replan
            << "  lon_move_dist_every_replan = "
            << ego_info_under_slot.lon_move_dist_every_replan;

  JSON_DEBUG_VALUE("move_slot_dist",
                   ego_info_under_slot.lat_move_dist_replan_success)

  JSON_DEBUG_VALUE("replan_move_slot_dist",
                   ego_info_under_slot.lat_move_dist_every_replan)

  return;
}

void PerpendicularTailInScenario::CalcProjPtForDynamicPlan(
    geometry_lib::PathPoint& proj_pt,
    std::vector<geometry_lib::PathPoint>& splicing_pt_vec) {
  splicing_pt_vec.clear();

  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const ApaParameters& param = apa_param.GetParam();

  proj_pt = ego_info_under_slot.cur_pose;

  if (frame_.replan_reason != ReplanReason::DYNAMIC) {
    return;
  }

  // 如果动态规划 从路径上往后取dt的点当做投影点
  ILOG_INFO << "decide ego pose or proj pt for dynamic path plan";

  if (apa_world_ptr_->GetPredictPathManagerPtr()->GetControlErrBig()) {
    ILOG_INFO << "control err is big, directly use ego pose to dynamic plan";
    return;
  }

  const double remain_dist =
      std::min(frame_.remain_dist_path, frame_.remain_dist_obs);

  // 根据剩余距离选择一个dt,
  // 剩余距离越长，可以选择更大的dt来给控制更大的反映时间
  std::vector<double> dist_vec{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  const double delta_t =
      (param.max_dynamic_plan_proj_dt - param.min_dynamic_plan_proj_dt) /
      (dist_vec.size() - 1);
  std::vector<double> dt_vec{};
  dt_vec.resize(dist_vec.size());
  for (size_t i = 0; i < dist_vec.size(); ++i) {
    dt_vec[i] = param.min_dynamic_plan_proj_dt + delta_t * i;
  }
  const double dt = mathlib::Interp1(dist_vec, dt_vec, remain_dist);
  const double s =
      std::fabs(apa_world_ptr_->GetMeasureDataManagerPtr()->GetVel()) * dt;

  ILOG_INFO << "s = " << s << "  dt = " << dt << "  delta_t = " << delta_t
            << "  remain_dist = " << remain_dist;

  JSON_DEBUG_VALUE("dynamic_plan_predict_dt", dt)
  JSON_DEBUG_VALUE("dynamic_plan_predict_ds", s)

  if (s > remain_dist - 0.68) {
    return;
  }

  const size_t pt_number = current_path_point_global_vec_.size();

  if (pt_number < 2) {
    return;
  }

  const double ds = current_path_point_global_vec_[pt_number - 1].s -
                    current_path_point_global_vec_[pt_number - 2].s;

  if (s < ds * 2) {
    return;
  }

  const double ego_s_proj =
      frame_.current_path_length - frame_.remain_dist_path;
  const double fur_s_proj = ego_s_proj + s;
  geometry_lib::PathPoint ego_proj_pt, fur_proj_pt;
  double ego_dist = std::numeric_limits<double>::infinity();
  double fur_dist = std::numeric_limits<double>::infinity();
  size_t ego_proj_index = 0, fur_proj_index = 0;
  for (size_t i = 0; i < current_path_point_global_vec_.size(); ++i) {
    const geometry_lib::PathPoint& pt = current_path_point_global_vec_[i];
    if (std::fabs(ego_s_proj - pt.s) < ego_dist) {
      ego_dist = std::fabs(ego_s_proj - pt.s);
      ego_proj_pt = pt;
      ego_proj_index = i;
    }
    if (std::fabs(fur_s_proj - pt.s) < fur_dist) {
      fur_dist = std::fabs(fur_s_proj - pt.s);
      fur_proj_pt = pt;
      fur_proj_index = i;
    }
  }

  proj_pt.pos = ego_info_under_slot.g2l_tf.GetPos(fur_proj_pt.pos);
  proj_pt.heading = ego_info_under_slot.g2l_tf.GetHeading(fur_proj_pt.heading);

  // 存储当前位置点到投影点之前的所有点
  splicing_pt_vec.reserve(fur_proj_index - ego_proj_index + 1);
  for (size_t i = ego_proj_index; i < fur_proj_index; ++i) {
    splicing_pt_vec.emplace_back(current_path_point_global_vec_[i]);
  }

  fur_proj_pt.PrintInfo();

  return;
}

void PerpendicularTailInScenario::SwitchProcessObsMethod() {
  switch (frame_.process_obs_method) {
    case ProcessObsMethod::DO_NOTHING:
      frame_.process_obs_method = ProcessObsMethod::MOVE_OBS_OUT_SLOT;
      break;
    case ProcessObsMethod::MOVE_OBS_OUT_SLOT:
      frame_.process_obs_method = ProcessObsMethod::MOVE_OBS_OUT_CAR_SAFE_POS;
      break;
    case ProcessObsMethod::MOVE_OBS_OUT_CAR_SAFE_POS:
    default:
      frame_.process_obs_method = ProcessObsMethod::DO_NOTHING;
      break;
  }
}

const bool PerpendicularTailInScenario::CheckFinished() {
  const ApaParameters& param = apa_param.GetParam();

  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const geometry_lib::PathPoint& cur_pose = ego_info_under_slot.cur_pose;
  const geometry_lib::PathPoint front_pose =
      GetCarFrontPoseFromCarPose(cur_pose);
  const geometry_lib::PathPoint& target_pose = ego_info_under_slot.target_pose;

  const double lon_err = cur_pose.pos.x() - target_pose.pos.x();
  const double lat_err1 = cur_pose.pos.y() - target_pose.pos.y();
  const double lat_err2 = front_pose.pos.y() - target_pose.pos.y();
  const double heading_err =
      (cur_pose.heading - target_pose.heading) * kRad2Deg;

  JSON_DEBUG_VALUE("terminal_error_x", lon_err)
  JSON_DEBUG_VALUE("terminal_error_y", lat_err1)
  JSON_DEBUG_VALUE("terminal_error_y_front", lat_err2)
  JSON_DEBUG_VALUE("terminal_error_heading", heading_err * kDeg2Rad)
  ILOG_INFO << "terminal_error_x = " << lon_err
            << "  terminal_error_y = " << lat_err1
            << "  lat_err2 = " << lat_err2 << "  heading_err = " << heading_err;

  const bool lon_condition = lon_err < param.finish_lon_err;

  const bool lat_condition_1 = std::fabs(lat_err1) <= param.finish_lat_err;

  const bool lat_condition_2 =
      std::fabs(lat_err1) <= param.finish_lat_err_strict &&
      std::fabs(lat_err2) <= param.finish_lat_err_strict;

  const bool heading_condition_1 =
      std::fabs(heading_err) <= param.finish_heading_err;

  const bool heading_condition_2 =
      std::fabs(heading_err) <= (param.finish_heading_err + 1.988);

  const bool lat_condition = (lat_condition_1 && heading_condition_1) &&
                             (lat_condition_2 && heading_condition_2);

  const bool static_condition =
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag();

  const bool remain_s_condition =
      frame_.remain_dist_path < param.max_replan_remain_dist;

  bool parking_finish =
      lon_condition && lat_condition && static_condition && remain_s_condition;

  if (parking_finish) {
    return true;
  }

  // stucked by directly behind uss
  const bool enter_slot_condition = ego_info_under_slot.slot_occupied_ratio >
                                    param.finish_uss_slot_occupied_ratio;
  const bool remain_uss_condition =
      frame_.remain_dist_obs < param.max_replan_remain_dist;

  GJKColDetRequest gjl_col_det_request(true, false);

  bool end_pos_has_obs_condition =
      apa_world_ptr_->GetCollisionDetectorInterfacePtr()
          ->GetGJKCollisionDetectorPtr()
          ->Update(
              std::vector<geometry_lib::PathPoint>{
                  ego_info_under_slot.target_pose},
              0.0, 0.0, gjl_col_det_request)
          .col_flag;

  ILOG_INFO << "end_pos_has_obs_condition = " << end_pos_has_obs_condition;

  if (!end_pos_has_obs_condition) {
    double move_back_dist = 0.168;
    if (lon_err < 0.368) {
      // if only can move 0.1m, no move, finish
      move_back_dist = param.safe_uss_remain_dist_in_slot + 0.1;
    }

    const geometry_lib::PathPoint uss_pose{
        ego_info_under_slot.target_pose.pos -
            move_back_dist * Eigen::Vector2d(1.0, 0.0),
        ego_info_under_slot.target_pose.heading};

    end_pos_has_obs_condition =
        apa_world_ptr_->GetCollisionDetectorInterfacePtr()
            ->GetGJKCollisionDetectorPtr()
            ->Update(std::vector<geometry_lib::PathPoint>{uss_pose}, 0.0, 0.0,
                     gjl_col_det_request)
            .col_flag;

    ILOG_INFO << "after_pos_has_obs_condition = " << end_pos_has_obs_condition;
  }

  parking_finish = lat_condition && static_condition && enter_slot_condition &&
                   remain_uss_condition && end_pos_has_obs_condition;

  // Consider whether there are really obstacles at target pos. If so, finish
  // it is indeed impossible to reach the target pos, if not, try replan again
  if (parking_finish) {
    return true;
  }

  // stucked by dynamic col det
  const bool remain_dist_col_det_condition =
      frame_.remain_dist_col_det < param.max_replan_remain_dist;

  parking_finish = lat_condition && static_condition && enter_slot_condition &&
                   remain_dist_col_det_condition && (lon_err < 0.568);

  return parking_finish;
}

const bool PerpendicularTailInScenario::PostProcessPathAccordingLimiter() {
  size_t origin_traj_size = current_path_point_global_vec_.size();

  if (origin_traj_size < 2) {
    frame_.spline_success = false;
    ILOG_INFO << "error: origin_traj_size = " << origin_traj_size;
    return false;
  }

  // cal proj point, need extend, add a point
  // if need extend according limiter, need to add another point
  const size_t traj_size = origin_traj_size + 2;

  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> s_vec;
  std::vector<double> heading_vec;
  x_vec.clear();
  y_vec.clear();
  s_vec.clear();
  heading_vec.clear();

  x_vec.reserve(traj_size);
  y_vec.reserve(traj_size);
  s_vec.reserve(traj_size);
  heading_vec.reserve(traj_size);

  const Eigen::Vector2d limiter_mid =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot().l2g_tf.GetPos(
          apa_world_ptr_->GetSlotManagerPtr()
              ->GetEgoInfoUnderSlot()
              .origin_target_pose.pos);

  // If the target pose is very close to the previously planned
  // endpoint, there is no need to run the following steps
  if ((limiter_mid - current_path_point_global_vec_.back().pos).norm() <
      0.026) {
    return true;
  }

  double s_proj = 0.0;
  bool success =
      frame_.spline_success &&
      geometry_lib::CalProjFromSplineByBisection(
          0.0, frame_.current_path_length + frame_.path_extended_dist, s_proj,
          limiter_mid, frame_.x_s_spline, frame_.y_s_spline);
  if (!success) {
    ILOG_INFO << "path is err";
    return false;
  }
  if (s_proj < apa_world_ptr_->GetSimuParam().sample_ds * 1.5) {
    ILOG_INFO << "limiter s_proj is too small";
    return false;
  }
  double ds = 0.0;
  double s = 0.0;
  for (size_t i = 0; i < current_path_point_global_vec_.size(); ++i) {
    if (i > 0) {
      ds = std::hypot(current_path_point_global_vec_[i].pos.x() -
                          current_path_point_global_vec_[i - 1].pos.x(),
                      current_path_point_global_vec_[i].pos.y() -
                          current_path_point_global_vec_[i - 1].pos.y());
      s += std::max(ds, 1e-3);
    }
    if (s > s_proj) {
      ILOG_INFO << "path shoule be shorten because of limiter";
      if (s_proj - s_vec.back() > 0.036 && frame_.spline_success) {
        x_vec.emplace_back(frame_.x_s_spline(s_proj));
        y_vec.emplace_back(frame_.y_s_spline(s_proj));
        heading_vec.emplace_back(heading_vec.back());
        s_vec.emplace_back(s_proj);
      }
      break;
    }
    x_vec.emplace_back(current_path_point_global_vec_[i].pos.x());
    y_vec.emplace_back(current_path_point_global_vec_[i].pos.y());
    heading_vec.emplace_back(current_path_point_global_vec_[i].heading);
    s_vec.emplace_back(s);
  }
  if (s < s_proj) {
    ILOG_INFO << "path shoule be extended because of limiter";
    using namespace pnc::geometry_lib;
    double init_length = 0.0;
    double extend_length = s_proj - s;
    if (current_plan_path_vec_.size() > 0 &&
        std::fabs(current_plan_path_vec_.back().GetEndHeading()) * kRad2Deg <
            1.08) {
      PathSegment& path_seg_global = current_plan_path_vec_.back();
      init_length = path_seg_global.Getlength();
      if (path_seg_global.seg_type == SEG_TYPE_LINE) {
        CompleteLineInfo(path_seg_global.line_seg, init_length + extend_length);
      } else if (path_seg_global.seg_type == SEG_TYPE_ARC) {
        CompleteArcInfo(path_seg_global.arc_seg, init_length + extend_length,
                        path_seg_global.arc_seg.is_anti_clockwise);
      }
      ILOG_INFO << "init_length = " << init_length
                << "  extend_length = " << extend_length
                << "  cur_path_length = " << path_seg_global.Getlength()
                << "  s_proj = " << s_proj << "  s = " << s;

      // this path is global, need to transform to local to col det
      path_seg_global.GlobalToLocal(
          apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot().g2l_tf);

      ColResult col_res =
          apa_world_ptr_->GetCollisionDetectorInterfacePtr()
              ->GetGeometryCollisionDetectorPtr()
              ->Update(path_seg_global,
                       apa_param.GetParam().car_lat_inflation_normal,
                       apa_param.GetParam().col_obs_safe_dist_normal);

      const double remain_dist =
          std::max(init_length,
                   std::min(col_res.remain_obs_dist -
                                apa_param.GetParam().col_obs_safe_dist_normal,
                            col_res.remain_car_dist));

      extend_length = remain_dist - init_length;

      pnc::geometry_lib::CompletePathSeg(path_seg_global, remain_dist);

      ILOG_INFO << "length = " << path_seg_global.Getlength()
                << "  remain_dist = " << remain_dist
                << "  extend_length = " << extend_length;
    }
    const double total_length = s + extend_length;
    while (s <= total_length) {
      if (x_vec.size() > PLANNING_TRAJ_POINTS_MAX_NUM - 1) {
        break;
      }
      s += apa_world_ptr_->GetSimuParam().sample_ds;
      x_vec.emplace_back(frame_.x_s_spline(s));
      y_vec.emplace_back(frame_.y_s_spline(s));
      heading_vec.emplace_back(heading_vec.back());
      s_vec.emplace_back(s);
    }
    // x_vec.emplace_back(frame_.x_s_spline(s_proj));
    // y_vec.emplace_back(frame_.y_s_spline(s_proj));
    // heading_vec.emplace_back(heading_vec.back());
    // s_vec.emplace_back(s_proj);
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
  success = pnc::geometry_lib::CalExtendedPointByTwoPoints(
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

const double PerpendicularTailInScenario::CalRealTimeBrakeDist() {
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const geometry_lib::PathPoint& cur_pose = ego_info_under_slot.cur_pose;

  const ApaParameters& param = apa_param.GetParam();
  double lon_buffer = (ego_info_under_slot.slot_occupied_ratio < 0.05)
                          ? param.safe_uss_remain_dist_out_slot
                          : param.safe_uss_remain_dist_in_slot;

  geometry_lib::GeometryPath geometry_path_brake(all_plan_path_vec_);
  if (geometry_path_brake.gear_change_count > 0 &&
      geometry_path_brake.gear_index_vec.size() > 0) {
    const size_t start_index = geometry_path_brake.gear_index_vec[0];
    size_t end_index = geometry_path_brake.path_count - 1;
    if (geometry_path_brake.gear_index_vec.size() > 1) {
      end_index = geometry_path_brake.gear_index_vec[1] - 1;
    }
    std::vector<geometry_lib::PathSegment> seg_vec;
    seg_vec.reserve(end_index - start_index + 1);
    for (size_t i = start_index; i <= end_index; ++i) {
      seg_vec.emplace_back(geometry_path_brake.path_segment_vec[i]);
    }
    geometry_path_brake.SetPath(seg_vec);
    if (geometry_path_brake.collide_flag &&
        geometry_path_brake.total_length < 0.76) {
      lon_buffer = param.limited_safe_uss_remain_dist;
      ILOG_INFO << "next path is col and length is short, so this path need be "
                   "more radical";
    }
  }

  // 如果在库外或者在库内，但是车位跳动较大时 横向buffer要更大
  const bool increase_lat_err_flag =
      (frame_.gear_command == geometry_lib::SEG_GEAR_DRIVE &&
       ego_info_under_slot.cur_pose.pos.x() >
           ego_info_under_slot.slot.GetOriginCornerCoordLocal().pt_01_mid.x() +
               2.168) ||
      (frame_.gear_command == geometry_lib::SEG_GEAR_REVERSE &&
       ego_info_under_slot.slot_occupied_ratio > 0.168 &&
       frame_.slot_jump_lat_err > param.finish_lat_err_strict - 0.02) ||
      (frame_.gear_command == geometry_lib::SEG_GEAR_REVERSE &&
       apa_world_ptr_->GetPredictPathManagerPtr()->GetControlErrBig());

  // adopting a graded lat buffer real-time braking
  std::vector<RealTimeBrakeInfo> real_time_brake_info_vec;
  real_time_brake_info_vec.resize(4);
  RealTimeBrakeInfo real_time_brake_info(
      RealTimeBrakeType::STOP, param.stop_lat_inflation, param.stop_lon_dist);
  real_time_brake_info_vec[0] = real_time_brake_info;

  real_time_brake_info.Set(RealTimeBrakeType::HEAVY_BRAKE,
                           param.heavy_brake_lat_inflation,
                           param.heavy_brake_lon_dist);
  real_time_brake_info_vec[1] = real_time_brake_info;

  real_time_brake_info.Set(RealTimeBrakeType::MODERATE_BRAKE,
                           param.moderate_brake_lat_inflation,
                           param.moderate_brake_lon_dist);
  real_time_brake_info_vec[2] = real_time_brake_info;

  real_time_brake_info.Set(RealTimeBrakeType::SLIGHT_BRAKE,
                           param.slight_brake_lat_inflation,
                           param.slight_brake_lon_dist);
  real_time_brake_info_vec[3] = real_time_brake_info;

  for (size_t i = 0;
       i < real_time_brake_info_vec.size() && increase_lat_err_flag; ++i) {
    real_time_brake_info_vec[i].lat_buffer =
        std::max(real_time_brake_info_vec[i].lat_buffer,
                 param.moderate_brake_lat_inflation);
  }

  if (frame_.gear_command == geometry_lib::SEG_GEAR_REVERSE) {
    if (!frame_.is_last_path || std::fabs(cur_pose.pos.y()) > 0.08 ||
        std::fabs(cur_pose.heading) * kRad2Deg > 2.068) {
      lon_buffer += 0.05;  // extra lon buffer when ref gear in special case
    }
  }

  double safe_remain_dist = std::numeric_limits<double>::infinity();
  for (const auto& real_time_brake_info : real_time_brake_info_vec) {
    double remain_dist =
        CalRemainDistFromObs(lon_buffer, real_time_brake_info.lat_buffer);
    remain_dist = std::max(remain_dist, real_time_brake_info.min_lon_dist);
    safe_remain_dist = std::min(safe_remain_dist, remain_dist);
  }

  JSON_DEBUG_VALUE("car_real_time_col_lat_buffer",
                   real_time_brake_info_vec[0].lat_buffer)

  ILOG_INFO << "real time brake safe_remain_dist = " << safe_remain_dist
            << "  lon_buffer = " << lon_buffer
            << "  lat_buffer = " << real_time_brake_info_vec[0].lat_buffer;

  return safe_remain_dist;
}

const bool PerpendicularTailInScenario::CheckShouldStopWhenSlotJumpsMuch() {
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const double ego_stop_dist = 1.0;

  if (frame_.ego_stop_when_slot_jumps_much ||
      frame_.replan_reason != ReplanReason::DYNAMIC || !frame_.is_last_path ||
      ego_info_under_slot.slot_occupied_ratio < 1e-3 ||
      ego_info_under_slot.slot_occupied_ratio > 0.708 ||
      ego_info_under_slot.fix_slot ||
      frame_.remain_dist_path < ego_stop_dist + 0.168 ||
      (frame_.remain_dist_obs < ego_stop_dist + 0.168 && false)) {
    ILOG_INFO << "should not stop when slot jumps much 1";
    return false;
  }

  if (!frame_.dynamic_plan_fail_flag && frame_.dynamic_plan_path_superior) {
    ILOG_INFO << "should not stop when slot jumps much 2";
    frame_.dynamic_replan_fail_count = 0;
    return false;
  }

  if (frame_.dynamic_replan_fail_count < 3) {
    ILOG_INFO << "should not stop when slot jumps much 3";
    return false;
  }

  // 计算上一次规划终点
  geometry_lib::GeometryPath geometry_path_bef(all_plan_path_vec_);
  geometry_path_bef.GlobalToLocal(
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot().g2l_tf);
  const geometry_lib::PathPoint tar_pose_bef = geometry_path_bef.end_pose;
  const geometry_lib::PathPoint front_tar_pose_bef =
      GetCarFrontPoseFromCarPose(tar_pose_bef);

  // 计算当前的真实车位终点位置
  geometry_lib::PathPoint tar_pose_now = ego_info_under_slot.origin_target_pose;
  tar_pose_now.pos.x() += ego_info_under_slot.lon_move_dist_every_replan;
  tar_pose_now.pos.y() += ego_info_under_slot.lat_move_dist_every_replan;
  const geometry_lib::PathPoint front_tar_pose_now =
      GetCarFrontPoseFromCarPose(tar_pose_now);

  const auto& param = apa_param.GetParam();

  std::vector<double> fail_count_tab{3.0, 4.0, 5.0, 6.0, 7.0};
  const double dlat =
      (param.should_stop_lat_err - (param.finish_lat_err_strict - 1e-3)) /
      (fail_count_tab.size() - 1);
  const double dheading =
      (param.should_stop_heading_err - (param.finish_heading_err - 1e-3)) /
      (fail_count_tab.size() - 1);
  std::vector<double> lat_err_tab{};
  std::vector<double> heading_err_tab{};
  lat_err_tab.reserve(fail_count_tab.size());
  heading_err_tab.reserve(fail_count_tab.size());
  for (size_t i = 0; i < fail_count_tab.size(); ++i) {
    lat_err_tab.emplace_back(param.should_stop_lat_err - i * dlat);
    heading_err_tab.emplace_back(param.should_stop_heading_err - i * dheading);
  }

  const double lat_err_threshold =
      mathlib::Interp1(fail_count_tab, lat_err_tab,
                       static_cast<double>(frame_.dynamic_replan_fail_count));

  const double heading_err_threshold =
      mathlib::Interp1(fail_count_tab, heading_err_tab,
                       static_cast<double>(frame_.dynamic_replan_fail_count));

  const double lat_err = std::max(
      std::fabs(tar_pose_now.pos.y() - tar_pose_bef.pos.y()),
      std::fabs(front_tar_pose_now.pos.y() - front_tar_pose_bef.pos.y()));

  const double heading_err =
      std::fabs(tar_pose_now.heading - tar_pose_bef.heading) * kRad2Deg;

  ILOG_INFO << "lat_err = " << lat_err << "  heading_err = " << heading_err
            << "  lat_err_threshold = " << lat_err_threshold
            << "  heading_err_threshold = " << heading_err_threshold;

  if (std::fabs(lat_err) < lat_err_threshold &&
      std::fabs(heading_err) < heading_err_threshold) {
    ILOG_INFO << "should not stop when slot jumps much 4";
    return false;
  }

  frame_.ego_stop_when_slot_jumps_much = true;
  frame_.can_correct_path_for_limiter = false;

  ILOG_INFO << "the slot jumps too much, ego should stop, trim path, and then "
               "plan reverse path";

  PostProcessPathAccordingRemainDist(frame_.current_path_length -
                                     frame_.remain_dist_path + ego_stop_dist);

  return true;
}

void PerpendicularTailInScenario::CalSlotJumpErr() {
  if (current_path_point_global_vec_.size() < 1) {
    return;
  }

  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  // 计算上次规划终点位置
  geometry_lib::PathPoint tar_pose_bef = current_path_point_global_vec_.back();
  geometry_lib::PathPoint front_tar_pose_bef =
      GetCarFrontPoseFromCarPose(tar_pose_bef);
  tar_pose_bef.GlobalToLocal(ego_info_under_slot.g2l_tf);
  front_tar_pose_bef.GlobalToLocal(ego_info_under_slot.g2l_tf);

  // 计算当前真实终点位置
  geometry_lib::PathPoint tar_pose_now = ego_info_under_slot.origin_target_pose;
  tar_pose_now.pos.x() += ego_info_under_slot.lon_move_dist_every_replan;
  tar_pose_now.pos.y() += ego_info_under_slot.lat_move_dist_every_replan;
  const geometry_lib::PathPoint front_tar_pose_now =
      GetCarFrontPoseFromCarPose(tar_pose_now);

  // 计算两次终点位置误差
  const double lat_err = std::max(
      std::fabs(tar_pose_now.pos.y() - tar_pose_bef.pos.y()),
      std::fabs(front_tar_pose_now.pos.y() - front_tar_pose_bef.pos.y()));

  const double heading_err =
      std::fabs(tar_pose_now.heading - tar_pose_bef.heading) * kRad2Deg;

  const double lon_err = std::fabs(tar_pose_now.pos.x() - tar_pose_bef.pos.x());

  const auto& param = apa_param.GetParam();

  ILOG_INFO << "lat_err = " << lat_err << "  lon_err = " << lon_err
            << "  heading_err = " << heading_err;

  frame_.slot_jump_lat_err = lat_err;
  frame_.slot_jump_lon_err = lon_err;
  frame_.slot_jump_heading_err = heading_err;
}

const double PerpendicularTailInScenario::CalRemainDistBySlotJump() {
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  if (!frame_.is_last_path || current_path_point_global_vec_.size() < 1 ||
      frame_.gear_command == geometry_lib::SEG_GEAR_DRIVE ||
      ego_info_under_slot.slot_occupied_ratio < 0.168 ||
      (ego_info_under_slot.slot_occupied_ratio > 0.708 &&
       !frame_.ego_should_stop_by_slot_jump)) {
    frame_.car_already_move_dist = 0.0;
    frame_.ego_should_stop_by_slot_jump = false;
    return 5.01;
  }

  const auto& param = apa_param.GetParam();

  if (frame_.slot_jump_lat_err < param.finish_lat_err_strict - 1e-3 &&
      frame_.slot_jump_heading_err < param.finish_heading_err - 1e-2) {
    frame_.car_already_move_dist = 0.0;
    frame_.ego_should_stop_by_slot_jump = false;
    return 5.01;
  }

  // 上一帧自车沿路径已经行驶的距离
  const double car_already_move_dist_last =
      frame_.current_path_length - frame_.remain_dist_path_last;

  // 当前帧自车沿路径已经行驶的距离
  const double car_already_move_dist =
      frame_.current_path_length - frame_.remain_dist_path;

  if (!frame_.ego_should_stop_by_slot_jump) {
    double stop_dist1, stop_dist2;
    for (const auto& path_point : current_path_point_global_vec_) {
      if (path_point.pos.x() > ego_info_under_slot.target_pose.pos.x() + 1.68) {
        stop_dist1 = path_point.s;
      }
      if (std::fabs(path_point.heading) * kRad2Deg > 12.68) {
        stop_dist2 = path_point.s;
      }
    }

    stop_dist1 -= car_already_move_dist;
    stop_dist2 -= car_already_move_dist;
    frame_.ego_should_stop_dist_by_slot_jump =
        std::max(std::min(stop_dist1, stop_dist2), 1.268);
  }

  // if (frame_.remain_dist_path < ego_stop_dist + 0.168) {

  // }

  // 车位跳动剩余距离等于停止距离减去从应该停车时刻到现在行驶的累计距离
  const double remain_dist_slot_jump =
      frame_.ego_should_stop_dist_by_slot_jump - frame_.car_already_move_dist;

  // 计算从应该停车时刻到现在行驶的累计距离
  frame_.car_already_move_dist +=
      std::fabs(car_already_move_dist - car_already_move_dist_last);

  ILOG_INFO
      << "should stop because of the slot jump much, remain_dist_slot_jump = "
      << remain_dist_slot_jump
      << "  ego_stop_dist = " << frame_.ego_should_stop_dist_by_slot_jump;

  frame_.ego_should_stop_by_slot_jump = true;

  return remain_dist_slot_jump;
}

const bool PerpendicularTailInScenario::PostProcessPathAccordingRemainDist(
    const double remain_dist) {
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
    if (s > remain_dist) {
      ILOG_INFO << "path shoule be shorten";
      if (remain_dist - s_vec.back() > 0.036 && frame_.spline_success) {
        x_vec.emplace_back(frame_.x_s_spline(remain_dist));
        y_vec.emplace_back(frame_.y_s_spline(remain_dist));
        heading_vec.emplace_back(heading_vec.back());
        s_vec.emplace_back(remain_dist);
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

const bool PerpendicularTailInScenario::CheckDynamicPlanPathOptimal() {
  const ApaParameters& param = apa_param.GetParam();
  // 拿到之前的路径 并转换到如今的车位坐标系下
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();
  geometry_lib::GeometryPath geometry_path_bef(all_plan_path_vec_);
  geometry_path_bef.GlobalToLocal(ego_info_under_slot.g2l_tf);

  // 拿到现在的路径
  const geometry_lib::GeometryPath geometry_path_now(
      apa_world_ptr_->GetParkingTaskInterfacePtr()
          ->GetPerpendicularTailInPathGeneratorPtr()
          ->GetOutput()
          .path_segment_vec);

  if (geometry_path_now.path_segment_vec.size() < 1) {
    return false;
  }

  if (geometry_path_now.path_segment_vec.back().seg_type !=
      geometry_lib::SEG_TYPE_LINE) {
    return false;
  }

  if (geometry_path_now.gear_change_count > 0) {
    ILOG_INFO << "now path gear change count bigger than 0";
    return false;
  }

  if (geometry_path_bef.gear_change_count > 0) {
    return true;
  }

  // 拿到之前的规划终点
  const geometry_lib::PathPoint tar_pose_bef = geometry_path_bef.end_pose;
  const geometry_lib::PathPoint front_pose_bef =
      GetCarFrontPoseFromCarPose(tar_pose_bef);

  // 拿到现在的规划终点
  const geometry_lib::PathPoint tar_pose_now = geometry_path_now.end_pose;
  const geometry_lib::PathPoint front_pose_now =
      GetCarFrontPoseFromCarPose(tar_pose_now);

  // 拿到实际终点
  const geometry_lib::PathPoint tar_pose_real = ego_info_under_slot.target_pose;
  const geometry_lib::PathPoint front_pose_real =
      GetCarFrontPoseFromCarPose(tar_pose_real);

  // 计算之前规划终点与当前实际终点的横纵航向误差
  const double lat_err_bef = (tar_pose_bef.pos - tar_pose_real.pos).y();
  const double front_lat_err_bef = (front_pose_bef.pos - tar_pose_real.pos).y();
  const double lon_err_bef = (tar_pose_bef.pos - tar_pose_real.pos).x();
  const double heading_err_bef =
      (tar_pose_bef.heading - tar_pose_real.heading) * kRad2Deg;

  // 计算现在规划终点与当前实际终点的横纵航向误差
  const double lat_err_now = (tar_pose_now.pos - tar_pose_real.pos).y();
  const double front_lat_err_now = (front_pose_now.pos - tar_pose_real.pos).y();
  const double lon_err_now = (tar_pose_now.pos - tar_pose_real.pos).x();
  const double heading_err_now =
      (tar_pose_now.heading - tar_pose_real.heading) * kRad2Deg;

  ILOG_INFO << "lat_err_bef = " << lat_err_bef
            << "  front_lat_err_bef = " << front_lat_err_bef
            << "  lon_err_bef = " << lon_err_bef
            << "  heading_err_bef = " << heading_err_bef
            << "  lat_err_now = " << lat_err_now
            << "  front_lat_err_now = " << front_lat_err_now
            << "  lon_err_now = " << lon_err_now
            << "  heading_err_now = " << heading_err_now;

  if (lon_err_bef > param.car_to_limiter_dis - 0.068 &&
      geometry_lib::IsTwoNumerEqual(lon_err_now, 0.0)) {
    return true;
  }

  bool bef_path_meet_finish = true;
  if (std::fabs(lat_err_bef) > param.finish_lat_err ||
      std::fabs(front_lat_err_bef) > param.finish_lat_err ||
      std::fabs(heading_err_bef) > param.finish_heading_err) {
    bef_path_meet_finish = false;
  }

  bool now_path_meet_finish = true;
  if (std::fabs(lat_err_now) > param.finish_lat_err ||
      std::fabs(front_lat_err_now) > param.finish_lat_err ||
      std::fabs(heading_err_now) > param.finish_heading_err) {
    now_path_meet_finish = false;
  }

  if (!bef_path_meet_finish && now_path_meet_finish) {
    return true;
  }

  if (bef_path_meet_finish && !now_path_meet_finish) {
    return false;
  }

  // 如果之前路径误差越小 那么对现在路径的最后一段直线长度要求就越高
  double line_length = 0.0;
  for (int i = geometry_path_now.path_segment_vec.size() - 1; i >= 0; i--) {
    const auto& seg = geometry_path_now.path_segment_vec[i];
    if (seg.seg_steer == geometry_lib::SEG_TYPE_LINE &&
        seg.seg_gear == geometry_lib::SEG_GEAR_REVERSE) {
      line_length += geometry_path_now.path_segment_vec[i].Getlength();
    } else {
      break;
    }
  }
  const std::vector<double> lat_err_tab{0.01, 0.03, 0.05, 0.07, 0.09, 0.10};
  const std::vector<double> line_length_tab{1.3, 1.1, 0.9, 0.7, 0.5, 0.368};
  const double min_line_length = mathlib::Interp1(
      lat_err_tab, line_length_tab,
      std::max(std::fabs(lat_err_bef), std::fabs(front_lat_err_bef)));

  ILOG_INFO << "now line length = " << line_length
            << "  min line length = " << min_line_length;
  if (line_length < min_line_length) {
    ILOG_INFO << "the last line length is small";
    return false;
  }

  const double allow_err = 0.04;
  const double front_allow_err = 0.06;

  if (std::fabs(lat_err_bef) < std::fabs(lat_err_now) + allow_err &&
      std::fabs(front_lat_err_bef) <
          std::fabs(front_lat_err_now) + front_allow_err) {
    if (geometry_path_now.IsHasSTurnPath()) {
      ILOG_INFO << "now path has s turn";
      return false;
    } else {
      return true;
    }
  }

  if (std::fabs(lat_err_bef) > std::fabs(lat_err_now) + allow_err ||
      std::fabs(front_lat_err_bef) >
          std::fabs(front_lat_err_now) + front_allow_err) {
    return true;
  }

  return false;

  // 如果现在路径当前转向与当前转角方向相同
  const double steer_angle =
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetSteerWheelAngle() *
      kRad2Deg;
  const double straight_angle = 68.0;
  const bool steer_case_1 =
      steer_angle > straight_angle &&
      geometry_path_now.cur_steer == geometry_lib::SEG_STEER_LEFT;
  const bool steer_case_2 =
      steer_angle < -straight_angle &&
      geometry_path_now.cur_steer == geometry_lib::SEG_STEER_RIGHT;
  const bool steer_case_3 =
      mathlib::IsInBound(steer_angle, -straight_angle, straight_angle) &&
      geometry_path_now.cur_steer == geometry_lib::SEG_STEER_STRAIGHT;
  if (steer_case_1 || steer_case_2 || steer_case_3) {
    return true;
  }

  return false;
}

const bool PerpendicularTailInScenario::LateralPathOptimize(
    std::vector<geometry_lib::PathPoint>& optimal_path_vec) {
  const GeometryPathOutput& path_plan_output =
      apa_world_ptr_->GetParkingTaskInterfacePtr()
          ->GetPerpendicularTailInPathGeneratorPtr()
          ->GetOutput();
  const double sample_ds = path_plan_output.actual_ds;
  const std::vector<pnc::geometry_lib::PathPoint>& pt_vec =
      path_plan_output.path_point_vec;
  const double cur_gear_length = path_plan_output.cur_gear_length;
  const SimulationParam& simu_param = apa_world_ptr_->GetSimuParam();
  if (pt_vec.size() < 3 ||
      cur_gear_length < apa_param.GetParam().min_opt_path_length) {
    return false;
  }
  bool cilqr_optimization_enable = true;
  bool perpendicular_optimization_enable = true;
  if (!simu_param.is_simulation) {
    perpendicular_optimization_enable =
        apa_param.GetParam().perpendicular_lat_opt_enable;

    cilqr_optimization_enable =
        apa_param.GetParam().cilqr_path_optimization_enable;
  } else {
    perpendicular_optimization_enable = simu_param.is_path_optimization;
    cilqr_optimization_enable = simu_param.is_cilqr_optimization;
  }

  if (!perpendicular_optimization_enable) {
    return false;
  }

  LateralPathOptimizer::Parameter param;
  param.sample_ds = 0.01;
  if (simu_param.is_simulation) {
    param.q_ref_xy = simu_param.q_ref_xy;
    param.q_ref_theta = simu_param.q_ref_theta;
    param.q_terminal_xy = simu_param.q_terminal_xy;
    param.q_terminal_theta = simu_param.q_terminal_theta;
    param.q_k = simu_param.q_k;
    param.q_u = simu_param.q_u;
    param.q_k_bound = simu_param.q_k_bound;
    param.q_u_bound = simu_param.q_u_bound;
  } else {
    param.q_ref_xy = 50.0;
    param.q_ref_theta = 50.0;
    param.q_terminal_xy = 5000.0;
    param.q_terminal_theta = 168000.0;
    param.q_k = 5.0;
    param.q_u = 5.0;
    param.q_k_bound = 100.0;
    param.q_u_bound = 100.0;
  }

  const double start_time = IflyTime::Now_ms();

  apa_world_ptr_->GetLateralPathOptimizerPtr()->Init(cilqr_optimization_enable);
  apa_world_ptr_->GetLateralPathOptimizerPtr()->SetParam(param);
  apa_world_ptr_->GetLateralPathOptimizerPtr()->Update(pt_vec,
                                                       frame_.gear_command);

  ILOG_INFO << "lat_path_opt_cost_time_ms = " << IflyTime::Now_ms() - start_time
            << " ms";

  const std::vector<pnc::geometry_lib::PathPoint>& origin_optimized_path_vec =
      apa_world_ptr_->GetLateralPathOptimizerPtr()->GetOriginOutputPathVec();

  if (origin_optimized_path_vec.size() < 3) {
    ILOG_INFO << "origin_optimized_path_vec.size() < 3";
    return false;
  }

  const size_t max_pt_number =
      PLANNING_TRAJ_POINTS_MAX_NUM - APA_COMPARE_PLANNING_TRAJ_POINTS_MAX_NUM;
  optimal_path_vec.clear();
  optimal_path_vec.reserve(max_pt_number + 2);
  if (origin_optimized_path_vec.size() <= max_pt_number) {
    optimal_path_vec = origin_optimized_path_vec;
  } else {
    const double length = origin_optimized_path_vec.back().s;
    const double resampled_ds = length / max_pt_number;
    std::vector<double> x_vec;
    std::vector<double> y_vec;
    std::vector<double> s_vec;
    std::vector<double> heading_vec;
    x_vec.reserve(origin_optimized_path_vec.size());
    y_vec.reserve(origin_optimized_path_vec.size());
    s_vec.reserve(origin_optimized_path_vec.size());
    heading_vec.reserve(origin_optimized_path_vec.size());
    for (const auto& pt : origin_optimized_path_vec) {
      x_vec.emplace_back(pt.pos.x());
      y_vec.emplace_back(pt.pos.y());
      s_vec.emplace_back(pt.s);
      heading_vec.emplace_back(pt.heading);
    }
    mathlib::spline x_s_spline;
    mathlib::spline y_s_spline;
    mathlib::spline heading_s_spline;
    x_s_spline.set_points(s_vec, x_vec);
    y_s_spline.set_points(s_vec, y_vec);
    heading_s_spline.set_points(s_vec, heading_vec);
    geometry_lib::PathPoint tmp_pt;
    double ds = 0.0;
    for (size_t i = 1; i < max_pt_number; ++i) {
      tmp_pt.pos << x_s_spline(ds), y_s_spline(ds);
      tmp_pt.heading = heading_s_spline(ds);
      tmp_pt.s = ds;
      ds += resampled_ds;
      if (optimal_path_vec.size() > 0) {
        const double dist_err =
            (tmp_pt.pos - optimal_path_vec.back().pos).norm();
        const double heading_err =
            geometry_lib::NormalizeAngle(tmp_pt.heading -
                                         optimal_path_vec.back().heading) *
            kRad2Deg;
        if (dist_err < 0.01 || std::fabs(heading_err) > 36.8) {
          continue;
        }
      }
      optimal_path_vec.emplace_back(tmp_pt);
    }

    // 可能没采样到终点
    if (optimal_path_vec.size() > 0) {
      tmp_pt = optimal_path_vec.back();
      if (s_vec.back() - optimal_path_vec.back().s > 1e-2) {
        tmp_pt.pos << x_s_spline(s_vec.back()), y_s_spline(s_vec.back());
        tmp_pt.heading = heading_s_spline(s_vec.back());
        tmp_pt.s = s_vec.back();
        optimal_path_vec.emplace_back(tmp_pt);
      }
    }
  }

  if (optimal_path_vec.size() < 3) {
    ILOG_INFO << "optimized_path_vec.size() < 3";
    return false;
  }

  // 检查路径长度是否有较大变化
  if (std::fabs(cur_gear_length - optimal_path_vec.back().s) > 5e-2) {
    ILOG_INFO << "length is not the same, cur_gear_length = " << cur_gear_length
              << "  optimized_path length = " << optimal_path_vec.back().s;
    return false;
  }

  // 检查起点终点是否一致
  if (!pnc::geometry_lib::CheckTwoPoseIsSame(
          optimal_path_vec.front(), pt_vec.front(), 0.02, 0.3 / 57.3) ||
      !pnc::geometry_lib::CheckTwoPoseIsSame(optimal_path_vec.back(),
                                             pt_vec.back(), 0.02, 0.3 / 57.3)) {
    ILOG_INFO << "start or end pose is not same";
    return false;
  }

  // 检查优化后的路径是否很奇怪

  // 检查是否碰撞
  if (apa_world_ptr_->GetCollisionDetectorInterfacePtr()
          ->GetGJKCollisionDetectorPtr()
          ->Update(optimal_path_vec, 0.08, 0.0, GJKColDetRequest())
          .col_flag) {
    ILOG_INFO << "the optimal path is col";
    return false;
  }

  ILOG_INFO << "optimal path success, pos_err = "
            << (pt_vec.back().pos - optimal_path_vec.back().pos).norm()
            << "  heading_err = "
            << (pt_vec.back().heading - optimal_path_vec.back().heading) *
                   kRad2Deg;

  return true;
}

const PerpendicularTailInScenario::SlotObsType
PerpendicularTailInScenario::CalSlotObsType(const Eigen::Vector2d& obs_slot) {
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();
  // 2米2的车位重规划考虑的障碍物单侧最多入侵车位20厘米
  // 给障碍物跳动留下些许余量
  const std::vector<double> slot_width_tab{2.2, 2.4, 2.6, 2.8, 3.0, 3.2};
  const std::vector<double> invasion_dist_tab{0.20, 0.25, 0.30, 0.35, 0.40};
  const double dy1 = mathlib::Interp1(slot_width_tab, invasion_dist_tab,
                                      ego_info_under_slot.slot.slot_width_);

  // 内外侧障碍物往远离车位的一遍考虑1.68米就可以
  const double dy2 = 1.68;

  // 最多高于车位3.468米的障碍物可以当做内外侧障碍物
  double dx1 = 3.468;
  // 但是如果自车位置本身较低 那么内外侧障碍物考虑的x值也应该降低
  dx1 = std::min(dx1, ego_info_under_slot.cur_pose.pos.x() -
                          apa_param.GetParam().car_width * 0.5 -
                          ego_info_under_slot.slot.slot_length_);
  // 也需要有个最低考虑位置
  dx1 = std::max(dx1, 0.368);

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

  // Firstly, the default right side is the inner side, and the left side is
  // the outer side
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

const bool PerpendicularTailInScenario::CheckDynamicUpdate() {
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const ApaParameters& param = apa_param.GetParam();
  const bool gear_case =
      (frame_.gear_command == geometry_lib::SEG_GEAR_REVERSE);

  const bool car_motion_case =
      !apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag();

  const bool slot_confidence_case = (ego_info_under_slot.confidence == 1);

  const bool car_pos_case =
      ego_info_under_slot.cur_pose.pos.x() <
      (ego_info_under_slot.slot.GetOriginCornerCoordLocal().pt_01_mid.x() +
       2.68);

  const bool occupied_ratio_case = (ego_info_under_slot.slot_occupied_ratio <
                                    param.pose_slot_occupied_ratio_3);

  const bool dynamic_update_flag = gear_case && car_motion_case &&
                                   slot_confidence_case && car_pos_case &&
                                   occupied_ratio_case;

  if (dynamic_update_flag) {
    frame_.dynamic_plan_time += param.plan_time;
  } else {
    frame_.dynamic_plan_time = 0.0;
  }

  if (frame_.dynamic_plan_time > param.dynamic_plan_interval_time) {
    frame_.dynamic_plan_time = 0.0;
    return true;
  }

  return false;
}

void PerpendicularTailInScenario::Log() const {
  JSON_DEBUG_VALUE("correct_path_for_limiter", frame_.correct_path_for_limiter)
  JSON_DEBUG_VALUE("replan_flag", frame_.replan_flag)

  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const geometry_lib::LocalToGlobalTf& l2g_tf = ego_info_under_slot.l2g_tf;

  std::vector<double> obstaclesX;
  obstaclesX.reserve(100);
  std::vector<double> obstaclesY;
  obstaclesY.reserve(100);

  for (const auto& pair :
       apa_world_ptr_->GetObstacleManagerPtr()->GetObstacles()) {
    if (pair.second.GetObsAttributeType() !=
        ApaObsAttributeType::VIRTUAL_POINT_CLOUD) {
      continue;
    }
    for (const Eigen::Vector2d& pt : pair.second.GetPtClout2dLocal()) {
      const Eigen::Vector2d tmp_obstacle = l2g_tf.GetPos(pt);
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

  pt_vec[0] = pt_vec[0] + ego_info_under_slot.lat_move_dist_every_replan *
                              origin_corner_coord_global.pt_01_unit_vec;
  pt_vec[1] = pt_vec[1] + ego_info_under_slot.lat_move_dist_every_replan *
                              origin_corner_coord_global.pt_01_unit_vec;
  pt_vec[2] = pt_vec[2] + ego_info_under_slot.lat_move_dist_every_replan *
                              origin_corner_coord_global.pt_23_unit_vec;
  pt_vec[3] = pt_vec[3] + ego_info_under_slot.lat_move_dist_every_replan *
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

  JSON_DEBUG_VALUE("replan_flag", frame_.replan_flag)
  JSON_DEBUG_VALUE("is_replan_first", frame_.is_replan_first)
  JSON_DEBUG_VALUE("current_path_length", frame_.current_path_length)
  JSON_DEBUG_VALUE("path_plan_success", frame_.plan_stm.path_plan_success)
  JSON_DEBUG_VALUE("planning_status", frame_.plan_stm.planning_status)
  JSON_DEBUG_VALUE("spline_success", frame_.spline_success)
  JSON_DEBUG_VALUE("remain_dist", frame_.remain_dist_path)
  JSON_DEBUG_VALUE("remain_dist_col_det", frame_.remain_dist_col_det)
  JSON_DEBUG_VALUE("remain_dist_obs", frame_.remain_dist_obs)
  JSON_DEBUG_VALUE("remain_dist_slot_jump", frame_.remain_dist_slot_jump)
  JSON_DEBUG_VALUE("stuck_time", frame_.stuck_time)
  JSON_DEBUG_VALUE("replan_reason", frame_.replan_reason)
  JSON_DEBUG_VALUE("plan_fail_reason", frame_.plan_fail_reason)

  JSON_DEBUG_VALUE("ego_should_stop_by_slot_jump",
                   frame_.ego_should_stop_by_slot_jump)

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

  const auto& path_plan_output = apa_world_ptr_->GetParkingTaskInterfacePtr()
                                     ->GetPerpendicularTailInPathGeneratorPtr()
                                     ->GetOutput();

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